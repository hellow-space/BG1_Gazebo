#!/usr/bin/env python3
"""Exercise the BG1 navigation interfaces and write JSON runtime evidence."""

import json
import math
from pathlib import Path
import time
from typing import NamedTuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from tf2_msgs.msg import TFMessage


class MotionSample(NamedTuple):
    timestamp: float
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    linear_speed: float
    angular_speed: float


def _distance(first, last):
    return math.hypot(last.x - first.x, last.y - first.y)


def _wrapped_delta(current, previous):
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


def stationary_window_ok(samples, max_drift=0.03, max_tilt=0.12, max_height_change=0.03):
    if len(samples) < 2:
        return False
    return (
        _distance(samples[0], samples[-1]) <= max_drift
        and max(abs(sample.roll) for sample in samples) <= max_tilt
        and max(abs(sample.pitch) for sample in samples) <= max_tilt
        and max(sample.z for sample in samples) - min(sample.z for sample in samples) <= max_height_change
    )


def forward_motion_ok(samples, minimum_x_progress=0.10, maximum_lateral_drift=0.20):
    if len(samples) < 2:
        return False
    return (
        samples[-1].x - samples[0].x >= minimum_x_progress
        and abs(samples[-1].y - samples[0].y) <= maximum_lateral_drift
    )


def positive_rotation_ok(samples, minimum_yaw_change=0.15):
    if len(samples) < 2:
        return False
    change = sum(_wrapped_delta(current.yaw, previous.yaw) for previous, current in zip(samples, samples[1:]))
    return change >= minimum_yaw_change


def stop_window_ok(
    samples,
    maximum_linear_speed=0.05,
    maximum_angular_speed=0.05,
    settling_seconds=0.0,
):
    if not samples:
        return False
    settled = [sample for sample in samples if sample.timestamp - samples[0].timestamp >= settling_seconds]
    return bool(settled) and all(
        abs(sample.linear_speed) <= maximum_linear_speed
        and abs(sample.angular_speed) <= maximum_angular_speed
        for sample in settled
    )


def speed_ceiling_ok(samples, maximum=1.0, tolerance=0.03):
    return bool(samples) and max(abs(sample.linear_speed) for sample in samples) <= maximum + tolerance


def quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class RuntimeVerifier(Node):
    REQUIRED_TOPICS = ("odom", "scan", "joint_states", "tf", "clock")

    def __init__(self):
        super().__init__("bg1_runtime_verifier")
        self.last_received = {name: None for name in self.REQUIRED_TOPICS}
        self.odom_samples = []
        self.command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(LaserScan, "/scan", lambda _: self._mark("scan"), qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", lambda _: self._mark("joint_states"), 20)
        self.create_subscription(TFMessage, "/tf", lambda _: self._mark("tf"), 20)
        self.create_subscription(Clock, "/clock", lambda _: self._mark("clock"), 20)

    def _mark(self, topic):
        self.last_received[topic] = time.monotonic()

    def _on_odom(self, message):
        self._mark("odom")
        pose = message.pose.pose
        twist = message.twist.twist
        roll, pitch, yaw = quaternion_to_euler(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        self.odom_samples.append(
            MotionSample(
                time.monotonic(),
                pose.position.x,
                pose.position.y,
                pose.position.z,
                roll,
                pitch,
                yaw,
                math.hypot(twist.linear.x, twist.linear.y),
                twist.angular.z,
            )
        )

    def publish_command(self, linear=0.0, angular=0.0):
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.command_publisher.publish(message)

    def wait_for_topics(self, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(value is not None for value in self.last_received.values()):
                now = time.monotonic()
                return {name: now - stamp for name, stamp in self.last_received.items()}
        missing = [name for name, stamp in self.last_received.items() if stamp is None]
        raise RuntimeError(f"Timed out waiting for topics: {missing}")

    def collect(self, duration, linear=0.0, angular=0.0):
        start_index = len(self.odom_samples)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publish_command(linear, angular)
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.odom_samples[start_index:]

    def stop(self):
        for _ in range(10):
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.02)


def _sample_dicts(samples):
    return [dict(sample._asdict()) for sample in samples]


def run_verification(node):
    evidence = {"passed": False, "checks": {}, "phases": {}}
    evidence["topic_age_seconds"] = node.wait_for_topics()

    stationary = node.collect(2.0)
    evidence["phases"]["stationary"] = _sample_dicts(stationary)
    evidence["checks"]["stationary"] = stationary_window_ok(stationary)

    forward = node.collect(3.0, linear=0.25)
    evidence["phases"]["forward"] = _sample_dicts(forward)
    evidence["checks"]["forward"] = forward_motion_ok(forward)

    rotation = node.collect(2.5, angular=0.4)
    evidence["phases"]["rotation"] = _sample_dicts(rotation)
    evidence["checks"]["rotation"] = positive_rotation_ok(rotation)

    node.stop()
    stopped = node.collect(1.5)
    evidence["phases"]["stopped"] = _sample_dicts(stopped)
    evidence["checks"]["stopped"] = stop_window_ok(stopped, settling_seconds=0.75)

    speed = node.collect(3.0, linear=1.0)
    evidence["phases"]["speed_ceiling"] = _sample_dicts(speed)
    evidence["checks"]["speed_ceiling"] = speed_ceiling_ok(speed)
    evidence["passed"] = all(evidence["checks"].values())
    return evidence


def main():
    output_directory = Path("/tmp/bg1_gazebo_verification")
    output_directory.mkdir(parents=True, exist_ok=True)
    output_path = output_directory / "runtime_evidence.json"
    rclpy.init()
    node = RuntimeVerifier()
    evidence = {"passed": False, "error": "verification did not complete"}
    exit_code = 0
    try:
        evidence = run_verification(node)
        if evidence["passed"]:
            node.get_logger().info(f"All BG1 runtime checks passed; evidence: {output_path}")
        else:
            failed = [name for name, passed in evidence["checks"].items() if not passed]
            node.get_logger().error(f"Runtime checks failed: {failed}")
            exit_code = 1
    except Exception as error:
        evidence = {"passed": False, "error": str(error)}
        node.get_logger().error(str(error))
        exit_code = 1
    finally:
        node.stop()
        output_path.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n")
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
