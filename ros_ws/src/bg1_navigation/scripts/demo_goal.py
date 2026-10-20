#!/usr/bin/env python3
"""Send a Nav2 goal and record Gazebo ground truth (simulation only)."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path as NavPath
from action_msgs.msg import GoalStatus


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x',type=float,default=4.8)
    parser.add_argument('--y',type=float,default=0.0)
    parser.add_argument('--yaw',type=float,default=0.0)
    parser.add_argument('--timeout',type=float,default=180.0)
    parser.add_argument('--output',type=Path,default=Path('/tmp/bg1_nav2_result.json'))
    args = parser.parse_args()
    rclpy.init()
    node = Node('bg1_navigation_demo', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    stop = node.create_publisher(Twist,'/cmd_vel',10)
    state = {'scan':None,'amcl':None,'path':[]}
    node.create_subscription(LaserScan,'/scan',lambda m: state.update(scan=m),qos_profile_sensor_data)
    pose_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m: state.update(amcl=m),pose_qos)
    node.create_subscription(NavPath,'/plan',lambda m: state.update(path=[[p.pose.position.x,p.pose.position.y] for p in m.poses]),10)
    samples = []
    process = subprocess.Popen(['gz','topic','-e','-t','/world/bg1_navigation/dynamic_pose/info','--json-output'],
        stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)
    def collect():
        last = 0.0
        for line in process.stdout:
            now = time.monotonic()
            if now-last < .1:
                continue
            try:
                data = json.loads(line)
                pose = next(p for p in data['pose'] if p['name']=='bg1_pro')
                q = pose['orientation']
                yaw = math.atan2(2*(q.get('w',0)*q.get('z',0)+q.get('x',0)*q.get('y',0)),
                                 1-2*(q.get('y',0)**2+q.get('z',0)**2))
                samples.append({'t':now, 'x':pose['position'].get('x',0),
                                'y':pose['position'].get('y',0),'yaw':yaw})
                last = now
            except (ValueError,KeyError,StopIteration):
                continue
    thread = threading.Thread(target=collect,daemon=True)
    thread.start()
    handle = None
    report = {'passed':False, 'goal':[args.x,args.y,args.yaw]}
    try:
        deadline = time.monotonic()+60
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if client.server_is_ready() and state['amcl'] is not None and state['scan'] is not None and samples:
                break
        else:
            raise RuntimeError('Nav2/AMCL/laser/ground truth did not become ready within 60 s')
        scan = state['scan']
        valid = sum(math.isfinite(v) and scan.range_min <= v <= scan.range_max for v in scan.ranges)
        report['scan_frame'] = scan.header.frame_id
        report['finite_scan_points'] = valid
        if valid < 30:
            raise RuntimeError(f'Laser has too few valid obstacle returns: {valid}')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x,goal.pose.pose.position.y = args.x,args.y
        goal.pose.pose.orientation.z = math.sin(args.yaw/2)
        goal.pose.pose.orientation.w = math.cos(args.yaw/2)
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        if not future.done():
            raise RuntimeError('Goal acceptance timed out')
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('Nav2 rejected goal')
        result = handle.get_result_async()
        started = time.monotonic()
        last_log = 0
        while not result.done() and time.monotonic()-started<args.timeout:
            rclpy.spin_once(node,timeout_sec=.1)
            if time.monotonic()-last_log>10:
                last_log=time.monotonic()
                print(f'Navigating: elapsed={time.monotonic()-started:.1f}s, actual={samples[-1] if samples else None}',flush=True)
        report['elapsed_seconds'] = time.monotonic()-started
        if not result.done():
            raise RuntimeError('Navigation timed out')
        report['action_status'] = result.result().status
        time.sleep(.5)
        actual = samples[-1]
        report['position_error_m'] = math.hypot(actual['x']-args.x,actual['y']-args.y)
        report['yaw_error_rad'] = abs(math.atan2(math.sin(actual['yaw']-args.yaw),math.cos(actual['yaw']-args.yaw)))
        report['passed'] = (report['action_status']==GoalStatus.STATUS_SUCCEEDED
                            and report['position_error_m']<.35 and report['yaw_error_rad']<.35)
    except Exception as error:
        report['error'] = str(error)
    finally:
        if handle is not None and not report['passed']:
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node,cancel,timeout_sec=5)
        stop.publish(Twist())
        rclpy.spin_once(node,timeout_sec=.1)
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        thread.join(timeout=1)
        report['ground_truth_trajectory'] = samples
        report['last_planned_path'] = state['path']
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('ground_truth_trajectory','last_planned_path')},indent=2))
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
