"""Dedicated ROS environment process. Publishes a map and makes one planning call."""
from array import array
import json
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point32
from bg1_leader_coverage_interfaces.srv import PlanCoverage


def main():
    directory = Path(sys.argv[1])
    config = json.loads((directory/'leader_input.json').read_text())
    rclpy.init()
    node = Node('offline_coverage_benchmark')
    process = None
    result = dict(success=False, path=[], details={})
    try:
        with (directory/'leader_server.log').open('w') as log:
            process = subprocess.Popen([config['executable']], stdout=log, stderr=subprocess.STDOUT)
        pub = node.create_publisher(OccupancyGrid, '/map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        client = node.create_client(PlanCoverage, '/leader_coverage/plan')
        if not client.wait_for_service(timeout_sec=30):
            raise TimeoutError('Leader service unavailable')
        target = np.load(directory/'leader_input.npz')['target']
        h, w = target.shape
        r = config['resolution']
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.info.width, message.info.height = w, h
        message.info.resolution = r
        message.info.origin.orientation.w = 1.
        message.data = array('b', np.where(np.flipud(target), 0, 100).astype(np.int8).tobytes())
        deadline = time.monotonic()+20
        while pub.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                raise TimeoutError('Leader map subscription unavailable')
            rclpy.spin_once(node, timeout_sec=.05)
        pub.publish(message)
        deadline = time.monotonic()+.3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        request = PlanCoverage.Request()
        request.region.header.frame_id = 'map'
        request.region.polygon.points = [Point32(x=float(x), y=float(y)) for x,y in
                                        [(r,r),((w-1)*r,r),((w-1)*r,(h-1)*r),(r,(h-1)*r)]]
        request.start.x, request.start.y = config['start']
        request.pattern = 0
        request.stripe_spacing = config['tool_width']
        request.clearance = config['clearance']
        future = client.call_async(request)
        deadline = time.monotonic()+120
        while not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError('Leader planning timed out')
            if process.poll() is not None:
                raise RuntimeError(f'Leader exited {process.returncode}')
            rclpy.spin_once(node, timeout_sec=.01)
        response = future.result()
        result.update(success=response.success, message=response.message,
                      path=[[p.pose.position.x,p.pose.position.y] for p in response.coverage_path.poses],
                      details={'request_region': [[p.x,p.y] for p in request.region.polygon.points],
                               'requested_start_m': config['start'], 'pattern': 0})
    except Exception as error:
        result['message'] = f'{type(error).__name__}: {error}'
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        if process is not None:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            result['details']['server_exit_code'] = process.returncode
        (directory/'leader_output.json').write_text(json.dumps(result)+'\n')


if __name__ == '__main__':
    main()
