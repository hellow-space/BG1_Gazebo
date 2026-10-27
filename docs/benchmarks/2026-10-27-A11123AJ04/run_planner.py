"""Run one unmodified planner in its own overlay and ROS domain; no motion."""
import argparse
from array import array
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import yaml

out = Path(__file__).resolve().parent
root = out.parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--planner', choices=['leader','f2c'], required=True)
args = parser.parse_args()
config = json.loads((out/'inputs.json').read_text())
run = dict(planner=args.planner, runs=[], completed=False, motion_executed=False,
           ros_domain_id=os.environ.get('ROS_DOMAIN_ID'), config_file='inputs.json')
process = None
rclpy.init()
node = Node('benchmark_'+args.planner)


def wait(future, timeout=180):
    deadline = time.monotonic()+timeout
    while not future.done():
        if time.monotonic() > deadline:
            raise TimeoutError('Planner response timed out')
        if process is not None and process.poll() is not None:
            raise RuntimeError(f'Planner process exited with {process.returncode}')
        rclpy.spin_once(node, timeout_sec=.01)
    return future.result()


try:
    log = open(out/(args.planner+'_server.log'), 'w')
    if args.planner == 'leader':
        from nav_msgs.msg import OccupancyGrid
        from geometry_msgs.msg import Point32
        from bg1_leader_coverage_interfaces.srv import PlanCoverage
        executable = root/'leader_coverage_ws/build/bg1_leader_coverage/planner_server'
        process = subprocess.Popen([str(executable)], stdout=log, stderr=subprocess.STDOUT)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub = node.create_publisher(OccupancyGrid, '/map', qos)
        client = node.create_client(PlanCoverage, '/leader_coverage/plan')
        assert client.wait_for_service(timeout_sec=30)
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.info.width = config['width']; m.info.height = config['height']
        m.info.resolution = config['resolution_m']; m.info.origin.orientation.w = 1.
        target = np.load(out/'inputs.npz')['target']
        m.data = array('b', np.where(target, 0, 100).astype(np.int8).tobytes())
        deadline = time.monotonic()+20
        while pub.get_subscription_count() == 0:
            assert time.monotonic() < deadline, 'Planner map subscription unavailable'
            rclpy.spin_once(node, timeout_sec=.05)
        pub.publish(m)
        until = time.monotonic()+.3
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.01)
        request = PlanCoverage.Request()
        request.region.header.frame_id = 'map'
        r = config['resolution_m']; w = config['width']; h = config['height']
        request.region.polygon.points = [Point32(x=float(x),y=float(y)) for x,y in
                                        [(r,r),((w-1)*r,r),((w-1)*r,(h-1)*r),(r,(h-1)*r)]]
        request.start.x, request.start.y = config['start']
        request.pattern = 0; request.stripe_spacing = .6; request.clearance = 1.
        run['request'] = dict(region=[[p.x,p.y] for p in request.region.polygon.points],
                              start=config['start'], pattern=0, stripe_spacing_m=.6, clearance_m=1.)

        def compute():
            started = time.perf_counter()
            response = wait(client.call_async(request))
            seconds = time.perf_counter()-started
            return response.coverage_path, dict(success=response.success, message=response.message,
                                                call_seconds=seconds)
    else:
        from lifecycle_msgs.srv import ChangeState
        from rclpy.action import ActionClient
        from opennav_coverage_msgs.action import ComputeCoveragePath
        from opennav_coverage_msgs.msg import Coordinate, Coordinates
        executable = root/'coverage_ws/build/opennav_coverage/opennav_coverage'
        parameters = yaml.safe_load((root/'coverage_ws/src/bg1_coverage_demo/config/coverage.yaml').read_text())
        parameters['coverage_server']['ros__parameters'].update(
            use_sim_time=False, operation_width=.6, default_swath_angle=math.pi/2,
            default_headland_width=1., min_turning_radius=.6)
        parameter_file = out/'f2c_config.yaml'
        parameter_file.write_text(yaml.safe_dump(parameters,sort_keys=False))
        process = subprocess.Popen([str(executable),'--ros-args','-r','__node:=coverage_server',
                                    '--params-file',str(parameter_file)],stdout=log,stderr=subprocess.STDOUT)
        lifecycle = node.create_client(ChangeState, '/coverage_server/change_state')
        assert lifecycle.wait_for_service(timeout_sec=30)
        for transition in (1,3):
            change = ChangeState.Request(); change.transition.id = transition
            assert wait(lifecycle.call_async(change),30).success
        client = ActionClient(node, ComputeCoveragePath, '/compute_coverage_path')
        assert client.wait_for_server(timeout_sec=30)
        polygon = json.loads((out/'target_polygon.json').read_text())
        request = ComputeCoveragePath.Goal()
        request.frame_id = 'map'
        request.generate_headland = request.generate_route = request.generate_path = True
        request.polygons = [Coordinates(coordinates=[Coordinate(axis1=float(x),axis2=float(y))
                                                     for x,y in ring])
                            for ring in polygon['geometry']['coordinates']]
        run['request'] = dict(polygon_file='target_polygon.json', generate_headland=True,
                              generate_route=True, generate_path=True, parameters_file='f2c_config.yaml',
                              start_constraint_supported=False)

        def compute():
            started = time.perf_counter()
            goal = wait(client.send_goal_async(request),30)
            assert goal.accepted, 'Coverage action goal rejected'
            response = wait(goal.get_result_async())
            seconds = time.perf_counter()-started
            msg = response.result
            return msg.nav_path, dict(success=response.status==4 and msg.error_code==0,
                                     action_status=response.status,error_code=msg.error_code,
                                     call_seconds=seconds,
                                     server_planning_seconds=msg.planning_time.sec+msg.planning_time.nanosec*1e-9,
                                     swaths=len(msg.coverage_path.swaths))
    run['executable'] = str(executable.relative_to(root))
    run['server_pid'] = process.pid
    for index in range(config['warmup_runs']+config['measured_runs']):
        message, result = compute()
        path = []
        for pose in message.poses:
            p, q = pose.pose.position, pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            path.append([p.x,p.y,yaw])
        result.update(warmup=index<config['warmup_runs'],poses=len(path),
                      path_sha256=hashlib.sha256(np.asarray(path,dtype='<f8').tobytes()).hexdigest())
        run['runs'].append(result)
        run['path'] = path
        print(json.dumps(dict(index=index,**result)),flush=True)
        if not result['success'] or len(path)<2:
            raise RuntimeError(f'{args.planner} did not return a successful nonempty path')
    run['completed'] = True
except Exception as error:
    run['error'] = str(error)
    print('FAILED: '+str(error),flush=True)
finally:
    node.destroy_node()
    rclpy.try_shutdown()
    if process is not None:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill();process.wait()
        run['server_exit_code'] = process.returncode
    (out/(args.planner+'_raw.json')).write_text(json.dumps(run,indent=2)+'\n')
raise SystemExit(0 if run['completed'] else 1)
