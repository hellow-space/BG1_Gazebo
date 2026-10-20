#!/usr/bin/env python3
"""Explicit plan/execute client for the obstacle-free BG1 demonstration."""
import argparse
import json
import math
import signal
from pathlib import Path
import subprocess
import threading
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Path as NavPath,Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from tf2_ros import Buffer,TransformListener
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinates,Coordinate
from nav2_msgs.action import FollowPath
from action_msgs.srv import CancelGoal
from lifecycle_msgs.srv import ChangeState
from coverage_geometry import validate_polygon,validate_path,validate_result,connector,finish_cleanup,quaternion_yaw,validate_motion


def yaw(q):
    return quaternion_yaw(q.x,q.y,q.z,q.w)


def poses_of(path):
    for p in path.poses:
        if p.header.frame_id not in ('',path.header.frame_id) or not math.isfinite(p.pose.position.z):
            raise ValueError('Inconsistent pose frame or nonfinite pose height')
    return [(p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)) for p in path.poses]


def to_path(node,poses):
    path=NavPath()
    path.header.frame_id='map';path.header.stamp=node.get_clock().now().to_msg()
    for x,y,a in poses:
        p=PoseStamped();p.header=path.header
        p.pose.position.x=float(x);p.pose.position.y=float(y)
        p.pose.orientation.z=math.sin(a/2);p.pose.orientation.w=math.cos(a/2)
        path.poses.append(p)
    return path


class Task(Node):
    def __init__(self):
        super().__init__('bg1_coverage_task',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.coverage=ActionClient(self,ComputeCoveragePath,'/compute_coverage_path')
        self.follow=ActionClient(self,FollowPath,'/follow_path')
        self.stop_pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.path_pub=self.create_publisher(NavPath,'/coverage_demo/path_input',10)
        self.active=None;self.pending_goal=None;self.active_client=None
        self.tf_buffer=Buffer();self.tf_listener=TransformListener(self.tf_buffer,self)
        self.odom=None;self.scan=None;self.samples=[];self.process=None;self.thread=None
        self.create_subscription(Odometry,'/odom',lambda m:setattr(self,'odom',m),10)
        self.create_subscription(LaserScan,'/scan',lambda m:setattr(self,'scan',m),qos_profile_sensor_data)

    def wait(self,future,timeout):
        deadline=time.monotonic()+timeout
        last_log=time.monotonic()
        while rclpy.ok() and not future.done() and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.1)
            if timeout>30 and self.samples and time.monotonic()-last_log>15:
                last_log=time.monotonic();p=self.samples[-1]
                print(f'Running: actual x={p["x"]:.2f}, y={p["y"]:.2f}, yaw={p["yaw"]:.2f}',flush=True)
        if not future.done():raise TimeoutError(f'Action/service wait exceeded {timeout:.0f} s')
        return future.result()

    def action(self,client,goal,timeout):
        deadline=time.monotonic()+30
        while not client.server_is_ready():
            if time.monotonic()>deadline:raise TimeoutError('Action server unavailable')
            rclpy.spin_once(self,timeout_sec=.1)
        self.active_client=client
        self.pending_goal=client.send_goal_async(goal)
        self.active=self.wait(self.pending_goal,10)
        self.pending_goal=None
        if not self.active.accepted:
            self.active=None
            raise RuntimeError('Action rejected')
        result=self.wait(self.active.get_result_async(),timeout)
        self.active=None
        return result

    def record(self):
        self.process=subprocess.Popen(['gz','topic','-e','-t','/world/bg1_flat/dynamic_pose/info','--json-output'],
            stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)
        def collect():
            last=0
            for line in self.process.stdout:
                now=time.monotonic()
                if now-last<.1:continue
                try:
                    d=json.loads(line)
                    p=next(v for v in d['pose'] if v['name']=='bg1_pro')
                    q=p['orientation'];pos=p['position']
                    a=math.atan2(2*(q.get('w',0)*q.get('z',0)+q.get('x',0)*q.get('y',0)),1-2*(q.get('y',0)**2+q.get('z',0)**2))
                    self.samples.append({'t':now,'x':pos.get('x',0),'y':pos.get('y',0),'yaw':a});last=now
                except (ValueError,KeyError,StopIteration):continue
        self.thread=threading.Thread(target=collect,daemon=True);self.thread.start()

    def follow_path(self,poses,timeout):
        goal=FollowPath.Goal();goal.path=to_path(self,poses)
        goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker';goal.progress_checker_id='progress_checker'
        result=self.action(self.follow,goal,timeout)
        if result.status!=4 or result.result.error_code!=0:
            raise RuntimeError(f'FollowPath failed: status={result.status}, code={result.result.error_code}, {result.result.error_msg}')
        return result.status

    def cancel_active(self,motion):
        try:
            if self.pending_goal is not None:
                self.active=self.wait(self.pending_goal,5)
                self.pending_goal=None
            if self.active is not None and self.active.accepted:
                response=self.wait(self.active.cancel_goal_async(),5)
                if response.return_code!=0:
                    # Goal may have finished just before cancellation.
                    result=self.wait(self.active.get_result_async(),2)
                    if result.status not in (4,5,6):raise RuntimeError('Cancellation rejected')
                else:
                    result=self.wait(self.active.get_result_async(),5)
                    if result.status not in (4,5,6):raise RuntimeError('Action did not terminate')
                self.active=None
        except (Exception,KeyboardInterrupt):
            if motion:
                # Dedicated demo controller: stop even if a goal acknowledgement was lost.
                def cancel_all():
                    service=self.create_client(CancelGoal,'/follow_path/_action/cancel_goal')
                    try:
                        if not service.wait_for_service(timeout_sec=2):raise RuntimeError('Cancel-all service unavailable')
                        self.wait(service.call_async(CancelGoal.Request()),3)
                    finally:
                        self.destroy_client(service)
                def deactivate():
                    service=self.create_client(ChangeState,'/controller_server/change_state')
                    try:
                        if not service.wait_for_service(timeout_sec=2):raise RuntimeError('Controller deactivation unavailable')
                        request=ChangeState.Request();request.transition.id=4
                        if not self.wait(service.call_async(request),5).success:
                            raise RuntimeError('Controller deactivation failed; stop the demo launch')
                    finally:
                        self.destroy_client(service)
                finish_cleanup([cancel_all,deactivate])
            raise

    def cleanup(self,motion):
        def stop():
            if not motion:return
            # Keep zero output through the smoother's command timeout.
            until=time.monotonic()+1.2
            while time.monotonic()<until:
                self.stop_pub.publish(Twist());rclpy.spin_once(self,timeout_sec=.05)
        def reap():
            if self.process is None:return
            self.process.terminate()
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
            self.thread.join(timeout=1)
        finish_cleanup([lambda:self.cancel_active(motion),stop,reap])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true',help='Approach start and execute coverage (default: plan only)')
    parser.add_argument('--output',type=Path,default=Path('/tmp/bg1_coverage_result.json'))
    parser.add_argument('--task-config',type=Path)
    args=parser.parse_args()
    own=Path(get_package_share_directory('bg1_coverage_demo'))
    config=yaml.safe_load((args.task_config or own/'config/task.yaml').read_text())
    fp=yaml.safe_load((Path(get_package_share_directory('bg1_description'))/'config/nav2_footprint.yaml').read_text())['footprint']
    polygon=validate_polygon(config['polygon']);bounds=config['allowed_bounds']
    # Keep the ROS context alive through cancellation on Ctrl+C/SIGTERM.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    node=Task();report={'passed':False,'execute':args.execute,'polygon':polygon,'allowed_bounds':bounds,
        'localization':'Gazebo ground-truth map-to-odom correction (simulation only)','operation_width_m':0.6}
    started=time.monotonic()
    try:
        request=ComputeCoveragePath.Goal();request.frame_id='map'
        request.generate_headland=True;request.generate_route=True;request.generate_path=True
        request.polygons=[Coordinates(coordinates=[Coordinate(axis1=float(x),axis2=float(y)) for x,y in polygon])]
        result=node.action(node.coverage,request,float(config['planning_timeout_s']))
        msg=result.result;path=poses_of(msg.nav_path)
        report['path_length_m']=validate_result(result.status,msg.error_code,msg.nav_path.header.frame_id,path,bounds,fp)
        report.update(planning_status=result.status,planning_error_code=msg.error_code,
            planning_seconds=msg.planning_time.sec+msg.planning_time.nanosec*1e-9,
            swath_count=len(msg.coverage_path.swaths),path=path)
        if report['swath_count']<2:raise ValueError('Expected more than one coverage swath')
        # Allow DDS discovery, then hand off the path to the persistent preview node.
        until=time.monotonic()+1
        while time.monotonic()<until:
            node.path_pub.publish(to_path(node,path));rclpy.spin_once(node,timeout_sec=.1)
        print(f'Planned {len(path)} poses, {report["swath_count"]} swaths, {report["path_length_m"]:.2f} m',flush=True)
        if args.execute:
            node.record();deadline=time.monotonic()+20
            while (node.odom is None or node.scan is None or not node.samples
                   or not node.tf_buffer.can_transform('map','base_footprint',rclpy.time.Time())):
                if time.monotonic()>deadline:raise TimeoutError('Odometry/laser/Gazebo ground truth unavailable')
                rclpy.spin_once(node,timeout_sec=.1)
            report['ground_truth_start']=node.samples[-1].copy()
            transform=node.tf_buffer.lookup_transform('map','base_footprint',rclpy.time.Time()).transform
            start=(transform.translation.x,transform.translation.y,yaw(transform.rotation))
            approach=connector(start,path[0]);validate_path(approach,bounds,fp)
            print('Approaching coverage start',flush=True)
            node.follow_path(approach,90)
            print('Following coverage path',flush=True)
            report['execution_status']=node.follow_path(path,float(config['execution_timeout_s']))
            until=time.monotonic()+.6
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
            actual=node.samples[-1]
            report['actual_travel_m']=validate_motion(node.samples,time.monotonic())
            validate_path([(p['x'],p['y'],p['yaw']) for p in node.samples],bounds,fp)
            error=math.hypot(actual['x']-path[-1][0],actual['y']-path[-1][1])
            report['actual_final']=actual.copy();report['actual_position_error_m']=error
            report['actual_yaw_error_rad']=abs(math.atan2(math.sin(actual['yaw']-path[-1][2]),math.cos(actual['yaw']-path[-1][2])))
            report['actual_displacement_m']=math.hypot(actual['x']-report['ground_truth_start']['x'],actual['y']-report['ground_truth_start']['y'])
            if error>.35:raise RuntimeError(f'Gazebo final position error {error:.3f} m exceeds 0.35 m')
            if len(node.samples)<10:raise RuntimeError('Insufficient actual motion evidence')
        report['passed']=True
    except (Exception,KeyboardInterrupt) as error:
        report['error']=str(error) or 'Interrupted'
    finally:
        try:node.cleanup(args.execute)
        except Exception as error:report['cleanup_error']=str(error);report['passed']=False
        report['elapsed_seconds']=time.monotonic()-started
        report['ground_truth_trajectory']=node.samples
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('path','ground_truth_trajectory')},indent=2),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':main()
