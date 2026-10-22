#!/usr/bin/env python3
"""Explicit Nav2 approach + RPP execution of the current coverage path."""
import copy
import json
import math
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose,FollowPath
from action_msgs.srv import CancelGoal
from lifecycle_msgs.srv import ChangeState


def prepare_path(path):
    if path.header.frame_id!='map' or len(path.poses)<2:
        raise ValueError('A planned map-frame path with at least two poses is required')
    snapshot=copy.deepcopy(path)
    for pose in snapshot.poses:
        p=pose.pose.position;q=pose.pose.orientation
        if pose.header.frame_id not in ('','map') or not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):
            raise ValueError('Invalid path coordinates or frame')
        norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if abs(norm-1.)>.01:raise ValueError('Path orientations must be normalized')
    return snapshot


def signature(path):
    return tuple((p.pose.position.x,p.pose.position.y,p.pose.position.z,p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w) for p in path.poses)


class CoverageMotion(Node):
    def __init__(self):
        super().__init__('leader_coverage_motion')
        self.path=None;self.snapshot=None;self.phase='ready';self.message='Select a region and explicitly start motion.'
        self.pending_send=None;self.goal_handle=None;self.result_future=None
        self.generation=0;self.stopping_since=0.;self.stop_failed=False;self.deadline=0.;self.fallback=False
        self.approach_succeeded=False;self.follow_succeeded=False
        self.nav=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.follow=ActionClient(self,FollowPath,'/follow_path')
        self.cancel_clients=[self.create_client(CancelGoal,f'/{name}/_action/cancel_goal') for name in ('navigate_to_pose','follow_path')]
        self.deactivate_clients=[self.create_client(ChangeState,f'/{name}/change_state') for name in ('bt_navigator','behavior_server','controller_server')]
        self.stop_pub=self.create_publisher(Twist,'/cmd_vel',10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub=self.create_publisher(String,'/leader_coverage/motion_status',qos)
        self.create_subscription(NavPath,'/leader_coverage/coverage_path',self.path_received,qos)
        self.command_services=[self.create_service(Trigger,'/leader_coverage/motion/start',lambda req,resp:self.start(resp)),
            self.create_service(Trigger,'/leader_coverage/motion/stop',lambda req,resp:self.stop(resp)),
            self.create_service(Trigger,'/leader_coverage/motion/status',lambda req,resp:self.status_service(resp))]
        self.timer=self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_status()

    @property
    def active(self):return self.phase in ('approaching','following','stopping')

    def data(self):
        return {'phase':self.phase,'message':self.message,'approach_succeeded':self.approach_succeeded,
                'follow_succeeded':self.follow_succeeded,'path_poses':len(self.path.poses) if self.path else 0}

    def publish_status(self):self.status_pub.publish(String(data=json.dumps(self.data())))

    def set_phase(self,phase,message):
        self.phase=phase;self.message=message;self.publish_status();self.get_logger().info(message)

    def path_received(self,path):
        try:new_path=prepare_path(path)
        except ValueError:new_path=None
        changed=self.path is not None and (new_path is None or signature(new_path)!=signature(self.path))
        self.path=new_path
        if self.active and changed:self.request_stop('Selection/path changed; execution cancelled.')
        self.publish_status()

    def start(self,response):
        if self.active:
            response.success=False;response.message='Execution or cancellation is still active.';return response
        if self.fallback:
            response.success=False;response.message='Restart the simulation after cancellation fallback.';return response
        if self.path is None:
            response.success=False;response.message='No valid coverage path; select and plan a region first.';return response
        if not self.nav.server_is_ready() or not self.follow.server_is_ready():
            response.success=False;response.message='Nav2 action servers are not ready.';return response
        self.snapshot=prepare_path(self.path);self.generation+=1
        self.approach_succeeded=False;self.follow_succeeded=False;self.stop_failed=False
        goal=NavigateToPose.Goal();goal.pose=copy.deepcopy(self.snapshot.poses[0])
        goal.pose.header.frame_id='map';goal.pose.header.stamp=self.get_clock().now().to_msg()
        self.set_phase('approaching','Navigating to coverage start.')
        self.send(self.nav,goal,180.)
        response.success=True;response.message='Execution started: approach first, then follow coverage path.';return response

    def send(self,client,goal,timeout):
        self.deadline=time.monotonic()+timeout;generation=self.generation
        try:
            self.pending_send=client.send_goal_async(goal)
            self.pending_send.add_done_callback(lambda future:self.accepted(future,generation))
        except Exception as error:
            self.pending_send=None;self.request_stop(f'Cannot send goal: {error}',failed=True)

    def accepted(self,future,generation):
        try:handle=future.result()
        except Exception as error:
            if generation==self.generation:
                self.request_stop(f'Goal acknowledgement failed: {error}',failed=True)
            return
        if generation!=self.generation:
            if handle.accepted:handle.cancel_goal_async()
            return
        self.pending_send=None
        if not handle.accepted:
            self.request_stop('Nav2 rejected the goal.',failed=True);return
        self.goal_handle=handle
        self.result_future=handle.get_result_async()
        self.result_future.add_done_callback(lambda result:self.finished(result,generation))
        if self.phase=='stopping':handle.cancel_goal_async()

    def finished(self,future,generation):
        if generation!=self.generation:return
        try:
            result=future.result()
            success=result.status==4 and getattr(result.result,'error_code',0)==0
        except Exception as error:
            self.request_stop(f'Action result failed: {error}',failed=True);return
        self.goal_handle=None;self.result_future=None
        if self.phase=='stopping':return
        if not success:
            self.request_stop(f'Nav2 failed: status={result.status}, code={getattr(result.result,"error_code",0)}, {getattr(result.result,"error_msg","")}',failed=True);return
        if self.phase=='approaching':
            self.approach_succeeded=True
            goal=FollowPath.Goal();goal.path=copy.deepcopy(self.snapshot)
            goal.path.header.frame_id='map';goal.path.header.stamp=self.get_clock().now().to_msg()
            for pose in goal.path.poses:pose.header=goal.path.header
            goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker';goal.progress_checker_id='progress_checker'
            self.set_phase('following','Following coverage path with RPP.')
            length=sum(math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y) for a,b in zip(goal.path.poses,goal.path.poses[1:]))
            self.send(self.follow,goal,min(3600.,length/.05+120.))
        elif self.phase=='following':
            self.follow_succeeded=True
            self.request_stop('Coverage path completed.')

    def request_stop(self,message,failed=False):
        self.stop_failed|=failed
        if self.phase!='stopping':
            self.stopping_since=time.monotonic();self.set_phase('stopping',message)
        elif failed:self.message=message;self.publish_status()
        if self.goal_handle is not None:self.goal_handle.cancel_goal_async()
        # Also covers a lost/in-flight goal acknowledgement. Late accepts are cancelled above.
        for client in self.cancel_clients:
            if client.service_is_ready():client.call_async(CancelGoal.Request())
        self.stop_pub.publish(Twist())

    def stop(self,response):
        if self.active:self.request_stop('Stopped by user.')
        response.success=True;response.message='Stopping execution.' if self.active else 'Robot execution is idle.';return response

    def status_service(self,response):
        response.success=True;response.message=json.dumps(self.data());return response

    def tick(self):
        now=time.monotonic()
        if self.phase=='stopping':
            self.stop_pub.publish(Twist())
            if self.pending_send is None and self.goal_handle is None and now-self.stopping_since>=1.3:
                phase='failed' if self.stop_failed else ('completed' if self.follow_succeeded else 'stopped')
                self.set_phase(phase,self.message)
            elif now-self.stopping_since>8 and not self.fallback:
                self.fallback=True
                self.message='Cancellation unconfirmed; deactivating Nav2. Restart the simulation to recover.'
                for client in self.deactivate_clients:
                    if client.service_is_ready():
                        request=ChangeState.Request();request.transition.id=4;client.call_async(request)
                self.publish_status();self.get_logger().error(self.message)
        elif self.phase in ('approaching','following') and now>self.deadline:
            self.request_stop('Motion action timed out.',failed=True)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=CoverageMotion()
    interrupted=False
    def interrupt(signum,frame):
        nonlocal interrupted
        interrupted=True
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    try:
        while rclpy.ok() and not interrupted:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        if node.active:
            node.request_stop('Motion node shutting down.')
            deadline=time.monotonic()+10
            while node.active and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
