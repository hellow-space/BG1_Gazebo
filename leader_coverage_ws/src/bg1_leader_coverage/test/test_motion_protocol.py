"""Real ROS action protocol tests with a controllable, non-physics Nav2 peer."""
import importlib.util
from pathlib import Path
import threading,time
import pytest
import rclpy
from rclpy.action import ActionServer,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose,FollowPath
from std_srvs.srv import Trigger


@pytest.mark.parametrize('case',['early_stop','path_change','rejected'])
def test_action_cancellation_races(case):
    path=Path(__file__).parents[1]/'scripts/coverage_motion.py'
    spec=importlib.util.spec_from_file_location('coverage_motion',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    rclpy.init();motion=module.CoverageMotion();peer=rclpy.create_node('controlled_nav2_peer');tester=rclpy.create_node('motion_protocol_tester')
    counts={'nav_cancelled':0,'follow':0,'nav_entered':0}
    def goal_callback(request):
        time.sleep(.15)
        return GoalResponse.REJECT if case=='rejected' else GoalResponse.ACCEPT
    def navigate(handle):
        counts['nav_entered']+=1
        deadline=time.monotonic()+5
        while not handle.is_cancel_requested and time.monotonic()<deadline:time.sleep(.01)
        result=NavigateToPose.Result()
        if handle.is_cancel_requested:counts['nav_cancelled']+=1;handle.canceled()
        else:handle.abort();result.error_code=1
        return result
    def follow(handle):
        counts['follow']+=1;handle.succeed();return FollowPath.Result()
    group=ReentrantCallbackGroup()
    nav=ActionServer(peer,NavigateToPose,'/navigate_to_pose',execute_callback=navigate,goal_callback=goal_callback,
                     cancel_callback=lambda handle:CancelResponse.ACCEPT,callback_group=group)
    follow_server=ActionServer(peer,FollowPath,'/follow_path',execute_callback=follow,callback_group=group)
    executor=MultiThreadedExecutor(num_threads=4);executor.add_node(motion);executor.add_node(peer)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    clients={cmd:tester.create_client(Trigger,f'/leader_coverage/motion/{cmd}') for cmd in ('start','stop','status')}
    pub=tester.create_publisher(NavPath,'/leader_coverage/coverage_path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def until(condition,timeout=5):
        end=time.monotonic()+timeout
        while not condition():
            assert time.monotonic()<end, motion.data()
            rclpy.spin_once(tester,timeout_sec=.02)
    def call(command):
        assert clients[command].wait_for_service(timeout_sec=3)
        future=clients[command].call_async(Trigger.Request());until(future.done);return future.result()
    try:
        until(lambda:motion.nav.server_is_ready() and motion.follow.server_is_ready())
        msg=NavPath();msg.header.frame_id='map'
        for x in (0.,1.):
            p=PoseStamped();p.header.frame_id='map';p.pose.position.x=x;p.pose.orientation.w=1.;msg.poses.append(p)
        pub.publish(msg);until(lambda:motion.path is not None)
        assert call('start').success
        assert not call('start').success # a duplicate start must not replace the active goal
        if case=='early_stop':assert call('stop').success
        elif case=='path_change':
            until(lambda:counts['nav_entered']>0)
            pub.publish(NavPath())
        until(lambda:motion.phase in ('stopped','failed'))
        assert motion.phase==('failed' if case=='rejected' else 'stopped')
        assert counts['follow']==0
        if case!='rejected':assert counts['nav_cancelled']==1
    finally:
        call('stop')
        executor.shutdown(timeout_sec=6);thread.join(timeout=6)
        nav.destroy();follow_server.destroy();tester.destroy_node();peer.destroy_node();motion.destroy_node();rclpy.shutdown()
