"""Failure-path tests using fake actions; no ROS graph or robot required."""
from pathlib import Path
import sys
from types import SimpleNamespace
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
pytest.importorskip('rclpy')
from coverage_task import Task,poses_of
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import PoseStamped


def test_empty_pose_frame_inherits_path_frame_but_conflicts_rejected():
    path=NavPath();path.header.frame_id='map'
    p=PoseStamped();p.pose.orientation.w=1.0
    path.poses=[p]
    assert poses_of(path)==[(0,0,0)]
    path.poses[0].header.frame_id='odom'
    with pytest.raises(ValueError):poses_of(path)


def test_goal_ack_timeout_keeps_pending_future_for_cleanup():
    future=object()
    client=SimpleNamespace(server_is_ready=lambda:True,send_goal_async=lambda goal:future)
    def timeout(*args):raise TimeoutError('ack lost')
    node=SimpleNamespace(wait=timeout,pending_goal=None,active=None,active_client=None)
    with pytest.raises(TimeoutError):Task.action(node,client,object(),5)
    assert node.pending_goal is future
    assert node.active_client is client


def test_cancel_service_failure_still_attempts_controller_deactivation():
    calls=[]
    def client(kind,name):
        calls.append(name)
        return SimpleNamespace(wait_for_service=lambda timeout_sec:name.endswith('change_state'),
                               call_async=lambda req:object())
    def wait(future,timeout):
        if future=='pending':raise TimeoutError('ack lost')
        return SimpleNamespace(success=True)
    node=SimpleNamespace(pending_goal='pending',active=None,wait=wait,create_client=client,destroy_client=lambda c:None)
    with pytest.raises(Exception):Task.cancel_active(node,True)
    assert '/controller_server/change_state' in calls
