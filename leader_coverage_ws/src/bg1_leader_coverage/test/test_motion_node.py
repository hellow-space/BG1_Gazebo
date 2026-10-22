import importlib.util
from pathlib import Path
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from std_srvs.srv import Trigger
import pytest

MODULE=Path(__file__).parents[1]/'scripts/coverage_motion.py'

def load():
    assert MODULE.exists(),'motion execution is not implemented'
    spec=importlib.util.spec_from_file_location('coverage_motion',MODULE)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def path():
    msg=NavPath();msg.header.frame_id='map'
    for x,y in [(0.,0.),(1.,0.),(1.,1.)]:
        pose=PoseStamped();pose.header.frame_id='map';pose.pose.position.x=x;pose.pose.position.y=y
        pose.pose.orientation.w=1.;msg.poses.append(pose)
    return msg


def test_path_snapshot_validates_and_does_not_mutate_source():
    module=load();source=path();snapshot=module.prepare_path(source)
    snapshot.poses[0].pose.position.x=99.
    assert source.poses[0].pose.position.x==0.
    bad=path();bad.header.frame_id='odom'
    with pytest.raises(ValueError):module.prepare_path(bad)
    bad=path();bad.poses[0].pose.position.x=float('nan')
    with pytest.raises(ValueError):module.prepare_path(bad)
    bad=path();bad.poses[0].pose.orientation.w=0.
    with pytest.raises(ValueError):module.prepare_path(bad)


def test_motion_starts_idle_and_rejects_missing_path():
    module=load();rclpy.init();node=module.CoverageMotion()
    try:
        assert not node.start(Trigger.Response()).success
        assert node.stop(Trigger.Response()).success
        assert node.phase=='ready'
        node.path_received(path());assert len(node.path.poses)==3
        node.path_received(NavPath());assert node.path is None
    finally:node.destroy_node();rclpy.shutdown()
