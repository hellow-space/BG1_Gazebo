import importlib.util
from pathlib import Path
import sys
import rclpy
from std_srvs.srv import Trigger

SCRIPTS=Path(__file__).parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))


def test_selector_starts_and_can_reset():
    spec=importlib.util.spec_from_file_location('region_selector',SCRIPTS/'region_selector.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    rclpy.init()
    node=None
    try:
        node=module.RegionSelector()
        node.state.add((1,2))
        response=node.command('reset',Trigger.Response())
        assert response.success and node.state.points==[]
        assert node.command('plan',Trigger.Response()).success is False
    finally:
        if node:node.destroy_node()
        rclpy.shutdown()


def test_pending_request_reset_and_timeout_recover():
    import time
    from geometry_msgs.msg import TransformStamped
    from bg1_leader_coverage_interfaces.srv import PlanCoverage
    from rclpy.parameter import Parameter
    spec=importlib.util.spec_from_file_location('region_selector',SCRIPTS/'region_selector.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    rclpy.init();node=module.RegionSelector();peer=rclpy.create_node('delayed_coverage_server')
    # A real discovered service with no executor processing its requests.
    server=peer.create_service(PlanCoverage,'/leader_coverage/plan',lambda req,resp:resp)
    try:
        assert node.client.wait_for_service(timeout_sec=3)
        transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='base_footprint'
        transform.transform.rotation.w=1.
        node.buffer.set_transform_static(transform,'test')
        node.state.points=[(0.,0.),(4.,0.),(4.,3.)];node.plan()
        old=node.pending
        assert node.busy and not old.done()
        assert node.command('reset',Trigger.Response()).success
        assert not node.busy and old.cancelled()
        node.state.points=[(0.,0.),(4.,0.),(4.,3.)];node.plan()
        current=node.pending;node.finished(old,0)
        assert node.busy and node.pending is current
        node.command('reset',Trigger.Response())
        node.set_parameters([Parameter('planning_timeout',value=.01)])
        node.state.points=[(0.,0.),(4.,0.),(4.,3.)];node.plan()
        time.sleep(.03);node.check_timeout()
        assert not node.busy and node.pending is None
    finally:
        peer.destroy_node();node.destroy_node();rclpy.shutdown()
