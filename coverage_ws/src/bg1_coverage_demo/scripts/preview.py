#!/usr/bin/env python3
"""Keep the last planned path available for RViz, without commanding motion."""
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PolygonStamped,Point32
from nav_msgs.msg import Path as NavPath


def main():
    rclpy.init()
    node=Node('coverage_preview')
    config=yaml.safe_load((Path(get_package_share_directory('bg1_coverage_demo'))/'config/task.yaml').read_text())
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    path_pub=node.create_publisher(NavPath,'/coverage_demo/path',qos)
    region_pub=node.create_publisher(PolygonStamped,'/coverage_demo/region',qos)
    state={'path':None}
    node.create_subscription(NavPath,'/coverage_demo/path_input',lambda m:state.update(path=m),10)
    def publish():
        stamp=node.get_clock().now().to_msg()
        region=PolygonStamped()
        region.header.frame_id='map';region.header.stamp=stamp
        region.polygon.points=[Point32(x=float(x),y=float(y)) for x,y in config['polygon']]
        region_pub.publish(region)
        if state['path'] is not None:
            state['path'].header.stamp=stamp
            path_pub.publish(state['path'])
    node.create_timer(.5,publish)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
