#!/usr/bin/env python3
"""RViz Publish Point -> polygon -> leader C++ coverage preview."""
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PointStamped,Point32,PolygonStamped,Point
from nav_msgs.msg import Path as NavPath
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
from bg1_leader_coverage_interfaces.srv import PlanCoverage
from tf2_ros import Buffer,TransformListener
from selection import Selection,validate_polygon
import yaml


class RegionSelector(Node):
    def __init__(self):
        super().__init__('leader_region_selector')
        self.declare_parameter('stripe_spacing',.7)
        self.declare_parameter('clearance',1.0)
        self.declare_parameter('demo_region','')
        self.declare_parameter('planning_timeout',30.)
        self.state=Selection();self.revision=0;self.busy=False;self.pattern=0
        self.pending=None;self.deadline=0.
        self.timeout_timer=self.create_timer(.25,self.check_timeout,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers=self.create_publisher(MarkerArray,'/leader_coverage/selection_markers',qos)
        self.region_pub=self.create_publisher(PolygonStamped,'/leader_coverage/selected_region',qos)
        self.path_pub=self.create_publisher(NavPath,'/leader_coverage/coverage_path',qos)
        self.boundary_pub=self.create_publisher(NavPath,'/leader_coverage/boundary_path',qos)
        self.status_pub=self.create_publisher(String,'/leader_coverage/status',qos)
        self.create_subscription(PointStamped,'/clicked_point',self.clicked,10)
        self.client=self.create_client(PlanCoverage,'/leader_coverage/plan')
        self.command_services=[]
        for command in ('plan','reset','undo','demo','zigzag','backshaped'):
            self.command_services.append(self.create_service(Trigger,f'/leader_coverage/selection/{command}',
                lambda req,resp,command=command:self.command(command,resp)))
        self.publish_selection()
        self.status('Ready: Publish Point -> click vertices -> click first vertex to close and plan. Motion requires motion.sh start.')

    def status(self,text):
        self.status_pub.publish(String(data=text));self.get_logger().info(text)

    def clear_paths(self):
        empty=NavPath();empty.header.frame_id='map';empty.header.stamp=self.get_clock().now().to_msg()
        self.path_pub.publish(empty);self.boundary_pub.publish(empty)

    def cancel_request(self):
        future=self.pending
        self.pending=None;self.busy=False
        if future is not None:
            self.client.remove_pending_request(future)
            future.cancel()

    def check_timeout(self):
        if self.pending is not None and time.monotonic()>self.deadline:
            self.cancel_request();self.clear_paths()
            self.status('Planning timed out; reset/plan can retry when the server is available.')

    def changed(self):
        self.cancel_request()
        self.revision+=1;self.clear_paths();self.publish_selection()

    def clicked(self,msg):
        try:
            if msg.header.frame_id!='map':raise ValueError('请将 RViz Fixed Frame 设为 map')
            if not all(math.isfinite(v) for v in (msg.point.x,msg.point.y)):raise ValueError('坐标无效')
            closed=self.state.add((msg.point.x,msg.point.y));self.changed()
            if closed:self.plan()
            else:self.status(f'{len(self.state.points)} vertices selected; click first vertex to close.')
        except ValueError as error:self.status(str(error))

    def publish_selection(self):
        stamp=self.get_clock().now().to_msg()
        polygon=PolygonStamped();polygon.header.frame_id='map';polygon.header.stamp=stamp
        polygon.polygon.points=[Point32(x=float(x),y=float(y)) for x,y in self.state.points]
        self.region_pub.publish(polygon)
        delete=Marker();delete.action=Marker.DELETEALL
        line=Marker();line.header=polygon.header;line.ns='selection';line.id=0
        line.type=Marker.LINE_STRIP;line.action=Marker.ADD;line.pose.orientation.w=1.
        line.scale.x=.10;line.color.r=1.;line.color.g=.85;line.color.a=1.
        points=self.state.points+([self.state.points[0]] if self.state.closed and self.state.points else [])
        line.points=[Point(x=float(x),y=float(y),z=.12) for x,y in points]
        markers=[delete,line]
        for i,(x,y) in enumerate(self.state.points):
            vertex=Marker();vertex.header=polygon.header;vertex.ns='vertices';vertex.id=i
            vertex.type=Marker.SPHERE;vertex.action=Marker.ADD;vertex.pose.orientation.w=1.
            vertex.pose.position=Point(x=float(x),y=float(y),z=.15)
            vertex.scale.x=vertex.scale.y=vertex.scale.z=.28
            vertex.color.r=0.2 if i==0 else 1.;vertex.color.g=1. if i==0 else .25;vertex.color.b=.1;vertex.color.a=1.
            markers.append(vertex)
        self.markers.publish(MarkerArray(markers=markers))

    def command(self,command,response):
        try:
            if command=='reset':self.state.reset();self.changed();self.status('Selection cleared.')
            elif command=='undo':self.state.undo();self.changed();self.status('Last vertex removed.')
            elif command=='demo':
                data=yaml.safe_load(Path(self.get_parameter('demo_region').value).read_text())
                self.state.points=validate_polygon([tuple(p) for p in data['polygon']]);self.state.closed=True
                self.changed();self.plan()
            elif command in ('zigzag','backshaped'):
                self.pattern=int(command=='backshaped');self.changed()
                self.status(f'Pattern: {command}.')
                if self.state.closed:self.plan()
            else:self.plan()
            response.success=True;response.message='Command accepted; read /leader_coverage/status for planning outcome.'
        except (ValueError,OSError) as error:
            response.success=False;response.message=str(error);self.status(str(error))
        return response

    def plan(self):
        validate_polygon(self.state.points)
        if self.busy:raise ValueError('上一次规划仍在运行，请稍后再点击 plan')
        if not self.client.service_is_ready():raise ValueError('规划服务尚未就绪，请稍后执行 plan')
        request=PlanCoverage.Request();request.region.header.frame_id='map'
        request.region.header.stamp=self.get_clock().now().to_msg()
        request.region.polygon.points=[Point32(x=float(x),y=float(y)) for x,y in self.state.points]
        try:
            pose=self.buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
            request.start.x=pose.transform.translation.x;request.start.y=pose.transform.translation.y
        except Exception as error:
            raise ValueError('机器人定位 TF 尚未就绪，请稍后执行 plan') from error
        request.pattern=self.pattern
        request.stripe_spacing=float(self.get_parameter('stripe_spacing').value)
        request.clearance=float(self.get_parameter('clearance').value)
        self.state.closed=True;self.publish_selection();self.clear_paths();self.busy=True
        version=self.revision
        future=self.client.call_async(request)
        self.pending=future;self.deadline=time.monotonic()+float(self.get_parameter('planning_timeout').value)
        self.status(f'Planning {"backshaped" if self.pattern else "zigzag"}; planning only; use motion.sh start after the path is ready.')
        future.add_done_callback(lambda f:self.finished(f,version))

    def finished(self,future,version):
        if future is not self.pending:return
        self.pending=None;self.busy=False
        if version!=self.revision:
            self.status('Selection changed; old result discarded. Click plan to generate current selection.');return
        try:
            result=future.result()
            if result.success:
                self.path_pub.publish(result.coverage_path);self.boundary_pub.publish(result.boundary_path)
                self.status(f'Preview ready: {len(result.coverage_path.poses)} poses. Use motion.sh start to execute.')
            else:self.clear_paths();self.status(f'Planning rejected: {result.message}')
        except Exception as error:self.clear_paths();self.status(f'Planning error: {error}')


def main():
    rclpy.init();node=RegionSelector()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
