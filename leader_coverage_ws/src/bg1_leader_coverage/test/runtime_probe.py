#!/usr/bin/env python3
"""Exercise actual ROS services and RViz click topic without moving the robot."""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from bg1_leader_coverage_interfaces.srv import PlanCoverage
from std_srvs.srv import Trigger
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid,Path as NavPath
from geometry_msgs.msg import Point32,PointStamped


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    rclpy.init();node=Node('leader_coverage_runtime_probe');state={'map':None,'path':None,'status':''}
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid,'/map',lambda m:state.update(map=m),qos)
    node.create_subscription(NavPath,'/leader_coverage/coverage_path',lambda m:state.update(path=m),qos)
    node.create_subscription(String,'/leader_coverage/status',lambda m:state.update(status=m.data),qos)
    def until(condition,timeout=60):
        deadline=time.monotonic()+timeout
        while not condition():
            if time.monotonic()>deadline:raise TimeoutError(state['status'])
            rclpy.spin_once(node,timeout_sec=.1)
    def call(client,req):
        assert client.wait_for_service(timeout_sec=30)
        f=client.call_async(req);until(f.done,120);return f.result()
    client=node.create_client(PlanCoverage,'/leader_coverage/plan')
    trigger={cmd:node.create_client(Trigger,f'/leader_coverage/selection/{cmd}') for cmd in ('reset','undo','plan')}
    points=[(-10.,-8.),(10.,-8.),(10.,8.),(-10.,8.)]
    def request(pattern=0):
        req=PlanCoverage.Request();req.region.header.frame_id='map'
        req.region.polygon.points=[Point32(x=x,y=y) for x,y in points]
        req.pattern=pattern;req.stripe_spacing=.7;req.clearance=1.
        return req
    report={'passed':False,'patterns':[]}
    try:
        until(lambda:state['map'] is not None)
        bad=request();bad.region.polygon.points=bad.region.polygon.points[:2]
        assert not call(client,bad).success
        bad=request();bad.region.header.frame_id='odom';assert not call(client,bad).success
        for pattern in (0,1):
            start=time.monotonic();response=call(client,request(pattern))
            assert response.success,response.message
            path=[[p.pose.position.x,p.pose.position.y] for p in response.coverage_path.poses]
            assert len(path)>20
            report['patterns'].append({'pattern':pattern,'poses':len(path),
                'length_m':sum(math.dist(a,b) for a,b in zip(path,path[1:])),
                'duration_s':time.monotonic()-start,'path':path})
        # Same topic that RViz Publish Point sends, including clicking first vertex to close.
        assert call(trigger['reset'],Trigger.Request()).success
        until(lambda:state['path'] is not None and not state['path'].poses)
        pub=node.create_publisher(PointStamped,'/clicked_point',10)
        until(lambda:pub.get_subscription_count()>0)
        for x,y in points+[points[0]]:
            msg=PointStamped();msg.header.frame_id='map';msg.point.x=x;msg.point.y=y
            pub.publish(msg)
            deadline=time.monotonic()+.25
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        until(lambda:state['path'] is not None and len(state['path'].poses)>20)
        report['rviz_click_topic_poses']=len(state['path'].poses)
        # New selection clears stale preview; undo/reset and invalid explicit plan are rejected.
        assert call(trigger['reset'],Trigger.Request()).success
        until(lambda:not state['path'].poses)
        assert not call(trigger['plan'],Trigger.Request()).success
        report['passed']=True
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np
        m=state['map'];origin=m.info.origin.position;r=m.info.resolution
        bitmap=np.array(m.data).reshape(m.info.height,m.info.width)
        fig,axes=plt.subplots(1,2,figsize=(16,7),layout='constrained')
        for ax,item in zip(axes,report['patterns']):
            ax.imshow(bitmap,origin='lower',extent=[origin.x,origin.x+m.info.width*r,origin.y,origin.y+m.info.height*r],cmap='gray_r',vmin=-1,vmax=100)
            x,y=zip(*(points+[points[0]]));ax.plot(x,y,color='orange',linewidth=2)
            x,y=zip(*item['path']);ax.plot(x,y,color='#008c9d',linewidth=.8)
            ax.set_title(('Zigzag' if item['pattern']==0 else 'Backshaped')+f" / {item['length_m']:.1f} m")
            ax.set_aspect('equal');ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)')
        args.output.parent.mkdir(parents=True,exist_ok=True)
        fig.savefig(args.output.with_suffix('.png'),dpi=140);plt.close(fig)
    except Exception as error:report['error']=str(error) or type(error).__name__
    finally:
        args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k!='patterns'},indent=2))
        for item in report['patterns']:print({k:v for k,v in item.items() if k!='path'})
        node.destroy_node();rclpy.try_shutdown()
    raise SystemExit(0 if report['passed'] else 1)

if __name__=='__main__':main()
