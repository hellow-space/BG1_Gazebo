#!/usr/bin/env python3
"""Read-only runtime scene check; optionally ask Nav2 to plan to every probe."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer,TransformListener
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from lifecycle_msgs.srv import GetState
from ament_index_python.packages import get_package_share_directory


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',choices=['warehouse','office'],required=True)
    parser.add_argument('--mode',choices=['navigation','mapping'],required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    metadata=json.loads((Path(get_package_share_directory('bg1_scenarios'))/'metadata'/f'{args.scene}.json').read_text())
    rclpy.init();node=Node('bg1_scene_verifier',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    state={'map':None,'scan':None};buffer=Buffer();listener=TransformListener(buffer,node)
    def on_map(m):
        state['map']={'frame':m.header.frame_id,'width':m.info.width,'height':m.info.height,
            'known':sum(v>=0 for v in m.data),'occupied':sum(v>50 for v in m.data),'resolution':m.info.resolution}
    node.create_subscription(OccupancyGrid,'/map',on_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(LaserScan,'/scan',lambda m:state.update(scan={'frame':m.header.frame_id,
        'finite':sum(math.isfinite(v) for v in m.ranges)}),qos_profile_sensor_data)
    report={'scene':args.scene,'mode':args.mode,'passed':False,'plans':[]}
    def wait(future,timeout=20):
        rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
        if not future.done():raise TimeoutError('Service/action timed out')
        return future.result()
    try:
        deadline=time.monotonic()+60
        while not (state['map'] and state['scan'] and buffer.can_transform('map','base_footprint',rclpy.time.Time())):
            if time.monotonic()>deadline:raise TimeoutError('Map, laser or TF unavailable')
            rclpy.spin_once(node,timeout_sec=.1)
        assert state['map']['frame']=='map' and state['map']['known']>100 and state['map']['occupied']>0
        assert state['scan']['frame']=='nav_lidar_link' and state['scan']['finite']>30
        topics=subprocess.check_output(['gz','topic','-l'],text=True,timeout=10)
        assert f'/world/bg1_{args.scene}/' in topics
        if args.mode=='navigation':
            service=node.create_client(GetState,'/planner_server/get_state')
            assert service.wait_for_service(timeout_sec=20)
            deadline=time.monotonic()+30
            while wait(service.call_async(GetState.Request())).current_state.id!=3:
                if time.monotonic()>deadline:raise TimeoutError('Planner inactive')
                rclpy.spin_once(node,timeout_sec=.2)
            client=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
            assert client.wait_for_server(timeout_sec=20)
            def pose(x,y):
                msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
                msg.pose.position.x=float(x);msg.pose.position.y=float(y);msg.pose.orientation.w=1.0
                return msg
            for x,y in metadata['goals']:
                goal=ComputePathToPose.Goal();goal.goal=pose(x,y);goal.start=pose(0,0)
                goal.use_start=True;goal.planner_id='GridBased'
                handle=wait(client.send_goal_async(goal));assert handle.accepted
                result=wait(handle.get_result_async())
                assert result.status==4 and result.result.error_code==0 and len(result.result.path.poses)>1,f'No plan to {(x,y)}'
                path=result.result.path.poses
                length=sum(math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y) for a,b in zip(path,path[1:]))
                report['plans'].append({'goal':[x,y],'poses':len(path),'length_m':length})
        else:
            nodes={name for name,_ in node.get_node_names_and_namespaces()}
            assert 'slam_toolbox' in nodes and 'amcl' not in nodes and 'coverage_sim_localization' not in nodes
        report.update(state);report['passed']=True
    except Exception as error:
        report['error']=str(error) or type(error).__name__
    finally:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
        node.destroy_node();rclpy.shutdown()
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':main()
