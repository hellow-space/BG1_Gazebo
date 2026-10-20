#!/usr/bin/env python3
"""Simulation-only map->odom correction from time-aligned Gazebo model poses."""
from collections import deque
import json
import math
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from coverage_geometry import quaternion_yaw,map_to_odom,stamp_seconds


def main():
    rclpy.init();node=Node('coverage_sim_localization')
    broadcaster=TransformBroadcaster(node)
    odometry=deque(maxlen=500)
    state={'actual':None}
    def on_odom(msg):
        p=msg.pose.pose;q=p.orientation
        try:a=quaternion_yaw(q.x,q.y,q.z,q.w)
        except ValueError:return
        odometry.append((msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,(p.position.x,p.position.y,a)))
    node.create_subscription(Odometry,'/odom',on_odom,100)
    process=subprocess.Popen(['gz','topic','-e','-t','/world/bg1_flat/dynamic_pose/info','--json-output'],
        stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)
    def collect():
        last=0
        for line in process.stdout:
            wall=time.monotonic()
            if wall-last<.02:continue
            try:
                d=json.loads(line);p=next(p for p in d['pose'] if p['name']=='bg1_pro')
                q=p['orientation'];s=d['header']['stamp'];pos=p['position']
                a=quaternion_yaw(q.get('x',0),q.get('y',0),q.get('z',0),q.get('w',1))
                state['actual']=(stamp_seconds(s),(pos.get('x',0),pos.get('y',0),a),wall)
                last=wall
            except (ValueError,KeyError,StopIteration):continue
    thread=threading.Thread(target=collect,daemon=True);thread.start()
    def publish():
        actual=state['actual']
        if actual is None or not odometry or time.monotonic()-actual[2]>.5:return
        stamp,odom=min(odometry,key=lambda item:abs(item[0]-actual[0]))
        if abs(stamp-actual[0])>.1:return
        x,y,a=map_to_odom(actual[1],odom)
        msg=TransformStamped();msg.header.stamp=node.get_clock().now().to_msg()
        msg.header.frame_id='map';msg.child_frame_id='odom'
        msg.transform.translation.x=x;msg.transform.translation.y=y
        msg.transform.rotation.z=math.sin(a/2);msg.transform.rotation.w=math.cos(a/2)
        broadcaster.sendTransform(msg)
    node.create_timer(.02,publish)
    node.get_logger().info('Using Gazebo ground truth for DEMO localization; not AMCL or real-world localization')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        process.terminate()
        try:process.wait(timeout=3)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        thread.join(timeout=1);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
