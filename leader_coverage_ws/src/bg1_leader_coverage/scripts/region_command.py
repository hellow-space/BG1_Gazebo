#!/usr/bin/env python3
import argparse
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main():
    parser=argparse.ArgumentParser(description='Control polygon selection and preview; never commands robot motion.')
    parser.add_argument('command',choices=['demo','plan','reset','undo','zigzag','backshaped'])
    args=parser.parse_args();rclpy.init();node=Node('leader_region_command')
    try:
        client=node.create_client(Trigger,f'/leader_coverage/selection/{args.command}')
        if not client.wait_for_service(timeout_sec=15):raise RuntimeError('Selection service unavailable; start leader_coverage_ws/run.sh first')
        future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=20)
        if not future.done():raise RuntimeError('Selection command timed out')
        response=future.result();print(response.message)
        if not response.success:raise RuntimeError('Selection command rejected')
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
