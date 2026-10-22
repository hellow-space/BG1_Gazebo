#!/usr/bin/env python3
"""Save navigation occupancy map and resumable SLAM pose graph."""
import argparse
from pathlib import Path
import subprocess
import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SerializePoseGraph


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix',type=Path,help='Output path without extension; refuses existing files')
    args=parser.parse_args();prefix=args.prefix.expanduser().resolve()
    for suffix in ('.yaml','.pgm','.posegraph','.data'):
        if Path(str(prefix)+suffix).exists():parser.error(f'Output already exists: {prefix}{suffix}')
    prefix.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=Node('bg1_save_map')
    client=node.create_client(SerializePoseGraph,'/slam_toolbox/serialize_map')
    try:
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('SLAM Toolbox is not running')
        subprocess.run(['ros2','run','nav2_map_server','map_saver_cli','-f',str(prefix),
            '--ros-args','-p','use_sim_time:=true','-p','save_map_timeout:=10.0'],check=True)
        request=SerializePoseGraph.Request();request.filename=str(prefix)
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=30)
        if not future.done() or future.result() is None or future.result().result!=0:
            raise RuntimeError('Occupancy map saved, but pose-graph serialization failed')
        for suffix in ('.yaml','.pgm','.posegraph','.data'):
            if not Path(str(prefix)+suffix).is_file():raise RuntimeError(f'Missing saved file: {prefix}{suffix}')
        print(f'Saved map: {prefix}.yaml / .pgm\nSaved resumable graph: {prefix}.posegraph / .data')
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
