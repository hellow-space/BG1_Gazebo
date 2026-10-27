"""Standalone PNG map service and original Leader planner; no motion nodes."""
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = str(Path(__file__).resolve().with_name('map.yaml'))
    return LaunchDescription([
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='leader_preview_lifecycle_manager', output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server'],
                          'use_sim_time': False}]),
        Node(package='bg1_leader_coverage', executable='planner_server',
             output='screen', parameters=[{'use_sim_time': False}]),
    ])
