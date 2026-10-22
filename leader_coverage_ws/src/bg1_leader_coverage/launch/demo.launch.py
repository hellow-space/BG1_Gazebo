from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    own=FindPackageShare('bg1_leader_coverage')
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('headless',default_value='false'),
        DeclareLaunchArgument('rviz',default_value='true'),
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('bg1_navigation'),'launch/demo.launch.py'])),
            launch_arguments={'scene':'warehouse','gui':LaunchConfiguration('gui'),
                'headless':LaunchConfiguration('headless'),'rviz':'false'}.items())],scoped=True),
        Node(package='bg1_leader_coverage',executable='planner_server',output='screen',parameters=[{'use_sim_time':True}]),
        Node(package='bg1_leader_coverage',executable='coverage_motion.py',output='screen',parameters=[{'use_sim_time':True}]),
        Node(package='bg1_leader_coverage',executable='region_selector.py',output='screen',parameters=[{
            'use_sim_time':True,'stripe_spacing':.7,'clearance':1.0,
            'demo_region':PathJoinSubstitution([own,'config/warehouse_region.yaml'])}]),
        Node(package='rviz2',executable='rviz2',name='leader_coverage_rviz',output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d',PathJoinSubstitution([own,'rviz/coverage.rviz'])],parameters=[{'use_sim_time':True}])])
