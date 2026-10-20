from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('bg1_navigation')
    config = PathJoinSubstitution([share,'config','nav2.yaml'])
    world = PathJoinSubstitution([share,'worlds','demo.sdf'])
    map_file = PathJoinSubstitution([share,'maps','demo.yaml'])
    simulation = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('bg1_gazebo'),'launch','sim.launch.py'])),
        launch_arguments={'world':world, 'world_name':'bg1_navigation',
                          'gui':LaunchConfiguration('gui'),
                          'headless':LaunchConfiguration('headless')}.items())
    nodes = [('nav2_map_server','map_server'),('nav2_amcl','amcl'),
             ('nav2_controller','controller_server'),('nav2_planner','planner_server'),
             ('nav2_smoother','smoother_server'),('nav2_behaviors','behavior_server'),
             ('nav2_bt_navigator','bt_navigator'),('nav2_velocity_smoother','velocity_smoother')]
    actions = [DeclareLaunchArgument('gui',default_value='true'),
               DeclareLaunchArgument('headless',default_value='false'),
               DeclareLaunchArgument('rviz',default_value='true'), simulation]
    for package, executable in nodes:
        extra = {'use_sim_time':True}
        if executable == 'map_server':
            extra['yaml_filename'] = map_file
        remaps = []
        if executable in ('controller_server','behavior_server'):
            remaps = [('cmd_vel','/cmd_vel_nav')]
        if executable == 'velocity_smoother':
            remaps = [('cmd_vel','/cmd_vel_nav'),('cmd_vel_smoothed','/cmd_vel')]
        actions.append(Node(package=package,executable=executable,name=executable,
                            output='screen',parameters=[config,extra],remappings=remaps))
    actions.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
        name='lifecycle_manager_bg1',output='screen',parameters=[{
            'use_sim_time':True,'autostart':True,'bond_timeout':10.0,
            'node_names':[name for _,name in nodes]}]))
    actions.append(Node(package='rviz2',executable='rviz2',output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d',PathJoinSubstitution([share,'rviz','navigation.rviz'])],
        parameters=[{'use_sim_time':True}]))
    return LaunchDescription(actions)
