from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from bg1_scenarios.catalog import resolve_scene


def _launch_setup(context):
    share = FindPackageShare('bg1_navigation')
    config = PathJoinSubstitution([share,'config','nav2.yaml'])
    scene = resolve_scene(LaunchConfiguration('scene').perform(context))
    world = str(scene.world)
    map_file = str(scene.map_file)
    simulation = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('bg1_gazebo'),'launch','sim.launch.py'])),
        launch_arguments={'world':world, 'world_name':scene.world_name,
                          'gui':LaunchConfiguration('gui'),
                          'headless':LaunchConfiguration('headless')}.items())
    nodes = [('nav2_map_server','map_server'),('nav2_amcl','amcl'),
             ('nav2_controller','controller_server'),('nav2_planner','planner_server'),
             ('nav2_smoother','smoother_server'),('nav2_behaviors','behavior_server'),
             ('nav2_bt_navigator','bt_navigator'),('nav2_velocity_smoother','velocity_smoother')]
    actions = [simulation]
    for package, executable in nodes:
        extra = {'use_sim_time':True}
        if executable == 'map_server':
            extra['yaml_filename'] = map_file
        remaps = []
        if executable in ('controller_server','behavior_server'):
            remaps = [('cmd_vel','/cmd_vel_nav')]
        if executable == 'velocity_smoother':
            remaps = [('cmd_vel','/cmd_vel_nav'),('cmd_vel_smoothed','/cmd_vel')]
        parameters = [config]
        if scene.navigation_overrides is not None:
            parameters.append(str(scene.navigation_overrides))
        parameters.append(extra)
        actions.append(Node(package=package,executable=executable,name=executable,
                            output='screen',parameters=parameters,remappings=remaps))
    actions.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
        name='lifecycle_manager_bg1',output='screen',parameters=[{
            'use_sim_time':True,'autostart':True,'bond_timeout':10.0,
            'node_names':[name for _,name in nodes]}]))
    actions.append(Node(package='rviz2',executable='rviz2',output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d',str(scene.navigation_rviz)],
        parameters=[{'use_sim_time':True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene',default_value='demo',choices=['demo','warehouse','office']),
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('headless',default_value='false'),
        DeclareLaunchArgument('rviz',default_value='true'),OpaqueFunction(function=_launch_setup)])
