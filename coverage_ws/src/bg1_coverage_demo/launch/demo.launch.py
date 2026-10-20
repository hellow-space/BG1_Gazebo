from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    own=Path(get_package_share_directory('bg1_coverage_demo'))
    gazebo=Path(get_package_share_directory('bg1_gazebo'))
    nav=Path(get_package_share_directory('bg1_navigation'))
    config=str(nav/'config/nav2.yaml')
    # Read current framework config without rewriting any underlay file.
    base=yaml.safe_load(Path(config).read_text())
    if base['controller_server']['ros__parameters']['FollowPath']['plugin'] != 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController':
        raise RuntimeError('Coverage demo expects RPP in the underlay nav2.yaml')
    nodes=[
        Node(package='bg1_coverage_demo',executable='sim_localization.py',name='coverage_sim_localization',
             parameters=[{'use_sim_time':True}],output='screen'),
        Node(package='opennav_coverage',executable='opennav_coverage',name='coverage_server',
             parameters=[str(own/'config/coverage.yaml')],output='screen'),
        Node(package='nav2_controller',executable='controller_server',name='controller_server',
             parameters=[config,{'use_sim_time':True}],remappings=[('cmd_vel','/cmd_vel_nav')],output='screen'),
        Node(package='nav2_velocity_smoother',executable='velocity_smoother',name='velocity_smoother',
             parameters=[config,{'use_sim_time':True}],remappings=[('cmd_vel','/cmd_vel_nav'),('cmd_vel_smoothed','/cmd_vel')],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_coverage',
             parameters=[{'use_sim_time':True,'autostart':True,'bond_timeout':10.0,
                          'node_names':['coverage_server','controller_server','velocity_smoother']}],output='screen'),
        Node(package='bg1_coverage_demo',executable='preview.py',name='coverage_preview',
             parameters=[{'use_sim_time':True}],output='screen'),
        Node(package='rviz2',executable='rviz2',name='coverage_rviz',condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d',str(own/'rviz/coverage.rviz')],parameters=[{'use_sim_time':True}],output='screen')]
    sim=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo/'launch/sim.launch.py')),
        launch_arguments={'world':str(gazebo/'worlds/bg1_flat.sdf'),'world_name':'bg1_flat',
                          'gui':LaunchConfiguration('gui'),'headless':LaunchConfiguration('headless')}.items())
    return LaunchDescription([DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('headless',default_value='false'),sim,*nodes])
