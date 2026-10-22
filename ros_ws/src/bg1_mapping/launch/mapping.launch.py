from pathlib import Path
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,ExecuteProcess,OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from bg1_scenarios.catalog import resolve_scene


def _launch_setup(context):
    own=Path(get_package_share_directory('bg1_mapping'))
    gazebo=Path(get_package_share_directory('bg1_gazebo'))
    scene=resolve_scene(LaunchConfiguration('scene').perform(context))
    slam=Path(get_package_share_directory('slam_toolbox'))
    keyboard=Path(get_package_prefix('bg1_mapping'))/'lib/bg1_mapping/keyboard.sh'
    simulation=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo/'launch/sim.launch.py')),
        launch_arguments={'world':str(scene.world),'world_name':scene.world_name,
            'gui':LaunchConfiguration('gui'),'headless':LaunchConfiguration('headless')}.items())
    mapper=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam/'launch/online_async_launch.py')),
        launch_arguments={'use_sim_time':'true','autostart':'true','use_lifecycle_manager':'false',
            'slam_params_file':str(own/'config/slam.yaml')}.items())
    return [
        simulation,mapper,
        Node(package='rviz2',executable='rviz2',name='mapping_rviz',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d',str(scene.mapping_rviz or own/'rviz/mapping.rviz')],parameters=[{'use_sim_time':True}],output='screen'),
        ExecuteProcess(cmd=['gnome-terminal','--wait','--title=BG1 Mapping Keyboard','--','bash',str(keyboard)],
            condition=IfCondition(LaunchConfiguration('teleop')),output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene',default_value='demo',choices=['demo','warehouse','office']),
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('headless',default_value='false'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('teleop',default_value='true'),OpaqueFunction(function=_launch_setup)])
