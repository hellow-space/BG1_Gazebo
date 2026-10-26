from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
 root=Path(__file__).resolve().parents[5];data=root/'data/maps/imported/A11123AJ04_231127_094529/gazebo_demo'
 config=str(Path(get_package_share_directory('bg1_navigation'))/'config/nav2.yaml')
 sim=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('bg1_gazebo'))/'launch/sim.launch.py')),launch_arguments={'world':str(data/'world.sdf'),'world_name':'leader_imported','gui':'true','headless':'false','spawn_x':'34.325','spawn_y':'52.425','spawn_yaw':'0'}.items())
 nodes=[sim];names=[]
 for package,name in [('nav2_map_server','map_server'),('nav2_amcl','amcl'),('nav2_controller','controller_server'),('nav2_planner','planner_server'),('nav2_smoother','smoother_server'),('nav2_behaviors','behavior_server'),('nav2_bt_navigator','bt_navigator'),('nav2_velocity_smoother','velocity_smoother')]:
  extra={'use_sim_time':True}
  if name=='map_server':extra['yaml_filename']=str(data/'map.yaml')
  if name=='amcl':extra.update({'initial_pose.x':34.325,'initial_pose.y':52.425,'initial_pose.yaw':0.})
  remaps=[]
  if name in ('controller_server','behavior_server'):remaps=[('cmd_vel','/cmd_vel_nav')]
  if name=='velocity_smoother':remaps=[('cmd_vel','/cmd_vel_nav'),('cmd_vel_smoothed','/cmd_vel')]
  nodes.append(Node(package=package,executable=name,name=name,output='screen',parameters=[config,extra],remappings=remaps));names.append(name)
 nodes.extend([
  Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='leader_motion_lifecycle',output='screen',parameters=[{'use_sim_time':True,'autostart':True,'bond_timeout':10.,'node_names':names}]),
  Node(package='bg1_leader_coverage',executable='planner_server',output='screen',parameters=[{'use_sim_time':True}]),
  Node(package='bg1_leader_coverage',executable='coverage_motion.py',output='screen',parameters=[{'use_sim_time':True}]),
  Node(package='rviz2',executable='rviz2',name='leader_motion_rviz',output='screen',arguments=['-d',str(Path(__file__).with_name('motion.rviz'))],parameters=[{'use_sim_time':True}])])
 return LaunchDescription(nodes)
