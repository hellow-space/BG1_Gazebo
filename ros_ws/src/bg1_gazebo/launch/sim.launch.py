import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def _enabled(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context):
    description_share = FindPackageShare("bg1_description")
    gazebo_share = FindPackageShare("bg1_gazebo")
    model = PathJoinSubstitution([description_share, "urdf", "bg1.urdf.xacro"])
    geometry = PathJoinSubstitution([description_share, "config", "bg1_geometry.yaml"])
    physics = PathJoinSubstitution([description_share, "config", "bg1_physics.yaml"])
    bridge_config = PathJoinSubstitution([gazebo_share, "config", "bridge_topics.yaml"])

    robot_description = Command(
        [
            "xacro ", model,
            " geometry:=", geometry,
            " physics:=", physics,
            " appearance:=", LaunchConfiguration("appearance"),
        ]
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
    )

    gz_args = []
    if not _enabled(context, "paused"):
        gz_args.append("-r")
    if _enabled(context, "headless") or not _enabled(context, "gui"):
        gz_args.append("-s")
    gz_args.append(LaunchConfiguration("world").perform(context))
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": " ".join(gz_args)}.items(),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_bg1_pro",
        output="screen",
        arguments=[
            "-world", LaunchConfiguration("world_name"),
            "-name", "bg1_pro",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
            "-z", LaunchConfiguration("spawn_z"),
            "-Y", LaunchConfiguration("spawn_yaw"),
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bg1_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )
    return [gazebo, state_publisher, spawn, bridge]


def generate_launch_description():
    gazebo_share = FindPackageShare("bg1_gazebo")
    default_world = PathJoinSubstitution([gazebo_share, "worlds", "bg1_flat.sdf"])
    resource_path = [
        PathJoinSubstitution([FindPackagePrefix("bg1_description"), "share"]),
        os.pathsep,
        PathJoinSubstitution([FindPackagePrefix("bg1_gazebo"), "share"]),
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("paused", default_value="false"),
            DeclareLaunchArgument("appearance", default_value="wrapped"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("world_name", default_value="bg1_flat"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.02"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            OpaqueFunction(function=_launch_setup),
        ]
    )
