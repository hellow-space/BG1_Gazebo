from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("bg1_description")
    model = PathJoinSubstitution([package_share, "urdf", "bg1.urdf.xacro"])
    geometry = PathJoinSubstitution([package_share, "config", "bg1_geometry.yaml"])
    physics = PathJoinSubstitution([package_share, "config", "bg1_physics.yaml"])
    appearance = LaunchConfiguration("appearance")

    robot_description = Command(
        [
            "xacro ",
            model,
            " geometry:=",
            geometry,
            " physics:=",
            physics,
            " appearance:=",
            appearance,
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "appearance",
                default_value="primitive",
                description="Robot visual mode: primitive or wrapped",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": True, "robot_description": robot_description}],
            ),
        ]
    )
