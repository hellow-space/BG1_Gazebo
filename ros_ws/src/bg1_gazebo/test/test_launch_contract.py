from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
WORLD = PACKAGE / "worlds" / "bg1_flat.sdf"
LAUNCH = PACKAGE / "launch" / "sim.launch.py"
BRIDGE = PACKAGE / "config" / "bridge_topics.yaml"


def test_world_loads_required_harmonic_systems():
    root = ET.parse(WORLD).getroot()
    assert root.tag == "sdf"
    assert tuple(int(part) for part in root.get("version").split(".")) >= (1, 10)
    world = root.find("world[@name='bg1_flat']")
    assert world is not None
    plugins = {plugin.get("filename") for plugin in world.findall("plugin")}
    assert {
        "gz-sim-physics-system",
        "gz-sim-user-commands-system",
        "gz-sim-scene-broadcaster-system",
        "gz-sim-sensors-system",
    }.issubset(plugins)
    sensors = world.find("plugin[@filename='gz-sim-sensors-system']")
    assert sensors.findtext("render_engine") == "ogre2"
    ground = world.find("model[@name='ground_plane']")
    assert ground is not None
    assert ground.findtext("link/collision/geometry/plane/size") == "100 100"


def test_launch_uses_installed_model_and_headless_switch():
    source = LAUNCH.read_text()
    for argument in ("gui", "headless", "paused", "appearance", "world", "spawn_x", "spawn_y", "spawn_z", "spawn_yaw"):
        assert f'"{argument}"' in source
    assert 'FindPackageShare("bg1_description")' in source
    assert 'FindPackageShare("bg1_gazebo")' in source
    assert 'package="ros_gz_sim"' in source
    assert 'executable="create"' in source
    assert '"-topic", "robot_description"' in source
    assert "GZ_SIM_RESOURCE_PATH" in source
    assert "parameter_bridge" in source


def test_bridge_contract_contains_navigation_topics():
    entries = yaml.safe_load(BRIDGE.read_text())
    by_ros_topic = {entry["ros_topic_name"]: entry for entry in entries}
    expected = {
        "/cmd_vel": ("/model/bg1_pro/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist", "ROS_TO_GZ"),
        "/odom": ("/model/bg1_pro/odometry", "nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS"),
        "/scan": ("/model/bg1_pro/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan", "GZ_TO_ROS"),
        "/clock": ("/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", "GZ_TO_ROS"),
        "/joint_states": ("/model/bg1_pro/joint_state", "sensor_msgs/msg/JointState", "gz.msgs.Model", "GZ_TO_ROS"),
        "/tf": ("/model/bg1_pro/tf", "tf2_msgs/msg/TFMessage", "gz.msgs.Pose_V", "GZ_TO_ROS"),
    }
    assert set(by_ros_topic) == set(expected)
    for topic, (gz_topic, ros_type, gz_type, direction) in expected.items():
        entry = by_ros_topic[topic]
        assert entry["gz_topic_name"] == gz_topic
        assert entry["ros_type_name"] == ros_type
        assert entry["gz_type_name"] == gz_type
        assert entry["direction"] == direction


def test_no_ros1_or_classic_dependencies_are_declared():
    source = "\n".join(
        path.read_text(errors="ignore")
        for path in PACKAGE.rglob("*")
        if path.is_file()
        and path.resolve() != Path(__file__).resolve()
        and path.suffix in {".xml", ".py", ".yaml", ".sdf", ".txt"}
    )
    forbidden = ("catkin", "gazebo_ros", "libgazebo_ros", "gazebo-classic")
    assert not any(token in source for token in forbidden)
