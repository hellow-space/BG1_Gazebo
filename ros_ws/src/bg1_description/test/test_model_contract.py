from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
MODEL = PACKAGE / "urdf" / "bg1.urdf.xacro"
GEOMETRY = PACKAGE / "config" / "bg1_geometry.yaml"
PHYSICS = PACKAGE / "config" / "bg1_physics.yaml"


def expand(appearance="primitive"):
    output = subprocess.check_output(
        [
            "xacro",
            str(MODEL),
            f"geometry:={GEOMETRY}",
            f"physics:={PHYSICS}",
            f"appearance:={appearance}",
        ],
        text=True,
    )
    return ET.fromstring(output)


def floats(value):
    return [float(item) for item in value.split()]


def test_required_tree_and_joint_axes():
    robot = expand()
    links = {link.get("name") for link in robot.findall("link")}
    assert links == {
        "base_footprint",
        "base_link",
        "left_drive_wheel_link",
        "right_drive_wheel_link",
        "caster_swivel_link",
        "caster_wheel_link",
        "nav_lidar_link",
    }
    assert robot.find("joint[@name='base_footprint_joint']").get("type") == "fixed"
    for name in ("left_drive_wheel_joint", "right_drive_wheel_joint", "caster_wheel_joint"):
        joint = robot.find(f"joint[@name='{name}']")
        assert joint.get("type") == "continuous"
        assert joint.find("axis").get("xyz") == "0 1 0"
    swivel = robot.find("joint[@name='caster_swivel_joint']")
    assert swivel.get("type") == "continuous"
    assert swivel.find("axis").get("xyz") == "0 0 1"


def test_body_and_squeegee_collision_envelopes():
    robot = expand()
    geometry = yaml.safe_load(GEOMETRY.read_text())
    body = robot.find("link[@name='base_link']/collision[@name='body_collision']")
    squeegee = robot.find("link[@name='base_link']/collision[@name='squeegee_collision']")
    assert floats(body.find("geometry/box").get("size")) == pytest.approx(
        [geometry["body_length_m"], geometry["body_width_m"], geometry["overall_height_m"] - geometry["body_bottom_z_m"]]
    )
    assert floats(squeegee.find("geometry/box").get("size")) == pytest.approx(
        [
            geometry["body_rear_extent_m"] - geometry["rear_extent_m"],
            geometry["overall_width_m"],
            geometry["squeegee_height_m"],
        ]
    )
    body_origin = floats(body.find("origin").get("xyz"))
    squeegee_origin = floats(squeegee.find("origin").get("xyz"))
    assert body_origin[0] - geometry["body_length_m"] / 2 == pytest.approx(geometry["body_rear_extent_m"])
    assert body_origin[0] + geometry["body_length_m"] / 2 == pytest.approx(geometry["front_extent_m"])
    assert squeegee_origin[0] - (geometry["body_rear_extent_m"] - geometry["rear_extent_m"]) / 2 == pytest.approx(geometry["rear_extent_m"])


def test_total_mass_is_348_5_kg():
    robot = expand()
    masses = [float(mass.get("value")) for mass in robot.findall("link/inertial/mass")]
    assert sum(masses) == pytest.approx(348.5, abs=1e-6)


def test_inertias_are_positive_and_symmetric():
    robot = expand()
    for inertia in robot.findall("link/inertial/inertia"):
        assert all(float(inertia.get(axis)) > 0 for axis in ("ixx", "iyy", "izz"))
        assert all(float(inertia.get(axis)) == 0 for axis in ("ixy", "ixz", "iyz"))


def test_diff_drive_matches_joint_geometry():
    robot = expand()
    geometry = yaml.safe_load(GEOMETRY.read_text())
    plugin = robot.find(".//plugin[@name='gz::sim::systems::DiffDrive']")
    assert plugin is not None
    assert plugin.findtext("left_joint") == "left_drive_wheel_joint"
    assert plugin.findtext("right_joint") == "right_drive_wheel_joint"
    assert float(plugin.findtext("wheel_separation")) == geometry["wheel_track_m"]
    assert float(plugin.findtext("wheel_radius")) == geometry["wheel_radius_m"]
    assert float(plugin.findtext("max_linear_velocity")) == geometry["max_linear_mps"]
    assert float(plugin.findtext("max_angular_velocity")) == geometry["max_angular_rps"]
    left = floats(robot.find("joint[@name='left_drive_wheel_joint']/origin").get("xyz"))
    right = floats(robot.find("joint[@name='right_drive_wheel_joint']/origin").get("xyz"))
    assert left[1] - right[1] == pytest.approx(geometry["wheel_track_m"])
    assert left[2] == right[2] == geometry["wheel_radius_m"]


def test_harmonic_plugins_and_gpu_lidar_contract():
    robot = expand()
    plugins = {plugin.get("filename") for plugin in robot.findall(".//plugin")}
    assert {
        "gz-sim-diff-drive-system",
        "gz-sim-joint-state-publisher-system",
        "gz-sim-pose-publisher-system",
    }.issubset(plugins)
    assert not any("gazebo_ros" in filename for filename in plugins)
    sensor = robot.find(".//sensor[@name='nav_lidar']")
    assert sensor is not None
    assert sensor.get("type") == "gpu_lidar"
    assert sensor.findtext("topic") == "/model/bg1_pro/scan"
    assert int(sensor.findtext("lidar/scan/horizontal/samples")) == 360


def test_xacro_converts_to_sdf_14():
    robot = expand()
    with tempfile.TemporaryDirectory() as directory:
        urdf = Path(directory) / "bg1.urdf"
        ET.ElementTree(robot).write(urdf, encoding="unicode")
        result = subprocess.run(["gz", "sdf", "-p", str(urdf)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    sdf = ET.fromstring(result.stdout)
    assert sdf.tag == "sdf"
    assert int(sdf.get("version").split(".")[0]) >= 1
    assert sdf.find("model") is not None


def test_contact_parameters_survive_sdf_conversion():
    physics = yaml.safe_load(PHYSICS.read_text())
    with tempfile.TemporaryDirectory() as directory:
        urdf = Path(directory) / 'bg1.urdf'
        ET.ElementTree(expand()).write(urdf, encoding='unicode')
        sdf = ET.fromstring(subprocess.check_output(['gz', 'sdf', '-p', str(urdf)], text=True))
    for link, coefficient in (
        ('left_drive_wheel_link', physics['drive_wheel_mu']),
        ('right_drive_wheel_link', physics['drive_wheel_mu']),
        ('caster_wheel_link', physics['caster_mu']),
    ):
        value = sdf.findtext(f"model/link[@name='{link}']/collision/surface/friction/ode/mu")
        assert value is not None, f'{link}: friction was lost in URDF-to-SDF conversion'
        assert float(value) == pytest.approx(coefficient)


def test_navigation_squeegee_does_not_drag_on_floor():
    robot = expand()
    collision = robot.find("link[@name='base_link']/collision[@name='squeegee_collision']")
    center_z = floats(collision.find('origin').get('xyz'))[2]
    height = floats(collision.find('geometry/box').get('size'))[2]
    assert center_z - height / 2 >= 0.019
