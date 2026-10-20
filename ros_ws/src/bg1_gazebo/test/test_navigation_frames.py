import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET


def test_navigation_has_one_tf_parent_and_matching_scan_frame():
    package = Path(__file__).resolve().parents[2] / 'bg1_description'
    robot = ET.fromstring(subprocess.check_output([
        'xacro', str(package/'urdf/bg1.urdf.xacro'),
        f'geometry:={package}/config/bg1_geometry.yaml',
        f'physics:={package}/config/bg1_physics.yaml'], text=True))
    drive = robot.find(".//plugin[@name='gz::sim::systems::DiffDrive']")
    assert drive.findtext('child_frame_id') == 'base_footprint'
    assert robot.find('.//sensor/gz_frame_id').text == 'nav_lidar_link'
