import json
from pathlib import Path
import xml.etree.ElementTree as ET
from PIL import Image
import yaml

PACKAGE = Path(__file__).resolve().parents[1]


def test_costmaps_use_bg1_footprint_and_laser():
    config = yaml.safe_load((PACKAGE/'config/nav2.yaml').read_text())
    footprint = yaml.safe_load((PACKAGE.parent/'bg1_description/config/nav2_footprint.yaml').read_text())['footprint']
    for name in ('local_costmap','global_costmap'):
        params = config[name][name]['ros__parameters']
        assert json.loads(params['footprint']) == footprint
        assert params['robot_base_frame'] == 'base_footprint'
        assert params['obstacle_layer']['scan']['topic'] == '/scan'
    controller = config['controller_server']['ros__parameters']
    assert controller['enable_stamped_cmd_vel'] is False
    assert controller['FollowPath']['desired_linear_vel'] <= .3


def test_map_obstacles_match_world():
    scene = yaml.safe_load((PACKAGE/'config/demo_scene.yaml').read_text())
    world = ET.parse(PACKAGE/'worlds/demo.sdf').getroot().find('world')
    with Image.open(PACKAGE/'maps/demo.pgm') as bitmap:
        for name,x,y,l,w,h in scene['boxes']:
            model = world.find(f"model[@name='{name}']")
            assert [float(v) for v in model.findtext('link/collision/geometry/box/size').split()] == [l,w,h]
            px = int((x-scene['origin'][0])/scene['resolution'])
            py = bitmap.height-1-int((y-scene['origin'][1])/scene['resolution'])
            assert bitmap.getpixel((px,py)) == 0
        px = int(-scene['origin'][0]/scene['resolution'])
        py = bitmap.height-1-int(-scene['origin'][1]/scene['resolution'])
        assert bitmap.getpixel((px,py)) == 254
