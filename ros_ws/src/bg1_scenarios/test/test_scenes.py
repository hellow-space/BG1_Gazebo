from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
from scipy.ndimage import distance_transform_edt,label
import pytest
import yaml

PACKAGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PACKAGE))
from bg1_scenarios.build import expand_layout,render_map,build_scene


@pytest.mark.parametrize('name',['warehouse','office'])
def test_large_layout_has_connected_robot_clearance(name):
    config=yaml.safe_load((PACKAGE/'config'/f'{name}.yaml').read_text())
    objects,goals=expand_layout(config)
    bitmap,origin=render_map(config,objects)
    assert config['size'][0]>=30 and config['size'][1]>=24
    assert len(objects)>=20
    assert len({o['name'] for o in objects})==len(objects)
    occupancy=np.asarray(bitmap)
    clearance=distance_transform_edt(occupancy==254)*config['resolution']
    # Nav2 pads both axes; include cell-area uncertainty rather than just centres.
    required_radius=np.hypot(.84+.03,.38+.03)+config['resolution']/np.sqrt(2)
    safe=clearance>=required_radius
    components,_=label(safe)
    def cell(x,y):
        return bitmap.height-1-int((y-origin[1])/config['resolution']),int((x-origin[0])/config['resolution'])
    start=cell(*config['spawn'][:2]);component=components[start]
    assert component>0
    for goal in goals:
        assert components[cell(*goal)]==component, f'{name}: inaccessible probe {goal}'
    for obstacle in objects:
        assert occupancy[cell(obstacle['x'],obstacle['y'])]==0
    assert occupancy[0,0]==205


@pytest.mark.parametrize('name',['warehouse','office'])
def test_generated_world_and_map_match(name,tmp_path):
    config=PACKAGE/'config'/f'{name}.yaml'
    base=PACKAGE.parent/'bg1_gazebo/worlds/bg1_flat.sdf'
    report=build_scene(config,base,tmp_path)
    map_config=yaml.safe_load((tmp_path/'maps'/f'{name}.yaml').read_text())
    unknown_probability=1-205/255
    assert map_config['free_thresh']<unknown_probability<map_config['occupied_thresh']
    world=ET.parse(tmp_path/'worlds'/f'{name}.sdf').getroot().find('world')
    assert world.attrib['name']==f'bg1_{name}'
    models={m.attrib['name']:m for m in world.findall('model')}
    for obstacle in report['objects']:
        model=models[obstacle['name']]
        position=[float(v) for v in model.findtext('pose').split()]
        assert position[:2]==[obstacle['x'],obstacle['y']]
        geometry=model.find('link/collision/geometry')
        if obstacle['shape']=='box':
            assert list(map(float,geometry.findtext('box/size').split()))==obstacle['size']
        else:
            assert float(geometry.findtext('cylinder/radius'))==obstacle['radius']
    snapshot=(tmp_path/'worlds'/f'{name}.sdf').read_bytes()
    build_scene(config,base,tmp_path)
    assert snapshot==(tmp_path/'worlds'/f'{name}.sdf').read_bytes()


def test_unknown_layout_rejected():
    with pytest.raises(ValueError):expand_layout({'name':'unknown'})


def test_scene_resolution_preserves_demo_and_pairs_world_map():
    from bg1_scenarios.catalog import resolve_scene
    demo=resolve_scene('demo',Path('/navigation'),Path('/scenarios'))
    assert demo.world==Path('/navigation/worlds/demo.sdf')
    assert demo.world_name=='bg1_navigation'
    assert demo.navigation_overrides is None
    for name in ('warehouse','office'):
        scene=resolve_scene(name,Path('/navigation'),Path('/scenarios'))
        assert scene.world==Path('/scenarios/worlds')/f'{name}.sdf'
        assert scene.map_file==Path('/scenarios/maps')/f'{name}.yaml'
        assert scene.world_name==f'bg1_{name}'
        assert scene.navigation_overrides==(Path('/scenarios/config/office_navigation.yaml') if name=='office' else None)
    with pytest.raises(ValueError):resolve_scene('../bad',Path('/navigation'),Path('/scenarios'))
