"""Resolve a scene consistently for Gazebo, navigation and SLAM launches."""
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Scene:
    world: Path
    world_name: str
    map_file: Path
    navigation_rviz: Path
    mapping_rviz: Path | None
    navigation_overrides: Path | None = None


def resolve_scene(name,navigation_share=None,scenarios_share=None):
    if name not in ('demo','warehouse','office'):
        raise ValueError(f'Unknown scene {name!r}; choose demo, warehouse or office')
    if navigation_share is None or scenarios_share is None:
        from ament_index_python.packages import get_package_share_directory
        navigation_share=navigation_share or get_package_share_directory('bg1_navigation')
        scenarios_share=scenarios_share or get_package_share_directory('bg1_scenarios')
    nav=Path(navigation_share);scenes=Path(scenarios_share)
    if name=='demo':
        return Scene(nav/'worlds/demo.sdf','bg1_navigation',nav/'maps/demo.yaml',nav/'rviz/navigation.rviz',None)
    return Scene(scenes/'worlds'/f'{name}.sdf',f'bg1_{name}',scenes/'maps'/f'{name}.yaml',
                 scenes/'rviz/navigation.rviz',scenes/'rviz/mapping.rviz',
                 scenes/'config/office_navigation.yaml' if name=='office' else None)
