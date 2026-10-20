#!/usr/bin/env python3
"""Generate matching world and map from demo_scene.yaml."""
import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from PIL import Image, ImageDraw
import yaml


def build(package, base_world):
    scene = yaml.safe_load((package/'config/demo_scene.yaml').read_text())
    root = ET.parse(base_world).getroot()
    world = root.find('world')
    world.set('name', 'bg1_navigation')
    resolution = scene['resolution']
    ox, oy, _ = scene['origin']
    width, height = [round(v/resolution) for v in scene['size']]
    bitmap = Image.new('L', (width,height), 254)
    draw = ImageDraw.Draw(bitmap)
    for name, x, y, length, breadth, z in scene['boxes']:
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = f'{x} {y} {z/2} 0 0 0'
        link = ET.SubElement(model, 'link', name='link')
        for kind in ('collision','visual'):
            element = ET.SubElement(link, kind, name=kind)
            box = ET.SubElement(ET.SubElement(element,'geometry'),'box')
            ET.SubElement(box,'size').text = f'{length} {breadth} {z}'
            if kind == 'visual':
                mat = ET.SubElement(element,'material')
                color = '0.25 0.40 0.48 1' if 'wall' not in name else '0.65 0.70 0.73 1'
                ET.SubElement(mat,'ambient').text = color
                ET.SubElement(mat,'diffuse').text = color
        x0 = math.floor((x-length/2-ox)/resolution)
        x1 = math.ceil((x+length/2-ox)/resolution)-1
        y0 = height-math.ceil((y+breadth/2-oy)/resolution)
        y1 = height-math.floor((y-breadth/2-oy)/resolution)-1
        draw.rectangle((x0,y0,x1,y1), fill=0)
    (package/'maps').mkdir(exist_ok=True)
    (package/'worlds').mkdir(exist_ok=True)
    bitmap.save(package/'maps/demo.pgm')
    (package/'maps/demo.yaml').write_text(yaml.safe_dump(dict(
        image='demo.pgm', mode='trinary', resolution=resolution, origin=scene['origin'],
        negate=0, occupied_thresh=0.65, free_thresh=0.25)))
    ET.indent(root)
    ET.ElementTree(root).write(package/'worlds/demo.sdf', encoding='unicode', xml_declaration=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--package', required=True, type=Path)
    parser.add_argument('--base-world', required=True, type=Path)
    args = parser.parse_args()
    build(args.package, args.base_world)
