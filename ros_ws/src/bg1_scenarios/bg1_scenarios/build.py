"""Compile matching Gazebo geometry and reference occupancy maps."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from PIL import Image,ImageDraw
import yaml

COLORS={'wall':'0.78 0.81 0.83 1','rack':'0.12 0.27 0.38 1',
        'pallet':'0.63 0.40 0.19 1','column':'0.52 0.57 0.61 1',
        'desk':'0.78 0.64 0.46 1','cabinet':'0.36 0.43 0.48 1'}


def expand_layout(config):
    name=config['name']
    if name not in ('warehouse','office'):raise ValueError(f'Unknown scene: {name}')
    objects=[];goals=[list(p) for p in config['goals']]
    def box(label,x,y,l,w,h,kind='wall'):
        objects.append(dict(name=label,shape='box',x=float(x),y=float(y),size=[float(l),float(w),float(h)],kind=kind))
    lx,ly=config['size'];t=config['wall_thickness'];h=config['wall_height']
    box('wall_south',0,-ly/2,lx,t,h);box('wall_north',0,ly/2,lx,t,h)
    box('wall_west',-lx/2,0,t,ly,h);box('wall_east',lx/2,0,t,ly,h)
    if name=='warehouse':
        racks=config['racks']
        for i,x in enumerate(racks['x']):
            for j,y in enumerate(racks['y']):box(f'rack_{i}_{j}',x,y,*racks['size'],'rack')
        for i,(x,y) in enumerate(config['pallets']['centers']):box(f'pallet_{i}',x,y,*config['pallets']['size'],'pallet')
        for i,(x,y) in enumerate(config['columns']['centers']):
            objects.append(dict(name=f'column_{i}',shape='cylinder',x=x,y=y,
                radius=config['columns']['radius'],height=config['columns']['height'],kind='column'))
    else:
        def partition(axis,fixed,lo,hi,doors,label):
            half=config['door_width']/2;cursor=lo;index=0
            for center in sorted(v for v in doors if lo+half<v<hi-half):
                if center-half>cursor:
                    length=center-half-cursor;mid=(cursor+center-half)/2
                    box(f'{label}_{index}',mid if axis=='x' else fixed,fixed if axis=='x' else mid,
                        length if axis=='x' else t,t if axis=='x' else length,h)
                    index+=1
                cursor=center+half
            if cursor<hi:
                mid=(cursor+hi)/2
                box(f'{label}_{index}',mid if axis=='x' else fixed,fixed if axis=='x' else mid,
                    hi-cursor if axis=='x' else t,t if axis=='x' else hi-cursor,h)
        c=config['corridor_half_width']
        for i,x in enumerate(config['vertical_walls']):
            for j,(lo,hi) in enumerate([(-ly/2,-c),(c,ly/2)]):
                partition('y',x,lo,hi,config['vertical_door_centers'],f'wall_vertical_{i}_{j}')
        for i,y in enumerate(config['horizontal_walls']):
            for j,(lo,hi) in enumerate([(-lx/2,-c),(c,lx/2)]):
                partition('x',y,lo,hi,config['horizontal_door_centers'],f'wall_horizontal_{i}_{j}')
        for i,(xl,xr) in enumerate(config['room_x_intervals']):
            for j,(yl,yr) in enumerate(config['room_y_intervals']):
                box(f'desk_{i}_{j}',xl+1.2,yr-1.0,1.6,.7,.75,'desk')
                box(f'cabinet_{i}_{j}',xr-.65,yr-1.0,.8,1.2,1.1,'cabinet')
                goals.append([(xl+xr)/2,(yl+yr)/2])
    if any(min(o.get('size',[o.get('radius',0),o.get('height',0)]))<=0 for o in objects):
        raise ValueError('Nonpositive obstacle geometry')
    return objects,goals


def render_map(config,objects):
    resolution=config['resolution'];lx,ly=config['size'];margin=config['map_margin']
    origin=[-lx/2-margin,-ly/2-margin,0.0]
    width=round((lx+2*margin)/resolution);height=round((ly+2*margin)/resolution)
    bitmap=Image.new('L',(width,height),205);draw=ImageDraw.Draw(bitmap)
    def pixels(xlo,ylo,xhi,yhi):
        return (math.floor((xlo-origin[0])/resolution),height-math.ceil((yhi-origin[1])/resolution),
                math.ceil((xhi-origin[0])/resolution)-1,height-math.floor((ylo-origin[1])/resolution)-1)
    draw.rectangle(pixels(-lx/2,-ly/2,lx/2,ly/2),fill=254)
    for obstacle in objects:
        x,y=obstacle['x'],obstacle['y']
        if obstacle['shape']=='box':
            l,w,_=obstacle['size'];draw.rectangle(pixels(x-l/2,y-w/2,x+l/2,y+w/2),fill=0)
        else:
            r=obstacle['radius'];draw.ellipse(pixels(x-r,y-r,x+r,y+r),fill=0)
    return bitmap,origin


def visual(link,name,size,pose,color):
    element=ET.SubElement(link,'visual',name=name)
    ET.SubElement(element,'pose').text=' '.join(map(str,pose))
    geometry=ET.SubElement(element,'geometry');box=ET.SubElement(geometry,'box')
    ET.SubElement(box,'size').text=' '.join(map(str,size))
    material=ET.SubElement(element,'material')
    ET.SubElement(material,'ambient').text=color;ET.SubElement(material,'diffuse').text=color


def add_floor_details(world,config):
    model=ET.SubElement(world,'model',name='floor_markings');ET.SubElement(model,'static').text='true'
    link=ET.SubElement(model,'link',name='floor')
    lx,ly=config['size']
    def patch(label,x,y,l,w,color,z=.004):visual(link,label,[l,w,.003],[x,y,z,0,0,0],color)
    if config['name']=='warehouse':
        patch('cross_aisle',0,0,lx-.5,5.5,'0.42 0.47 0.49 1')
        patch('center_aisle',0,0,5.3,ly-.5,'0.42 0.47 0.49 1',.008)
        for y in (-2.8,2.8):patch(f'cross_line_{y}',0,y,lx-.5,.08,'0.95 0.73 0.22 1',.012)
        for y in range(-12,13,3):patch(f'center_dash_{y}',0,y,.09,1.2,'0.95 0.80 0.38 1',.012)
        for i,x in enumerate(config['racks']['x']):
            for j,y in enumerate(config['racks']['y']):
                patch(f'rack_zone_{i}_{j}',x,y,1.85,8.2,'0.69 0.58 0.36 1')
    else:
        patch('main_hall',0,0,lx-.4,3.8,'0.59 0.66 0.69 1')
        patch('cross_hall',0,0,3.8,ly-.4,'0.59 0.66 0.69 1',.008)
        for i,(xl,xr) in enumerate(config['room_x_intervals']):
            for j,(yl,yr) in enumerate(config['room_y_intervals']):
                color='0.75 0.72 0.64 1' if (i+j)%2 else '0.65 0.72 0.70 1'
                patch(f'room_floor_{i}_{j}',(xl+xr)/2,(yl+yr)/2,xr-xl-.24,yr-yl-.24,color)


def build_scene(config_path,base_world,output):
    config=yaml.safe_load(Path(config_path).read_text());name=config['name']
    objects,goals=expand_layout(config);root=ET.parse(base_world).getroot();world=root.find('world')
    world.set('name',f'bg1_{name}')
    add_floor_details(world,config)
    for obstacle in objects:
        model=ET.SubElement(world,'model',name=obstacle['name']);ET.SubElement(model,'static').text='true'
        height=obstacle['size'][2] if obstacle['shape']=='box' else obstacle['height']
        ET.SubElement(model,'pose').text=f'{obstacle["x"]} {obstacle["y"]} {height/2} 0 0 0'
        link=ET.SubElement(model,'link',name='link')
        for kind in ('collision','visual'):
            element=ET.SubElement(link,kind,name=kind);geometry=ET.SubElement(element,'geometry')
            if obstacle['shape']=='box':
                shape=ET.SubElement(geometry,'box');ET.SubElement(shape,'size').text=' '.join(map(str,obstacle['size']))
            else:
                shape=ET.SubElement(geometry,'cylinder')
                ET.SubElement(shape,'radius').text=str(obstacle['radius']);ET.SubElement(shape,'length').text=str(height)
            if kind=='visual':
                mat=ET.SubElement(element,'material');color=COLORS[obstacle['kind']]
                ET.SubElement(mat,'ambient').text=color;ET.SubElement(mat,'diffuse').text=color
        # All accents remain inside the solid collision/laser silhouette.
        if obstacle['kind']=='rack':
            l,w,h=obstacle['size']
            for k,z in enumerate((.20,.95,1.70,2.25)):
                visual(link,f'shelf_band_{k}',[l,w-.05,.065],[0,0,z-h/2,0,0,0],'0.91 0.54 0.17 1')
            for side in (-1,1):
                for k,y in enumerate((-3.8,0,3.8)):
                    visual(link,f'upright_{side}_{k}',[.06,.13,h],[side*(l/2-.03),y,0,0,0,0],'0.22 0.47 0.64 1')
        elif obstacle['kind']=='wall':
            l,w,h=obstacle['size']
            visual(link,'wall_cap',[l,w,.08],[0,0,h/2-.04,0,0,0],'0.37 0.47 0.52 1')
        elif obstacle['kind']=='pallet':
            l,w,h=obstacle['size']
            visual(link,'crate_top',[l-.05,w-.05,.04],[0,0,h/2-.02,0,0,0],'0.83 0.64 0.35 1')
    output=Path(output)
    for directory in ('worlds','maps','metadata'):(output/directory).mkdir(parents=True,exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(output/'worlds'/f'{name}.sdf',encoding='unicode',xml_declaration=True)
    bitmap,origin=render_map(config,objects);bitmap.save(output/'maps'/f'{name}.pgm')
    (output/'maps'/f'{name}.yaml').write_text(yaml.safe_dump(dict(image=f'{name}.pgm',mode='trinary',
        resolution=config['resolution'],origin=origin,negate=0,occupied_thresh=.65,free_thresh=.196)))
    report=dict(name=name,title=config['title'],size=config['size'],world_name=f'bg1_{name}',
        spawn=config['spawn'],goals=goals,resolution=config['resolution'],objects=objects)
    (output/'metadata'/f'{name}.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
