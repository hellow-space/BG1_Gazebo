from pathlib import Path
import json,struct,hashlib
import numpy as np
from PIL import Image
import xml.etree.ElementTree as ET
import yaml
root=Path(__file__).resolve().parents[5];data=root/'data/maps/imported/A11123AJ04_231127_094529';out=data/'gazebo_demo';out.mkdir(exist_ok=True)
a=np.flipud(np.asarray(Image.open(data/(data.name+'.pgm'))));blocked=a<=191
active={};rects=[]
for y,row in enumerate(blocked):
 changes=np.diff(np.r_[False,row,False].astype('i1'));runs=set(zip(np.where(changes==1)[0],np.where(changes==-1)[0]))
 for key in list(active):
  if key not in runs:rects.append((*key,active.pop(key),y))
 for key in runs:active.setdefault(key,y)
for key,y in active.items():rects.append((*key,y,blocked.shape[0]))
projected=np.zeros(blocked.shape,dtype=bool)
faces=[(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),(1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)]
with (out/'obstacles.stl').open('wb') as f:
 f.write(b'BG1 original PGM grid extrusion'.ljust(80,b'\0'));f.write(struct.pack('<I',12*len(rects)))
 for x0,x1,y0,y1 in rects:
  projected[y0:y1,x0:x1]=True
  v=np.array([(x0*.05,y0*.05,0),(x1*.05,y0*.05,0),(x1*.05,y1*.05,0),(x0*.05,y1*.05,0),(x0*.05,y0*.05,1.2),(x1*.05,y0*.05,1.2),(x1*.05,y1*.05,1.2),(x0*.05,y1*.05,1.2)])
  for face in faces:
   t=v[list(face)];n=np.cross(t[1]-t[0],t[2]-t[0]);n=n/np.linalg.norm(n);f.write(struct.pack('<12fH',*n,*t.ravel(),0))
assert np.array_equal(projected,blocked)
sdf=ET.parse(root/'ros_ws/src/bg1_gazebo/worlds/bg1_flat.sdf');world=sdf.find('world');world.set('name','leader_imported')
ground=world.find("model[@name='ground_plane']");ET.SubElement(ground,'pose').text='30.2 42.75 0 0 0 0'
model=ET.SubElement(world,'model',name='imported_grid_obstacles');ET.SubElement(model,'static').text='true';link=ET.SubElement(model,'link',name='obstacles')
for kind in ['collision','visual']:
 el=ET.SubElement(link,kind,name=kind);mesh=ET.SubElement(ET.SubElement(el,'geometry'),'mesh');ET.SubElement(mesh,'uri').text=str(out/'obstacles.stl')
 if kind=='visual':
  mat=ET.SubElement(el,'material');ET.SubElement(mat,'ambient').text='.5 .55 .62 1';ET.SubElement(mat,'diffuse').text='.6 .65 .72 1'
# Use a local copy of existing GUI plugins, with a camera on this scene.
ET.indent(sdf);sdf.write(out/'world.sdf',encoding='unicode',xml_declaration=True)
mapcfg=dict(image='../'+data.name+'.pgm',mode='trinary',resolution=.05,origin=[0.,0.,0.],negate=0,occupied_thresh=.65,free_thresh=.25)
(out/'map.yaml').write_text(yaml.safe_dump(mapcfg,sort_keys=False))
report=dict(rectangles=len(rects),triangles=len(rects)*12,projection_matches_map=True,blocked_cells=int(blocked.sum()),height_m=1.2,resolution=.05,unknown_is_obstacle=True,source_sha256=hashlib.sha256((data/(data.name+'.pgm')).read_bytes()).hexdigest())
(out/'generation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
rviz=yaml.safe_load((root/'leader_coverage_ws/src/bg1_leader_coverage/rviz/coverage.rviz').read_text());rviz['Visualization Manager']['Views']['Current'].update(Scale=18.,X=37.,Y=52.)
Path(__file__).with_name('motion.rviz').write_text(yaml.safe_dump(rviz,sort_keys=False))
