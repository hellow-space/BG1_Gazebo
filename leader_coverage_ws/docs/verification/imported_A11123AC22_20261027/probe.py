import json, math, time, hashlib
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point32
from bg1_leader_coverage_interfaces.srv import PlanCoverage

root=Path(__file__).resolve().parents[4]
out=root/'leader_coverage_ws/docs/verification/imported_A11123AC22_20261027'
out.mkdir(exist_ok=True)
rclpy.init();node=Node('leader_imported_map_preview');maps=[]
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
sub=node.create_subscription(OccupancyGrid,'/map',maps.append,qos)
client=node.create_client(PlanCoverage,'/leader_coverage/plan')
deadline=time.monotonic()+30
while not maps:
 if time.monotonic()>deadline:raise TimeoutError('No map')
 rclpy.spin_once(node,timeout_sec=.1)
m=maps[-1];a=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
r=m.info.resolution;ox=m.info.origin.position.x;oy=m.info.origin.position.y
from PIL import Image
source_image=root/'data/maps/imported/A11123AC22_231213_163036/A11123AC22_231213_163036.png'
pixels=np.asarray(Image.open(source_image).convert('L'))
expected=np.flipud(np.where(pixels==255,0,np.where(pixels==0,100,-1)))
assert np.array_equal(a,expected), 'Loaded map does not match requested PNG'
clear=distance_transform_edt(a==0);sy,sx=np.unravel_index(clear.argmax(),clear.shape)
# Select the complete image extent, leaving one cell inside map bounds.
region=[(ox+r,oy+r),(ox+(m.info.width-1)*r,oy+r),(ox+(m.info.width-1)*r,oy+(m.info.height-1)*r),(ox+r,oy+(m.info.height-1)*r)]
req=PlanCoverage.Request();req.region.header.frame_id='map'
req.region.polygon.points=[Point32(x=float(x),y=float(y)) for x,y in region]
req.start.x=ox+(sx+.5)*r;req.start.y=oy+(sy+.5)*r
req.pattern=0;req.stripe_spacing=.7;req.clearance=1.
assert client.wait_for_service(timeout_sec=30)
start=time.monotonic();future=client.call_async(req)
while not future.done():
 if time.monotonic()-start>180:raise TimeoutError('Planning timeout')
 rclpy.spin_once(node,timeout_sec=.1)
resp=future.result();elapsed=time.monotonic()-start
path=[[p.pose.position.x,p.pose.position.y] for p in resp.coverage_path.poses]
png=root/'data/maps/imported/A11123AC22_231213_163036/A11123AC22_231213_163036.png'
hashes=json.loads((root/'leader_coverage_ws/install/bg1_leader_coverage/share/bg1_leader_coverage/source_hashes.json').read_text())
assert all(hashlib.sha256((root/'third_party/leader/coverage_path_planner'/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
report=dict(map_id=png.stem,source_image=str(png.relative_to(root)),source_sha256=hashlib.sha256(png.read_bytes()).hexdigest(),source_hashes_match=True,loaded_map_matches_png=True,
 algorithm='Leader CoveragePathPlanner, build-generated ROS 2 adaptation',pattern='zigzag',resolution=r,width=m.info.width,height=m.info.height,
 origin=[ox,oy,0],metadata_assumptions=['0.05 m/pixel inferred from same-name, same-size PGM header; PNG metadata unavailable','Origin [0,0,0] is an arbitrary local planning frame; original origin unavailable','Trinary free_thresh=0.25, occupied_thresh=0.65, negate=0'],
 region=region,start=[req.start.x,req.start.y],stripe_spacing_m=req.stripe_spacing,clearance_m=req.clearance,
 success=resp.success,message=resp.message,duration_s=elapsed,poses=len(path),length_m=sum(math.dist(x,y) for x,y in zip(path,path[1:])),
 known_free_area_m2=float((a==0).sum()*r*r),motion_executed=False,full_coverage_verified=False,path=path)
(out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
(out/'source_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='path'},indent=2),flush=True)
assert resp.success,resp.message
assert len(path)>20
samples=[]
for x,y in zip(path,path[1:]):
 n=max(1,math.ceil(math.dist(x,y)/(r*.5)))
 samples.extend([(x[0]+(y[0]-x[0])*j/n,x[1]+(y[1]-x[1])*j/n) for j in range(n+1)])
xy=np.array(samples);ix=((xy[:,0]-ox)/r).astype(int);iy=((xy[:,1]-oy)/r).astype(int)
assert np.all((ix>=0)&(ix<m.info.width)&(iy>=0)&(iy<m.info.height))
assert np.all(a[iy,ix]==0)
report['independent_check']=dict(segment_samples=len(samples),all_samples_in_known_free_space=True,min_obstacle_distance_m=float(clear[iy,ix].min()*r))
(out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
fig,axes=plt.subplots(1,2,figsize=(20,8),layout='constrained')
img=np.where(a<0,0,np.where(a==0,1,2))
ys,xs=np.where(a==0)
for ax in axes:
 ax.imshow(img,origin='lower',extent=[ox,ox+m.info.width*r,oy,oy+m.info.height*r],cmap=ListedColormap(['#808080','#f7f7f7','#171717']),interpolation='nearest',vmin=0,vmax=2)
 ax.set_xlim(max(0,xs.min()*r-1),min(m.info.width*r,xs.max()*r+1));ax.set_ylim(max(0,ys.min()*r-1),min(m.info.height*r,ys.max()*r+1))
 ax.set_aspect('equal');ax.set_xlabel('Local map x (m)');ax.set_ylabel('Local map y (m)')
axes[0].set_title('Input PNG / '+png.stem)
x,y=zip(*path);axes[1].plot(x,y,color='#008b9b',lw=.8,label='Leader zigzag coverage')
axes[1].scatter(*path[0],color='#00a040',s=50,zorder=5,label='Path start')
axes[1].scatter(*path[-1],color='#e84b35',s=50,zorder=5,label='Path end')
axes[1].set_title(f'Original Leader algorithm / {len(path):,} poses / {report["length_m"]:.1f} m')
axes[1].legend(loc='lower right',fontsize=8)
fig.suptitle('Planning only | 0.05 m/pixel | spacing 0.7 m | clearance 1.0 m | arbitrary local origin',fontsize=11)
fig.savefig(out/'comparison.png',dpi=160);plt.close(fig)
fig,ax=plt.subplots(figsize=(15,10),layout='constrained')
ax.imshow(img,origin='lower',extent=[ox,ox+m.info.width*r,oy,oy+m.info.height*r],cmap=ListedColormap(['#808080','#f7f7f7','#171717']),interpolation='nearest',vmin=0,vmax=2)
ax.plot(x,y,color='#008b9b',lw=.9,label='Leader zigzag path')
ax.scatter(*path[0],color='#00a040',s=65,zorder=5,label='Path start')
ax.scatter(*path[-1],color='#e84b35',s=65,zorder=5,label='Path end')
ax.set_xlim(max(0,xs.min()*r-1),min(m.info.width*r,xs.max()*r+1));ax.set_ylim(max(0,ys.min()*r-1),min(m.info.height*r,ys.max()*r+1))
ax.set_aspect('equal');ax.set_xlabel('Local map x (m)');ax.set_ylabel('Local map y (m)')
ax.set_title(f'{png.stem}.png\nLeader zigzag | {len(path):,} poses | {report["length_m"]:.1f} m | spacing 0.7 m | clearance 1.0 m')
ax.legend(loc='upper right');fig.savefig(out/'coverage.png',dpi=160);plt.close(fig)
print('VERIFIED: '+json.dumps(report['independent_check'])+'; finite planning probe completed',flush=True)
node.destroy_node();rclpy.try_shutdown()
