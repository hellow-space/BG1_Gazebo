import json,math,time,threading,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import Point32
from std_msgs.msg import String
from bg1_leader_coverage_interfaces.srv import PlanCoverage
from tf2_ros import Buffer,TransformListener
root=Path(__file__).resolve().parents[5];out=root/'leader_coverage_ws/docs/verification/imported_A11123AJ04_20261022/motion';out.mkdir(exist_ok=True)
saved=json.loads((out.parent/'result.json').read_text())
rclpy.init();node=Node('leader_imported_motion_probe',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
buf=Buffer();listener=TransformListener(buf,node)
state={'motion':None};node.create_subscription(String,'/leader_coverage/motion_status',lambda m:state.update(motion=json.loads(m.data)),qos)
client=node.create_client(PlanCoverage,'/leader_coverage/plan');assert client.wait_for_service(timeout_sec=30)
req=PlanCoverage.Request();req.region.header.frame_id='map';req.region.polygon.points=[Point32(x=float(x),y=float(y)) for x,y in saved['region']];req.start.x=saved['start'][0];req.start.y=saved['start'][1];req.pattern=0;req.stripe_spacing=.7;req.clearance=1.
f=client.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=60);resp=f.result();assert resp and resp.success,resp.message if resp else 'no response'
path=resp.coverage_path;coords=np.array([(p.pose.position.x,p.pose.position.y) for p in path.poses]);tree=cKDTree(coords)
report={'map_id':saved['map_id'],'localization':'AMCL, known simulated spawn pose','ground_truth_localization':False,'world':'leader_imported','spawn':[34.325,52.425,0.], 'path_poses':len(path.poses),'path_length_m':sum(math.dist(a,b) for a,b in zip(coords,coords[1:])),'motion':None,'actual_travel_m':0.,'samples':[],'full_route_completed':False}
pub=node.create_publisher(NavPath,'/leader_coverage/coverage_path',qos);bound=node.create_publisher(NavPath,'/leader_coverage/boundary_path',qos);pub.publish(path);bound.publish(resp.boundary_path)
# Independent simulator pose stream, not odometry.
gz=subprocess.Popen(['gz','topic','-e','-t','/world/leader_imported/dynamic_pose/info','--json-output'],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)
actual={'value':None}
def collect():
 for line in gz.stdout:
  try:
   d=json.loads(line);p=next(p for p in d['pose'] if p['name']=='bg1_pro');pos=p['position'];q=p['orientation'];s=d['header']['stamp'];qx=q.get('x',0);qy=q.get('y',0);qz=q.get('z',0);qw=q.get('w',1)
   actual['value']=(float(s.get('sec',0))+float(s.get('nsec',0))*1e-9,pos.get('x',0),pos.get('y',0),math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz)))
  except (KeyError,ValueError,StopIteration):pass
threading.Thread(target=collect,daemon=True).start()
laststamp=-1;lastsave=0;start=time.monotonic()
print('Path published: '+str(len(path.poses))+'; waiting for explicit motion start',flush=True)
try:
 while rclpy.ok():
  rclpy.spin_once(node,timeout_sec=.05);p=actual['value'];now=time.monotonic()
  if p and p[0]-laststamp>=.15:
   sample={'sim_time':p[0],'wall_elapsed_s':now-start,'x':p[1],'y':p[2],'yaw':p[3]};laststamp=p[0]
   dist,index=tree.query(p[1:3]);sample.update(path_distance_m=float(dist),nearest_path_index=int(index))
   try:
    tf=buf.lookup_transform('map','base_footprint',rclpy.time.Time());x=tf.transform.translation.x;y=tf.transform.translation.y;sample.update(amcl_x=x,amcl_y=y,localization_error_m=math.hypot(x-p[1],y-p[2]))
   except Exception:pass
   if report['samples']:
    prev=report['samples'][-1];report['actual_travel_m']+=math.hypot(p[1]-prev['x'],p[2]-prev['y'])
   report['samples'].append(sample)
  if now-lastsave>10:
   lastsave=now;report['motion']=state['motion'];report['full_route_completed']=bool(state['motion'] and state['motion']['phase']=='completed');report['wall_elapsed_s']=now-start
   (out/'runtime.json').write_text(json.dumps(report,indent=2)+'\n')
   print(json.dumps({k:v for k,v in report.items() if k!='samples'})+' latest='+str(report['samples'][-1] if report['samples'] else None),flush=True)
finally:
 gz.terminate()
 try:gz.wait(timeout=5)
 except subprocess.TimeoutExpired:gz.kill();gz.wait()
 node.destroy_node();rclpy.try_shutdown()
