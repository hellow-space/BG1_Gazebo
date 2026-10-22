#!/usr/bin/env python3
"""Actual Gazebo motion/cancellation test; preserve and restore user's region."""
import copy,json,math,os,signal,subprocess,threading,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rosidl_runtime_py.set_message import set_message_fields

ROOT=Path(__file__).resolve().parents[4]
OUT=ROOT/'leader_coverage_ws/docs/verification/motion'


def main():
    saved=json.loads((OUT/'original_selection.json').read_text())
    original=NavPath();set_message_fields(original,saved['path'])
    vertices=saved['region']['polygon']['points']
    rclpy.init();node=Node('leader_motion_runtime_probe');state={'motion':None,'path':None,'selection':''}
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String,'/leader_coverage/motion_status',lambda m:state.update(motion=json.loads(m.data)),qos)
    node.create_subscription(String,'/leader_coverage/status',lambda m:state.update(selection=m.data),qos)
    node.create_subscription(NavPath,'/leader_coverage/coverage_path',lambda m:state.update(path=m),qos)
    pub=node.create_publisher(NavPath,'/leader_coverage/coverage_path',qos)
    clicks=node.create_publisher(PointStamped,'/clicked_point',10)
    clients={cmd:node.create_client(Trigger,f'/leader_coverage/selection/{cmd}') for cmd in ('reset','undo')}
    samples=[];processes=[]
    gz=subprocess.Popen(['gz','topic','-e','-t','/world/bg1_warehouse/dynamic_pose/info','--json-output'],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)
    def collect():
        for line in gz.stdout:
            try:
                message=json.loads(line);pose=next(p for p in message['pose'] if p['name']=='bg1_pro');p=pose['position']
                samples.append({'t':time.monotonic(),'x':p.get('x',0.),'y':p.get('y',0.)})
            except (ValueError,KeyError,StopIteration):pass
    thread=threading.Thread(target=collect,daemon=True);thread.start()
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    def until(condition,timeout=120):
        end=time.monotonic()+timeout;last=time.monotonic()
        while not condition():
            if time.monotonic()>end:raise TimeoutError(str(state['motion']))
            rclpy.spin_once(node,timeout_sec=.1)
            if time.monotonic()-last>10:
                print('Motion:',state['motion'],'actual:',samples[-1] if samples else None,flush=True);last=time.monotonic()
    def trigger(command):
        client=clients[command];assert client.wait_for_service(timeout_sec=15)
        future=client.call_async(Trigger.Request());until(future.done,20);assert future.result().success,future.result().message
    def restore():
        trigger('reset');spin(.2)
        until(lambda:clicks.get_subscription_count()>0,10)
        for point in vertices+[vertices[0]]:
            msg=PointStamped();msg.header.frame_id='map';msg.point.x=float(point['x']);msg.point.y=float(point['y'])
            clicks.publish(msg);spin(.2)
        until(lambda:state['path'] is not None and len(state['path'].poses)>1,30)
    def publish(path):
        pub.publish(path);spin(.4)
    def start():
        proc=subprocess.Popen(['bash','leader_coverage_ws/motion.sh','start'],cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
        processes.append(proc);return proc
    def wait_terminal(timeout=120):until(lambda:state['motion'] and state['motion']['phase'] in ('completed','stopped','failed'),timeout)
    def stopped_drift():
        spin(1);a=copy.copy(samples[-1]);spin(2);b=samples[-1]
        drift=math.hypot(b['x']-a['x'],b['y']-a['y']);assert drift<.03,drift;return drift
    def cli_done(proc):
        text=proc.communicate(timeout=10)[0];print(text,flush=True);assert proc.returncode==0,text
    report={'passed':False,'tests':[]}
    try:
        until(lambda:state['motion'] is not None and samples,30);restore()
        # 120 original leader-generated poses include a U turn; not a new artificial route.
        short=copy.deepcopy(original);short.poses=short.poses[:120]
        publish(short);before=copy.copy(samples[-1]);index=len(samples)
        proc=start();until(lambda:state['motion']['phase'] in ('approaching','following'))
        wait_terminal(180);cli_done(proc)
        assert state['motion']['phase']=='completed',state['motion']
        assert state['motion']['approach_succeeded'] and state['motion']['follow_succeeded']
        travel=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(samples[index:],samples[index+1:]))
        endpoint=short.poses[-1].pose.position;actual=samples[-1]
        error=math.hypot(actual['x']-endpoint.x,actual['y']-endpoint.y)
        assert travel>2 and error<.4,(travel,error)
        report['tests'].append({'name':'approach_and_follow_short_leader_path','poses':120,'actual_travel_m':travel,'endpoint_error_m':error,'drift_m':stopped_drift()})
        # Real external stop command while original full selected path is being followed.
        publish(original);proc=start();until(lambda:state['motion']['phase']=='following',120);spin(3)
        stopped=subprocess.run(['bash','leader_coverage_ws/motion.sh','stop'],cwd=ROOT,capture_output=True,text=True,timeout=20)
        assert stopped.returncode==0,stopped.stdout+stopped.stderr
        wait_terminal(20);cli_done(proc);assert state['motion']['phase']=='stopped'
        report['tests'].append({'name':'manual_stop_during_follow','drift_m':stopped_drift()})
        # A real selector undo publishes an empty path and must cancel active navigation.
        proc=start();until(lambda:state['motion']['phase'] in ('approaching','following'));spin(.5)
        trigger('undo');wait_terminal(20);cli_done(proc);assert state['motion']['phase']=='stopped'
        report['tests'].append({'name':'selection_change_cancels','drift_m':stopped_drift()})
        restore()
        # Ctrl+C of foreground command must cancel the central action, not only the CLI.
        proc=start();until(lambda:state['motion']['phase'] in ('approaching','following'));spin(.5)
        os.killpg(proc.pid,signal.SIGINT);wait_terminal(20);cli_done(proc)
        assert state['motion']['phase']=='stopped'
        report['tests'].append({'name':'ctrl_c_cancels','drift_m':stopped_drift()})
        report['passed']=True
    except Exception as error:report['error']=str(error) or type(error).__name__
    finally:
        subprocess.run(['bash','leader_coverage_ws/motion.sh','stop'],cwd=ROOT,capture_output=True,text=True,timeout=20)
        for proc in processes:
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=15)
                except subprocess.TimeoutExpired:proc.terminate()
        try:restore()
        except Exception as error:report['restore_error']=str(error);report['passed']=False
        report['samples']=samples
        OUT.mkdir(parents=True,exist_ok=True);(OUT/'runtime.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2),flush=True)
        gz.terminate();gz.wait(timeout=5);node.destroy_node();rclpy.try_shutdown()
    raise SystemExit(0 if report['passed'] else 1)

if __name__=='__main__':main()
