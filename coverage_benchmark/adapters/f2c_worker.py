"""Two native F2C action requests in a dedicated coverage overlay; no motion."""
import json
import math
from pathlib import Path
import signal
import resource
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from lifecycle_msgs.srv import ChangeState
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates
import yaml


def main():
    directory=Path(sys.argv[1])
    config=json.loads((directory/'f2c_input.json').read_text())
    runtime=Path(config['runtime_root'])
    parameters=yaml.safe_load((runtime/'coverage_ws/src/bg1_coverage_demo/config/coverage.yaml').read_text())
    parameters['coverage_server']['ros__parameters'].update(
        use_sim_time=False,operation_width=config['tool_width'],default_swath_angle=math.pi/2,
        default_headland_width=config['clearance'],min_turning_radius=.6)
    parameter_file=directory/'f2c_parameters.yaml'
    parameter_file.write_text(yaml.safe_dump(parameters,sort_keys=False))
    rclpy.init()
    node=Node('f2c_obstacle_benchmark')
    process=None
    address_limit=2*1024**3
    output={'success':False,'details':{'parameters':parameters},'native_address_space_limit_bytes':address_limit}
    def wait(future,timeout=120):
        deadline=time.monotonic()+timeout
        while not future.done():
            if time.monotonic()>deadline:raise TimeoutError('Native F2C action timed out')
            if process is not None and process.poll() is not None:raise RuntimeError(f'Native server exited {process.returncode}')
            rclpy.spin_once(node,timeout_sec=.01)
        return future.result()
    try:
        with (directory/'f2c_server.log').open('w') as log:
            process=subprocess.Popen([config['executable'],'--ros-args','-r','__node:=coverage_server',
                                      '--params-file',str(parameter_file)],stdout=log,stderr=subprocess.STDOUT)
        output['server_pid']=process.pid
        resource.prlimit(process.pid,resource.RLIMIT_AS,(address_limit,address_limit))
        lifecycle=node.create_client(ChangeState,'/coverage_server/change_state')
        if not lifecycle.wait_for_service(timeout_sec=30):raise TimeoutError('Coverage lifecycle unavailable')
        for transition in (1,3):
            change=ChangeState.Request();change.transition.id=transition
            if not wait(lifecycle.call_async(change),30).success:raise RuntimeError('Coverage lifecycle failed')
        client=ActionClient(node,ComputeCoveragePath,'/compute_coverage_path')
        if not client.wait_for_server(timeout_sec=30):raise TimeoutError('Coverage action unavailable')
        for name in ('baseline','candidate_route','candidate'):
            item=config['candidate' if name=='candidate_route' else name]
            request=ComputeCoveragePath.Goal()
            request.frame_id='map'
            request.generate_headland=item['generate_headland']
            request.generate_route=True
            request.generate_path=(name!='candidate_route')
            request.polygons=[Coordinates(coordinates=[Coordinate(axis1=float(x),axis2=float(y)) for x,y in ring])
                              for ring in item['polygon']['geometry']['coordinates']]
            goal=wait(client.send_goal_async(request),30)
            if not goal.accepted:raise RuntimeError('Native action rejected')
            response=wait(goal.get_result_async())
            native=response.result
            components=native.coverage_path
            output[name]=dict(success=response.status==4 and native.error_code==0,
                              action_status=response.status,error_code=native.error_code,
                              path=[[p.pose.position.x,p.pose.position.y] for p in native.nav_path.poses],
                              swaths=[[[s.start.x,s.start.y],[s.end.x,s.end.y]] for s in components.swaths],
                              turns=[[[p.pose.position.x,p.pose.position.y] for p in turn.poses] for turn in components.turns],
                              contains_turns=components.contains_turns,swaths_ordered=components.swaths_ordered,
                              generate_headland=item['generate_headland'])
        # Obtain exact ordered swaths directly; full-path messages group states by type.
        candidate=output['candidate']
        route=output.pop('candidate_route')
        candidate['path_swaths']=candidate['swaths']
        candidate['native_path_success']=candidate['success']
        if not candidate['success']:
            candidate['native_path_error']=f"Native path action status={candidate['action_status']} error_code={candidate['error_code']}"
        candidate['swaths']=route['swaths']
        candidate['swaths_ordered']=route['swaths_ordered']
        candidate['success']=route['success']
        candidate['route_action_status']=route['action_status']
        candidate['route_error_code']=route['error_code']
        candidate['ordered_swaths_source']='native action generate_path=False'
        output['success']=True
    except Exception as error:
        output['error']=f'{type(error).__name__}: {error}'
    finally:
        node.destroy_node();rclpy.try_shutdown()
        if process is not None:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:process.kill();process.wait()
            output['server_exit_code']=process.returncode
        (directory/'f2c_output.json').write_text(json.dumps(output,separators=(',',':'))+'\n')


if __name__=='__main__':main()
