"""Fixed native Fields2Cover swaths plus an explicitly separate raster connector."""
import hashlib
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import numpy as np
from scipy.ndimage import label
from coverage_benchmark.models import PlanningResult
from coverage_benchmark.map_io import world_to_pixel
from coverage_benchmark.raster_geometry import polygonize, rasterize
from coverage_benchmark.obstacle_connector import assemble_path, usable_native_turns
from coverage_benchmark.provenance import array_fingerprint, native_fingerprints


def reachable_mask(request):
    row,col=world_to_pixel(request.start,request.navigable.shape[0],request.resolution)
    h,w=request.navigable.shape
    if not (0<=row<h and 0<=col<w and request.navigable[row,col]):
        raise ValueError('Requested start is outside clearance-allowed raster')
    components,count=label(request.navigable)
    reachable=components==components[row,col]
    return reachable,dict(navigable_components=count,reachable_pixels=int(reachable.sum()),
                          excluded_navigable_pixels=int(request.navigable.sum()-reachable.sum()),
                          scope='common start clearance component; other regions remain uncovered',
                          all_map_coverage_claimed=False)


def _polygon(mask,resolution):
    polygons=polygonize(mask,resolution)
    if len(polygons)!=1 or not polygons[0]['valid']:
        raise ValueError('Requested component does not form one valid polygon')
    if not np.array_equal(rasterize(polygons,mask.shape,resolution),mask):
        raise ValueError('Native polygon raster roundtrip mismatch')
    return polygons[0]


def extract_native_candidate(native):
    """Keep a completed ordered-route response even if optional native turns fail."""
    candidate=native.get('candidate')
    if candidate and candidate.get('ordered_swaths_source'):
        return candidate
    route=native.get('candidate_route')
    if route and route.get('success') and route.get('swaths_ordered'):
        return dict(route,path=[],path_swaths=[],turns=[],contains_turns=False,
                    native_path_success=False,native_path_error=native.get('error','Native turns unavailable'),
                    route_action_status=route['action_status'],route_error_code=route['error_code'],
                    ordered_swaths_source='native action generate_path=False')
    return candidate or {}


def plan(request,work_dir):
    work_dir=Path(work_dir).resolve()
    work_dir.mkdir(parents=True,exist_ok=True)
    details={}
    try:
        reachable,scope=reachable_mask(request)
        details.update(scope)
        details['raster_sha256']={name:array_fingerprint(mask) for name,mask in
                                  (('target',request.free),('navigable',request.navigable),('reachable',reachable))}
        details['native_fingerprints']=native_fingerprints(request.runtime_root)
        executable=request.runtime_root/'coverage_ws/build/opennav_coverage/opennav_coverage'
        config=dict(runtime_root=str(request.runtime_root),executable=str(executable),tool_width=request.tool_width,
                    clearance=request.clearance,
                    baseline=dict(polygon=_polygon(request.free,request.resolution),generate_headland=True),
                    candidate=dict(polygon=_polygon(reachable,request.resolution),generate_headland=False))
        (work_dir/'f2c_input.json').write_text(json.dumps(config,separators=(',',':'))+'\n')
        worker=Path(__file__).with_name('f2c_worker.py')
        command=['bash','--noprofile','--norc','-c',
                 'set -e; source "$1"; source "$2"; source "$3"; exec python3 "$4" "$5"',
                 'f2c-obstacle','/opt/ros/jazzy/setup.bash',str(request.runtime_root/'ros_ws/install/setup.bash'),
                 str(request.runtime_root/'coverage_ws/install/setup.bash'),str(worker),str(work_dir)]
        env=dict(os.environ,ROS_DOMAIN_ID='102',ROS_LOG_DIR=str(work_dir/'ros_logs'))
        with Path('/tmp/bg1_f2c_domain_102.lock').open('w') as domain_lock, (work_dir/'f2c_worker.log').open('w') as log:
            fcntl.flock(domain_lock,fcntl.LOCK_EX)
            process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:process.wait(timeout=300)
            except BaseException:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=12)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
                raise
        if process.returncode:raise RuntimeError(f'Native F2C worker exited {process.returncode}; see {work_dir}/f2c_worker.log')
        native=json.loads((work_dir/'f2c_output.json').read_text())
        details['native_baseline']=native.get('baseline',{})
        details['native_candidate']=extract_native_candidate(native)
        details['native_worker_error']=native.get('error')
        details['native_address_space_limit_bytes']=native.get('native_address_space_limit_bytes')
        details['native_server_exit_code']=native.get('server_exit_code')
        details['native_server_pid']=native.get('server_pid')
        details['native_parameters']=native.get('details',{}).get('parameters')
        details['native_input']=config
        details['native_executable_sha256']=hashlib.sha256(executable.read_bytes()).hexdigest()
        for name in ('Fields2Cover','opennav_coverage'):
            directory=request.runtime_root/'third_party'/name
            details[name+'_commit']=subprocess.check_output(['git','-C',str(directory),'rev-parse','HEAD'],text=True).strip()
            if subprocess.check_output(['git','-C',str(directory),'status','--porcelain'],text=True).strip():
                raise ValueError(f'{name} upstream contains uncommitted modifications')
        candidate=details['native_candidate']
        if not candidate.get('success'):
            raise RuntimeError(native.get('error','Native F2C candidate failed'))
        if not candidate['swaths_ordered']:
            raise ValueError('Expected ordered native swaths')
        turns=usable_native_turns(candidate['swaths'],candidate['path_swaths'],candidate['turns'])
        details['native_turn_mapping_verified']=bool(turns)
        path,connection=assemble_path(candidate['swaths'],turns,np.flipud(reachable),
                                      request.resolution,request.start)
        details.update(connection)
        return PlanningResult(path,True,'Complete raster-audited route in explicit start-reachable scope',details)
    except Exception as error:
        return PlanningResult(success=False,message=f'{type(error).__name__}: {error}',details=details)
