"""Native IPA Local Energy order plus explicitly separate audited A* connections."""
import json
import math
import os
from pathlib import Path
import resource
import signal
import subprocess
import time
import numpy as np
from PIL import Image
from coverage_benchmark.adapters.ipa_build import build,sha
from coverage_benchmark.adapters.f2c_safe import reachable_mask
from coverage_benchmark.ipa_connection import assemble,strict_path_valid
from coverage_benchmark.models import PlanningResult
from coverage_benchmark.provenance import array_fingerprint

RUN_TIMEOUT_SECONDS=180
ADDRESS_SPACE_BYTES=3*1024**3


def prepare_input(request):
    if (request.free.shape!=request.navigable.shape or request.navigable.ndim!=2
            or np.any(request.navigable & ~request.free)):
        raise ValueError('Navigable raster must be a same-shape subset of target free raster')
    if not math.isfinite(request.tool_width) or request.tool_width<=0:raise ValueError('Invalid tool width')
    if not math.isfinite(request.resolution) or request.resolution<=0:raise ValueError('Invalid resolution')
    if not np.isfinite(request.start).all():raise ValueError('Invalid start')
    reachable,scope=reachable_mask(request)
    config=dict(start=list(request.start),resolution=request.resolution,tool_width=request.tool_width,
                clearance=request.clearance,coverage_radius=request.tool_width/2,
                grid_spacing_m=request.tool_width/2*math.sqrt(2),planning_mode='footprint',algorithm=6,
                native_internal_half_grid_erosion=True,pixel_origin=[request.resolution/2]*2)
    scope['raster_sha256']={name:array_fingerprint(value) for name,value in
                           (('target',request.free),('navigable',request.navigable),('reachable',reachable))}
    return config,scope


def _limits():
    resource.setrlimit(resource.RLIMIT_AS,(ADDRESS_SPACE_BYTES,ADDRESS_SPACE_BYTES))
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))


def run_native(command,env,log_path,timeout):
    with Path(log_path).open('w') as log:
        process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,
                                 start_new_session=True,preexec_fn=_limits)
        try:process.wait(timeout=timeout)
        except BaseException:
            if process.poll() is None:
                try:os.killpg(process.pid,signal.SIGKILL)
                except ProcessLookupError:pass
            process.wait();raise
        return process.returncode


def plan(request,work_dir):
    work=Path(work_dir);work.mkdir(parents=True,exist_ok=True)
    output=work/'ipa_output.txt';output.unlink(missing_ok=True)
    details=dict(native_invoked=False,timeout_seconds=RUN_TIMEOUT_SECONDS,address_space_limit_bytes=ADDRESS_SPACE_BYTES,
                 native_path=[],path_policy='preserve all native targets in order; common-start approach and audited A* connections')
    try:
        config,scope=prepare_input(request);details.update(scope,native_input=config)
        reachable,_=reachable_mask(request)
        Image.fromarray(np.flipud(reachable).astype('uint8')*255).save(work/'ipa_input.png')
        details['native_input_sha256']=sha(work/'ipa_input.png')
        (work/'ipa_input.json').write_text(json.dumps(config,indent=2)+'\n')
        executable,build_info=build(request.source_root);details['build']=build_info
        command=[str(executable),str(work/'ipa_input.png'),str(output),str(request.resolution),
                 *map(str,request.start),str(config['coverage_radius']),'IPA1']
        details['native_invoked']=True;started=time.monotonic()
        try:details['native_returncode']=run_native(command,dict(os.environ),work/'ipa_native.log',RUN_TIMEOUT_SECONDS)
        except subprocess.TimeoutExpired:
            details['timed_out']=True;raise ValueError('IPA native planning timed out')
        finally:
            details['native_elapsed_seconds']=time.monotonic()-started
            if (work/'ipa_native.log').exists():details['native_log_sha256']=sha(work/'ipa_native.log')
        if details['native_returncode']!=0:raise ValueError('IPA native exited '+str(details['native_returncode']))
        lines=output.read_text().splitlines();tag,count=lines[0].split()
        points=[list(map(float,line.split())) for line in lines[1:]]
        if tag!='IPA1' or len(points)!=int(count) or not points:raise ValueError('No complete native IPA target sequence')
        details['native_path']=points;details['native_output_sha256']=sha(output)
        lower=np.flipud(reachable)
        details['raw_native_closed_cell_valid']=strict_path_valid(points,lower,request.resolution)
        path,connections=assemble(request.start,points,lower,request.resolution)
        details['connections']=connections
        return PlanningResult(path,True,'Native Local Energy order with complete raster-safe connections',details)
    except (ValueError,RuntimeError,OSError,subprocess.SubprocessError,IndexError) as error:
        details['error']=str(error)
        return PlanningResult([],False,str(error),details)
