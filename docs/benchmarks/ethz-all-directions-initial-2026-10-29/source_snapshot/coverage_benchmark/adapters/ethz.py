"""Raster-to-polygon adapter; all coverage/connection/ordering is native ETHZ."""
import json
import math
import os
from pathlib import Path
import resource
import signal
import subprocess
import time
import numpy as np
from coverage_benchmark.models import PlanningResult
from coverage_benchmark.raster_geometry import polygonize, rasterize
from coverage_benchmark.adapters.f2c_safe import reachable_mask
from coverage_benchmark.provenance import array_fingerprint
from coverage_benchmark.adapters.ethz_build import build, environment, sha

RUN_TIMEOUT_SECONDS=180
ADDRESS_SPACE_BYTES=3*1024**3
BOUNDARY_INSET_M=1e-6


def _inset_orthogonal_ring(coordinates,outer):
    """Shift axis-aligned pixel edges inward by 1 micrometre, preserving corners.

    This prevents half-open raster boundary ambiguities when the native
    visibility graph legitimately travels along polygon walls. No output path
    is nudged or repaired. Free space is left of both CCW outer and CW hole.
    """
    ring=np.asarray(coordinates[:-1],float)
    signed=np.sum(ring[:,0]*np.roll(ring[:,1],-1)-np.roll(ring[:,0],-1)*ring[:,1])
    if (signed>0)!=outer:ring=ring[::-1]
    edges=np.roll(ring,-1,axis=0)-ring
    if not np.all(np.logical_xor(np.abs(edges[:,0])<1e-10,np.abs(edges[:,1])<1e-10)):
        raise ValueError('Expected axis-aligned raster polygon edges')
    length=np.linalg.norm(edges,axis=1)
    left=np.column_stack((-edges[:,1],edges[:,0]))/length[:,None]
    moved=ring+BOUNDARY_INSET_M*(left+np.roll(left,1,axis=0))
    return np.vstack((moved,moved[0])).tolist()


def prepare_input(request):
    if not math.isfinite(request.tool_width) or request.tool_width<=0:
        raise ValueError('Invalid tool width')
    if not math.isfinite(request.resolution) or request.resolution<=0:
        raise ValueError('Invalid resolution')
    if not np.isfinite(request.start).all():raise ValueError('Invalid start')
    reachable,details=reachable_mask(request)
    polygons=polygonize(reachable,request.resolution)
    if len(polygons)!=1 or not polygons[0]['valid']:
        raise ValueError('Start component must form one valid polygon with holes')
    if not np.array_equal(rasterize(polygons,reachable.shape,request.resolution),reachable):
        raise ValueError('Exact polygon raster roundtrip mismatch')
    polygon=dict(polygons[0])
    polygon['geometry']=dict(type='Polygon',coordinates=[_inset_orthogonal_ring(r,i==0)
                                      for i,r in enumerate(polygons[0]['geometry']['coordinates'])])
    # Re-evaluate geometry metadata after the conservative microscopic offset.
    from coverage_benchmark.raster_geometry import from_json, valid, area, geometry_close
    geom=from_json(json.dumps(polygon['geometry']).encode())
    try:polygon.update(valid=bool(valid(geom)),area_m2=area(geom))
    finally:geometry_close(geom)
    if not polygon['valid']:raise ValueError('Inset polygon is invalid')
    if not np.array_equal(rasterize([polygon],reachable.shape,request.resolution),reachable):
        raise ValueError('Boundary inset changed raster pixel membership')
    config=dict(polygon=polygon,start=list(request.start),goal=list(request.start),
                sweep_spacing_m=request.tool_width,decomposition='BCD',cost='Euclidean distance',
                wall_distance_m=0.,offset_polygons=False,sweep_single_direction=False)
    details.update(boundary_inset_m=BOUNDARY_INSET_M,geometry_policy='exact pixel boundary + conservative microscopic axis-aligned inset',
                   input_vertices=sum(len(r)-1 for r in polygon['geometry']['coordinates']),
                   input_holes=len(polygon['geometry']['coordinates'])-1,
                   raster_sha256={name:array_fingerprint(value) for name,value in
                                   (('target',request.free),('navigable',request.navigable),('reachable',reachable))},
                   start_goal_policy='closed route returning to the common start; native approach and return included')
    return config,details


def _limits():
    resource.setrlimit(resource.RLIMIT_AS,(ADDRESS_SPACE_BYTES,ADDRESS_SPACE_BYTES))
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))


def run_native(command,env,log_path,timeout):
    """Wait for the real process, killing/reaping its session on any interruption."""
    with Path(log_path).open('w') as log:
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,
                                 env=env,start_new_session=True,preexec_fn=_limits)
        try:
            process.wait(timeout=timeout)
        except BaseException:
            if process.poll() is None:
                try:os.killpg(process.pid,signal.SIGKILL)
                except ProcessLookupError:pass
            process.wait()
            raise
        return process.returncode


def plan(request,work_dir):
    details=dict(native_invoked=False,path_policy='complete native route, no repair or filtering',
                 timeout_seconds=RUN_TIMEOUT_SECONDS,address_space_limit_bytes=ADDRESS_SPACE_BYTES)
    try:
        config,scope=prepare_input(request);details.update(scope);details['native_input']=config
        executable,build_info=build(request.source_root);details['build']=build_info
        work_dir=Path(work_dir).resolve();work_dir.mkdir(parents=True,exist_ok=True)
        (work_dir/'ethz_input.json').write_text(json.dumps(config,separators=(',',':'))+'\n')
        rings=config['polygon']['geometry']['coordinates']
        lines=['ETHZ1 '+' '.join(map(str,[request.tool_width,*request.start,*request.start,len(rings)]))]
        for ring in rings:
            lines.append(str(len(ring)-1))
            lines.extend(f'{x:.17g} {y:.17g}' for x,y in ring[:-1])
        input_file=work_dir/'ethz_input.txt';output_file=work_dir/'ethz_output.txt'
        input_file.write_text('\n'.join(lines)+'\n');output_file.unlink(missing_ok=True)
        command=[str(executable),str(input_file),str(output_file)]
        details.update(native_invoked=True,native_command=command,input_sha256=sha(input_file))
        started=time.monotonic()
        try:
            returncode=run_native(command,environment(executable.parent.parent),work_dir/'ethz_native.log',RUN_TIMEOUT_SECONDS)
            details['native_returncode']=returncode
        except subprocess.TimeoutExpired:
            details.update(timed_out=True,native_returncode=-signal.SIGKILL)
            raise
        finally:
            details['native_elapsed_seconds']=time.monotonic()-started
            details['native_log_sha256']=sha(work_dir/'ethz_native.log')
        if returncode:
            tail=(work_dir/'ethz_native.log').read_text(errors='replace')[-2000:]
            raise RuntimeError(f'Native ETHZ exited {returncode}: {tail}')
        lines=output_file.read_text().splitlines();marker,cells,count=lines[0].split()
        path=[list(map(float,line.split())) for line in lines[1:]]
        if marker!='ETHZ1' or len(path)!=int(count) or len(path)<2 or np.asarray(path).shape!=(len(path),2) or not np.isfinite(path).all():
            raise ValueError('Invalid native output')
        if not np.allclose(path[0],request.start,rtol=0,atol=1e-8) or not np.allclose(path[-1],request.start,rtol=0,atol=1e-8):
            raise ValueError('Native route does not respect common start/goal')
        details.update(native=dict(solver='GK-MA',decomposition_cells=int(cells),point_count=int(count)),
                       native_output_sha256=sha(output_file),native_input=config)
        return PlanningResult(path,True,'Native ETHZ BCD + sweep graph + visibility + GK-MA closed route; independently audit geometry',details)
    except Exception as error:
        details['error']=f'{type(error).__name__}: {error}'
        return PlanningResult(success=False,message=details['error'],details=details)
