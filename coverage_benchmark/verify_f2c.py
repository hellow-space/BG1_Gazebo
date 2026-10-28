"""Verify hashes, native inputs, source trace and independent geometric witnesses."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from PIL import Image
from coverage_benchmark.map_io import load_map
from coverage_benchmark.models import PlanningInput
from coverage_benchmark.metrics import evaluate_path, sweep_mask
from coverage_benchmark.obstacle_connector import assemble_path, usable_native_turns
from coverage_benchmark.adapters.f2c_safe import reachable_mask
from coverage_benchmark.provenance import file_sha256, array_fingerprint, implementation_fingerprints, native_fingerprints
from coverage_benchmark.raster_geometry import rasterize


def verify(root,runtime,output):
    manifest=json.loads((output/'manifest.json').read_text())
    assert manifest['completed'] and manifest['completed_scenes']==manifest['parameters']['scenes']
    assert implementation_fingerprints(root)==manifest['implementation_sha256'],'Custom implementation changed since run'
    for name,digest in manifest['artifacts_sha256'].items():
        assert file_sha256(output/name)==digest,('Changed artifact',name)
    rows=list(csv.DictReader((output/'summary.csv').open()))
    assert len(rows)==6
    checks=[]
    config=manifest['parameters']
    native_hashes=native_fingerprints(runtime)
    for scene in config['scenes']:
        image=runtime/'data/maps/imported'/scene/(scene+'.png')
        data=load_map(image,config['resolution'],config['clearance'])
        assert manifest['maps'][scene]==data.metadata
        request=PlanningInput(scene,data.target,data.navigable,data.resolution,data.start,
                              config['tool_width'],config['clearance'],root,runtime)
        safe=json.loads((output/(scene+'_safe.json')).read_text())
        native=json.loads((output/(scene+'_native.json')).read_text())
        candidate=json.loads((output/(scene+'_candidate.json')).read_text())
        details=safe['details']
        reachable,scope=reachable_mask(request)
        for key,value in scope.items():assert details[key]==value
        assert details['raster_sha256']=={name:array_fingerprint(mask) for name,mask in
                                         (('target',data.target),('navigable',data.navigable),('reachable',reachable))}
        assert details['native_fingerprints']==native_hashes
        assert details['native_address_space_limit_bytes']==2*1024**3
        if details['native_server_exit_code']!=0:
            assert candidate.get('success') and not candidate.get('native_path_success')
            assert candidate.get('native_path_error') and details.get('native_worker_error')
        for name,mask in (('baseline',data.target),('candidate',reachable)):
            polygon=details['native_input'][name]['polygon']
            assert polygon['valid'] and np.array_equal(rasterize([polygon],mask.shape,data.resolution),mask)
        if safe['planner_success']:
            assert candidate['ordered_swaths_source']=='native action generate_path=False'
            turns=usable_native_turns(candidate['swaths'],candidate['path_swaths'],candidate['turns'])
            replay,trace=assemble_path(candidate['swaths'],turns,np.flipud(reachable),data.resolution,data.start)
            assert replay==safe['path'],'Native swaths -> safe route replay mismatch'
            for key,value in trace.items():assert details[key]==value,('Trace mismatch',key)
            cursor=0
            for section in details['path_sections']:
                assert section['begin_vertex']==cursor and section['end_vertex']>cursor
                cursor=section['end_vertex']
                assert section['source_swath_index']<len(candidate['swaths'])
            assert cursor==len(safe['path'])-1 and safe['path'][0]==list(data.start)
        for record in (native,safe):
            assert record['run_id']==manifest['run_id']
            actual=evaluate_path(record['path'],data.free,data.navigable,data.resolution,
                                 config['tool_width'],record['planner_success'])
            assert actual==record['metrics']
            row=next(row for row in rows if row['scene']==scene and row['algorithm']==record['algorithm'])
            assert float(row['coverage_percent'])==actual['coverage_percent']
            assert (float(row['path_length_m']) if row['path_length_m'] else None)==actual['path_length_m']
            assert row['path_valid']==str(actual['path_valid'])
            if record['algorithm']=='f2c_safe' and record['planner_success']:
                assert actual['path_valid'] and actual['diagnostics']['invalid_segments']==0
            points=np.asarray(record['path'],dtype=float)
            witnesses=0
            if len(points)>1 and record['planner_success']:
                assert math.isclose(sum(math.dist(a,b) for a,b in zip(points,points[1:])),actual['path_length_m'],abs_tol=1e-7)
                swept=sweep_mask(points,data.free.shape,data.resolution,config['tool_width'])
                indices=np.random.default_rng(20261028).choice(np.flatnonzero(np.flipud(data.free)),size=256,replace=False)
                a,b=points[:-1],points[1:];v=b-a;vv=np.einsum('ij,ij->i',v,v)
                for index in indices:
                    y,x=np.unravel_index(index,data.free.shape)
                    point=np.array([(x+.5)*data.resolution,(y+.5)*data.resolution])
                    t=np.divide(np.einsum('ij,ij->i',point-a,v),vv,out=np.zeros_like(vv),where=vv>0)
                    delta=point-(a+np.clip(t,0,1)[:,None]*v)
                    inside=bool((np.einsum('ij,ij->i',delta,delta)<=config['tool_width']**2/4+1e-12).any())
                    assert inside==bool(swept.flat[index]);witnesses+=1
            checks.append(dict(scene=scene,algorithm=record['algorithm'],metrics_recomputed=True,
                               independent_capsule_witnesses=witnesses,
                               path_sha256=hashlib.sha256(points.astype('<f8').tobytes()).hexdigest()))
        with Image.open(output/(scene+'_comparison.png')) as picture:picture.verify()
    result=dict(passed=True,run_id=manifest['run_id'],source_and_artifact_hashes_match=True,
                native_input_roundtrip=True,complete_trace_replayed=True,checks=checks,
                verifier_sha256=file_sha256(Path(__file__)),motion_executed=False)
    (output/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser()
    parser.add_argument('--runtime-root',type=Path,default=root)
    parser.add_argument('--output',type=Path,default=root/'docs/benchmarks/f2c-obstacle-2026-10-28')
    args=parser.parse_args()
    verify(root,args.runtime_root.resolve(),args.output.resolve())
    print('Verified 6 cases, source/artifact hashes, native input masks, per-section route replay and independent sweep witnesses.')
