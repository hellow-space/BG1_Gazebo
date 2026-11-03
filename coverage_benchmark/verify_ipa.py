"""Replay native IPA targets/connections and independently certify 18 saved cases."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import numpy as np
from PIL import Image
from coverage_benchmark.adapters.ipa import prepare_input
from coverage_benchmark.adapters.ipa_build import cache_valid,UPSTREAM_COMMIT,sha
from coverage_benchmark.adapters.f2c_safe import reachable_mask
from coverage_benchmark.ipa_connection import assemble
from coverage_benchmark.ipa_audit import path_box_audit
from coverage_benchmark.ipa_compare import DEFAULT_OUTPUT,load_previous,invalidate_failure
from coverage_benchmark.map_io import load_map
from coverage_benchmark.metrics import evaluate_path,sweep_mask
from coverage_benchmark.models import PlanningInput
from coverage_benchmark.provenance import file_sha256


def require(condition,message):
    if not condition:raise ValueError(message)


def verify_replay(record,start,allowed,resolution):
    expected,connections=assemble(start,record['details']['native_path'],allowed,resolution)
    require(expected==record['path'],'Final path does not replay native targets in order')
    require(connections==record['details']['connections'],'Connection provenance does not replay')
    require(path_box_audit(record['path'],allowed,resolution),'Independent segment/blocked-box intersection detected')


def independent_witnesses(record,data,tool_width):
    points=np.asarray(record['path'],float)
    if not points.size:return 0
    require(math.isclose(sum(math.dist(a,b) for a,b in zip(points,points[1:])),record['metrics']['path_length_m'],abs_tol=1e-7),'Independent length mismatch')
    mask=sweep_mask(points,data.free.shape,data.resolution,tool_width)
    indices=np.random.default_rng(20261030).choice(np.flatnonzero(np.flipud(data.free)),size=256,replace=False)
    a,b=points[:-1],points[1:];v=b-a;vv=np.einsum('ij,ij->i',v,v)
    for index in indices:
        row,col=np.unravel_index(index,data.free.shape);p=np.array([(col+.5)*data.resolution,(row+.5)*data.resolution])
        t=np.divide(np.einsum('ij,ij->i',p-a,v),vv,out=np.zeros_like(vv),where=vv>0)
        d=p-(a+np.clip(t,0,1)[:,None]*v)
        covered=bool((np.einsum('ij,ij->i',d,d)<=tool_width**2/4+1e-12).any())
        require(covered==bool(mask.flat[index]),'Independent capsule witness mismatch')
    return len(indices)


def _verify(root,runtime,output):
    manifest=json.loads((output/'manifest.json').read_text())
    require(manifest.get('completed'),'Incomplete IPA comparison run')
    for group,base in (('implementation_sha256',root),('source_sha256',root),('artifacts_sha256',output)):
        for name,digest in manifest[group].items():require(file_sha256(base/name)==digest,group+' mismatch: '+name)
    scenes,old,sources,common=load_previous(root)
    require(manifest['source_sha256']==sources,'Historical source manifest mismatch')
    require(manifest['maps']==common['maps'] and manifest['parameters']==common['parameters'],'Historical common input mismatch')
    config=manifest['parameters'];csv_rows=list(csv.DictReader((output/'summary.csv').open()))
    require(len(csv_rows)==18,'Expected exactly 18 comparison rows')
    require(len({(r['scene'],r['algorithm']) for r in csv_rows})==18,'Duplicate comparison row')
    checks=[]
    for scene in scenes:
        data=load_map(runtime/'data/maps/imported'/scene/(scene+'.png'),config['resolution'],config['clearance'])
        require(data.metadata==manifest['maps'][scene],'Original map metadata mismatch')
        request=PlanningInput(scene,data.target,data.navigable,data.resolution,data.start,config['tool_width'],config['clearance'],root,runtime)
        record=json.loads((output/(scene+'_ipa_safe.json')).read_text());d=record['details']
        require(record['scene']==scene and record['algorithm']=='ipa_safe' and record['run_id']==manifest['run_id'],'IPA record identity mismatch')
        native_input,scope=prepare_input(request);reachable,_=reachable_mask(request)
        require(d['native_input']==native_input==json.loads((output/(scene+'_ipa_input.json')).read_text()),'Native input configuration mismatch')
        require(d['raster_sha256']==scope['raster_sha256'],'Common raster fingerprint mismatch')
        mask_image=output/(scene+'_ipa_input.png')
        require(np.array_equal(np.asarray(Image.open(mask_image)),np.flipud(reachable).astype('uint8')*255),'Native binary map differs from common start component')
        require(file_sha256(mask_image)==d['native_input_sha256'],'Native input image SHA mismatch')
        build=d['build'];require(build['upstream_commit']==UPSTREAM_COMMIT,'Upstream version mismatch')
        require(cache_valid(build,build['identity']),'Native binary or actual runtime dependency changed')
        for name,digest in build['identity']['source_sha256'].items():require(file_sha256(root/name)==digest,'Native compiled source changed: '+name)
        require(sha(build['identity']['compiler'])==build['identity']['compiler_sha256'],'Compiler changed')
        up=root/'third_party/ipa_coverage_planning'
        require(subprocess.check_output(['git','-C',str(up),'rev-parse','HEAD'],text=True).strip()==UPSTREAM_COMMIT,'Upstream commit changed')
        require(not subprocess.check_output(['git','-C',str(up),'status','--porcelain']).strip(),'Upstream has modifications')
        require(file_sha256(output/(scene+'_ipa_native.log'))==d['native_log_sha256'],'Native log changed')
        if d.get('native_returncode')==0:
            nativefile=output/(scene+'_ipa_output.txt');lines=nativefile.read_text().splitlines();tag,count=lines[0].split()
            raw=[list(map(float,line.split())) for line in lines[1:]]
            require(tag=='IPA1' and int(count)==len(raw) and raw==d['native_path'],'Native complete output replay mismatch')
            if raw:require(file_sha256(nativefile)==d['native_output_sha256'],'Native output fingerprint mismatch')
        require(evaluate_path(d['native_path'],data.free,data.navigable,data.resolution,config['tool_width'],bool(d['native_path']))==d['raw_metrics'],'Raw native diagnostic mismatch')
        if record['planner_success']:
            verify_replay(record,list(data.start),np.flipud(reachable),data.resolution)
            require(record['metrics']['path_valid'],'Successful IPA route is invalid')
        else:require(record['path']==[] and d.get('error'),'Failed IPA route must be empty with an explicit error')
        for r in [r for r in old if r['scene']==scene]+[record]:
            actual=evaluate_path(r['path'],data.free,data.navigable,data.resolution,config['tool_width'],r['planner_success'])
            require(actual==r['metrics'],'Metric recomputation mismatch: '+r['algorithm'])
            row=next(x for x in csv_rows if x['scene']==scene and x['algorithm']==r['algorithm'])
            require(float(row['coverage_percent'])==actual['coverage_percent'] and row['path_valid']==str(actual['path_valid']) and
                    (float(row['path_length_m']) if row['path_length_m'] else None)==actual['path_length_m'],'CSV metric mismatch')
            witnesses=independent_witnesses(r,data,config['tool_width'])
            checks.append(dict(scene=scene,algorithm=r['algorithm'],metrics_recomputed=True,independent_capsule_witnesses=witnesses))
        with Image.open(output/(scene+'_comparison.png')) as image:image.verify()
    result=dict(passed=True,run_id=manifest['run_id'],cases=18,native_targets_and_connections_replayed=True,
                independent_segment_box_audit=True,source_and_input_fingerprints_match=True,checks=checks,motion_executed=False)
    (output/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def verify(root,runtime,output):
    (output/'verification.json').unlink(missing_ok=True)
    try:return _verify(root,runtime,output)
    except BaseException as error:
        invalidate_failure(output,error)
        raise


if __name__=='__main__':
    root=Path(__file__).resolve().parents[1];parser=argparse.ArgumentParser()
    parser.add_argument('--runtime-root',type=Path,default=root);parser.add_argument('--output',type=Path,default=root/DEFAULT_OUTPUT)
    args=parser.parse_args();verify(root,args.runtime_root.resolve(),args.output.resolve())
    print('Verified 18 cases, native IPA target/connection replay and independent segment-box safety audit.')
