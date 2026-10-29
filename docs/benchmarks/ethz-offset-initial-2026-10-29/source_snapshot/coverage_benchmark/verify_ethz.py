"""Replay all 15 metrics and verify ETHZ native evidence without calling planners."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import numpy as np
from PIL import Image
from coverage_benchmark.adapters.ethz import prepare_input
from coverage_benchmark.compare_saved import load_sources,THREE
from coverage_benchmark.ethz_compare import DEFAULT_OUTPUT
from coverage_benchmark.map_io import load_map
from coverage_benchmark.metrics import evaluate_path,sweep_mask
from coverage_benchmark.models import PlanningInput
from coverage_benchmark.provenance import file_sha256


def verify(root,runtime,output):
    manifest=json.loads((output/'manifest.json').read_text());config=manifest['parameters']
    if not manifest['completed']:raise ValueError('Incomplete ETHZ run')
    for name,digest in manifest['implementation_sha256'].items():
        if file_sha256(root/name)!=digest:raise ValueError('Implementation fingerprint mismatch: '+name)
    for name,digest in manifest['source_sha256'].items():
        if file_sha256(root/name)!=digest:raise ValueError('Historical evidence fingerprint mismatch: '+name)
    for name,digest in manifest['artifacts_sha256'].items():
        if file_sha256(output/name)!=digest:raise ValueError('Output fingerprint mismatch: '+name)
    scenes,old,_=load_sources(root)
    source_manifest=json.loads((root/THREE/'manifest.json').read_text())
    assert manifest['maps']==source_manifest['maps'] and config==source_manifest['parameters']
    csv_rows=list(csv.DictReader((output/'summary.csv').open()));assert len(csv_rows)==15
    checks=[]
    for scene in scenes:
        image=runtime/'data/maps/imported'/scene/(scene+'.png')
        data=load_map(image,config['resolution'],config['clearance']);assert data.metadata==manifest['maps'][scene]
        ethz=json.loads((output/(scene+'_ethz.json')).read_text());assert ethz['scene']==scene and ethz['algorithm']=='ethz'
        request=PlanningInput(scene,data.target,data.navigable,data.resolution,data.start,config['tool_width'],config['clearance'],root,runtime)
        native_input,scope=prepare_input(request)
        assert native_input==json.loads((output/(scene+'_ethz_input.json')).read_text())
        assert native_input==ethz['details']['native_input']
        assert scope['raster_sha256']==ethz['details']['raster_sha256']
        build=ethz['details']['build'];base=root/'coverage_benchmark/.build/ethz'
        for name,digest in build['identity']['build_source_sha256'].items():assert file_sha256(root/name)==digest
        for name,digest in build['artifacts_sha256'].items():assert file_sha256(base/name)==digest
        for name,digest in build['runtime_sha256'].items():assert file_sha256(base/name)==digest
        upstream=root/'third_party/polygon_coverage_planning'
        assert subprocess.check_output(['git','-C',str(upstream),'rev-parse','HEAD'],text=True).strip()==build['identity']['upstream_commit']
        assert not subprocess.check_output(['git','-C',str(upstream),'status','--porcelain']).strip()
        assert file_sha256(output/(scene+'_ethz_input.txt'))==ethz['details']['input_sha256']
        assert file_sha256(output/(scene+'_ethz_native.log'))==ethz['details']['native_log_sha256']
        if ethz['planner_success']:
            lines=(output/(scene+'_ethz_output.txt')).read_text().splitlines()
            marker,cells,count=lines[0].split()
            assert marker=='ETHZ1' and int(cells)==ethz['details']['native']['decomposition_cells']
            raw=[list(map(float,line.split())) for line in lines[1:]]
            assert int(count)==len(raw) and raw==ethz['path']
            assert np.allclose(raw[0],data.start,rtol=0,atol=1e-8) and np.allclose(raw[-1],data.start,rtol=0,atol=1e-8)
        else:assert ethz['path']==[] and ethz['details']['native_invoked'] and ethz['details']['error']
        for record in [r for r in old if r['scene']==scene]+[ethz]:
            actual=evaluate_path(record['path'],data.free,data.navigable,data.resolution,config['tool_width'],record['planner_success'])
            assert actual==record['metrics']
            row=next(x for x in csv_rows if x['scene']==scene and x['algorithm']==record['algorithm'])
            assert float(row['coverage_percent'])==actual['coverage_percent'] and row['path_valid']==str(actual['path_valid'])
            assert (float(row['path_length_m']) if row['path_length_m'] else None)==actual['path_length_m']
            xy=np.asarray(record['path']);witnesses=0
            if xy.size:
                assert math.isclose(sum(math.dist(a,b) for a,b in zip(xy,xy[1:])),actual['path_length_m'],abs_tol=1e-7)
                swept=sweep_mask(xy,data.free.shape,data.resolution,config['tool_width'])
                rng=np.random.default_rng(20261029)
                indices=rng.choice(np.flatnonzero(np.flipud(data.free)),size=256,replace=False)
                a,b=xy[:-1],xy[1:];v=b-a;vv=np.einsum('ij,ij->i',v,v)
                for index in indices:
                    rr,cc=np.unravel_index(index,data.free.shape)
                    p=np.array([(cc+.5)*data.resolution,(rr+.5)*data.resolution])
                    t=np.divide(np.einsum('ij,ij->i',p-a,v),vv,out=np.zeros_like(vv),where=vv>0)
                    delta=p-(a+np.clip(t,0,1)[:,None]*v)
                    inside=bool((np.einsum('ij,ij->i',delta,delta)<=config['tool_width']**2/4+1e-12).any())
                    assert inside==bool(swept.flat[index]);witnesses+=1
            checks.append(dict(scene=scene,algorithm=record['algorithm'],metric_recomputation=True,independent_capsule_witnesses=witnesses))
        with Image.open(output/(scene+'_comparison.png')) as im:im.verify()
    result=dict(passed=True,cases=15,checks=checks,native_output_replayed=True,source_and_input_fingerprints_match=True,motion_executed=False)
    (output/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    root=Path(__file__).resolve().parents[1];parser=argparse.ArgumentParser()
    parser.add_argument('--runtime-root',type=Path,default=root)
    parser.add_argument('--output',type=Path,default=root/DEFAULT_OUTPUT)
    args=parser.parse_args();verify(root,args.runtime_root.resolve(),args.output.resolve())
    print('Verified 15 cases, ETHZ raw native output, fingerprints and independent geometry witnesses.')
