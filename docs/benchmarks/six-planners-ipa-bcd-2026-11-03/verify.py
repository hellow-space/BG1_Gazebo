"""Verify this fresh run without reading or changing any historical report."""
import csv
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from coverage_benchmark.map_io import load_map
from coverage_benchmark.models import PlanningInput
from coverage_benchmark.metrics import evaluate_path
from coverage_benchmark.provenance import file_sha256,native_fingerprints
from coverage_benchmark.adapters.f2c_safe import reachable_mask
from coverage_benchmark.ipa_audit import path_box_audit
from coverage_benchmark.ipa_connection import strict_path_valid
from coverage_benchmark.verify_ipa import independent_witnesses,verify_replay
from coverage_benchmark.raster_geometry import rasterize
from coverage_benchmark.obstacle_connector import assemble_path,usable_native_turns
from coverage_benchmark.hybrid.planner import candidates,_plan
from coverage_benchmark.hybrid.build import verify_build
from coverage_benchmark.adapters.ipa_build import cache_valid
from rerun import sources,versions
OUT=Path(__file__).resolve().parent


def check(value,message):
    if not value:raise ValueError(message)


def native_replay(rec,m,request,work):
    d=rec['details'];name=rec['algorithm'];r=m.resolution
    reachable,_=reachable_mask(request);f=np.flipud(reachable)
    if name=='leader':
        raw=json.loads((work/'leader_output.json').read_text())
        check(raw['path']==rec['path'] and raw['success']==rec['planner_success'],'Leader response differs')
        check(np.array_equal(np.load(work/'leader_input.npz')['target'],m.target),'Leader input differs')
        return 'ROS planning response replay'
    if name=='stc':
        from coverage_benchmark.adapters.stc import plan
        replay=plan(request,work)
        check(replay.path==rec['path'] and replay.success==rec['planner_success'] and replay.message==rec['message'],'STC deterministic replay differs')
        return 'unchanged upstream deterministic replay'
    if name=='ipa_bcd':
        evidence=d['decomposition']['evidence']
        for name,digest in evidence['files_sha256'].items():check(file_sha256(Path(name))==digest,'BCD evidence changed')
        check(verify_build(d['decomposition']['build'],ROOT),'BCD build changed')
        check(np.array_equal(np.asarray(Image.open(evidence['request']['allowed']))==255,f),'BCD input F differs')
        raw=json.loads(next(Path(p) for p in evidence['files_sha256'] if Path(p).name=='output.json').read_text())
        check(raw['cells']==d['decomposition']['cells'],'BCD native cells differ')
        if rec['planner_success']:
            pieces=[candidates(cell,f,r)[0] for cell in raw['cells']]
            replay=_plan(request,f,raw['cells'],pieces,'ipa_bcd',work)
            check(replay.path==rec['path'] and replay.details['schedule']==d['schedule'] and replay.details['sections']==d['sections'],'BCD cell scheduling/route replay differs')
        else:check(not rec['path'],'Failed BCD has path')
        return 'native cell sweeps, schedule and every connection replay'
    if name=='ipa_safe':
        from coverage_benchmark.adapters.ipa import prepare_input
        config,scope=prepare_input(request)
        check(config==d['native_input']==json.loads((work/'ipa_input.json').read_text()),'Local native input differs')
        check(scope['raster_sha256']==d['raster_sha256'],'Local raster identity differs')
        check(np.array_equal(np.asarray(Image.open(work/'ipa_input.png')),np.flipud(reachable).astype('uint8')*255),'Local raster differs')
        check(cache_valid(d['build'],d['build']['identity']),'Local native build/runtime changed')
        check(file_sha256(work/'ipa_native.log')==d['native_log_sha256'],'Local log changed')
        if d.get('native_returncode')==0:
            lines=(work/'ipa_output.txt').read_text().splitlines();tag,count=lines[0].split()
            points=[list(map(float,line.split())) for line in lines[1:]]
            check(tag=='IPA1' and len(points)==int(count) and points==d['native_path'],'Local native target mismatch')
        if rec['planner_success']:verify_replay(rec,list(m.start),f,r)
        else:check(not rec['path'] and d.get('error'),'Local failure lacks error')
        return 'complete native Local targets and safe connection replay'
    if name=='ethz':
        from coverage_benchmark.adapters.ethz import prepare_input
        config,scope=prepare_input(request)
        check(config==d['native_input']==json.loads((work/'ethz_input.json').read_text()),'ETHZ input differs')
        check(scope['raster_sha256']==d['raster_sha256'],'ETHZ mask differs')
        build=d['build'];base=ROOT/'coverage_benchmark/.build/ethz'
        for p,digest in build['identity']['build_source_sha256'].items():check(file_sha256(ROOT/p)==digest,'ETHZ build source differs')
        for group in ('artifacts_sha256','runtime_sha256'):
            for p,digest in build[group].items():check(file_sha256(base/p)==digest,'ETHZ binary/runtime differs')
        if rec['planner_success']:
            lines=(work/'ethz_output.txt').read_text().splitlines();tag,cells,count=lines[0].split()
            points=[list(map(float,line.split())) for line in lines[1:]]
            check(tag=='ETHZ1' and len(points)==int(count) and points==rec['path'],'ETHZ raw route differs')
            check(np.allclose(points[0],m.start,rtol=0,atol=1e-8) and np.allclose(points[-1],m.start,rtol=0,atol=1e-8),'ETHZ common start/goal differs')
        else:check(not rec['path'] and d.get('error'),'ETHZ failure lacks error')
        return 'native BCD/GTSP raw route and input polygon replay'
    if name=='f2c_safe':
        check(d['native_fingerprints']==native_fingerprints(ROOT),'F2C binary/runtime differs')
        for key,mask in (('baseline',m.target),('candidate',reachable)):
            check(np.array_equal(rasterize([d['native_input'][key]['polygon']],m.free.shape,r),mask),'F2C polygon differs')
        if rec['planner_success']:
            c=d['native_candidate'];turns=usable_native_turns(c['swaths'],c['path_swaths'],c['turns'])
            path,trace=assemble_path(c['swaths'],turns,f,r,m.start)
            check(path==rec['path'] and all(d[k]==v for k,v in trace.items()),'F2C native swath/route replay differs')
            check(path[0]==list(m.start),'F2C common start missing')
        else:check(not rec['path'],'Failed F2C has path')
        return 'native ordered swaths and every connection replay'
    raise ValueError('Unknown algorithm')


def verify():
    manifest=json.loads((OUT/'manifest.json').read_text());p=manifest['parameters']
    check(manifest['completed'] and manifest['planning_completed'],'Run incomplete')
    check(manifest['implementation_sha256']==sources(),'Planner source changed')
    check(manifest['versions']==versions(ROOT,ROOT),'Runtime/Leader build changed')
    check(manifest['f2c_dependencies']==native_fingerprints(ROOT),'F2C dependency changed')
    for name,commit in manifest['submodules'].items():
        base=ROOT/'third_party'/name
        check(subprocess.check_output(['git','-C',str(base),'rev-parse','HEAD'],text=True).strip()==commit,'Upstream commit changed')
        check(not subprocess.check_output(['git','-C',str(base),'status','--porcelain']),'Dirty upstream')
    for name,digest in manifest['artifacts_sha256'].items():check(file_sha256(OUT/name)==digest,'Artifact changed: '+name)
    rows=list(csv.DictReader((OUT/'summary.csv').open()));checks=[]
    check(len(rows)==18 and len(manifest['records'])==18,'Expected 18 fresh cases')
    for scene in p['scenes']:
        print('VERIFY',scene,flush=True)
        m=load_map(ROOT/'data/maps/imported'/scene/(scene+'.png'),p['resolution'],p['clearance'])
        check(m.metadata==manifest['maps'][scene],'Map changed')
        req=PlanningInput(scene,m.target,m.navigable,m.resolution,m.start,p['tool_width'],p['clearance'],ROOT,ROOT)
        for name in manifest['algorithms']:
            rec=json.loads((OUT/(scene+'_'+name+'.json')).read_text())
            check(rec['scene']==scene and rec['algorithm']==name and rec['run_id']==manifest['run_id'] and rec['fresh_native_call'],'Identity differs')
            check(rec['input']==m.metadata,'Per-record input differs')
            metric=evaluate_path(rec['path'],m.free,m.navigable,m.resolution,p['tool_width'],rec['planner_success'])
            check(metric==rec['metrics'],'Metric differs')
            row=next(row for row in rows if row['scene']==scene and row['algorithm']==name)
            check(float(row['coverage_percent'])==metric['coverage_percent'] and row['path_valid']==str(metric['path_valid']) and (float(row['path_length_m']) if row['path_length_m'] else None)==metric['path_length_m'],'CSV differs')
            check(rec['path'] or not rec['planner_success'],'Success without path')
            provenance=native_replay(rec,m,req,OUT/'evidence'/scene/name)
            witnesses=independent_witnesses(rec,m,p['tool_width'])
            strict=box=None
            if name in ('ipa_bcd','f2c_safe','ethz','ipa_safe') and rec['planner_success']:
                reachable,_=reachable_mask(req);f=np.flipud(reachable)
                strict=strict_path_valid(rec['path'],f,m.resolution);box=path_box_audit(rec['path'],f,m.resolution)
                check(metric['path_valid'] and strict and box,'Audited planner returned unsafe path')
            checks.append(dict(scene=scene,algorithm=name,metric_recomputation=True,provenance=provenance,independent_coverage_witnesses=witnesses,strict_audit=strict,box_audit=box))
            print('  PASS',name,flush=True)
        with Image.open(OUT/(scene+'_comparison.png')) as im:im.verify()
    result=dict(passed=True,run_id=manifest['run_id'],cases=len(checks),independent_coverage_witnesses=sum(c['independent_coverage_witnesses'] for c in checks),checks=checks,motion_executed=False,historical_paths_reused=False)
    (OUT/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return result


if __name__=='__main__':
    (OUT/'verification.json').unlink(missing_ok=True)
    try:print(json.dumps(verify(),ensure_ascii=False),flush=True)
    except BaseException:
        (OUT/'verification.json').unlink(missing_ok=True)
        raise
