"""Fresh sequential calls to all six current planners; never import saved paths."""
from datetime import datetime, timezone
import importlib
import json
from pathlib import Path
import subprocess
import sys
import time
import uuid
import yaml

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from coverage_benchmark.models import PlanningInput
from coverage_benchmark.map_io import load_map
from coverage_benchmark.metrics import evaluate_path
from coverage_benchmark.provenance import file_sha256, native_fingerprints
from coverage_benchmark.runner import versions

OUT=Path(__file__).resolve().parent
ALGORITHMS={'leader':'leader','ipa_bcd':'ipa_bcd','stc':'stc','f2c_safe':'f2c_safe','ethz':'ethz','ipa_safe':'ipa'}
SUBMODULES=('opennav_coverage','Fields2Cover','python_robotics','polygon_coverage_planning','ipa_coverage_planning')


def save(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n')


def sources():
    paths=[p for p in (ROOT/'coverage_benchmark').rglob('*') if p.is_file() and p.suffix in ('.py','.cpp','.hpp','.h','.yaml','.in') and not any(part in ('.build','.work','__pycache__','tests') for part in p.parts)]
    return {str(p.relative_to(ROOT)):file_sha256(p) for p in paths}


def main():
    if (OUT/'manifest.json').exists():raise RuntimeError('Use a fresh output directory; this run already has a manifest')
    config=yaml.safe_load((ROOT/'coverage_benchmark/configs/three_maps.yaml').read_text())
    manifest=dict(run_id=uuid.uuid4().hex,started_utc=datetime.now(timezone.utc).isoformat(),completed=False,planning_completed=False,
        parameters=config,algorithms=list(ALGORITHMS),maps={},records=[],implementation_sha256=sources(),motion_executed=False,
        historical_paths_reused=False,versions=versions(ROOT,ROOT),f2c_dependencies=native_fingerprints(ROOT),submodules={})
    for name in SUBMODULES:
        base=ROOT/'third_party'/name
        if subprocess.check_output(['git','-C',str(base),'status','--porcelain']):raise RuntimeError('Dirty upstream '+name)
        manifest['submodules'][name]=subprocess.check_output(['git','-C',str(base),'rev-parse','HEAD'],text=True).strip()
    save(OUT/'manifest.json',manifest)
    for scene in config['scenes']:
        source=ROOT/'data/maps/imported'/scene/(scene+'.png')
        m=load_map(source,config['resolution'],config['clearance']);manifest['maps'][scene]=m.metadata
        request=PlanningInput(scene,m.target,m.navigable,m.resolution,m.start,config['tool_width'],config['clearance'],ROOT,ROOT)
        for name,module in ALGORITHMS.items():
            print('START',scene,name,datetime.now(timezone.utc).isoformat(),flush=True)
            work=OUT/'evidence'/scene/name;work.mkdir(parents=True)
            started=time.monotonic()
            result=importlib.import_module('coverage_benchmark.adapters.'+module).plan(request,work)
            elapsed=time.monotonic()-started
            metrics=evaluate_path(result.path,m.free,m.navigable,m.resolution,config['tool_width'],result.success)
            rec=dict(run_id=manifest['run_id'],scene=scene,algorithm=name,input=m.metadata,planner_success=result.success,message=result.message,
                path=result.path,details=result.details,metrics=metrics,adapter_elapsed_seconds=elapsed,fresh_native_call=True)
            filename=scene+'_'+name+'.json';save(OUT/filename,rec)
            manifest['records'].append(filename);save(OUT/'manifest.json',manifest)
            print('DONE',scene,name,'success='+str(result.success),'seconds='+str(round(elapsed,2)),json.dumps(metrics),flush=True)
    if sources()!=manifest['implementation_sha256']:raise RuntimeError('Implementation changed during run')
    manifest.update(planning_completed=True,planning_finished_utc=datetime.now(timezone.utc).isoformat())
    save(OUT/'manifest.json',manifest)
    print('ALL_18_PLANNING_CALLS_FINISHED',flush=True)


if __name__=='__main__':main()
