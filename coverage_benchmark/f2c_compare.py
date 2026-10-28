"""Separate Fields2Cover native / raster connector experiment; old results untouched."""
import argparse
import csv
import json
import subprocess
from datetime import datetime, timezone
import uuid
from pathlib import Path
import numpy as np
import yaml
from coverage_benchmark.models import PlanningInput
from coverage_benchmark.map_io import load_map
from coverage_benchmark.metrics import evaluate_path
from coverage_benchmark.runner import plot_scene
from coverage_benchmark.adapters.f2c_safe import plan
from coverage_benchmark.provenance import implementation_fingerprints, file_sha256


def run(root,runtime,output):
    output.mkdir(parents=True,exist_ok=True)
    config=yaml.safe_load((root/'coverage_benchmark/configs/three_maps.yaml').read_text())
    manifest=dict(run_id=uuid.uuid4().hex,started_utc=datetime.now(timezone.utc).isoformat(),
                  completed=False,completed_scenes=[],parameters=config,maps={},motion_executed=False,
                  implementation_sha256=implementation_fingerprints(root),
                  code_base_commit=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip(),
                  artifacts_sha256={},
                  new_variant='Fields2Cover + start-component safe swaths + raster A*; stop-and-turn geometry')
    rows=[]
    def checkpoint():
        with (output/'summary.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=['scene','algorithm','coverage_percent','path_length_m','path_valid'],lineterminator='\n')
            writer.writeheader();writer.writerows(rows)
        manifest['artifacts_sha256']['summary.csv']=file_sha256(output/'summary.csv')
        (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    checkpoint()
    for scene in config['scenes']:
        print('Planning native and safe F2C: '+scene,flush=True)
        image=runtime/'data/maps/imported'/scene/(scene+'.png')
        data=load_map(image,config['resolution'],config['clearance'])
        manifest['maps'][scene]=data.metadata
        request=PlanningInput(scene,data.target,data.navigable,data.resolution,data.start,
                              config['tool_width'],config['clearance'],root,runtime)
        result=plan(request,root/'coverage_benchmark/.work/f2c_obstacle'/scene)
        baseline=result.details.pop('native_baseline',{})
        candidate=result.details.pop('native_candidate',{})
        baseline_file=scene+'_native.json'
        candidate_file=scene+'_candidate.json'
        (output/candidate_file).write_text(json.dumps(candidate,separators=(',',':'))+'\n')
        manifest['artifacts_sha256'][candidate_file]=file_sha256(output/candidate_file)
        result.details.update(native_baseline_file=baseline_file,native_candidate_file=candidate_file)
        records=[]
        for name,path,success,message,details in (
            ('f2c_native',baseline.get('path',[]),baseline.get('success',False),
             'Unmodified native Fields2Cover, original target + 1m headland',
             {key:value for key,value in baseline.items() if key!='path'}),
            ('f2c_safe',result.path,result.success,result.message,result.details)):
            metric=evaluate_path(path,data.free,data.navigable,data.resolution,config['tool_width'],success)
            record=dict(run_id=manifest['run_id'],scene=scene,algorithm=name,path=path,planner_success=success,message=message,
                        details=details,metrics=metric)
            destination=output/(baseline_file if name=='f2c_native' else scene+'_safe.json')
            destination.write_text(json.dumps(record,separators=(',',':'),allow_nan=False)+'\n')
            manifest['artifacts_sha256'][destination.name]=file_sha256(destination)
            records.append(record)
            rows.append(dict(scene=scene,algorithm=name,**{key:metric[key] for key in
                             ('coverage_percent','path_length_m','path_valid')}))
            print(json.dumps(dict(scene=scene,algorithm=name,**metric)),flush=True)
        plot_scene(image,records,data,output/(scene+'_comparison.png'),config['tool_width'])
        manifest['artifacts_sha256'][scene+'_comparison.png']=file_sha256(output/(scene+'_comparison.png'))
        manifest['completed_scenes'].append(scene)
        checkpoint()
    manifest['completed']=True
    manifest['finished_utc']=datetime.now(timezone.utc).isoformat()
    checkpoint()
    return rows


if __name__=='__main__':
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser()
    parser.add_argument('--runtime-root',type=Path,default=root)
    parser.add_argument('--output',type=Path,default=root/'docs/benchmarks/f2c-obstacle-2026-10-28')
    args=parser.parse_args()
    run(root,args.runtime_root.resolve(),args.output.resolve())
