"""Recompute persisted metrics and independent geometry witnesses; no planner calls."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import numpy as np
from PIL import Image
from coverage_benchmark.map_io import load_map
from coverage_benchmark.metrics import evaluate_path, sweep_mask
from coverage_benchmark.runner import ALGORITHMS, versions


def verify(source_root, runtime_root, output):
    manifest = json.loads((output/'manifest.json').read_text())
    config = manifest['parameters']
    assert versions(source_root,runtime_root) == manifest['versions']
    for name in ('bcd_planner','python_robotics'):
        assert not subprocess.check_output(['git','-C',str(source_root/'third_party'/name),'status','--porcelain'],text=True).strip()
    rows = list(csv.DictReader((output/'summary.csv').open()))
    assert len(rows)==9
    checks = []
    for scene in config['scenes']:
        path = runtime_root/'data/maps/imported'/scene/(scene+'.png')
        data = load_map(path,config['resolution'],config['clearance'])
        assert data.metadata==manifest['maps'][scene]
        # Companion metadata is the basis of the scale assumption, never the PNG input substitute.
        with path.with_suffix('.pgm').open('rb') as stream:
            assert b'0.05' in b''.join(stream.readline() for _ in range(6))
        for algorithm in ALGORITHMS:
            record = json.loads((output/f'{scene}_{algorithm}.json').read_text())
            assert record['scene']==scene and record['algorithm']==algorithm
            actual = evaluate_path(record['path'],data.free,data.navigable,data.resolution,
                                   config['tool_width'],record['planner_success'])
            assert actual==record['metrics']
            row = next(x for x in rows if x['scene']==scene and x['algorithm']==algorithm)
            assert float(row['coverage_percent'])==actual['coverage_percent']
            assert row['path_valid']==str(actual['path_valid'])
            assert (float(row['path_length_m']) if row['path_length_m'] else None)==actual['path_length_m']
            xy = np.asarray(record['path'])
            count = 0
            if xy.size:
                independent_length = sum(math.dist(a,b) for a,b in zip(xy,xy[1:]))
                assert math.isclose(independent_length,actual['path_length_m'],abs_tol=1e-7)
                # Pointwise full-path distance, independent of bounding-box capsule rasterization.
                swept = sweep_mask(xy,data.free.shape,data.resolution,config['tool_width'])
                rng = np.random.default_rng(20261027)
                indices = rng.choice(np.flatnonzero(np.flipud(data.free)),size=256,replace=False)
                a,b = xy[:-1],xy[1:]
                v = b-a
                vv = np.einsum('ij,ij->i',v,v)
                for index in indices:
                    row_,col = np.unravel_index(index,data.free.shape)
                    point = np.array([(col+.5)*data.resolution,(row_+.5)*data.resolution])
                    t = np.divide(np.einsum('ij,ij->i',point-a,v),vv,out=np.zeros_like(vv),where=vv>0)
                    delta = point-(a+np.clip(t,0,1)[:,None]*v)
                    inside = bool((np.einsum('ij,ij->i',delta,delta)<=config['tool_width']**2/4+1e-12).any())
                    assert inside==bool(swept.flat[index])
                    count += 1
            else:
                assert not record['planner_success'] and actual['path_length_m'] is None
            checks.append(dict(scene=scene,algorithm=algorithm,path_sha256=hashlib.sha256(np.asarray(record['path'],dtype='<f8').tobytes()).hexdigest(),
                               metric_recomputation=True,independent_capsule_witnesses=count))
        with Image.open(output/(scene+'_comparison.png')) as image:
            image.verify()
    result = dict(passed=True,nine_cases=True,source_map_and_version_hashes_match=True,
                  metrics_recomputed=True,checks=checks,motion_executed=False)
    (output/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument('--runtime-root',type=Path,default=root)
    parser.add_argument('--output',type=Path,default=root/'docs/benchmarks/three-planners-2026-10-27')
    args = parser.parse_args()
    verify(root,args.runtime_root.resolve(),args.output.resolve())
    print('Verified 9 cases: input/version hashes, metrics, CSV, images and independent geometry witnesses.')
