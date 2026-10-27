"""Recheck persisted benchmark evidence independently of the ROS processes."""
import ast
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import subprocess
import sys
import cv2
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml
from metrics import path_length, segment_validity, sweep_mask
from raster_geometry import rasterize

out=Path(__file__).resolve().parent
root=out.parents[2]
config=json.loads((out/'inputs.json').read_text())
summary=json.loads((out/'summary.json').read_text())
source=root/config['source_image']
assert hashlib.sha256(source.read_bytes()).hexdigest()==config['source_sha256']
data=np.load(out/'inputs.npz');r=config['resolution_m']
pixels=np.asarray(Image.open(source).convert('L'))
original_free=(255-pixels.astype(float))/255 < .25
assert np.array_equal(data['free'],np.flipud(original_free))
n,labels,stats,_=cv2.connectedComponentsWithStats(original_free.astype(np.uint8),connectivity=4)
target=labels==1+stats[1:,cv2.CC_STAT_AREA].argmax()
assert np.array_equal(data['target'],np.flipud(target))
assert config['free_pixels']==507209 and config['target_pixels']==470743
assert n-1==config['original_free_components_4connected']
assert np.array_equal(data['allowed'],distance_transform_edt(data['target'])>20)
assert int(data['free'].sum())==config['free_pixels']
polygon=json.loads((out/'target_polygon.json').read_text())
assert len(polygon['geometry']['coordinates'])==93
assert np.array_equal(rasterize([polygon],target.shape,r),target)
assert abs(polygon['area_m2']-target.sum()*r*r)<1e-6
for name,item in config['versions'].items():
    directory=root/'third_party'/name
    assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=directory,text=True).strip()==item['commit']
    assert not subprocess.check_output(['git','status','--porcelain'],cwd=directory,text=True).strip()
for name,digest in config['leader_source_hashes'].items():
    assert hashlib.sha256((root/'third_party/leader/coverage_path_planner'/name).read_bytes()).hexdigest()==digest
binaries={'leader':root/'leader_coverage_ws/build/bg1_leader_coverage/planner_server',
          'f2c':root/'coverage_ws/build/opennav_coverage/opennav_coverage',
          'f2c_library':root/'coverage_ws/install/fields2cover/lib/libFields2Cover.so'}
for name,path in binaries.items():
    assert hashlib.sha256(path.read_bytes()).hexdigest()==config['binary_sha256'][name]

details={}
rng=np.random.default_rng(20261027)
sample_ids=rng.choice(np.flatnonzero(data['free']),size=512,replace=False)
yy,xx=np.unravel_index(sample_ids,data['free'].shape)
witnesses=np.stack([(xx+.5)*r,(yy+.5)*r],axis=1)
for name in ('leader','f2c'):
    raw=json.loads((out/(name+'_raw.json')).read_text())
    metric=summary['planners'][name]
    evaluated=np.load(out/(name+'_evaluated.npz'))
    path=np.asarray(raw['path'],dtype='<f8');xy=path[:,:2]
    assert raw['completed'] and len(raw['runs'])==4
    assert sum(run['warmup'] for run in raw['runs'])==1
    assert all(run['success'] for run in raw['runs'])
    assert raw['server_exit_code']==0 and not Path('/proc/'+str(raw['server_pid'])).exists()
    assert all(run['path_sha256']==hashlib.sha256(path.tobytes()).hexdigest() for run in raw['runs'])
    assert abs(sum(math.dist(a,b) for a,b in zip(xy,xy[1:]))-metric['path_length_m'])<1e-7
    assert abs(path_length(path)-metric['path_length_m'])<1e-9
    lengths=np.linalg.norm(np.diff(xy,axis=0),axis=1)
    for key,mask in [('free',data['free']),('clear',data['allowed'])]:
        assert np.array_equal(segment_validity(path,mask,r),evaluated[key+'_valid_segments'])
    good=evaluated['clear_valid_segments']
    assert metric['invalid_clearance_segments']==int((~good).sum())
    assert abs(metric['invalid_clearance_length_m']-lengths[~good].sum())<1e-8
    assert np.array_equal(sweep_mask(path,data['free'].shape,r,.6),evaluated['geometric'])
    assert np.array_equal(sweep_mask(path,data['free'].shape,r,.6,good),evaluated['valid_sweep'])
    for kind,key in [('geometric','geometric'),('valid_fragment','valid_sweep')]:
        for domain in ('all_free','target'):
            mask=data['free'] if domain=='all_free' else data['target']
            count=int((evaluated[key]&mask).sum())
            assert count==metric[kind+'_'+domain+'_pixels']
            assert abs(100*count/mask.sum()-metric[kind+'_'+domain+'_coverage_pct'])<1e-10
    measured=[run['call_seconds'] for run in raw['runs'] if not run['warmup']]
    assert metric['call_median_s']==statistics.median(measured)
    # A separate point-wise nearest-segment calculation witnesses the capsule
    # union at 512 independently selected free cell centres.
    a,b=xy[:-1],xy[1:];v=b-a;vv=np.einsum('ij,ij->i',v,v)
    for point,index in zip(witnesses,sample_ids):
        delta=point-a
        projection=np.divide(np.einsum('ij,ij->i',delta,v),vv,out=np.zeros_like(vv),where=vv>0)
        closest=a+np.clip(projection,0,1)[:,None]*v
        dist2=np.einsum('ij,ij->i',point-closest,point-closest)
        assert bool((dist2<=.3**2+1e-12).any())==bool(evaluated['geometric'].flat[index])
        assert bool(((dist2<=.3**2+1e-12)&good).any())==bool(evaluated['valid_sweep'].flat[index])
    details[name]=dict(path_sha256=raw['runs'][-1]['path_sha256'],
                       exact_clearance_invalid_segments=int((~good).sum()),
                       source_collision_free=metric['known_free_route_valid'],
                       independent_capsule_witnesses=512,server_shutdown_confirmed=True)

rows=list(csv.DictReader((out/'summary.csv').open()))
for row in rows:
    metric=summary['planners'][row['planner']]
    for key in ('path_length_m','geometric_all_free_coverage_pct','valid_fragment_all_free_coverage_pct','call_median_s'):
        assert float(row[key])==metric[key]
params=yaml.safe_load((out/'f2c_config.yaml').read_text())['coverage_server']['ros__parameters']
assert not params['use_sim_time'] and params['operation_width']==.6
assert params['default_headland_width']==1. and params['min_turning_radius']==.6
assert params['default_route_type']=='BOUSTROPHEDON' and params['default_path_type']=='DUBIN'
for path in out.glob('*.py'):ast.parse(path.read_text())
for path in out.glob('*.png'):
    with Image.open(path) as image:image.verify()
for target_link in re.findall(r'\]\(([^)]+)\)',(out/'README.md').read_text()):
    if not target_link.startswith(('http:','https:')) and target_link!='verification.json':
        assert (out/target_link).exists(),target_link
tests=subprocess.run([sys.executable,'-m','pytest',str(out/'test_metrics.py'),str(out/'test_raster_geometry.py'),'-q'],
                     capture_output=True,text=True)
assert tests.returncode==0,tests.stdout+tests.stderr
subprocess.run(['git','diff','--check'],cwd=root,check=True)
verification=dict(passed=True,source_and_binary_hashes_match=True,submodules_unchanged=True,
                  polygon_roundtrip_equal=True,metrics_recomputed=True,details=details,
                  unit_tests=tests.stdout.strip(),ast_yaml_png_links_valid=True,
                  independent_review_findings_addressed=['Use exact EDT instead of OpenCV approximate EDT for independent clearance audit',
                                                         'Remove invalid coverable-upper-bound metric'],
                  motion_executed=False)
(out/'verification.json').write_text(json.dumps(verification,indent=2)+'\n')
print(json.dumps(verification,indent=2))
