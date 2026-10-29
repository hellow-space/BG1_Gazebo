"""ETHZ real native integration and map-scope contract."""
from dataclasses import replace
import importlib
from pathlib import Path
import subprocess
import numpy as np
import pytest
from coverage_benchmark.models import PlanningInput
from coverage_benchmark.metrics import segment_validity
from coverage_benchmark.raster_geometry import rasterize

ROOT=Path(__file__).resolve().parents[2]


def adapter():
    assert (ROOT/'coverage_benchmark/adapters/ethz.py').exists(), 'ETHZ adapter not implemented'
    return importlib.import_module('coverage_benchmark.adapters.ethz')


def request(with_hole=False):
    mask=np.zeros((100,140),bool)
    mask[5:95,5:135]=True
    if with_hole:mask[35:65,55:85]=False
    return PlanningInput('tiny',mask,mask.copy(),.05,(1.025,2.525),.6,1.,ROOT,ROOT)


def test_geometry_keeps_hole_and_only_start_component():
    source=request(True)
    # Disconnected second component must not be joined by a fake straight line.
    source.navigable[0:2,0:2]=True
    config,details=adapter().prepare_input(source)
    assert len(config['polygon']['geometry']['coordinates'])==2
    burn=rasterize([config['polygon']],source.free.shape,source.resolution)
    assert not np.any(burn & ~source.navigable)
    assert not burn[50,70]
    assert not burn[0,0]
    assert details['excluded_navigable_pixels']==4
    assert config['start']==list(source.start)
    assert config['goal']==list(source.start)


def test_invalid_start_fails_without_native_invocation(tmp_path):
    result=adapter().plan(replace(request(),start=(0.,0.)),tmp_path)
    assert not result.success and result.path==[]
    assert not result.details['native_invoked']
    assert 'start' in result.message.lower()


def test_native_rectangle_and_hole_return_audited_closed_routes(tmp_path):
    before=subprocess.check_output(['git','-C',str(ROOT/'third_party/polygon_coverage_planning'),'status','--porcelain'])
    for hole in (False,True):
        source=request(hole)
        result=adapter().plan(source,tmp_path/str(hole))
        assert result.success,(result.message,result.details)
        assert len(result.path)>5
        assert result.path[0]==pytest.approx(source.start)
        assert result.path[-1]==pytest.approx(source.start)
        assert segment_validity(result.path,np.flipud(source.navigable),source.resolution).all()
        assert result.details['native_invoked']
        assert result.details['native']['solver']=='GK-MA'
        assert result.details['native']['decomposition_cells'] >= (4 if hole else 1)
        assert result.details['path_policy']=='complete native route, no repair or filtering'
    assert before==subprocess.check_output(['git','-C',str(ROOT/'third_party/polygon_coverage_planning'),'status','--porcelain'])==b''


def test_real_native_timeout_is_explicit_failure(tmp_path,monkeypatch):
    module=adapter()
    monkeypatch.setattr(module,'RUN_TIMEOUT_SECONDS',1e-9)
    result=module.plan(request(),tmp_path)
    assert not result.success and result.path==[]
    assert result.details['native_invoked']
    assert result.details['timed_out']


def test_interrupted_native_process_is_killed_and_reaped(tmp_path):
    import os
    import signal
    import sys
    module=adapter()
    assert hasattr(module,'run_native'), 'Native supervisor must clean up on BaseException'
    pidfile=tmp_path/'pid'
    command=[sys.executable,'-c',f'import os,time;open({str(pidfile)!r},"w").write(str(os.getpid()));time.sleep(60)']
    def interrupt(*_):raise KeyboardInterrupt
    previous=signal.signal(signal.SIGALRM,interrupt)
    try:
        signal.setitimer(signal.ITIMER_REAL,.25)
        with pytest.raises(KeyboardInterrupt):
            module.run_native(command,dict(os.environ),tmp_path/'child.log',10)
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        signal.signal(signal.SIGALRM,previous)
    assert pidfile.exists()
    with pytest.raises(ProcessLookupError):os.kill(int(pidfile.read_text()),0)


def test_new_run_ignores_unrelated_old_native_output(tmp_path):
    import coverage_benchmark.ethz_compare as module
    assert hasattr(module,'copy_run_artifacts'), 'Managed evidence must not retain stale native output'
    scene='tiny';source=tmp_path/'current';dest=tmp_path/'report'
    source.mkdir();dest.mkdir()
    (source/'ethz_input.txt').write_text('fresh input')
    (dest/(scene+'_ethz_output.txt')).write_text('stale successful route')
    module.copy_run_artifacts(source,dest,scene)
    assert not (dest/(scene+'_ethz_output.txt')).exists()
    assert (dest/(scene+'_ethz_input.txt')).read_text()=='fresh input'


def test_modified_runtime_dependency_invalidates_native_cache(tmp_path):
    from coverage_benchmark.adapters import ethz_build
    assert hasattr(ethz_build,'cache_valid'), 'Runtime dependency fingerprints must be checked before reusing binary'
    artifact=tmp_path/'planner';library=tmp_path/'runtime.so'
    artifact.write_text('executable');library.write_text('original library')
    info=dict(identity={'version':1},artifacts_sha256={'planner':ethz_build.sha(artifact)},
              runtime_sha256={str(library):ethz_build.sha(library)})
    assert ethz_build.cache_valid(tmp_path,info,{'version':1})
    library.write_text('modified library')
    assert not ethz_build.cache_valid(tmp_path,info,{'version':1})
    library.unlink()
    assert not ethz_build.cache_valid(tmp_path,info,{'version':1})


def test_verifier_refuses_optimized_python_instead_of_skipping_checks():
    import sys
    code='from pathlib import Path;from coverage_benchmark.verify_ethz import verify;verify(Path("/missing"),Path("/missing"),Path("/missing"))'
    completed=subprocess.run([sys.executable,'-O','-c',code],capture_output=True,text=True)
    assert completed.returncode!=0
    assert 'requires assertions enabled' in completed.stderr


def test_native_bcd_collinear_cell_vertices_preserve_geometry(tmp_path,monkeypatch):
    # Seed 1 creates aligned obstacle edges whose native BCD cells include
    # redundant collinear vertices; original VisibilityGraph aborts on them.
    mask=np.zeros((100,140),bool);mask[5:95,5:135]=True
    rng=np.random.default_rng(1)
    for _ in range(8):
        x=int(rng.choice(np.arange(30,115,10)));y=int(rng.choice(np.arange(10,75,10)))
        w=int(rng.choice([10,20,30]));h=int(rng.choice([10,20,30]))
        mask[y:y+h,x:x+w]=False
    source=PlanningInput('collinear',mask,mask.copy(),.05,(.775,2.525),.6,1.,ROOT,ROOT)
    module=adapter();monkeypatch.setattr(module,'POLYGON_GRID_M',.05)
    result=module.plan(source,tmp_path)
    assert result.success,result.message
    assert segment_validity(result.path,np.flipud(mask),.05).all()


def test_conservative_polygon_grid_never_fills_obstacle_or_partial_block():
    module=adapter()
    assert hasattr(module,'conservative_polygon_mask'), 'Raster polygons need a declared conservative complexity bound'
    mask=np.ones((9,11),bool);mask[1,1]=False
    reduced,details=module.conservative_polygon_mask(mask,.05)
    assert reduced.shape==mask.shape
    assert not (reduced & ~mask).any()
    assert not reduced[:4,:4].any()  # One blocked pixel excludes the entire block.
    assert reduced[:8,4:8].all()
    assert not reduced[8:,:].any() and not reduced[:,8:].any() # Partial edge blocks are blocked.
    assert details['polygon_block_cells']==4
    assert details['effective_polygon_grid_m']==pytest.approx(.2)


def test_new_run_invalidates_previous_completed_verification(tmp_path):
    import json
    import coverage_benchmark.ethz_compare as module
    assert hasattr(module,'start_run'), 'A new attempt must mark its report incomplete before planning'
    (tmp_path/'verification.json').write_text('{"passed":true}')
    (tmp_path/'manifest.json').write_text('{"completed":true,"run_id":"old"}')
    module.start_run(tmp_path,{'completed':False,'run_id':'new'})
    assert not (tmp_path/'verification.json').exists()
    assert json.loads((tmp_path/'manifest.json').read_text())=={'completed':False,'run_id':'new'}
