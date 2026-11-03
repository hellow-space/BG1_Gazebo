"""IPA native order plus complete, collision-free raster connections."""
from dataclasses import replace
import importlib
from pathlib import Path
import os
import signal
import subprocess
import sys
import numpy as np
import pytest
from coverage_benchmark.models import PlanningInput
from coverage_benchmark.metrics import segment_validity

ROOT=Path(__file__).resolve().parents[2]


def module(name):
    assert (ROOT/('coverage_benchmark/'+name.replace('.','/')+'.py')).exists(), name+' not implemented'
    return importlib.import_module('coverage_benchmark.'+name)


def request(hole=False):
    mask=np.zeros((100,140),bool);mask[5:95,5:135]=True
    if hole:mask[35:65,55:85]=False
    return PlanningInput('tiny',mask,mask.copy(),.05,(1.025,2.525),.6,1.,ROOT,ROOT)


def test_assembly_preserves_every_target_in_order_and_routes_around_wall():
    api=module('ipa_connection')
    mask=np.ones((20,20),bool);mask[:,10]=False;mask[1:3,10]=True
    start=[3.5,10.5];targets=[[16.5,10.5],[3.5,16.5],[16.5,16.5]]
    path,details=api.assemble(start,targets,mask,1.)
    assert path[0]==start and path[-1]==targets[-1]
    assert [path[s['last_vertex']] for s in details['sections']]==targets
    assert details['astar_connections']==3
    assert segment_validity(path,mask,1.).all()
    assert api.strict_path_valid(path,mask,1.)
    assert sum(np.linalg.norm(np.diff(path,axis=0),axis=1))>60


def test_closed_cell_contact_audit_rejects_wall_boundary_and_corner_touch():
    api=module('ipa_connection')
    mask=np.ones((4,4),bool);mask[1,1]=False
    assert not api.strict_path_valid([[2.,1.2],[2.,1.8]],mask,1.)
    assert not api.strict_path_valid([[.5,1.5],[1.5,.5]],mask,1.)
    assert api.strict_path_valid([[2.5,.5],[3.5,3.5]],mask,1.)


def test_unreachable_or_invalid_native_target_fails_without_dropping_it():
    api=module('ipa_connection');mask=np.ones((8,8),bool);mask[:,4]=False
    for target in ([6.5,2.5],[4.5,2.5],[float('nan'),2.5]):
        with pytest.raises(ValueError):api.assemble([1.5,2.5],[target],mask,1.)
    with pytest.raises(ValueError):api.assemble([1.5,2.5],[],mask,1.)


def test_prepare_input_retains_common_raster_scale_start_and_tool_width():
    api=module('adapters.ipa');r=request(True);r.navigable[:2,:2]=True;r.free[:2,:2]=True
    config,scope=api.prepare_input(r)
    assert config['start']==list(r.start)
    assert config['coverage_radius']==.3
    assert config['grid_spacing_m']==pytest.approx(.3*np.sqrt(2))
    assert scope['excluded_navigable_pixels']==4
    assert config['resolution']==.05
    assert scope['raster_sha256']['navigable']


def test_real_native_rectangle_and_hole_keep_order_and_audited_approach(tmp_path):
    api=module('adapters.ipa');up=ROOT/'third_party/ipa_coverage_planning'
    before=subprocess.check_output(['git','-C',str(up),'status','--porcelain'])
    for hole in (False,True):
        r=request(hole);result=api.plan(r,tmp_path/str(hole))
        assert result.success,(result.message,result.details)
        raw=result.details['native_path']
        assert len(raw)>20 and result.path[0]==list(r.start)
        assert [result.path[s['last_vertex']] for s in result.details['connections']['sections']]==raw
        assert module('ipa_connection').strict_path_valid(result.path,np.flipud(r.navigable),r.resolution)
        assert result.details['native_invoked']
        assert result.details['build']['upstream_commit']=='986c18384ed884dadd3bc857cd0c47c13b7d4716'
    assert before==subprocess.check_output(['git','-C',str(up),'status','--porcelain'])==b''


def test_invalid_start_fails_before_native_and_real_native_timeout_is_empty(tmp_path,monkeypatch):
    api=module('adapters.ipa')
    result=api.plan(replace(request(),start=(0.,0.)),tmp_path/'invalid')
    assert not result.success and not result.path and not result.details['native_invoked']
    monkeypatch.setattr(api,'RUN_TIMEOUT_SECONDS',1e-9)
    result=api.plan(request(),tmp_path/'timeout')
    assert not result.success and not result.path and result.details['timed_out']


def test_native_interruption_kills_and_reaps_process(tmp_path):
    api=module('adapters.ipa');pidfile=tmp_path/'pid'
    command=[sys.executable,'-c',f'import os,time;open({str(pidfile)!r},"w").write(str(os.getpid()));time.sleep(60)']
    def interrupt(*_):raise KeyboardInterrupt
    previous=signal.signal(signal.SIGALRM,interrupt)
    try:
        signal.setitimer(signal.ITIMER_REAL,.25)
        with pytest.raises(KeyboardInterrupt):api.run_native(command,dict(os.environ),tmp_path/'log',10)
    finally:
        signal.setitimer(signal.ITIMER_REAL,0);signal.signal(signal.SIGALRM,previous)
    assert pidfile.exists()
    with pytest.raises(ProcessLookupError):os.kill(int(pidfile.read_text()),0)


def test_cache_rejects_modified_binary_generated_source_or_runtime(tmp_path):
    api=module('adapters.ipa_build')
    a=tmp_path/'binary';b=tmp_path/'generated';c=tmp_path/'lib.so'
    for p in (a,b,c):p.write_text('original')
    info={'identity':{'a':1},'artifacts_sha256':{str(p):api.sha(p) for p in (a,b)},'runtime_sha256':{str(c):api.sha(c)}}
    assert api.cache_valid(info,{'a':1})
    for p in (a,b,c):
        p.write_text('changed');assert not api.cache_valid(info,{'a':1});p.write_text('original')
    c.unlink();assert not api.cache_valid(info,{'a':1})


def test_input_rejects_allowed_pixels_outside_target_before_native(tmp_path):
    api=module('adapters.ipa');r=request();r.navigable[0,0]=True
    result=api.plan(r,tmp_path)
    assert not result.success and not result.details['native_invoked']
    assert 'subset' in result.message.lower()


def test_independent_box_audit_detects_boundary_contact_and_safe_detour():
    api=module('ipa_audit');mask=np.ones((8,8),bool);mask[2:6,3]=False
    assert not api.path_box_audit([[1.5,3.5],[6.5,3.5]],mask,1.)
    assert not api.path_box_audit([[4.,2.5],[4.,5.5]],mask,1.)
    assert api.path_box_audit([[1.5,3.5],[1.5,1.5],[6.5,1.5],[6.5,3.5]],mask,1.)


def test_new_report_attempt_invalidates_stale_completion(tmp_path):
    import json
    api=module('ipa_compare')
    (tmp_path/'verification.json').write_text('{"passed":true}')
    (tmp_path/'manifest.json').write_text('{"completed":true}')
    api.start_run(tmp_path,{'completed':False,'run_id':'new'})
    assert not (tmp_path/'verification.json').exists()
    assert json.loads((tmp_path/'manifest.json').read_text())=={'completed':False,'run_id':'new'}


def test_verifier_rejects_removed_reordered_target_and_changed_connection():
    import copy
    api=module('verify_ipa');mask=np.ones((5,5),bool);start=[.5,.5];targets=[[1.5,.5],[2.5,.5]]
    path,connections=module('ipa_connection').assemble(start,targets,mask,1.)
    record={'path':path,'details':{'native_path':targets,'connections':connections}}
    api.verify_replay(record,start,mask,1.)
    bad=copy.deepcopy(record);bad['path'].pop(1)
    with pytest.raises(ValueError):api.verify_replay(bad,start,mask,1.)
    bad=copy.deepcopy(record);bad['details']['native_path'].reverse()
    with pytest.raises(ValueError):api.verify_replay(bad,start,mask,1.)
    bad=copy.deepcopy(record);bad['details']['connections']['sections'][0]['method']='astar'
    with pytest.raises(ValueError):api.verify_replay(bad,start,mask,1.)


def copied_report(tmp_path):
    import json,shutil
    from coverage_benchmark.provenance import file_sha256
    source=ROOT/'docs/benchmarks/six-planners-2026-10-30';dest=tmp_path/'report'
    shutil.copytree(source,dest)
    p=dest/'manifest.json';m=json.loads(p.read_text())
    m['implementation_sha256']={n:file_sha256(ROOT/n) for n in m['implementation_sha256']}
    p.write_text(json.dumps(m))
    target=dest/'A11123AC22_231213_163036_ipa_safe.json'
    target.write_text(target.read_text()+' ')
    return dest


def test_failed_verification_invalidates_old_success_certificate(tmp_path):
    import json
    dest=copied_report(tmp_path);api=module('verify_ipa')
    with pytest.raises(ValueError,match='artifacts_sha256 mismatch'):api.verify(ROOT,ROOT,dest)
    assert not (dest/'verification.json').exists()
    assert not json.loads((dest/'manifest.json').read_text())['completed']


def test_failed_reuse_invalidates_old_success_certificate(tmp_path):
    import json
    dest=copied_report(tmp_path);api=module('ipa_compare')
    with pytest.raises(ValueError,match='fingerprint changed'):api.run(ROOT,ROOT,dest,True)
    assert not (dest/'verification.json').exists()
    assert not json.loads((dest/'manifest.json').read_text())['completed']


def test_real_timeout_retains_verifiable_native_log(tmp_path,monkeypatch):
    from coverage_benchmark.provenance import file_sha256
    api=module('adapters.ipa');monkeypatch.setattr(api,'RUN_TIMEOUT_SECONDS',1e-9)
    result=api.plan(request(),tmp_path)
    assert not result.success and result.details['timed_out']
    assert result.details.get('native_log_sha256')==file_sha256(tmp_path/'ipa_native.log')


def test_current_loader_binding_change_invalidates_native_cache(tmp_path,monkeypatch):
    import shutil
    api=module('adapters.ipa_build');executable,info=api.build(ROOT)
    assert api.cache_valid(info,info['identity'])
    line=next(x for x in subprocess.check_output(['ldd',str(executable)],text=True).splitlines() if 'libopencv_core.so' in x)
    soname,_,library,*_=line.split()
    alternative=tmp_path/soname;shutil.copyfile(library,alternative)
    with alternative.open('ab') as stream:stream.write(b'IPA test: different runtime fingerprint')
    monkeypatch.setenv('LD_LIBRARY_PATH',str(tmp_path))
    assert str(alternative) in subprocess.check_output(['ldd',str(executable)],text=True)
    assert not api.cache_valid(info,info['identity'])
