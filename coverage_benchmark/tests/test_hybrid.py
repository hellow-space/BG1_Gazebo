from pathlib import Path
import numpy as np
import pytest
from coverage_benchmark.models import PlanningInput

ROOT=Path(__file__).resolve().parents[2]

def api():
    assert (ROOT/'coverage_benchmark/hybrid/planner.py').exists(), 'Hybrid planner missing'
    from coverage_benchmark.hybrid import planner
    return planner

def fixture():
    f=np.zeros((70,130),bool);f[5:65,5:125]=True;f[25:50,45:65]=False
    return f

def test_partition_is_complete_disjoint_and_keeps_one_pixel_corridor():
    p=api();f=fixture();f[:,80]=False;f[35,80]=True
    cells=[dict(id=0,polygon=[[.55,.55],[4.95,.55],[4.95,6.45],[.55,6.45]],path=[[1,1],[4,1]])]
    regions,labels=p.partition(f,cells,.1,.6)
    assert np.array_equal(labels>=0,f)
    assert labels[35,80]>=0
    assert sum(int(r.mask.sum()) for r in regions)==int(f.sum())
    assert any(r.kind=='unassigned' for r in regions)

def test_ledger_includes_transit_and_is_monotone():
    p=api();ledger=p.CoverageLedger((50,80),.1,.6)
    ledger.add([[1,1],[6,1]])
    before=ledger.covered.copy();assert ledger.covered[10,30]
    gain=ledger.add([[6,1],[6,3]])
    assert gain>0 and np.all(ledger.covered[before])
    assert ledger.add([[1,1],[6,1]])==0

def test_composer_routes_outside_task_and_never_cuts_wall():
    p=api();f=fixture();ledger=p.CoverageLedger(f.shape,.1,.6)
    route=p.SafeRoute(f,.1,ledger,[2,3.5]);route.targets([[9,3.5]],{'kernel':'test','region':0})
    from coverage_benchmark.ipa_connection import strict_path_valid
    assert strict_path_valid(route.path,f,.1)
    assert any(y<2.5 or y>=5 for x,y in route.path)
    assert ledger.covered.any()

def test_native_room_retains_only_original_ipa_modes(tmp_path):
    p=api();f=fixture();top=np.flipud(f)
    data=PlanningInput('synthetic',top,top,.1,(1.05,1.05),.6,0.,ROOT,tmp_path)
    records=p.compare(data)
    assert set(records)=={'ipa_bcd','ipa_local'}
    for result in records.values():
        assert result.success,result.message
        assert result.details['strict_valid'] and result.details['box_audit']['passed']
        assert result.details['residual_pixels']>=0
    assert records['ipa_bcd'].details['kernel_calls']['local']==0
    assert records['ipa_local'].details['kernel_calls']['bcd']==0
    assert not hasattr(p,'cleanup')

def test_candidate_collinear_compression_keeps_geometry():
    p=api();f=np.ones((30,60),bool)
    cell=dict(path=[[1.05+i*.1,1.05] for i in range(30)]+[[3.95,1.15],[3.95,1.25]])
    pieces,_=p.candidates(cell,f,.1)
    assert len(pieces)==2
    from coverage_benchmark.metrics import path_length
    assert path_length([v for piece in pieces for v in piece])==pytest.approx(path_length(cell['path']))

def test_comparison_replay_rejects_tampering_and_clears_certificate(tmp_path):
    assert (ROOT/'coverage_benchmark/hybrid/experiment.py').exists(), 'Hybrid experiment interface missing'
    from coverage_benchmark.hybrid.experiment import run,verify
    from PIL import Image
    import json
    f=fixture();mid='room';folder=tmp_path/'maps'/mid;folder.mkdir(parents=True)
    Image.fromarray(np.flipud(f).astype('uint8')*255).save(folder/(mid+'.png'))
    out=tmp_path/'output';run(ROOT,tmp_path/'maps',out,[mid],.1,.6,0.,plots=False)
    assert verify(ROOT,tmp_path/'maps',out)['passed']
    rec=out/(mid+'_ipa_local.json');record=json.loads(rec.read_text());record['path'][1]=[-1,-1];rec.write_text(json.dumps(record))
    with pytest.raises(ValueError):verify(ROOT,tmp_path/'maps',out)
    assert not (out/'verification.json').exists()
    assert not json.loads((out/'manifest.json').read_text())['completed']

def test_failed_variants_remain_failures_in_a_verifiable_report(tmp_path,monkeypatch):
    p=api()
    from coverage_benchmark.hybrid.experiment import run
    from PIL import Image
    import json
    f=fixture();mid='failed';folder=tmp_path/'maps'/mid;folder.mkdir(parents=True)
    Image.fromarray(np.flipud(f).astype('uint8')*255).save(folder/(mid+'.png'))
    def fail(*args,**kwargs):raise ValueError('test solver failure')
    monkeypatch.setattr(p,'_plan',fail)
    out=tmp_path/'output';certificate=run(ROOT,tmp_path/'maps',out,[mid],.1,.6,0.,plots=False)
    assert certificate['passed']
    rec=json.loads((out/(mid+'_ipa_bcd.json')).read_text())
    assert not rec['planner_success'] and rec['path']==[] and rec['metrics']['path_length_m'] is None

def test_real_local_timeout_becomes_failed_variant_and_other_variants_continue(tmp_path,monkeypatch):
    p=api()
    from coverage_benchmark.adapters.ipa import run_native as actual_run
    from coverage_benchmark.hybrid.build import build
    from coverage_benchmark.provenance import file_sha256
    import json
    build(ROOT);f=fixture();top=np.flipud(f);data=PlanningInput('timeout',top,top,.1,(1.05,1.05),.6,0.,ROOT,tmp_path)
    def timeout_once(command,env,log,timeout):
        request=json.loads(Path(command[1]).read_text())
        return actual_run(command,env,log,1e-6 if request['mode']=='local' and 'ipa_local' in str(log) else timeout)
    monkeypatch.setattr(p.native,'run_native',timeout_once)
    records=p.compare(data)
    failed=records['ipa_local'];assert not failed.success and failed.path==[]
    assert 'timeout' in failed.message.lower()
    evidence=failed.details['native_failure']['evidence']
    assert any(Path(x).name=='native.log' for x in evidence['files_sha256'])
    assert all(file_sha256(Path(x))==digest for x,digest in evidence['files_sha256'].items())
    assert records['ipa_bcd'].success


def test_build_source_revalidation_detects_changed_compat_without_rebuilding(tmp_path,monkeypatch):
    from coverage_benchmark.hybrid import build as module
    assert hasattr(module,'verify_build'), 'Native build source verifier missing'
    from coverage_benchmark.provenance import file_sha256
    tracked=tmp_path/'compat.hpp';tracked.write_text('original')
    compiler=tmp_path/'compiler';compiler.write_text('compiler')
    info=dict(upstream_commit='fixed',identity=dict(upstream_commit='fixed',source_sha256={'compat.hpp':file_sha256(tracked)},compiler=str(compiler),compiler_sha256=file_sha256(compiler)))
    monkeypatch.setattr(module,'check_upstream',lambda root:None)
    monkeypatch.setattr(module,'cache_valid',lambda *a:True)
    assert module.verify_build(info,tmp_path)
    tracked.write_text('changed')
    assert not module.verify_build(info,tmp_path)

def test_true_timeout_report_keeps_failure_evidence_and_verifies(tmp_path,monkeypatch):
    p=api()
    from coverage_benchmark.hybrid.experiment import run
    from coverage_benchmark.adapters.ipa import run_native as actual_run
    from PIL import Image
    import json
    f=fixture();mid='timeout';folder=tmp_path/'maps'/mid;folder.mkdir(parents=True)
    Image.fromarray(np.flipud(f).astype('uint8')*255).save(folder/(mid+'.png'))
    def timeout_one(command,env,log,timeout):
        request=json.loads(Path(command[1]).read_text())
        return actual_run(command,env,log,1e-6 if request['mode']=='local' and 'ipa_local' in str(log) else timeout)
    monkeypatch.setattr(p.native,'run_native',timeout_one)
    out=tmp_path/'out';assert run(ROOT,tmp_path/'maps',out,[mid],.1,.6,0.,False)['passed']
    result=json.loads((out/(mid+'_ipa_local.json')).read_text())
    assert result['path']==[] and result['metrics']['path_length_m'] is None
    assert 'native_failure' in result['details']
