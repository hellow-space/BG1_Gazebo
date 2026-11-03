"""Real IPA BCD decomposition/sweeps and target/coverage-aware Local kernel."""
import importlib
from pathlib import Path
import subprocess
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[2]


def api():
    assert (ROOT/'coverage_benchmark/hybrid/native.py').exists(),'Hybrid native interface missing'
    return importlib.import_module('coverage_benchmark.hybrid.native')


def room(clutter=False):
    f=np.zeros((90,150),bool);f[5:85,5:145]=True
    if clutter:
        f[25:45,30:45]=False;f[50:65,60:80]=False;f[20:40,100:115]=False
    return f


def test_real_ipa_bcd_exports_decomposition_and_sweep_paths(tmp_path):
    native=api();before=subprocess.check_output(['git','-C',str(ROOT/'third_party/ipa_coverage_planning'),'status','--porcelain'])
    result=native.decompose(room(True),.05,.6,[.525,.525],ROOT,tmp_path)
    assert len(result['cells'])>=4
    assert sum(len(c['path']) for c in result['cells'])>20
    assert all(c['polygon'] and c['source']=='IPA BoustrophedonExplorer' for c in result['cells'])
    assert result['build']['upstream_commit']=='986c18384ed884dadd3bc857cd0c47c13b7d4716'
    assert before==subprocess.check_output(['git','-C',str(ROOT/'third_party/ipa_coverage_planning'),'status','--porcelain'])==b''


def test_local_task_mask_does_not_turn_outside_region_into_obstacles(tmp_path):
    native=api();f=room();task=np.zeros_like(f);task[15:75,95:135]=True
    c=np.zeros_like(f)
    r=native.local(f,task,c,.05,.6,[.525,.525],ROOT,tmp_path)
    assert len(r['path'])>20
    p=np.asarray(r['path']);cols=np.floor(p[:,0]/.05).astype(int);rows=np.floor(p[:,1]/.05).astype(int)
    assert task[rows,cols].all()
    assert r['diagnostics']['free_nodes']>r['diagnostics']['initial_target_nodes']>0
    assert r['diagnostics']['outside_task_visited_nodes']==0


def test_local_inherits_covered_state_and_does_not_plan_fully_covered_task(tmp_path):
    native=api();f=room();r=native.local(f,f,f.copy(),.05,.6,[.525,.525],ROOT,tmp_path)
    assert r['path']==[]
    assert r['diagnostics']['initial_target_nodes']==0
    assert r['diagnostics']['seed_visited_nodes']>0


def test_invalid_native_inputs_rejected_without_launch(tmp_path):
    native=api();f=room()
    with pytest.raises(ValueError):native.local(f,np.ones((2,2),bool),f,.05,.6,[.525,.525],ROOT,tmp_path)
    with pytest.raises(ValueError):native.decompose(f,.05,0.,[.525,.525],ROOT,tmp_path)
