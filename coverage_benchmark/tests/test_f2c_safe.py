import importlib
import os
from pathlib import Path
from dataclasses import replace
import numpy as np
import pytest
from coverage_benchmark.models import PlanningInput
from coverage_benchmark.metrics import evaluate_path

ROOT=Path(__file__).resolve().parents[2]

def adapter():
    assert (ROOT/'coverage_benchmark/adapters/f2c_safe.py').exists(), 'F2C safe adapter missing'
    return importlib.import_module('coverage_benchmark.adapters.f2c_safe')


def test_reachable_mask_preserves_denominator_and_does_not_snap_start():
    allowed=np.zeros((10,20),bool);allowed[2:8,2:8]=True;allowed[2:8,12:18]=True
    request=PlanningInput('two',np.ones_like(allowed),allowed,1.,(3.5,4.5),.6,1.,ROOT,ROOT)
    mask,details=adapter().reachable_mask(request)
    assert mask.sum()==36 and request.free.sum()==200
    assert details['excluded_navigable_pixels']==36 and details['navigable_components']==2
    with pytest.raises(ValueError,match='start'):
        adapter().reachable_mask(replace(request,start=(.5,.5)))


def test_native_polygonization_keeps_holes_and_exact_pixel_area():
    assert (ROOT/'coverage_benchmark/raster_geometry.py').exists(), 'Production polygonization missing'
    geometry=importlib.import_module('coverage_benchmark.raster_geometry')
    mask=np.zeros((35,45),bool);mask[2:32,3:40]=True;mask[12:20,15:23]=False
    polygons=geometry.polygonize(mask,.05)
    assert len(polygons)==1 and len(polygons[0]['geometry']['coordinates'])==2
    assert polygons[0]['valid'] and polygons[0]['area_m2']==pytest.approx(mask.sum()*.05**2)
    assert np.array_equal(geometry.rasterize(polygons,mask.shape,.05),mask)


def test_blocked_start_returns_explicit_failure_without_starting_ros(tmp_path):
    allowed=np.zeros((10,10),bool)
    request=PlanningInput('blocked',np.ones_like(allowed),allowed,1.,(.5,.5),.6,1.,ROOT,ROOT)
    result=adapter().plan(request,tmp_path)
    assert not result.success and not result.path and 'start' in result.message.lower()


def test_real_native_service_returns_swaths_and_safe_path_on_rectangle(tmp_path):
    runtime=Path(os.environ.get('BG1_RUNTIME_ROOT',ROOT))
    if not (runtime/'coverage_ws/build/opennav_coverage/opennav_coverage').exists():
        pytest.skip('Existing ROS coverage build required; set BG1_RUNTIME_ROOT')
    free=np.zeros((100,120),bool);free[3:97,3:117]=True
    allowed=np.zeros_like(free);allowed[24:76,24:96]=True
    request=PlanningInput('rectangle',free,allowed,float(np.float32(.05)),(3.025,2.525),.6,1.,ROOT,runtime)
    result=adapter().plan(request,tmp_path)
    assert result.success,result.message
    assert result.details['native_candidate']['swaths']
    assert result.details['native_baseline']['success']
    assert result.details['native_server_exit_code']==0
    assert result.path[0]==list(request.start)
    assert evaluate_path(result.path,free,allowed,request.resolution,.6,True)['path_valid']


def test_ordered_native_swaths_survive_optional_turn_process_failure():
    m=adapter()
    assert hasattr(m,'extract_native_candidate'),'Missing route-only native fallback'
    route=dict(success=True,swaths=[[[1.,1.],[4.,1.]],[[4.,2.],[1.,2.]]],
               swaths_ordered=True,contains_turns=False,path=[],turns=[],action_status=4,error_code=0,
               generate_headland=False)
    raw=dict(success=False,baseline={'success':True},candidate_route=route,
             error='RuntimeError: Native server exited -9',server_exit_code=-9)
    candidate=m.extract_native_candidate(raw)
    assert candidate['success'] and candidate['swaths']==route['swaths']
    assert candidate['ordered_swaths_source']=='native action generate_path=False'
    assert candidate['native_path_success'] is False and candidate['turns']==[]
    assert '-9' in candidate['native_path_error']
    assert raw['candidate_route']==route  # Normalization must preserve original native evidence.
