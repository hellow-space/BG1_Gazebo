from pathlib import Path
import numpy as np
import pytest
from PIL import Image
from coverage_benchmark.map_io import load_map, pixel_to_world, world_to_pixel
from coverage_benchmark.metrics import evaluate_path


def test_map_threshold_unknown_component_and_coordinate_roundtrip(tmp_path):
    pixels=np.full((9,13),128,np.uint8)
    pixels[1:8,1:8]=255
    pixels[4,4]=200
    pixels[2:4,10:12]=255
    path=tmp_path/'map.pgm';Image.fromarray(pixels).save(path)
    data=load_map(path,resolution=1.,clearance=1.)
    assert data.free.sum()==53
    assert data.target.sum()==49
    assert data.target[4,4] and not data.target[2,10]
    assert not data.navigable[1,1] and data.navigable[4,4]
    xy=pixel_to_world((3,5),9,1.)
    assert xy==[5.5,5.5]
    assert world_to_pixel(xy,9,1.)==(3,5)


def test_three_metrics_use_full_free_denominator_and_union():
    free=np.ones((5,5),bool)
    path=[[.5,2.5],[4.5,2.5],[.5,2.5]]
    result=evaluate_path(path,free,free,1.,.8,success=True)
    assert result['coverage_percent']==20.
    assert result['path_length_m']==8.
    assert result['path_valid'] is True


def test_collision_line_not_accepted_despite_free_endpoints():
    free=np.ones((5,5),bool);free[2,2]=False
    result=evaluate_path([[.5,2.5],[4.5,2.5]],free,free,1.,.8,success=True)
    assert result['path_valid'] is False
    assert result['coverage_percent']==pytest.approx(100*4/24)
    assert result['path_length_m']==4.
    assert result['diagnostics']['invalid_segments']==1


def test_no_path_does_not_get_a_zero_length_advantage():
    free=np.ones((5,5),bool)
    result=evaluate_path([],free,free,1.,.8,success=False)
    assert result['coverage_percent']==0.
    assert result['path_length_m'] is None
    assert result['path_valid'] is False


def test_clearance_grid_and_out_of_bounds_are_invalid():
    free=np.ones((5,5),bool);allowed=free.copy();allowed[2,2]=False
    assert not evaluate_path([[.5,2.5],[4.5,2.5]],free,allowed,1.,.8,success=True)['path_valid']
    assert not evaluate_path([[-.01,.5],[2.5,.5]],free,free,1.,.8,success=True)['path_valid']


def test_nonfinite_native_output_is_invalid_not_crash():
    free=np.ones((5,5),bool)
    result=evaluate_path([[.5,.5],[float('nan'),2]],free,free,1.,.8,success=True)
    assert not result['path_valid'] and result['path_length_m'] is None
