import math
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from coverage_geometry import validate_polygon, validate_path, connector, validate_result


def test_cleanup_failure_still_stops_and_reaps():
    from coverage_geometry import finish_cleanup
    calls=[]
    def cancel():
        calls.append('cancel')
        raise TimeoutError('No cancellation acknowledgement')
    with pytest.raises(RuntimeError,match='No cancellation acknowledgement'):
        finish_cleanup([cancel,lambda:calls.append('stop'),lambda:calls.append('reap')])
    assert calls==['cancel','stop','reap']

FOOTPRINT = [(0.84,0.38),(0.84,-0.38),(-0.355,-0.38),(-0.355,0.38)]

@pytest.mark.parametrize('points', [[], [(0,0),(1,0),(2,0)], [(0,0),(1,1),(0,1),(1,0)], [(0,0),(math.nan,1),(1,0)]])
def test_invalid_polygon_rejected(points):
    with pytest.raises(ValueError):
        validate_polygon(points)

def test_polygon_is_closed_without_mutating_input():
    p = [(0,0),(6,0),(6,4),(0,4)]
    assert validate_polygon(p) == p+[p[0]]
    assert len(p)==4

def test_nonfinite_and_empty_paths_rejected():
    for path in ([], [(0,0,0),(math.inf,1,0)]):
        with pytest.raises(ValueError):
            validate_path(path, [-2,8,-2,6], FOOTPRINT)

def test_rotated_footprint_boundary_is_checked():
    validate_path([(0,0,0),(1.5,0,math.pi/2)], [-2,2,-2,2], FOOTPRINT)
    with pytest.raises(ValueError):
        validate_path([(0,0,0),(0,1.5,math.pi/2)], [-2,2,-2,2], FOOTPRINT)

def test_connector_spacing_and_endpoint_heading():
    path = connector((0,0,0),(1,1,math.pi/2),.1)
    assert path[0][:2] == (0,0)
    assert path[-1] == (1,1,math.pi/2)
    assert max(math.dist(a[:2],b[:2]) for a,b in zip(path,path[1:])) <= .100001

@pytest.mark.parametrize('status,code', [(6,0),(5,0),(4,803)])
def test_failed_coverage_result_never_executes(status,code):
    with pytest.raises(ValueError):
        validate_result(status,code,'map',[(0,0,0),(1,0,0)],[-2,8,-2,6],FOOTPRINT)

def test_wrong_frame_rejected():
    with pytest.raises(ValueError):
        validate_result(4,0,'odom',[(0,0,0),(1,0,0)],[-2,8,-2,6],FOOTPRINT)


def test_connector_preserves_initial_heading_and_rejects_rotation_sweep():
    path=connector((7.15,0,0),(7.15,0,math.pi/2))
    assert path[0]==(7.15,0,0)
    with pytest.raises(ValueError):validate_path(path,[-2,8,-2,6],FOOTPRINT)


def test_quaternion_must_be_finite_and_unit_length():
    from coverage_geometry import quaternion_yaw
    for q in [(0,0,math.inf,1),(0,0,0,0),(0,0,0,2)]:
        with pytest.raises(ValueError):quaternion_yaw(*q)
    assert quaternion_yaw(0,0,0,1)==0


def test_motion_rejects_stationary_stale_and_nonfinite_samples():
    from coverage_geometry import validate_motion
    stationary=[{'t':i*.1,'x':0,'y':0,'yaw':0} for i in range(12)]
    with pytest.raises(ValueError):validate_motion(stationary,1.2)
    moving=[dict(p,x=p['t']) for p in stationary]
    assert validate_motion(moving,1.2)>1
    with pytest.raises(ValueError):validate_motion(moving,5)
    moving[-1]['x']=math.nan
    with pytest.raises(ValueError):validate_motion(moving,1.2)


def test_map_odom_correction_composes_to_actual_pose():
    from coverage_geometry import map_to_odom
    actual=(3,4,math.pi/2);odom=(1,0,0)
    x,y,a=map_to_odom(actual,odom)
    assert x==pytest.approx(3)
    assert y==pytest.approx(3)
    assert a==pytest.approx(math.pi/2)


def test_gazebo_timestamp_accepts_protobuf_int64_string():
    from coverage_geometry import stamp_seconds
    assert stamp_seconds({'sec':'42','nsec':250000000})==42.25
