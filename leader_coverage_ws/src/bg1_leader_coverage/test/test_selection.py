import importlib.util
from pathlib import Path
import pytest

MODULE=Path(__file__).parents[1]/'scripts/selection.py'

def load():
    assert MODULE.exists(), 'selection validation not implemented'
    spec=importlib.util.spec_from_file_location('selection',MODULE)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_polygon_validation():
    validate=load().validate_polygon
    assert validate([(0,0),(4,0),(4,3),(0,3)])==[(0,0),(4,0),(4,3),(0,3)]
    for points in ([],[(0,0),(1,1),(2,2)],[(0,0),(3,3),(0,3),(3,0)],
                   [(0,0),(float('nan'),0),(1,1)],[(0,0),(3,0),(3,0),(0,3)]):
        with pytest.raises(ValueError):validate(points)


def test_close_reset_undo():
    state=load().Selection()
    assert not state.add((0,0))
    assert not state.add((4,0))
    assert not state.add((4,3))
    assert state.add((.1,.1))
    assert state.closed and len(state.points)==3
    assert not state.add((10,10))
    assert state.points==[(10,10)] and not state.closed
    state.undo();assert not state.points
    state.add((0,0));state.reset();assert not state.points


def test_bad_closure_does_not_lock_selection():
    state=load().Selection()
    for p in [(0,0),(1,1),(2,2)]:state.add(p)
    with pytest.raises(ValueError):state.add((0,0))
    assert not state.closed
