import importlib
from pathlib import Path
import numpy as np
import pytest
from coverage_benchmark.metrics import segment_validity, path_length


def module():
    assert (Path(__file__).parents[1]/'obstacle_connector.py').exists(), 'obstacle connector missing'
    return importlib.import_module('coverage_benchmark.obstacle_connector')


def test_astar_goes_around_wall_and_keeps_requested_endpoints():
    allowed=np.ones((12,12),bool);allowed[:10,5]=False
    path=module().GridConnector(allowed,1.).connect([1.2,5.2],[9.8,5.8])
    assert path[0]==[1.2,5.2] and path[-1]==[9.8,5.8]
    assert segment_validity(path,allowed,1.).all()
    assert max(p[1] for p in path)>=10.
    assert path_length(path)>8.6


def test_diagonal_cannot_cut_two_touching_obstacle_corners():
    allowed=np.eye(2,dtype=bool)
    with pytest.raises(ValueError,match='disconnect|route'):
        module().GridConnector(allowed,1.).connect([.5,.5],[1.5,1.5])


def test_clips_swath_crossing_hole_instead_of_keeping_free_endpoints_only():
    allowed=np.ones((5,9),bool);allowed[2,3:6]=False
    pieces=module().clip_swath([[.5,2.5],[8.5,2.5]],allowed,1.)
    assert len(pieces)==2
    assert pieces[0][0]==[.5,2.5] and pieces[-1][-1]==[8.5,2.5]
    assert pieces[0][-1][0]<3 and pieces[1][0][0]>=6
    assert all(segment_validity(p,allowed,1.).all() for p in pieces)


def test_clips_outside_image_and_preserves_reverse_direction():
    allowed=np.ones((4,4),bool)
    pieces=module().clip_swath([[5.,1.5],[-1.,1.5]],allowed,1.)
    assert len(pieces)==1 and pieces[0][0][0]>pieces[0][-1][0]
    assert segment_validity(pieces[0],allowed,1.).all()


def test_keeps_native_valid_turn_and_includes_start_approach():
    allowed=np.ones((12,12),bool)
    swaths=[[[1.5,1.5],[8.5,1.5]],[[8.5,4.5],[1.5,4.5]]]
    turns=[[[8.5,1.5],[9.5,2.5],[8.5,4.5]]]
    path,details=module().assemble_path(swaths,turns,allowed,1.,[.5,.5])
    assert path[0]==[.5,.5] and path[-1]==[1.5,4.5]
    assert [9.5,2.5] in path and details['native_turns_kept']==1
    assert segment_validity(path,allowed,1.).all()


def test_replaces_colliding_native_turn_with_connected_checked_path():
    allowed=np.ones((8,12),bool);allowed[2,8]=False
    swaths=[[[1.5,1.5],[8.5,1.5]],[[8.5,4.5],[1.5,4.5]]]
    path,details=module().assemble_path(swaths,[[[8.5,1.5],[8.5,4.5]]],allowed,1.,[1.5,1.5])
    assert details['native_turns_kept']==0 and details['astar_connections']>=1
    assert segment_validity(path,allowed,1.).all()


def test_disconnected_required_swaths_fail_instead_of_dropping_connection():
    allowed=np.ones((8,12),bool);allowed[:,5]=False
    swaths=[[[1.5,1.5],[3.5,1.5]],[[7.5,1.5],[9.5,1.5]]]
    with pytest.raises(ValueError,match='disconnect|route'):
        module().assemble_path(swaths,[],allowed,1.,[1.5,1.5])


def test_nonfinite_and_blocked_start_are_rejected():
    allowed=np.ones((5,5),bool);allowed[0,0]=False
    connector=module().GridConnector(allowed,1.)
    with pytest.raises(ValueError):connector.connect([float('nan'),1.],[2.,2.])
    with pytest.raises(ValueError):connector.connect([.5,.5],[2.,2.])


def test_trace_covers_every_segment_and_identifies_native_sources():
    allowed=np.ones((8,12),bool);allowed[2,8]=False
    swaths=[[[1.5,1.5],[8.5,1.5]],[[8.5,4.5],[1.5,4.5]]]
    path,details=module().assemble_path(swaths,[[[8.5,1.5],[8.5,4.5]]],allowed,1.,[.5,.5])
    sections=details['path_sections']
    cursor=0
    for section in sections:
        assert section['begin_vertex']==cursor
        assert section['end_vertex']>cursor
        cursor=section['end_vertex']
        assert section['source_swath_index'] in (0,1)
        if section['kind']=='swath':
            assert path[section['begin_vertex']]==section['clipped_swath'][0]
            assert path[section['end_vertex']]==section['clipped_swath'][-1]
        else:
            assert section['method'] in ('native_turn','direct_checked','astar')
    assert cursor==len(path)-1
    assert [s['source_swath_index'] for s in sections if s['kind']=='swath']==[0,1]
    assert any(s.get('method')=='astar' for s in sections)


def test_turn_mapping_rejects_mismatched_path_components_even_with_same_count():
    m=module()
    assert hasattr(m,'usable_native_turns'), 'Missing native turn provenance check'
    swaths=[[[1.,1.],[5.,1.]],[[5.,3.],[1.,3.]]]
    turn=[[[5.,1.],[6.,2.],[5.,3.]]]
    assert m.usable_native_turns(swaths,swaths,turn)==turn
    mismatched=[swaths[0],[[5.,3.],[2.,3.]]]
    assert m.usable_native_turns(swaths,mismatched,turn)==[]
    assert m.usable_native_turns(swaths,swaths,[])==[]
