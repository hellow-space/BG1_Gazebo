import numpy as np
import pytest
from metrics import path_length, sweep_mask, segment_validity, clearance_mask


def test_length_includes_connections_and_zero_length_steps():
    assert path_length([[0, 0], [3, 4], [3, 4], [6, 8]]) == 10
    assert path_length([]) == 0


def test_horizontal_sweep_has_caps_and_counts_union_once():
    path = [[.5, 1.5], [4.5, 1.5]]
    expected = np.zeros((4, 5), bool)
    expected[1] = True
    assert np.array_equal(sweep_mask(path, (4, 5), 1, .98), expected)
    assert np.array_equal(sweep_mask(path + path[::-1], (4, 5), 1, .98), expected)


def test_single_pose_sweeps_a_disk_and_empty_path_sweeps_nothing():
    expected = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], bool)
    assert np.array_equal(sweep_mask([[1.5, 1.5]], (3, 3), 1, 2.02), expected)
    assert not sweep_mask([], (3, 3), 1, 1).any()


def test_whole_crossing_segment_is_invalid_despite_free_endpoints():
    allowed = np.ones((5, 5), bool)
    allowed[2, 2] = False
    assert segment_validity([[.5, 2.5], [4.5, 2.5]], allowed, 1).tolist() == [False]
    assert segment_validity([[.5, .5], [4.5, .5]], allowed, 1).tolist() == [True]


def test_short_obstacle_intersection_is_not_missed_by_sampling():
    allowed = np.ones((3, 3), bool)
    allowed[1, 1] = False
    # The line enters cell (1,1) for only a tiny part of its length.
    assert not segment_validity([[.1, .8100001], [2.9, 1.0900001]], allowed, 1)[0]


def test_out_of_bounds_and_invalid_fragment_sweep():
    allowed = np.ones((3, 3), bool)
    assert not segment_validity([[-.01, .5], [1.5, .5]], allowed, 1)[0]
    assert not segment_validity([[.5, .5], [3., .5]], allowed, 1)[0]
    assert not sweep_mask([[.5, .5], [2.5, .5]], (3, 3), 1, .9,
                          valid_segments=np.array([False])).any()


def test_nonfinite_coordinates_are_rejected():
    with pytest.raises(ValueError):
        path_length([[0, 0], [float('nan'), 1]])


def test_clearance_uses_exact_distance_and_strict_pixel_radius():
    free = np.ones((65, 65), bool)
    free[32, 32] = False
    allowed = clearance_mask(free, .05, 1.)
    yy, xx = np.indices(free.shape)
    expected = (xx-32)**2+(yy-32)**2 > 20**2
    assert np.array_equal(allowed, expected)
