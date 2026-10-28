"""Real native BCD integration: geometry, failure handling, upstream immutability."""
from dataclasses import replace
import hashlib
import importlib
from pathlib import Path
import subprocess
import time

import numpy as np
import pytest

from coverage_benchmark.models import PlanningInput

ROOT = Path(__file__).resolve().parents[2]


def adapter():
    # A missing implementation is an assertion failure, not a collection error.
    assert (ROOT / 'coverage_benchmark/adapters/bcd.py').exists(), 'BCD adapter is not implemented'
    return importlib.import_module('coverage_benchmark.adapters.bcd')


def request(mask, *, start=(2.525, 2.025)):
    return PlanningInput('tiny', mask, mask.copy(), .05, start, .6, 1., ROOT, ROOT)


def upstream_hashes():
    upstream = ROOT / 'third_party/bcd_planner'
    names = subprocess.check_output(['git', '-C', str(upstream), 'ls-files', '-z']).split(b'\0')
    return {name.decode(): hashlib.sha256((upstream / name.decode()).read_bytes()).hexdigest()
            for name in names if name}


def rectangle():
    mask = np.zeros((80, 100), dtype=bool)
    mask[3:77, 3:97] = True
    return mask


def test_native_free_rectangle_headless_nonempty_and_upstream_unchanged(tmp_path, monkeypatch):
    monkeypatch.delenv('DISPLAY', raising=False)
    before = upstream_hashes()
    result = adapter().plan(request(rectangle()), tmp_path)
    assert result.success, (result.message, result.details)
    points = np.asarray(result.path)
    assert len(points) > 100
    assert np.isfinite(points).all()
    assert (points[:, 0] >= .175).all() and (points[:, 0] <= 4.825).all()
    assert (points[:, 1] >= .175).all() and (points[:, 1] <= 3.825).all()
    # Pixel output must use centres and the world y axis must point upwards.
    assert points[0].tolist() == pytest.approx([2.525, 2.025])
    assert result.details['native_point_count'] == len(points)
    assert result.details['sweep_spacing_pixels'] == 12
    assert result.details['native_inset_pixels'] == 11
    assert before == upstream_hashes()
    assert subprocess.check_output(['git', '-C', str(ROOT / 'third_party/bcd_planner'), 'status', '--porcelain']) == b''


def test_native_obstacle_hole_terminates_and_returns_complete_path_or_explicit_failure(tmp_path):
    # Leave room for the native 11px inset around both exterior and hole.
    mask = np.zeros((140, 160), dtype=bool)
    mask[3:137, 3:157] = True
    mask[50:90, 65:95] = False
    started = time.monotonic()
    result = adapter().plan(request(mask, start=(1.025, 3.025)), tmp_path)
    assert time.monotonic() - started < 125
    assert result.details['native_invoked'] is True
    if result.success:
        assert len(result.path) > 100
        assert np.isfinite(result.path).all()
        assert result.details['native_point_count'] == len(result.path)
        assert result.details['native_segment_count'] > 1  # Native cell connections are retained.
    else:
        assert result.path == []
        assert result.message
        assert result.details.get('error')


def test_empty_navigable_fails_instead_of_planning_on_raw_free(tmp_path):
    source = request(rectangle())
    result = adapter().plan(replace(source, navigable=np.zeros_like(source.free)), tmp_path)
    assert not result.success
    assert result.path == []
    assert 'empty' in result.message.lower()


def test_native_start_snap_is_disclosed_without_teleport_connection(tmp_path):
    source = request(rectangle(), start=(.225, .225))
    result = adapter().plan(source, tmp_path)
    assert result.success, (result.message, result.details)
    assert result.details['start_snap_distance_m'] > .4
    assert result.path[0] == pytest.approx(result.details['native_start_world'])
    assert result.path[0] != list(source.start)
    assert result.details['start_connection_added'] is False
    assert result.details['start_snap_offset_m'] == pytest.approx([
        result.path[0][0] - .225, result.path[0][1] - .225])


def test_native_inset_erasure_is_explicit_failure_with_stage(tmp_path):
    mask = np.zeros((20, 20), dtype=bool)
    mask[3:17, 3:17] = True
    result = adapter().plan(request(mask, start=(.525, .525)), tmp_path)
    assert not result.success
    assert result.path == []
    assert result.details['native_invoked'] is True
    assert result.details['native_stage'] == 'contour_extraction'
    assert result.details['native_returncode'] != 0
    assert result.details['error']


def test_real_native_timeout_is_failure(tmp_path, monkeypatch):
    module = adapter()
    monkeypatch.setattr(module, 'RUN_TIMEOUT_SECONDS', 1e-9)
    result = module.plan(request(rectangle()), tmp_path)
    assert not result.success
    assert result.path == []
    assert result.details['native_invoked'] is True
    assert result.details['timeout_seconds'] == 1e-9
    assert 'TimeoutExpired' in result.details['error']


def test_navigable_mask_not_raw_free_is_native_input(tmp_path):
    source = request(rectangle(), start=(1.025, 2.025))
    navigable = source.navigable.copy()
    navigable[:, 55:] = False
    assert source.free[40, 70]  # This part of raw target must not enter native input.
    result = adapter().plan(replace(source, navigable=navigable), tmp_path)
    assert result.success, (result.message, result.details)
    assert max(point[0] for point in result.path) < 2.75
    payload = (tmp_path / 'bcd_navigable.pgm').read_bytes().split(b'\n', 3)[3]
    assert payload[40 * 100 + 70] == 0
    assert payload[40 * 100 + 20] == 255
