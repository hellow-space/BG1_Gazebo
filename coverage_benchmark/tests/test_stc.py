from pathlib import Path
import numpy as np
from coverage_benchmark.models import PlanningInput
from coverage_benchmark.adapters.stc import plan, make_grid
from coverage_benchmark.metrics import evaluate_path

ROOT = Path(__file__).resolve().parents[2]

def test_pooling_blocks_holes_and_partial_tiles():
    image = np.ones((49, 51), dtype=bool)
    image[6, 6] = False
    fine, step = make_grid(image, .05, .6)
    assert step == 12 and fine.shape == (6, 6)
    assert not fine[0, 0] and not fine[4:, :].any() and not fine[:, 4:].any()

def test_native_rectangle_full_path_and_upstream_unchanged(tmp_path):
    source = ROOT/'third_party/python_robotics/PathPlanning/SpiralSpanningTreeCPP/spiral_spanning_tree_coverage_path_planner.py'
    before = source.read_bytes()
    free = np.ones((96, 120), dtype=bool)
    request = PlanningInput('rectangle', free, free, .05, (2., 2.), .6, 0., ROOT, ROOT)
    result = plan(request, tmp_path)
    assert result.success, result.message
    assert len(result.path) > 20
    metrics = evaluate_path(result.path, free, free, .05, .6, result.success)
    assert metrics['path_valid']
    assert metrics['coverage_percent'] > 80
    assert source.read_bytes() == before
