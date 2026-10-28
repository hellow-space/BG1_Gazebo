"""Conservative raster adapter; calls the pinned upstream Spiral-STC unchanged."""
import hashlib
import importlib.util
import sys
import numpy as np
from coverage_benchmark.models import PlanningResult
from coverage_benchmark.map_io import world_to_pixel


def make_grid(navigable, resolution, tool_width):
    step = round(tool_width / resolution)
    if step < 1 or abs(step * resolution - tool_width) > 1e-6:
        raise ValueError('STC tool width must be an integer number of map pixels')
    h, w = navigable.shape
    fh, fw = ((h + step - 1)//step + 1)//2*2, ((w + step - 1)//step + 1)//2*2
    padded = np.zeros((fh*step, fw*step), dtype=bool)
    padded[:h, :w] = navigable
    fine = padded.reshape(fh, step, fw, step).all(axis=(1, 3))
    return fine, step


def plan(request, work_dir):
    source = request.source_root/'third_party/python_robotics/PathPlanning/SpiralSpanningTreeCPP/spiral_spanning_tree_coverage_path_planner.py'
    details = {'upstream_sha256': hashlib.sha256(source.read_bytes()).hexdigest()}
    old_limit = sys.getrecursionlimit()
    try:
        fine, step = make_grid(request.navigable, request.resolution, request.tool_width)
        coarse = fine.reshape(fine.shape[0]//2, 2, fine.shape[1]//2, 2).all(axis=(1, 3))
        candidates = np.argwhere(coarse)
        details.update(fine_shape=list(fine.shape), fine_cell_pixels=step,
                       fine_cell_m=step*request.resolution, coarse_cell_m=2*step*request.resolution,
                       navigable_pixels=int(request.navigable.sum()),
                       represented_fine_pixels=int(fine.sum()*step**2),
                       valid_coarse_cells=int(coarse.sum()))
        if not len(candidates):
            return PlanningResult(message='No complete free 2x2 STC coarse cell', details=details)
        row, col = world_to_pixel(request.start, request.free.shape[0], request.resolution)
        requested = np.array([row/step/2, col/step/2])
        start = tuple(map(int, candidates[np.argmin(((candidates+.5-requested)**2).sum(axis=1))]))
        details['native_start_coarse'] = list(start)
        spec = importlib.util.spec_from_file_location('benchmark_native_spiral_stc', source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.do_animation = False
        sys.setrecursionlimit(max(old_limit, int(coarse.size*2+100)))
        planner = module.SpiralSpanningTreeCoveragePlanner(fine)
        edges, route, segments = planner.plan(start)
        details.update(tree_edges=len(edges), route_nodes=len(route), native_segments=len(segments))
        if not segments:
            return PlanningResult(message='Native STC returned no segments', details=details)
        # Match upstream visualize_path, including its Manhattan interpolation between segments.
        vertices = [segments[0][0]]
        last = segments[0][0]
        for p, q in segments:
            if np.linalg.norm(np.asarray(p)-last) > 1:
                rows, cols = planner.get_interpolated_path(last, p)
                vertices.extend(zip(rows, cols))
            else:
                vertices.append(p)
            vertices.append(q)
            last = q
        scale = step*request.resolution
        path = [[float((c+.5)*scale), float(request.free.shape[0]*request.resolution-(r+.5)*scale)]
                for r, c in vertices]
        details['actual_start_m'] = path[0]
        details['start_offset_m'] = float(np.linalg.norm(np.asarray(path[0])-request.start))
        return PlanningResult(path, True, 'Native Spiral-STC path with upstream connector interpolation', details)
    except (Exception, SystemExit) as error:
        return PlanningResult(message=f'{type(error).__name__}: {error}', details=details)
    finally:
        sys.setrecursionlimit(old_limit)
