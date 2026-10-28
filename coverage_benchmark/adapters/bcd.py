"""Offline adapter for the pinned RicheyHuang native BCD static planner."""
from __future__ import annotations

import math
from pathlib import Path
import subprocess

import numpy as np

from coverage_benchmark.models import PlanningInput, PlanningResult
from .bcd_build import build

RUN_TIMEOUT_SECONDS = 120


def plan(request: PlanningInput, work_dir: Path) -> PlanningResult:
    details = {'native_invoked': False, 'native_map': 'shared navigable mask',
               'start_connection_added': False,
               'path_policy': 'all native static segments and points in original order, no repair or filtering'}
    try:
        mask = np.asarray(request.navigable, dtype=bool)
        if mask.ndim != 2 or mask.shape != request.free.shape:
            raise ValueError('navigable must be a 2-D mask matching free')
        if not mask.any():
            raise ValueError('empty navigable map')
        if not math.isfinite(request.resolution) or request.resolution <= 0:
            raise ValueError('invalid map resolution')
        width_pixels = request.tool_width / request.resolution
        spacing = max(1, int(math.floor(width_pixels + .5)))
        details.update({'requested_tool_width_pixels': width_pixels,
                        'sweep_spacing_pixels': spacing,
                        'effective_sweep_spacing_m': spacing * request.resolution,
                        'native_inset_pixels': spacing - 1,
                        'native_additional_inset_m': (spacing - 1) * request.resolution,
                        'native_inset_behavior': 'ExtractContours retains native radius circles, MORPH_OPEN and 1px polygon simplification; largest exterior contour only'})
        executable, build_details = build(Path(request.source_root))
        details.update(build_details)
        work_dir = Path(work_dir)
        work_dir.mkdir(parents=True, exist_ok=True)
        image_path = work_dir / 'bcd_navigable.pgm'
        output_path = work_dir / 'bcd_native_path.txt'
        image_path.write_bytes(f'P5\n{mask.shape[1]} {mask.shape[0]}\n255\n'.encode() + (mask.astype(np.uint8) * 255).tobytes())
        output_path.unlink(missing_ok=True)
        # Requested point -> containing image pixel; output always uses pixel centres.
        col = math.floor(request.start[0] / request.resolution)
        row = mask.shape[0] - 1 - math.floor(request.start[1] / request.resolution)
        command = [str(executable), str(image_path), str(col), str(row), str(spacing), str(output_path)]
        details.update({'native_invoked': True, 'native_command': command,
                        'requested_start_pixel': [col, row], 'native_output': str(output_path)})
        completed = subprocess.run(command, capture_output=True, text=True, timeout=RUN_TIMEOUT_SECONDS)
        (work_dir / 'bcd_native.log').write_text(completed.stdout + completed.stderr)
        details['native_returncode'] = completed.returncode
        details['native_stderr'] = completed.stderr[-4000:]
        stages = [line.split(maxsplit=1)[1] for line in completed.stderr.splitlines() if line.startswith('BCD_STAGE ')]
        if stages:
            details['native_stage'] = stages[-1]
        if completed.returncode:
            signal_note = f' signal {-completed.returncode}' if completed.returncode < 0 else ''
            raise RuntimeError(f'BCD native process failed: returncode {completed.returncode}{signal_note}; {completed.stderr[-3000:].strip()}')
        lines = output_path.read_text().splitlines()
        header = lines[0].split()
        if len(header) != 9 or header[0] != 'BCD1':
            raise ValueError('invalid BCD machine output header')
        width, height, radius, sx, sy, cells, segment_count, point_count = map(int, header[1:])
        # Header has nine fields including marker (eight integer metadata fields).
        points = []
        segment_sizes = []
        cursor = 1
        for _ in range(segment_count):
            marker, size = lines[cursor].split()
            if marker != 'SEGMENT':
                raise ValueError('invalid BCD segment marker')
            count = int(size)
            segment_sizes.append(count)
            cursor += 1
            for _ in range(count):
                x, y = map(int, lines[cursor].split())
                points.append([(x + .5) * request.resolution, (height - y - .5) * request.resolution])
                cursor += 1
        if cursor != len(lines) or len(points) != point_count or not points or not np.isfinite(points).all():
            raise ValueError('empty, nonfinite or inconsistent BCD native path')
        snapped = [(sx + .5) * request.resolution, (height - sy - .5) * request.resolution]
        offset = [snapped[0] - request.start[0], snapped[1] - request.start[1]]
        details.update({'native_point_count': point_count, 'native_segment_count': segment_count,
                        'native_segment_sizes': segment_sizes, 'native_cell_count': cells,
                        'native_map_shape': [height, width], 'native_radius': radius,
                        'native_start_pixel': [sx, sy], 'native_start_world': snapped,
                        'start_snap_offset_m': offset, 'start_snap_distance_m': math.hypot(*offset)})
        return PlanningResult(path=points, success=True, message='Native BCD static path returned; geometry requires independent audit', details=details)
    except Exception as error:
        details['error'] = f'{type(error).__name__}: {error}'
        if isinstance(error, subprocess.TimeoutExpired):
            details['timeout_seconds'] = error.timeout
        return PlanningResult(success=False, message=str(error), details=details)
