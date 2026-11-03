#!/usr/bin/env python3
"""Reconstruct report data from measured evidence; do not re-plan the final route."""
import csv
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np
from PIL import Image
from scipy.ndimage import label

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT))
from coverage_benchmark.hybrid.planner import candidates, CoverageLedger
from coverage_benchmark.grid_connector import GridConnector
from coverage_benchmark.map_io import load_map
from coverage_benchmark.metrics import evaluate_path, path_length
from coverage_benchmark.ipa_connection import strict_path_valid
from coverage_benchmark.ipa_audit import path_box_audit
from coverage_benchmark.provenance import file_sha256

SCENE = 'A11123AJ04_231127_094529'
HISTORY = ROOT / 'docs/benchmarks/hybrid-local-bcd-2026-10-30'
TRACE = HERE / 'evidence/native_trace'


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, separators=(',', ':'), allow_nan=False) + '\n')


def main():
    manifest = json.loads((HISTORY / 'manifest.json').read_text())
    for p, digest in manifest['source_sha256'].items():
        assert file_sha256(ROOT / p) == digest, f'Current planner source differs from saved run: {p}'
    record_path = HISTORY / f'{SCENE}_ipa_bcd.json'
    assert file_sha256(record_path) == manifest['artifacts_sha256'][record_path.name]
    record = json.loads(record_path.read_text())
    trace = json.loads((TRACE / 'trace.json').read_text())
    raw = json.loads((TRACE / 'output.json').read_text())
    m = load_map(ROOT / f'data/maps/imported/{SCENE}/{SCENE}.png', .05, 1.)
    assert m.metadata == record['input']
    r = m.resolution
    allowed = np.flipud(m.navigable)
    components, _ = label(allowed)
    x, y = np.floor(np.asarray(m.start) / r).astype(int)
    f = components == components[y, x]
    saved = np.load(record['details']['partition_file'])
    assert np.array_equal(f, saved['f'])
    assert np.array_equal(np.asarray(Image.open(record['details']['decomposition']['evidence']['request']['allowed'])) == 255, f)
    native_labels = np.asarray(Image.open(TRACE / '04_labels_final.png'))
    R = np.asarray(trace['rotations'][0]['R'])
    inverse = cv2.invertAffineTransform(R)
    path = np.asarray(record['path'])
    grouped = defaultdict(list)
    for section in record['details']['sections']:
        grouped[section['source']['region']].append(section)
    schedule = record['details']['schedule']
    order = {item['region']: i + 1 for i, item in enumerate(schedule)}
    cell_data, pieces = [], []
    for i, cell in enumerate(raw['cells']):
        value, rejected = candidates(cell, f, r)
        assert rejected == 0
        pieces.append(value)
        sweep = trace['sweeps'][i]
        vertex = sweep['polygon_rotated'][0]
        assert int(native_labels[vertex[1], vertex[0]]) == i + 1
        local_inverse = cv2.invertAffineTransform(np.asarray(sweep['R_cell']))
        grid = []
        for line in sweep['grid']:
            for key in ('upper', 'lower'):
                if line[key]:
                    local_points = np.asarray(line[key], np.float32)
                    rotated = cv2.transform(local_points[None], local_inverse)[0]
                    original = cv2.transform(rotated[None], inverse)[0]
                    grid.append(((original + .5) * r).tolist())
        sections = grouped[i]
        first = sections[0]['first_vertex'] if sections else None
        last = sections[-1]['last_vertex'] if sections else None
        raw_path = cell['path']
        status = '已访问' if i in order else ('原生无扫线' if not raw_path else '原生有目标，安全裁剪后无可用段')
        native_area_px = round(cell['area_m2'] / r**2)
        bbox = np.asarray(cell['polygon'])
        cell_data.append(dict(id=i, label=i+1, name=f'C{i:02}', area_m2=cell['area_m2'],
            native_area_pixels=native_area_px, native_label_pixels=int((native_labels == i+1).sum()),
            polygon=cell['polygon'], raw_path=raw_path, raw_points=len(raw_path), raw_length_m=path_length(raw_path),
            safe_pieces=value, safe_piece_count=len(value), safe_length_m=sum(path_length(p) for p in value),
            grid=grid, grid_rows=len(sweep['grid']),
            grid_points=sum(len(v) for line in sweep['grid'] for v in [line['upper'], line['lower']]),
            rotated_white_pixels=sweep['rotated_white_pixels'],
            local_angle_deg=trace['rotations'][i+1]['angle_rad'] * 180 / math.pi,
            chosen_corner=sweep.get('chosen_corner'), corner_costs=sweep.get('corner_costs', []),
            corners_rotated=sweep.get('corners_rotated', []),
            status=status, visit_order=order.get(i), first_vertex=first, last_vertex=last,
            visible_sections=sum(s['method']=='visible' for s in sections),
            astar_sections=sum(s['method']=='astar' for s in sections),
            final_length_m=path_length(path[first:last+1]) if first is not None else 0,
            center=((bbox.min(axis=0)+bbox.max(axis=0))/2).tolist(),
            bbox=[*bbox.min(axis=0).tolist(), *bbox.max(axis=0).tolist()],
            native_bbox=list(cv2.boundingRect(np.asarray(sweep['polygon_rotated'], np.int32)))))
    print('Candidates reconstructed; replaying all 61 scheduling decisions', flush=True)
    connector = GridConnector(f, r)
    pending = {i for i, values in enumerate(pieces) if values}
    schedule_trace = []
    ledger = CoverageLedger(f.shape, r, .6)
    ledger.add([m.start])
    for step, item in enumerate(schedule):
        rid = item['region']
        cell = cell_data[rid]
        here = path[cell['first_vertex']]
        options = []
        # Reconstruct the original pending order from its cell index order.
        for i in sorted(pending):
            values = pieces[i]
            options.extend([(i, values[0][0], False), (i, values[-1][-1], True)])
        options.sort(key=lambda option: np.linalg.norm(np.asarray(option[1]) - here))
        costs = []
        for i, point, reverse in options[:8]:
            route = connector.connect(here, point)
            assert strict_path_valid(route, f, r)
            costs.append(dict(region=i, reverse=reverse, point=point,
                              euclidean_m=float(np.linalg.norm(np.asarray(point)-here)),
                              length_m=path_length(route), route=route))
        winner = min(costs, key=lambda c:(c['length_m'], c['region'], c['reverse']))
        assert (winner['region'], winner['reverse']) == (rid, item['reverse'])
        assert winner['length_m'] == item['approach_length_m']
        values = pieces[rid]
        if item['reverse']:
            values = [v[::-1] for v in values[::-1]]
        targets = np.asarray([p for pair in values for p in pair])
        sections = grouped[rid]
        assert len(targets) == len(sections)
        assert np.array_equal(path[[s['last_vertex'] for s in sections]], targets)
        first, last = cell['first_vertex'], cell['last_vertex']
        gain = ledger.add(path[first:last+1])
        cell.update(sweep_gain_all_pixels=gain, approach_length_m=winner['length_m'], reverse=item['reverse'])
        schedule_trace.append(dict(step=step+1, region=rid, reverse=item['reverse'], here=here.tolist(),
            candidates=costs, first_vertex=first, last_vertex=last, gain_all_pixels=gain))
        pending.remove(rid)
        if (step+1) % 10 == 0:
            print(f'  scheduling {step+1}/61', flush=True)
    assert not pending
    assert connector.astar_calls + sum(c['astar_sections'] for c in cell_data) == record['details']['astar_calls']
    print('Recomputing final coverage and safety', flush=True)
    metric = evaluate_path(path, m.free, m.navigable, r, .6, True)
    assert metric == record['metrics']
    assert strict_path_valid(path, f, r) and path_box_audit(path, f, r)
    covered = ledger.covered
    assert int((f & ~covered).sum()) == record['details']['residual_pixels']
    assert int((np.flipud(m.free) & covered).sum()) == metric['diagnostics']['covered_pixels']
    events = []
    for i, event in enumerate(trace['events']):
        cuts = []
        for x0, yy, x1, _ in event['cuts']:
            points = cv2.transform(np.array([[[x0, yy], [x1, yy]]], np.float32), inverse)[0]
            cuts.append(((points+.5)*r).tolist())
        critical = np.asarray(event['critical_points'], np.float32)
        points = ((cv2.transform(critical[None], inverse)[0]+.5)*r).tolist() if len(critical) else []
        sweep_line = cv2.transform(np.array([[[0,event['y']],[1722,event['y']]]], np.float32), inverse)[0]
        events.append(dict(index=i+1, **event, cuts_world=cuts, critical_world=points,
                           row_world=((sweep_line+.5)*r).tolist()))
    sections = record['details']['sections']
    astar = [path[s['first_vertex']:s['last_vertex']+1].tolist() for s in sections if s['method']=='astar']
    astar_metadata = [dict(index=i, region=s['source']['region'], first_vertex=s['first_vertex'],
        last_vertex=s['last_vertex'], length_m=path_length(path[s['first_vertex']:s['last_vertex']+1]),
        direct_m=path_length([path[s['first_vertex']], path[s['last_vertex']]]))
        for i,s in enumerate(sections) if s['method']=='astar']
    length_parts=defaultdict(float)
    for section in sections:
        ti=section['target_index']
        kind='safe_pieces' if ti%2 else ('cell_approaches' if ti==0 else 'between_pieces')
        length_parts[kind]+=path_length(path[section['first_vertex']:section['last_vertex']+1])
    summary = dict(scene=SCENE, parameters=record['input'], tool_width_m=.6,
        metrics=metric, cells=len(cell_data), visited=len(schedule),
        empty_raw=[c['id'] for c in cell_data if not c['raw_points']],
        empty_safe=[c['id'] for c in cell_data if c['raw_points'] and not c['safe_piece_count']],
        small_cells_lt_1m2=sum(c['area_m2']<1 for c in cell_data),
        raw_points=sum(c['raw_points'] for c in cell_data), raw_length_sum_m=sum(c['raw_length_m'] for c in cell_data),
        singleton_cells=[c['id'] for c in cell_data if c['raw_points']==1],
        short_clipped_cells=[c['id'] for c in cell_data if c['raw_points']>1 and not c['safe_piece_count']],
        path_length_parts_m=dict(length_parts),
        safe_pieces=sum(c['safe_piece_count'] for c in cell_data),
        safe_length_sum_m=sum(c['safe_length_m'] for c in cell_data),
        events=len(events), effective_events=sum(bool(e['cuts']) for e in events),
        cut_runs=sum(len(e['cuts']) for e in events), native_rotation_deg=trace['rotations'][0]['angle_rad']*180/math.pi,
        event_types=dict(Counter(e['type'] for e in events)),
        route_vertices=len(path), route_sections=len(sections),
        astar_sections=len(astar), astar_candidate_searches=connector.astar_calls,
        all_astar_searches=record['details']['astar_calls'],
        feasible_pixels=int(f.sum()), covered_feasible_pixels=int((f&covered).sum()),
        residual_feasible_pixels=int((f&~covered).sum()),
        feasible_coverage_percent=100*(f&covered).sum()/f.sum(),
        excluded_after_clearance_pixels=int((np.flipud(m.navigable)&~f).sum()),
        residual_free_pixels=int((np.flipud(m.free)&~covered).sum()),
        no_local_calls=True, history_date='2026-10-30', trace_date='2026-11-02')
    data = dict(summary=summary, cells=cell_data, schedule=schedule_trace, path=path.tolist(),
                astar=astar, astar_metadata=astar_metadata, events=events,
                rotation=trace['rotations'][0], hough=trace['directions'][0])
    (HERE/'assets').mkdir(exist_ok=True)
    np.savez_compressed(HERE/'evidence/report_masks.npz', f=f, covered=covered,
                        free=np.flipud(m.free), target=np.flipud(m.target), navigable=allowed,
                        native_labels=native_labels)
    save(HERE/'evidence/report_data.json', data)
    save(HERE/'summary.json', summary)
    fields=['name','id','label','area_m2','native_area_pixels','native_label_pixels','grid_rows','grid_points',
            'raw_points','raw_length_m','safe_piece_count','safe_length_m','visit_order','reverse',
            'approach_length_m','visible_sections','astar_sections','final_length_m','status']
    with (HERE/'cells.csv').open('w',newline='',encoding='utf-8-sig') as file:
        writer=csv.DictWriter(file,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(cell_data)
    save(HERE/'evidence/schedule_candidates.json',schedule_trace)
    save(HERE/'evidence/report_verification.json',dict(
        historical_source_hashes_match=True, original_png_metadata_match=True,
        all_80_cells_match_native_labels=True, all_61_scheduler_choices_replayed=True,
        all_78930_targets_replayed=True, final_metrics_equal_saved=True,
        strict_closed_cell_audit=True, independent_pixel_box_audit=True,
        covered_mask_replayed=True, motion_executed=False,
        source_files_sha256={str(p.relative_to(ROOT)):file_sha256(p) for p in [record_path,HISTORY/'manifest.json',Path(__file__)]},
        evidence_sha256={str(p.relative_to(HERE)):file_sha256(p) for p in (HERE/'evidence').rglob('*') if p.is_file() and p.name!='report_verification.json'}))
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
