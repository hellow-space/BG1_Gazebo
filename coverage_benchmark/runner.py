"""Reproducible offline comparison; no navigation or robot motion."""
import csv
import hashlib
import importlib
import json
from pathlib import Path
import subprocess
import textwrap
import numpy as np
from PIL import Image
import yaml
from coverage_benchmark.map_io import load_map
from coverage_benchmark.models import PlanningInput
from coverage_benchmark.metrics import evaluate_path, sweep_mask, segment_validity

ALGORITHMS = ('leader', 'bcd', 'stc')


def plot_scene(source, records, map_data, destination, tool_width):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    image = np.asarray(Image.open(source).convert('L'))
    r = map_data.resolution
    h, w = image.shape
    extent = (0, w*r, 0, h*r)
    fig, axes = plt.subplots(2, 3, figsize=(17, 12), constrained_layout=True)
    for column, record in enumerate(records):
        path = np.asarray(record['path'])
        metrics = record['metrics']
        for ax in axes[:, column]:
            ax.imshow(image, cmap='gray', vmin=0, vmax=255, extent=extent, origin='upper')
            ax.set_aspect('equal')
            ax.set_xlabel('x (m)')
            ax.set_ylabel('y (m)')
        length = metrics['path_length_m']
        title = f"{record['algorithm'].upper()} | {metrics['coverage_percent']:.2f}% | "
        title += f'{length:.2f} m' if length is not None else 'no path'
        verdict = 'PASS (raster geometry)' if metrics['path_valid'] else ('FAIL (raster geometry)' if length is not None else 'NO PATH')
        axes[0,column].set_title(title+'\n'+verdict)
        if record.get('details',{}).get('native_memory_audit',{}).get('status')=='failed':
            axes[0,column].set_title(axes[0,column].get_title()+'\nNative memory audit: FAILED',color='#a01c1c')
        axes[1,column].set_title('Swept area on original known-free pixels')
        if path.size:
            segments = np.stack([path[:-1], path[1:]], axis=1)
            valid = segment_validity(path, np.flipud(map_data.navigable), r)
            axes[0,column].add_collection(LineCollection(segments, colors=np.where(valid, '#1673cc', '#ef2525'), linewidths=.55))
            axes[0,column].plot(path[0,0],path[0,1],'o',color='#00ad55',markersize=4)
            swept = sweep_mask(path, image.shape, r, tool_width) & np.flipud(map_data.free)
            overlay = np.zeros((*image.shape,4))
            overlay[swept] = [0, .6, .25, .8]
            axes[1,column].imshow(overlay, extent=extent, origin='lower')
        else:
            axes[0,column].text(.5,.5,textwrap.fill(record['message'],width=40),transform=axes[0,column].transAxes,
                                ha='center',va='center',fontsize=8,in_layout=False,
                                bbox={'facecolor':'white','alpha':.9})
    fig.suptitle(source.name+'\nBlue: path; red: common clearance audit failure; green dot: returned start',fontsize=12)
    fig.savefig(destination,dpi=150)
    plt.close(fig)


def versions(source_root, runtime_root=None):
    result = {}
    for name in ('bcd_planner','python_robotics'):
        directory = source_root/'third_party'/name
        result[name] = subprocess.check_output(['git','-C',str(directory),'rev-parse','HEAD'],text=True).strip()
    for name, relative in {
        'bcd_main': 'third_party/bcd_planner/main.cpp',
        'stc_source': 'third_party/python_robotics/PathPlanning/SpiralSpanningTreeCPP/spiral_spanning_tree_coverage_path_planner.py',
    }.items():
        result[name+'_sha256'] = hashlib.sha256((source_root/relative).read_bytes()).hexdigest()
    if runtime_root is not None:
        manifest = runtime_root/'leader_coverage_ws/build/bg1_leader_coverage/generated/source_hashes.json'
        source_hashes = json.loads(manifest.read_text())
        for relative, digest in source_hashes.items():
            actual = hashlib.sha256((source_root/'third_party/leader/coverage_path_planner'/relative).read_bytes()).hexdigest()
            if actual != digest:
                raise ValueError(f'Leader build source mismatch: {relative}')
        result['leader_source_hashes'] = source_hashes
        result['leader_binary_sha256'] = hashlib.sha256((runtime_root/'leader_coverage_ws/build/bg1_leader_coverage/planner_server').read_bytes()).hexdigest()
    return result


def run(source_root, runtime_root, output, config_file, scene_filter=None, algorithm_filter=None):
    source_root, runtime_root, output = map(lambda x: Path(x).resolve(), (source_root,runtime_root,output))
    output.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load(Path(config_file).read_text())
    manifest = {'parameters':config, 'versions':versions(source_root,runtime_root), 'maps':{}, 'motion_executed':False}
    rows = []
    for scene in config['scenes']:
        if scene_filter and scene not in scene_filter:
            continue
        source = runtime_root/'data/maps/imported'/scene/(scene+'.png')
        data = load_map(source, config['resolution'], config['clearance'])
        manifest['maps'][scene] = data.metadata
        request = PlanningInput(scene, data.target, data.navigable, data.resolution, data.start,
                                config['tool_width'], config['clearance'], source_root, runtime_root)
        records = []
        for algorithm in ALGORITHMS:
            path_file = output/f'{scene}_{algorithm}.json'
            if algorithm_filter and algorithm not in algorithm_filter:
                if not path_file.exists():
                    continue
                record = json.loads(path_file.read_text())
            else:
                print(f'Planning {scene} / {algorithm}', flush=True)
                adapter = importlib.import_module(f'coverage_benchmark.adapters.{algorithm}')
                result = adapter.plan(request, source_root/'coverage_benchmark/.work'/scene/algorithm)
                metrics = evaluate_path(result.path, data.free, data.navigable, data.resolution,
                                        config['tool_width'],result.success)
                record = dict(scene=scene, algorithm=algorithm, planner_success=result.success,
                              message=result.message, path=result.path, details=result.details, metrics=metrics)
                path_file.write_text(json.dumps(record,separators=(',',':'),allow_nan=False)+'\n')
                print(json.dumps(dict(scene=scene,algorithm=algorithm,**metrics)),flush=True)
            records.append(record)
            rows.append(dict(scene=scene,algorithm=algorithm,**{key:record['metrics'][key] for key in
                             ('coverage_percent','path_length_m','path_valid')}))
        if len(records)==3:
            plot_scene(source,records,data,output/(scene+'_comparison.png'),config['tool_width'])
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    with (output/'summary.csv').open('w',newline='') as stream:
        writer = csv.DictWriter(stream,fieldnames=['scene','algorithm','coverage_percent','path_length_m','path_valid'],lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    return rows
