"""Apply the same independent geometric metrics to saved planner paths."""
from pathlib import Path
import csv
import json
import statistics
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap
from metrics import path_length, sweep_mask, segment_validity

out = Path(__file__).resolve().parent
config = json.loads((out/'inputs.json').read_text())
data = np.load(out/'inputs.npz')
free, target, allowed = data['free'], data['target'], data['allowed']
r = config['resolution_m']; tool = config['operation_width_m']
denominators = dict(all_free=int(free.sum()), target=int(target.sum()))
summary = dict(map_id=config['map_id'], operation_width_m=tool, clearance_m=config['clearance_m'],
               all_free_area_m2=float(free.sum()*r*r), target_area_m2=float(target.sum()*r*r),
               common_target_fraction_of_all_free=config['target_fraction_of_all_free'],
               metric_definitions={
                   'geometric_coverage':'Union of radius width/2 capsules around ALL returned segments, intersected with free pixels; ignores traversal validity.',
                   'valid_fragment_coverage':'Union of capsules only around segments entirely inside the common exact-EDT clearance raster (>20 pixels from non-target cell centres). Fragments are not a connected executable route.',
                   'full_route_valid':'Every returned segment passes the same exhaustive raster-cell crossing check. Allowed-cell centres satisfy the distance threshold; this is NOT a continuous geometric or footprint/robot execution guarantee.',
                   'raster_method':'Exact point-to-segment distances at 0.05 m raster cell centres; no double counting.',
                   'time':'One warmup excluded; median of three end-to-end ROS calls; geometry conversion and process startup excluded.'},
               motion_executed=False, planners={})
plot_data={}
for name in ('leader','f2c'):
    raw=json.loads((out/(name+'_raw.json')).read_text())
    if not raw['completed']:
        summary['planners'][name]=dict(interface_success=False,error=raw.get('error'),runs=raw['runs'])
        continue
    path=np.asarray(raw['path']);xy=path[:,:2]
    lengths=np.linalg.norm(np.diff(xy,axis=0),axis=1)
    started=time.perf_counter()
    free_valid=segment_validity(path,free,r)
    clear_valid=segment_validity(path,allowed,r)
    native_clear_valid=segment_validity(path,data['leader_allowed'],r)
    geometric=sweep_mask(path,free.shape,r,tool)
    valid_sweep=sweep_mask(path,free.shape,r,tool,clear_valid)
    bad=np.flatnonzero(~clear_valid)
    prefix_end=int(bad[0])+1 if len(bad) else len(path)
    prefix=sweep_mask(path[:prefix_end],free.shape,r,tool) if prefix_end>1 else np.zeros(free.shape,bool)
    measured=[run for run in raw['runs'] if not run['warmup']]
    metric=dict(interface_success=True,poses=len(path),path_length_m=path_length(path),
                known_free_route_valid=bool(free_valid.all()),clearance_route_valid=bool(clear_valid.all()),
                leader_style_clearance_raster_valid=bool(native_clear_valid.all()),
                segment_count=len(lengths),invalid_known_free_segments=int((~free_valid).sum()),
                invalid_clearance_segments=int((~clear_valid).sum()),
                known_free_valid_length_m=float(lengths[free_valid].sum()),
                clearance_valid_length_m=float(lengths[clear_valid].sum()),
                invalid_clearance_length_m=float(lengths[~clear_valid].sum()),
                invalid_known_free_length_m=float(lengths[~free_valid].sum()),
                clearance_valid_length_pct=float(100*lengths[clear_valid].sum()/lengths.sum()),
                call_seconds=[run['call_seconds'] for run in measured],
                call_median_s=statistics.median(run['call_seconds'] for run in measured),
                deterministic_paths=len(set(run['path_sha256'] for run in raw['runs']))==1,
                start=xy[0].tolist(),end=xy[-1].tolist(),
                requested_start_offset_m=float(np.linalg.norm(xy[0]-config['start'])),
                metrics_seconds=time.perf_counter()-started)
    for denominator, count in denominators.items():
        domain=free if denominator=='all_free' else target
        for kind,mask in [('geometric',geometric),('valid_fragment',valid_sweep),('valid_prefix',prefix)]:
            pixels=int((mask & domain).sum())
            metric[kind+'_'+denominator+'_pixels']=pixels
            metric[kind+'_'+denominator+'_area_m2']=float(pixels*r*r)
            metric[kind+'_'+denominator+'_coverage_pct']=float(100*pixels/count)
    if name=='f2c':
        metric['swaths']=measured[0]['swaths']
        metric['server_planning_median_s']=statistics.median(run['server_planning_seconds'] for run in measured)
    summary['planners'][name]=metric
    np.savez_compressed(out/(name+'_evaluated.npz'),geometric=geometric,valid_sweep=valid_sweep,
                        free_valid_segments=free_valid,clear_valid_segments=clear_valid)
    plot_data[name]=(xy,clear_valid,geometric,valid_sweep)
    print(name,json.dumps(metric,indent=2),flush=True)

(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
columns=['planner','interface_success','path_length_m','geometric_all_free_coverage_pct',
         'valid_fragment_all_free_coverage_pct','geometric_target_coverage_pct',
         'valid_fragment_target_coverage_pct','known_free_route_valid','clearance_route_valid',
         'clearance_valid_length_pct','call_median_s','poses']
with (out/'summary.csv').open('w',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=columns,lineterminator='\n');writer.writeheader()
    for name,metric in summary['planners'].items():
        writer.writerow(dict(planner=name,**{key:metric.get(key) for key in columns[1:]}))

fig,axes=plt.subplots(2,2,figsize=(16,20),layout='constrained')
extent=[0,free.shape[1]*r,0,free.shape[0]*r]
gray=np.where(free,1,0)
for column,name in enumerate(('leader','f2c')):
    ax=axes[0,column]
    ax.imshow(gray,origin='lower',extent=extent,cmap=ListedColormap(['#858585','#f7f7f7']),interpolation='nearest',vmin=0,vmax=1)
    if name not in plot_data:
        ax.set_title(name+' failed');continue
    xy,valid,geometric,safe=plot_data[name]
    segments=np.stack([xy[:-1],xy[1:]],axis=1)
    ax.add_collection(LineCollection(segments[valid],colors='#008b9b',linewidths=.6))
    ax.add_collection(LineCollection(segments[~valid],colors='#df443a',linewidths=.9))
    ax.scatter(*xy[0],color='#1b9e42',s=35,zorder=4);ax.scatter(*xy[-1],color='#2020bb',s=35,zorder=4)
    metric=summary['planners'][name]
    ax.set_title(f'{"Leader" if name=="leader" else "Fields2Cover / opennav"}\n{metric["path_length_m"]:.1f} m | teal/red: exact-EDT raster pass/fail')
    coverage=np.zeros(free.shape,np.uint8)
    coverage[free]=1;coverage[geometric & free]=2;coverage[safe & free]=3
    axes[1,column].imshow(coverage,origin='lower',extent=extent,
        cmap=ListedColormap(['#858585','#fff4df','#ed9b69','#2f9d86']),vmin=0,vmax=3,interpolation='nearest')
    axes[1,column].set_title(f'Coverage over ALL original free space\nGeometric {metric["geometric_all_free_coverage_pct"]:.2f}% | valid fragments {metric["valid_fragment_all_free_coverage_pct"]:.2f}%')
for ax in axes.flat:
    ax.set_xlim(0,extent[1]);ax.set_ylim(0,extent[3]);ax.set_aspect('equal')
    ax.set_xlabel('Local map x (m)');ax.set_ylabel('Local map y (m)')
fig.suptitle('A11123AJ04 PNG | same connected target, 0.6 m tool/spacing, 1 m clearance audit\nBottom: green = valid-fragment sweep; orange = geometric-only; cream = missed. Planning only.',fontsize=13)
fig.savefig(out/'comparison.png',dpi=150);plt.close(fig)
