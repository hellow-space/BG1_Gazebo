"""Audit saved selection and reproduce geometric filters without modifying sources."""
from pathlib import Path
import hashlib
import json
import math
import cv2
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle

out = Path(__file__).resolve().parent
root = out.parents[3]
report = json.loads((out/'result.json').read_text())
source = root/report['source_image']
assert hashlib.sha256(source.read_bytes()).hexdigest() == report['source_sha256']
raw = np.asarray(Image.open(source).convert('L'))
free = raw == 255
h, w = free.shape
r = report['resolution']
vertices = np.array([[int(float(np.float32(x))/r), int(float(np.float32(y))/r)]
                     for x, y in report['region']], dtype=np.int32)
selection_lower = np.zeros((h, w), np.uint8)
cv2.fillPoly(selection_lower, [vertices], 1)
selection = np.flipud(selection_lower).copy().astype(bool)
assert not np.any(free & ~selection)
selected = (free & selection).astype(np.uint8)*255
inflation = math.ceil(report['clearance_m']/r)
distance = cv2.distanceTransform(selected, cv2.DIST_L2, 5)
cost = np.where((selected != 0) & (distance > inflation), 0, 254).astype(np.uint8)
usable = (cost == 0).astype(np.uint8)*255
contours, _ = cv2.findContours(usable.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
boundary = cv2.approxPolyDP(max(contours, key=cv2.contourArea), .5, True)
polygon = np.zeros((h, w), np.uint8)
cv2.fillPoly(polygon, [boundary], 1)
task = np.full((h, w), 255, np.uint8)
min_x, min_y = boundary.reshape(-1, 2).min(axis=0)
max_x, max_y = boundary.reshape(-1, 2).max(axis=0)
task[min_y:max_y+1, min_x:max_x+1] = 254
task[polygon != 0] = cost[polygon != 0]
retained = task == 0

# Geometric reproduction of the coarse sampling rule. It does not reproduce
# the complete planner's polygon/world-coordinate round trip or traversal.
spacing = int(math.floor(report['stripe_spacing_m']/r + .5))
num_x, num_y = (int((max_x-min_x)/spacing), int((max_y-min_y)/spacing))
start_x = (min_x+max_x)/2 - (num_x-1)*spacing/2
start_y = (min_y+max_y)/2 - (num_y-1)*spacing/2
candidates = []
for iy in range(num_y):
    for ix in range(num_x):
        x, y = int(math.floor(start_x+ix*spacing+.5)), int(math.floor(start_y+iy*spacing+.5))
        radius = spacing//2
        block = task[y-radius:y+radius+1, x-radius:x+radius+1]
        if np.any((block >= 200) & (block < 255)):
            continue
        local = (block == 0) & (polygon[y-radius:y+radius+1, x-radius:x+radius+1] != 0)
        yy, xx = np.where(local)
        if len(xx):
            best = np.argmin(abs(xx-radius)+abs(yy-radius))
            candidates.append((x+int(xx[best])-radius, y+int(yy[best])-radius))
fully_free_window = cv2.erode(retained.astype(np.uint8),
                             np.ones((2*(spacing//2)+1, 2*(spacing//2)+1), np.uint8))

count, labels, stats, _ = cv2.connectedComponentsWithStats(usable, connectivity=8)
component_areas = sorted((int(v)*r*r for v in stats[1:, cv2.CC_STAT_AREA]), reverse=True)
sx = int(report['start'][0]/r)
sy = h-1-int(report['start'][1]/r)
main_label = int(labels[sy, sx])
assert main_label > 0

sections = []
specs = [('A', 'lower-left vertical', 23., 15., 'horizontal'),
         ('B', 'lower-middle vertical', 60., 25., 'horizontal'),
         ('C', 'upper-left vertical', 18.8, 62., 'horizontal'),
         ('D', 'upper-middle vertical', 52., 60., 'horizontal'),
         ('E', 'upper horizontal connector', 40., 48., 'vertical'),
         ('F', 'upper T crossbar', 50., 66., 'vertical')]
for name, description, wx, wy, axis in specs:
    x, y = int(wx/r), h-1-int(wy/r)
    assert free[y, x]
    values = free[y, :] if axis == 'horizontal' else free[:, x]
    at = x if axis == 'horizontal' else y
    lo = hi = at
    while lo > 0 and values[lo-1]:
        lo -= 1
    while hi < len(values)-1 and values[hi+1]:
        hi += 1
    if axis == 'horizontal':
        endpoints = [[lo*r, (h-y-.5)*r], [(hi+1)*r, (h-y-.5)*r]]
        target_x, target_y = (lo+hi)//2, y
    else:
        endpoints = [[(x+.5)*r, (h-hi-1)*r], [(x+.5)*r, (h-lo)*r]]
        target_x, target_y = x, (lo+hi)//2
    cx, cy = (target_x+.5)*r, (h-target_y-.5)*r
    nearby_candidates = sum(abs((px+.5)*r-cx) <= 1 and abs((h-py-.5)*r-cy) <= 1
                            for px, py in candidates)
    nearby_window_centers = int(fully_free_window[max(0,target_y-20):target_y+21,
                                                 max(0,target_x-20):target_x+21].sum())
    sections.append(dict(id=name, description=description, axis=axis,
                         width_pixels=hi-lo+1, width_m=(hi-lo+1)*r,
                         endpoints_m=endpoints, center_pixel=[target_x, target_y],
                         center_m=[cx, cy],
                         survives_1m_inflation=bool(usable[target_y, target_x]),
                         connected_to_start_after_inflation=bool(labels[target_y, target_x] == main_label),
                         retained_in_task=bool(retained[target_y, target_x]),
                         geometric_coarse_candidates_in_2m_square=int(nearby_candidates),
                         fully_free_window_centers_in_2m_square=nearby_window_centers))

# Maximum bottleneck clearance from start to each section midpoint, on an
# 8-connected raster. This is a disk-centre connectivity diagnostic, not an
# orientation-dependent footprint feasibility proof.
exact = distance_transform_edt(selected != 0)
for section in sections:
    tx, ty = section['center_pixel']
    low, high = 0., min(float(exact[sy, sx]), float(exact[ty, tx]))
    for _ in range(13):
        mid = (low+high)/2
        _, ll = cv2.connectedComponents((exact > mid).astype(np.uint8), connectivity=8)
        if ll[sy, sx] and ll[sy, sx] == ll[ty, tx]:
            low = mid
        else:
            high = mid
    section['widest_path_bottleneck_radius_m_8connected'] = low*r

audit = dict(source_sha256=report['source_sha256'], resolution_assumed_m=r,
             selection_world_m=report['region'], selection_pixels_lower=vertices.tolist(),
             known_free_pixels=int(free.sum()), known_free_pixels_outside_selection=int((free & ~selection).sum()),
             inflation_radius_pixels=inflation, coarse_spacing_pixels=spacing,
             coarse_test_window_pixels=2*(spacing//2)+1,
             free_pixels_after_inflation=int((usable != 0).sum()),
             components_after_inflation=count-1,
             component_areas_m2_descending=component_areas,
             task_free_pixels_after_largest_contour=int(retained.sum()),
             geometric_coarse_candidates=len(candidates), sections=sections,
             limitations=['Widths are local axis-aligned contiguous white-pixel cross sections.',
                          'Bottleneck radii use 8-connected free pixel centres and exact EDT; no robot orientation model.',
                          'Coarse sampling is an offline geometric reproduction, not the full C++ planner execution.',
                          '0.05 m/pixel inferred from companion PGM; no original PNG YAML metadata.',
                          'No simulation or motion executed.'])
(out/'passage_analysis.json').write_text(json.dumps(audit, indent=2)+'\n')

fig, axes = plt.subplots(1, 2, figsize=(23, 9), layout='constrained')
extent=[0, w*r, 0, h*r]
axes[0].imshow(raw, origin='upper', extent=[0,w*r,0,h*r], cmap='gray', vmin=0,vmax=255,interpolation='nearest')
# imshow extent requires the top image row to map to the greatest map y.
rx, ry = zip(*(report['region']+[report['region'][0]]))
axes[0].plot(rx,ry,color='#e6a000',lw=2.2,ls='--',label='Actual requested selection')
path=np.array(report['path']);axes[0].plot(path[:,0],path[:,1],color='#008c9d',lw=.65,label='Saved Leader path')
for s in sections:
    p=np.array(s['endpoints_m']);axes[0].plot(p[:,0],p[:,1],color='#d2167c',lw=2.5,marker='|')
    cx,cy=s['center_m'];dx,dy={'A':(4,0),'B':(5,0),'C':(-14,1),'D':(5,0),'E':(-8,5),'F':(2,5)}[s['id']]
    axes[0].annotate(f'{s["id"]}: {s["width_m"]:.2f} m',(cx,cy),xytext=(cx+dx,cy+dy),
                     fontsize=11,color='#9e0759',fontweight='bold',bbox=dict(facecolor='white',alpha=.9,edgecolor='none',pad=2),
                     arrowprops=dict(arrowstyle='-',color='#9e0759'))
axes[0].set_title('Full requested rectangle + measured local cross sections\nAll 645,425 white pixels are inside the selection')
axes[0].legend(loc='lower right',fontsize=9)

stages=np.zeros((h,w),np.uint8)
stages[free]=1
stages[usable != 0]=2
stages[retained]=3
axes[1].imshow(stages,origin='upper',extent=extent,cmap=ListedColormap(['#808080','#f5cfd5','#9270b1','#43b09a']),vmin=0,vmax=3,interpolation='nearest')
axes[1].plot(rx,ry,color='#e6a000',lw=2,ls='--')
for s in sections:
    cx,cy=s['center_m'];axes[1].text(cx,cy,s['id'],fontsize=12,color='black',weight='bold',ha='center',va='center',bbox=dict(facecolor='white',edgecolor='none',alpha=.85,pad=2))
axes[1].set_title('Reproduced 1 m inflation + largest-contour task selection\nPink: removed by inflation | Purple: excluded | Green: retained')
for ax in axes:
    ax.set_xlim(0,w*r);ax.set_ylim(0,h*r);ax.set_aspect('equal');ax.set_xlabel('Local map x (m)');ax.set_ylabel('Local map y (m)')
fig.suptitle('Selection and passage audit | 0.05 m/pixel assumed | local widths do not prove full-route robot feasibility',fontsize=13)
fig.savefig(out/'passage_analysis.png',dpi=160);plt.close(fig)
print(json.dumps({k:v for k,v in audit.items() if k!='component_areas_m2_descending'},indent=2))
