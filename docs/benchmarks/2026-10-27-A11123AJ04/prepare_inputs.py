"""Prepare identical map target and exact polygon input for the two pipelines."""
from pathlib import Path
import hashlib
import json
import subprocess
import time
import cv2
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from raster_geometry import polygonize, rasterize
from metrics import clearance_mask

out = Path(__file__).resolve().parent
root = out.parents[2]
source = root/'data/maps/imported/A11123AJ04_231127_094529/A11123AJ04_231127_094529.png'
started = time.perf_counter()
pixels = np.asarray(Image.open(source).convert('L'))
occupancy_probability = (255-pixels.astype(float))/255
free = occupancy_probability < .25
occupied = occupancy_probability > .65
resolution = float(np.float32(.05))
n, labels, stats, _ = cv2.connectedComponentsWithStats(free.astype(np.uint8), connectivity=4)
largest = 1+int(stats[1:, cv2.CC_STAT_AREA].argmax())
target = labels == largest
polygon_started = time.perf_counter()
polygons = polygonize(target, resolution)
assert len(polygons) == 1, 'Common connected target must have one polygon exterior'
assert np.array_equal(rasterize(polygons, target.shape, resolution), target)
assert abs(polygons[0]['area_m2']-target.sum()*resolution**2) < 1e-6
(out/'target_polygon.json').write_text(json.dumps(polygons[0], separators=(',', ':'))+'\n')
polygon_seconds = time.perf_counter()-polygon_started
distance = cv2.distanceTransform(target.astype(np.uint8)*255, cv2.DIST_L2, 5)
leader_allowed = distance > 20
allowed = clearance_mask(target, resolution, 1.)
sy, sx = np.unravel_index(distance_transform_edt(target).argmax(), target.shape)
h, w = target.shape
np.savez_compressed(out/'inputs.npz', free=np.flipud(free), target=np.flipud(target),
                    allowed=np.flipud(allowed), leader_allowed=np.flipud(leader_allowed), original_pixels=pixels)
Image.fromarray(np.where(target,255,0).astype(np.uint8)).save(out/'target_map.png')
hashes = json.loads((root/'leader_coverage_ws/install/bg1_leader_coverage/share/bg1_leader_coverage/source_hashes.json').read_text())
assert all(hashlib.sha256((root/'third_party/leader/coverage_path_planner'/p).read_bytes()).hexdigest()==v for p,v in hashes.items())
versions = json.loads((root/'coverage_ws/docs/versions.json').read_text())
for name in versions:
    commit = subprocess.check_output(['git','rev-parse','HEAD'],cwd=root/'third_party'/name,text=True).strip()
    assert commit == versions[name]['commit']
    assert not subprocess.check_output(['git','status','--porcelain'],cwd=root/'third_party'/name,text=True).strip()
binaries = {'leader': root/'leader_coverage_ws/build/bg1_leader_coverage/planner_server',
            'f2c': root/'coverage_ws/build/opennav_coverage/opennav_coverage',
            'f2c_library': root/'coverage_ws/install/fields2cover/lib/libFields2Cover.so'}
data = dict(map_id=source.stem, source_image=str(source.relative_to(root)),
            source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(), width=w,height=h,
            resolution_m=resolution, origin=[0.,0.,0.],
            metadata_assumptions=['0.05 m/pixel inferred from companion PGM header, original PNG YAML missing',
                                  'Arbitrary local origin [0,0,0]; trinary free<0.25, occupied>0.65'],
            free_pixels=int(free.sum()), occupied_pixels=int(occupied.sum()),
            unknown_pixels=int((~free & ~occupied).sum()),
            target_pixels=int(target.sum()), target_fraction_of_all_free=float(target.sum()/free.sum()),
            original_free_components_4connected=n-1,
            polygon_rings=len(polygons[0]['geometry']['coordinates']),
            polygon_vertices=sum(len(ring) for ring in polygons[0]['geometry']['coordinates']),
            polygon_valid=polygons[0]['valid'], polygon_area_m2=polygons[0]['area_m2'],
            polygon_raster_roundtrip_equal=True, polygonize_seconds=polygon_seconds,
            start=[float((sx+.5)*resolution), float((h-sy-.5)*resolution)],
            operation_width_m=.6,stripe_spacing_m=.6,clearance_m=1.,
            clearance_audit='Independent exact EDT > ceil(clearance/resolution) in pixels, measured between raster cell centres; no continuous footprint claim',
            f2c_headland_m=1.,f2c_min_turning_radius_m=.6,f2c_swath_angle_rad=float(np.pi/2),
            warmup_runs=1,measured_runs=3,versions=versions,leader_source_hashes=hashes,
            binary_sha256={name:hashlib.sha256(path.read_bytes()).hexdigest() for name,path in binaries.items()},
            preprocessing_seconds=time.perf_counter()-started,
            motion_executed=False)
(out/'inputs.json').write_text(json.dumps(data,indent=2)+'\n')
print(json.dumps({k:v for k,v in data.items() if k not in ('leader_source_hashes','binary_sha256','versions')},indent=2))
