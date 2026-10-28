from dataclasses import dataclass
from pathlib import Path
import hashlib
import math
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, label


@dataclass(frozen=True)
class MapData:
    free: np.ndarray
    target: np.ndarray
    navigable: np.ndarray
    resolution: float
    start: tuple[float, float]
    metadata: dict


def load_map(path, resolution, clearance):
    path=Path(path)
    if resolution <= 0 or clearance < 0:
        raise ValueError('Invalid map resolution or clearance')
    resolution=float(np.float32(resolution))
    pixels=np.asarray(Image.open(path).convert('L'))
    probability=(255-pixels.astype(float))/255
    free=probability < .25
    components,count=label(free)
    if count == 0:
        raise ValueError('No known free pixels in input image')
    sizes=np.bincount(components.ravel());sizes[0]=0
    target=components==sizes.argmax()
    distance=distance_transform_edt(np.pad(target,1,constant_values=False))[1:-1,1:-1]
    navigable=distance > math.ceil(clearance/resolution)
    row,col=np.unravel_index(distance.argmax(),distance.shape)
    start=tuple(pixel_to_world((row,col),free.shape[0],resolution))
    metadata=dict(source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  source_name=path.name,width=pixels.shape[1],height=pixels.shape[0],
                  free_pixels=int(free.sum()),target_pixels=int(target.sum()),
                  navigable_pixels=int(navigable.sum()),free_components=int(count),
                  resolution_m=resolution,origin=[0.,0.,0.],
                  free_thresh=.25,occupied_thresh=.65,
                  metadata_assumptions=['0.05 m/pixel inferred from companion PGM; PNG YAML unavailable',
                                        'arbitrary local origin [0,0,0]'],
                  start=list(start),clearance_m=clearance)
    return MapData(free,target,navigable,resolution,start,metadata)


def pixel_to_world(pixel, height, resolution):
    row,col=pixel
    return [(col+.5)*resolution,(height-row-.5)*resolution]


def world_to_pixel(xy, height, resolution):
    x,y=xy
    return (height-1-math.floor(y/resolution),math.floor(x/resolution))
