"""Independent raster-centre metrics used equally for all planner outputs."""
import math
import numpy as np


def clearance_mask(free, resolution, clearance):
    from scipy.ndimage import distance_transform_edt
    if resolution <= 0 or clearance < 0:
        raise ValueError('Invalid resolution or clearance')
    return distance_transform_edt(free) > math.ceil(clearance/resolution)


def _xy(path):
    points = np.asarray(path, dtype=float)
    if not points.size:
        return np.empty((0, 2))
    if points.ndim != 2 or points.shape[1] < 2 or not np.isfinite(points).all():
        raise ValueError('Expected finite N by 2 (or 3) path coordinates')
    return points[:, :2]


def path_length(path):
    points = _xy(path)
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def sweep_mask(path, shape, resolution, width, valid_segments=None):
    points = _xy(path)
    if resolution <= 0 or width <= 0:
        raise ValueError('Resolution and tool width must be positive')
    swept = np.zeros(shape, dtype=bool)
    if not len(points):
        return swept
    pairs = [(points[0], points[0])] if len(points) == 1 else zip(points, points[1:])
    if valid_segments is not None and len(valid_segments) != max(0, len(points)-1):
        raise ValueError('Segment validity length mismatch')
    radius = width/2
    height, width_cells = shape
    for index, (a, b) in enumerate(pairs):
        if valid_segments is not None and (len(points) == 1 or not valid_segments[index]):
            continue
        lo = np.ceil((np.minimum(a, b)-radius)/resolution-.5).astype(int)
        hi = np.floor((np.maximum(a, b)+radius)/resolution-.5).astype(int)
        x0, y0 = max(0, lo[0]), max(0, lo[1])
        x1, y1 = min(width_cells-1, hi[0]), min(height-1, hi[1])
        if x1 < x0 or y1 < y0:
            continue
        x = (np.arange(x0, x1+1)+.5)*resolution
        y = (np.arange(y0, y1+1)+.5)*resolution
        dx, dy = b-a
        norm = dx*dx+dy*dy
        t = np.zeros((len(y), len(x))) if norm == 0 else np.clip(
            ((x[None, :]-a[0])*dx+(y[:, None]-a[1])*dy)/norm, 0, 1)
        distance2 = (x[None, :]-a[0]-t*dx)**2+(y[:, None]-a[1]-t*dy)**2
        swept[y0:y1+1, x0:x1+1] |= distance2 <= radius*radius+1e-12
    return swept


def segment_validity(path, allowed, resolution):
    points = _xy(path)
    if resolution <= 0:
        raise ValueError('Resolution must be positive')
    height, width = allowed.shape
    validity = []
    for a, b in zip(points, points[1:]):
        # Split at every grid boundary; each open interval lies in one raster
        # cell. This catches arbitrarily short crossings, unlike fixed sampling.
        aa, bb = a/resolution, b/resolution
        if np.any(np.minimum(aa, bb) < 0) or np.any(np.maximum(aa, bb) >= [width, height]):
            validity.append(False)
            continue
        cuts = [0., 1.]
        for axis in (0, 1):
            delta = bb[axis]-aa[axis]
            if delta:
                edges = np.arange(math.floor(min(aa[axis], bb[axis]))+1,
                                  math.ceil(max(aa[axis], bb[axis])))
                cuts.extend(((edges-aa[axis])/delta).tolist())
        cuts = np.unique(cuts)
        times = np.r_[0., (cuts[:-1]+cuts[1:])/2, 1.]
        cells = np.floor(aa[None, :]+times[:, None]*(bb-aa)[None, :]).astype(int)
        validity.append(bool(allowed[cells[:, 1], cells[:, 0]].all()))
    return np.asarray(validity, dtype=bool)


def evaluate_path(path, original_free, allowed, resolution, tool_width, success):
    """Return exactly three report metrics plus non-ranking validity diagnostics."""
    empty=dict(coverage_percent=0.,path_length_m=None,path_valid=False,
               diagnostics={'reason':'planner failed or returned no complete path'})
    if not success:
        return empty
    try:
        points=_xy(path)
        if len(points)<2:
            return empty
        length=path_length(points)
        lower_free=np.flipud(original_free)
        lower_allowed=np.flipud(allowed)
        valid=segment_validity(points,lower_allowed,resolution)
        free_valid=segment_validity(points,lower_free,resolution)
        swept=sweep_mask(points,lower_free.shape,resolution,tool_width)
        covered=int((swept & lower_free).sum())
        total=int(lower_free.sum())
        lengths=np.linalg.norm(np.diff(points,axis=0),axis=1)
        return dict(coverage_percent=100.*covered/total,path_length_m=length,
                    path_valid=bool(valid.all() and free_valid.all()),
                    diagnostics=dict(covered_pixels=covered,total_free_pixels=total,
                                     invalid_segments=int((~valid).sum()),
                                     collision_segments=int((~free_valid).sum()),
                                     invalid_length_m=float(lengths[~valid].sum()),
                                     reason='passed common raster audit' if valid.all() and free_valid.all() else 'collision, boundary or clearance raster violation'))
    except (ValueError,OverflowError,IndexError) as error:
        empty['diagnostics']['reason']=str(error)
        return empty
