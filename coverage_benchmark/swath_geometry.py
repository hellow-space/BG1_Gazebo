"""Exact raster-boundary clipping of native coverage swaths (lower-left XY)."""
import math
import numpy as np
from coverage_benchmark.metrics import segment_validity


def xy_points(path):
    points=np.asarray(path,dtype=float)
    if points.ndim!=2 or points.shape[1]!=2 or not np.isfinite(points).all():
        raise ValueError('Expected finite XY points')
    return points


def grid_cuts(a,b,resolution):
    aa,bb=a/resolution,b/resolution
    cuts=[0.,1.]
    for axis in (0,1):
        delta=bb[axis]-aa[axis]
        if delta:
            edges=np.arange(math.floor(min(aa[axis],bb[axis]))+1,
                            math.ceil(max(aa[axis],bb[axis])))
            cuts.extend(((edges-aa[axis])/delta).tolist())
    return np.unique(cuts)


def contains_point(point,allowed,resolution):
    x,y=np.floor(point/resolution).astype(int)
    return 0<=y<allowed.shape[0] and 0<=x<allowed.shape[1] and bool(allowed[y,x])


def clip_swath(swath,allowed,resolution):
    a,b=xy_points(swath)
    length=float(np.linalg.norm(b-a))
    if length<1e-12:
        return []
    cuts=grid_cuts(a,b,resolution)
    mid=(cuts[:-1]+cuts[1:])/2
    cells=np.floor((a[None,:]+mid[:,None]*(b-a))/resolution).astype(int)
    x,y=cells.T
    good=(x>=0)&(y>=0)&(x<allowed.shape[1])&(y<allowed.shape[0])
    good[good]=allowed[y[good],x[good]]
    starts=np.flatnonzero(good & np.r_[True,~good[:-1]])
    ends=np.flatnonzero(good & np.r_[~good[1:],True])+1
    result=[]
    for begin,end in zip(starts,ends):
        lo,hi=cuts[begin],cuts[end]
        epsilon=min((hi-lo)/4,1e-6*resolution/length)
        if lo!=0 or not contains_point(a,allowed,resolution):lo+=epsilon
        if hi!=1 or not contains_point(b,allowed,resolution):hi-=epsilon
        piece=[(a+lo*(b-a)).tolist(),(a+hi*(b-a)).tolist()]
        if segment_validity(piece,allowed,resolution).all():
            result.append(piece)
        else:
            raise ValueError('Clipped swath failed exact raster audit')
    return result
