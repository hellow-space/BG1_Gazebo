"""Independent exact segment/blocked-pixel-box intersection certificate."""
import numpy as np
from scipy.spatial import cKDTree


def path_box_audit(path,allowed,resolution):
    points=np.asarray(path,dtype=float);mask=np.asarray(allowed,bool)
    if (points.ndim!=2 or points.shape[1]!=2 or len(points)<2 or not np.isfinite(points).all()
            or not np.isfinite(resolution) or resolution<=0 or mask.ndim!=2):return False
    if (points<=0).any() or (points[:,0]>=mask.shape[1]*resolution).any() or (points[:,1]>=mask.shape[0]*resolution).any():return False
    y,x=np.nonzero(~mask)
    if not len(x):return True
    centers=np.column_stack((x+.5,y+.5))*resolution;tree=cKDTree(centers)
    for a,b in zip(points,points[1:]):
        # Each possible intersection lies within half a sample step plus half
        # a pixel diagonal of a sample: <= .25r + sqrt(.5)r < r.
        steps=max(1,int(np.ceil(np.linalg.norm(b-a)/(resolution*.5))))
        samples=a+np.linspace(0,1,steps+1)[:,None]*(b-a)
        candidates=set(i for group in tree.query_ball_point(samples,resolution*(1+1e-10)) for i in group)
        if not candidates:continue
        c=centers[list(candidates)];lo=c-resolution*.5-1e-10*resolution;hi=c+resolution*.5+1e-10*resolution
        near=np.zeros(len(c));far=np.ones(len(c));possible=np.ones(len(c),bool)
        for axis in (0,1):
            delta=b[axis]-a[axis]
            if delta==0:possible &= (a[axis]>=lo[:,axis]) & (a[axis]<=hi[:,axis])
            else:
                u=(lo[:,axis]-a[axis])/delta;v=(hi[:,axis]-a[axis])/delta
                near=np.maximum(near,np.minimum(u,v));far=np.minimum(far,np.maximum(u,v))
        if np.any(possible & (near<=far)):return False
    return True
