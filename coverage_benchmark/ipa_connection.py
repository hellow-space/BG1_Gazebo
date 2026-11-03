"""Preserve native target order; connect and audit every closed raster cell touched."""
import math
import numpy as np
from coverage_benchmark.grid_connector import GridConnector


def strict_path_valid(path,allowed,resolution):
    """Closed-cell supercover: touching any blocked cell, including corners, fails."""
    points=np.asarray(path,dtype=float);mask=np.asarray(allowed,dtype=bool)
    if (points.ndim!=2 or points.shape[1]!=2 or len(points)<2 or not np.isfinite(points).all()
            or mask.ndim!=2 or not math.isfinite(resolution) or resolution<=0):return False
    height,width=mask.shape
    for a,b in zip(points/resolution,points[1:]/resolution):
        cuts=[0.,1.]
        for axis in (0,1):
            d=b[axis]-a[axis]
            if d:
                lo,hi=sorted((a[axis],b[axis]))
                cuts.extend(((np.arange(math.floor(lo)+1,math.ceil(hi))-a[axis])/d).tolist())
        cuts=np.unique(cuts);times=np.r_[cuts,(cuts[:-1]+cuts[1:])/2]
        xy=a+times[:,None]*(b-a)
        # A tiny conservative numerical tolerance includes both closed cells on a grid line.
        for dx,dy in ((-1,-1),(-1,1),(1,-1),(1,1)):
            cells=np.floor(xy+np.array([dx,dy])*1e-9).astype(int)
            x,y=cells[:,0],cells[:,1]
            if (x<0).any() or (y<0).any() or (x>=width).any() or (y>=height).any():return False
            if not mask[y,x].all():return False
    return True


def assemble(start,targets,allowed,resolution):
    values=np.asarray(targets,dtype=float)
    if values.ndim!=2 or values.shape[1]!=2 or not len(values) or not np.isfinite(values).all():
        raise ValueError('Native targets must be a nonempty finite N by 2 sequence')
    connector=GridConnector(allowed,resolution)
    path=[list(start)];sections=[]
    for index,target in enumerate(values):
        before=connector.astar_calls;first=len(path)-1
        connection=connector.connect(path[-1],target)
        if not strict_path_valid(connection,allowed,resolution):
            raise ValueError('Connection failed closed-cell contact audit at target '+str(index))
        path.extend(connection[1:])
        sections.append(dict(native_target_index=index,first_vertex=first,last_vertex=len(path)-1,
                             method='astar' if connector.astar_calls>before else 'visible',
                             start=connection[0],end=target.tolist()))
    if not strict_path_valid(path,allowed,resolution):raise ValueError('Complete path failed closed-cell audit')
    return path,dict(sections=sections,astar_connections=connector.astar_calls,
                     native_targets_preserved=len(values),closed_cell_audit=True,
                     start_goal_policy='common start approach included; finish at final native target, no return')
