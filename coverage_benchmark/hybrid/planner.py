"""IPA BCD and Local Energy planners. All internal rasters use lower-left row order.

F is traversable; region masks are tasks, never artificial walls. C is the
actual planned disk sweep, including every approach and inter-region transfer.
"""
from dataclasses import dataclass
from pathlib import Path
import cv2
import numpy as np
from scipy.ndimage import label
from coverage_benchmark.hybrid import native
from coverage_benchmark.models import PlanningResult
from coverage_benchmark.grid_connector import GridConnector
from coverage_benchmark.ipa_connection import strict_path_valid
from coverage_benchmark.ipa_audit import path_box_audit
from coverage_benchmark.metrics import sweep_mask,path_length
from coverage_benchmark.swath_geometry import clip_swath


@dataclass
class Region:
    id: int
    kind: str
    mask: np.ndarray
    cells: list
    features: dict


def partition(f,cells,resolution,width):
    """Rasterise native cells for inspection; retain gaps without hybrid classification."""
    owner=np.full(f.shape,-1,np.int32);regions=[]
    for index,cell in enumerate(cells):
        polygon=np.asarray(cell['polygon'],float)
        mask=np.zeros(f.shape,np.uint8)
        if len(polygon)>=3:
            cv2.fillPoly(mask,[np.rint(polygon/resolution-.5).astype(np.int32)],1)
        selected=mask.astype(bool)&f&(owner<0)
        owner[selected]=index
        regions.append(Region(index,'bcd',selected,[index],{}))
    gaps,count=label(f&(owner<0))
    for component in range(1,count+1):
        mask=gaps==component;rid=len(regions);owner[mask]=rid
        regions.append(Region(rid,'unassigned',mask,[],{}))
    if not np.array_equal(owner>=0,f):raise ValueError('Partition lost feasible pixels')
    return regions,owner


class CoverageLedger:
    def __init__(self,shape,resolution,width):
        self.covered=np.zeros(shape,bool);self.resolution=resolution;self.width=width
    def add(self,path):
        points=np.asarray(path,float)
        if not len(points):return 0
        r=self.resolution;radius=self.width/2
        lo=np.maximum(0,np.floor((points.min(axis=0)-radius)/r).astype(int)-1)
        hi=np.minimum(self.covered.shape[::-1],np.ceil((points.max(axis=0)+radius)/r).astype(int)+1)
        x0,y0=lo;x1,y1=hi
        if x1<=x0 or y1<=y0:return 0
        swept=sweep_mask(points-lo*r,(y1-y0,x1-x0),r,self.width)
        current=self.covered[y0:y1,x0:x1];gain=int((swept&~current).sum());current|=swept
        return gain


class SafeRoute:
    def __init__(self,f,resolution,ledger,start):
        self.f=f;self.resolution=resolution;self.ledger=ledger;self.connector=GridConnector(f,resolution)
        self.path=[list(start)];self.sections=[];self.ledger.add(self.path)
    def connection(self,target):
        path=self.connector.connect(self.path[-1],target)
        if not strict_path_valid(path,self.f,self.resolution):raise ValueError('Connector failed strict closed-cell audit')
        return path
    def targets(self,targets,source):
        for index,target in enumerate(targets):
            before=self.connector.astar_calls;connection=self.connection(target);first=len(self.path)-1
            self.path.extend(connection[1:]);gain=self.ledger.add(connection)
            self.sections.append(dict(source=source,target_index=index,first_vertex=first,last_vertex=len(self.path)-1,
                                      method='astar' if self.connector.astar_calls>before else 'visible',gain_pixels=gain))


def candidates(cell,f,resolution):
    """Retain raw native output separately; clip candidate segments, never final routes."""
    pieces=[];rejected=0
    raw=np.asarray(cell['path'],float)
    points=[]
    for point in raw:
        if points and np.linalg.norm(point-points[-1])<1e-12:continue
        while len(points)>=2:
            a=points[-1]-points[-2];b=point-points[-1]
            if abs(a[0]*b[1]-a[1]*b[0])>1e-10 or np.dot(a,b)<=0:break
            points.pop()
        points.append(point)
    for a,b in zip(points,points[1:]):
        for piece in clip_swath([a,b],f,resolution):
            if strict_path_valid(piece,f,resolution):pieces.append(piece)
            else:rejected+=1
    return pieces,rejected


def _entry_options(region,pieces):
    values=[p for index in region.cells for p in pieces[index]]
    if not values:return []
    return [(region,values[0][0],False),(region,values[-1][-1],True)]


def _plan(data,f,cells,pieces,mode,work):
    if mode not in ('ipa_bcd','ipa_local'):raise ValueError('Unsupported IPA mode: '+mode)
    ledger=CoverageLedger(f.shape,data.resolution,data.tool_width);route=SafeRoute(f,data.resolution,ledger,data.start)
    calls=dict(bcd=0,local=0);evidence=[];schedule=[]
    if mode=='ipa_local':
        out=native.local(f,f,ledger.covered,data.resolution,data.tool_width,data.start,data.source_root,work/'local');out['context']=dict(phase='main',region='all',vertex=len(route.path)-1);evidence.append(out);calls['local']+=1
        route.targets(out['path'],dict(kernel='ipa_local',phase='main',region='all'))
    else:
        pending=[Region(i,'bcd',np.zeros((0,0),bool),[i],{}) for i in range(len(cells)) if pieces[i]]
        while pending:
            options=[o for region in pending for o in _entry_options(region,pieces)]
            if not options:break
            # Bounded geometric shortlist; final entry choice uses actual safe A* length.
            options.sort(key=lambda o:np.linalg.norm(np.asarray(o[1])-route.path[-1]));scored=[]
            for region,point,reverse in options[:8]:
                path=route.connection(point);scored.append((path_length(path),region.id,reverse,region,point))
            cost,_,reverse,region,point=min(scored,key=lambda x:(x[0],x[1],x[2]))
            schedule.append(dict(region=region.id,kernel=region.kind,entry=point,reverse=reverse,approach_length_m=cost,candidates_evaluated=len(scored)))
            values=[p for index in region.cells for p in pieces[index]]
            if reverse:values=[p[::-1] for p in values[::-1]]
            route.targets([p for piece in values for p in piece],dict(kernel='ipa_bcd',phase='main',region=region.id));calls['bcd']+=1
            pending=[r for r in pending if r.id!=region.id]
    strict=strict_path_valid(route.path,f,data.resolution);box=path_box_audit(route.path,f,data.resolution)
    if not strict or not box:raise ValueError('Final path failed independent safety audit')
    return PlanningResult(route.path,True,'Static raster path; no dynamics/curvature validation',dict(
        strict_valid=strict,box_audit=dict(passed=box),kernel_calls=calls,schedule=schedule,sections=route.sections,
        residual_pixels=int((f&~ledger.covered).sum()),feasible_pixels=int(f.sum()),
        covered_feasible_pixels=int((f&ledger.covered).sum()),
        covered_free_pixels=int((np.flipud(data.free)&ledger.covered).sum()),
        residual_free_pixels=int((np.flipud(data.free)&~ledger.covered).sum()),native_calls=evidence,
        fallback_used=False,astar_calls=route.connector.astar_calls))


def compare(data,progress=None):
    r=data.resolution;allowed=np.flipud(data.navigable);components,_=label(allowed)
    x,y=np.floor(np.asarray(data.start)/r).astype(int)
    if not (0<=y<allowed.shape[0] and 0<=x<allowed.shape[1] and allowed[y,x]):raise ValueError('Start outside F')
    f=components==components[y,x];work=Path(data.runtime_root);work.mkdir(parents=True,exist_ok=True)
    decomposition=native.decompose(f,r,data.tool_width,data.start,data.source_root,work/'decomposition')
    cells=decomposition['cells'];regions,labels=partition(f,cells,r,data.tool_width)
    np.savez_compressed(work/'partition.npz',labels=labels,f=f)
    pieces=[];rejected=0
    for cell in cells:
        values,bad=candidates(cell,f,r);pieces.append(values);rejected+=bad
    results={}
    for mode in ('ipa_bcd','ipa_local'):
        if progress:progress(mode)
        try:result=_plan(data,f,cells,pieces,mode,work/mode)
        except (ValueError,RuntimeError) as error:
            details=dict(failure=True)
            if isinstance(error,native.NativeFailure):details['native_failure']=error.details
            result=PlanningResult(message=str(error),details=details)
        result.details.update(decomposition=decomposition,partition=[dict(id=x.id,kind=x.kind,cells=x.cells,features=x.features,pixels=int(x.mask.sum())) for x in regions],
                              rejected_candidate_segments=rejected,partition_file=str(work/'partition.npz'),
                              scheduler='safe A* entry distance among up to 8 nearest endpoint candidates; greedy, no global optimality')
        results[mode]=result
    return results
