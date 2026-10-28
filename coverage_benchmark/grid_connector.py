"""No-corner-cut 8-neighbor A* and checked visibility simplification."""
import heapq
import math
import numpy as np
from scipy.ndimage import label
from coverage_benchmark.metrics import segment_validity
from coverage_benchmark.swath_geometry import xy_points, grid_cuts, contains_point


class GridConnector:
    def __init__(self,allowed,resolution):
        self.allowed=np.asarray(allowed,dtype=bool)
        if self.allowed.ndim!=2 or resolution<=0:
            raise ValueError('Invalid allowed raster or resolution')
        self.resolution=resolution
        self.components,_=label(self.allowed)
        self.astar_calls=0

    def clear(self,path):
        points=xy_points(path)
        return len(points)>=2 and bool(segment_validity(points,self.allowed,self.resolution).all())

    def _shortcut_clear(self,a,b):
        if not self.clear([a,b]):return False
        # A visibility shortcut must not undo the search's no-corner-cut rule.
        points=(a[None,:]+grid_cuts(a,b,self.resolution)[:,None]*(b-a))/self.resolution
        corners=points[np.all(np.isclose(points,np.round(points),atol=1e-9,rtol=0),axis=1)]
        for point in corners:
            x,y=np.round(point).astype(int)
            if not (0<x<self.allowed.shape[1] and 0<y<self.allowed.shape[0]
                    and self.allowed[y-1:y+1,x-1:x+1].all()):return False
        return True

    def _simplify(self,points):
        # First discard only collinear grid centres; then checked line of sight.
        compressed=[points[0]]
        for index in range(1,len(points)-1):
            a=np.asarray(points[index])-points[index-1]
            b=np.asarray(points[index+1])-points[index]
            if abs(a[0]*b[1]-a[1]*b[0])>1e-12 or np.dot(a,b)<0:
                compressed.append(points[index])
        compressed.append(points[-1])
        result=[compressed[0]]
        index=0
        while index<len(compressed)-1:
            end=len(compressed)-1
            while end>index+1 and not self._shortcut_clear(np.asarray(compressed[index]),np.asarray(compressed[end])):
                end-=1
            result.append(compressed[end]);index=end
        return result

    def connect(self,start,end):
        start,end=xy_points([start,end])
        if not contains_point(start,self.allowed,self.resolution) or not contains_point(end,self.allowed,self.resolution):
            raise ValueError('Connection endpoint outside allowed raster')
        sx,sy=np.floor(start/self.resolution).astype(int)
        ex,ey=np.floor(end/self.resolution).astype(int)
        if self.components[sy,sx]!=self.components[ey,ex]:
            raise ValueError('Required endpoints are disconnected in clearance raster')
        if self._shortcut_clear(start,end):return [start.tolist(),end.tolist()]
        self.astar_calls+=1
        width=self.allowed.shape[1]
        source,goal=sy*width+sx,ey*width+ex
        def heuristic(x,y):
            dx,dy=abs(x-ex),abs(y-ey)
            return max(dx,dy)+(math.sqrt(2)-1)*min(dx,dy)
        queue=[(heuristic(sx,sy),0.,source)]
        costs={source:0.};parent={}
        moves=((-1,0,1.),(1,0,1.),(0,-1,1.),(0,1,1.),
               (-1,-1,math.sqrt(2)),(-1,1,math.sqrt(2)),(1,-1,math.sqrt(2)),(1,1,math.sqrt(2)))
        found=False
        while queue:
            _,cost,current=heapq.heappop(queue)
            if cost>costs[current]+1e-12:continue
            if current==goal:found=True;break
            y,x=divmod(current,width)
            for dx,dy,step in moves:
                nx,ny=x+dx,y+dy
                if not (0<=ny<self.allowed.shape[0] and 0<=nx<width and self.allowed[ny,nx]):continue
                if dx and dy and not (self.allowed[y,nx] and self.allowed[ny,x]):continue
                neighbor=ny*width+nx;new=cost+step
                if new+1e-12<costs.get(neighbor,math.inf):
                    costs[neighbor]=new;parent[neighbor]=current
                    heapq.heappush(queue,(new+heuristic(nx,ny),new,neighbor))
        if not found:raise ValueError('No collision-free route between required endpoints')
        route=[goal]
        while route[-1]!=source:route.append(parent[route[-1]])
        centres=[[(node%width+.5)*self.resolution,(node//width+.5)*self.resolution] for node in reversed(route)]
        path=self._simplify([start.tolist(),*centres,end.tolist()])
        if not self.clear(path):raise ValueError('A* output failed final raster audit')
        return path
