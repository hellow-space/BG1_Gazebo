"""ROS-independent polygon validation and click/close state."""
import math
def cross(a,b,c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def on(a,b,p):
    return abs(cross(a,b,p))<1e-9 and min(a[0],b[0])-1e-9<=p[0]<=max(a[0],b[0])+1e-9 and min(a[1],b[1])-1e-9<=p[1]<=max(a[1],b[1])+1e-9


def intersects(a,b,c,d):
    return (cross(a,b,c)*cross(a,b,d)<0 and cross(c,d,a)*cross(c,d,b)<0) or on(a,b,c) or on(a,b,d) or on(c,d,a) or on(c,d,b)


def validate_polygon(points):
    if not 3<=len(points)<=100:
        raise ValueError('需要 3 至 100 个顶点')
    if any(not math.isfinite(v) for point in points for v in point):
        raise ValueError('顶点坐标必须是有限数值')
    if len(set(points))!=len(points):
        raise ValueError('顶点不能重复')
    area=abs(sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(points,points[1:]+points[:1])))/2
    if area<.05:
        raise ValueError('区域不能退化成直线')
    for i,a in enumerate(points):
        b=points[(i+1)%len(points)]
        for j in range(i+1,len(points)):
            if j==i+1 or (i==0 and j==len(points)-1):continue
            if intersects(a,b,points[j],points[(j+1)%len(points)]):
                raise ValueError('区域不能交叉或重叠')
    return list(points)


class Selection:
    def __init__(self):
        self.points=[]
        self.closed=False

    def add(self,point):
        if self.closed:self.reset()
        if len(self.points)>=3 and math.dist(point,self.points[0])<=.35:
            validate_polygon(self.points)
            self.closed=True
            return True
        if self.points and math.dist(point,self.points[-1])<.05:
            raise ValueError('相邻顶点距离太近')
        self.points.append(point)
        return False

    def reset(self):
        self.points.clear();self.closed=False

    def undo(self):
        self.closed=False
        if self.points:self.points.pop()
