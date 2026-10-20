"""Geometry checks for the convex, obstacle-free coverage demonstration."""
import math


def stamp_seconds(stamp):
    return float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9


def finish_cleanup(steps):
    errors=[]
    for step in steps:
        try:
            step()
        except (Exception,KeyboardInterrupt) as error:
            errors.append(str(error) or 'Interrupted during cleanup')
    if errors:
        raise RuntimeError('; '.join(errors))


def quaternion_yaw(x,y,z,w):
    if not all(map(math.isfinite,(x,y,z,w))) or abs(x*x+y*y+z*z+w*w-1)>.01:
        raise ValueError('Invalid orientation quaternion')
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


def map_to_odom(actual,odom):
    a=math.atan2(math.sin(actual[2]-odom[2]),math.cos(actual[2]-odom[2]))
    c,s=math.cos(a),math.sin(a)
    return actual[0]-c*odom[0]+s*odom[1],actual[1]-s*odom[0]-c*odom[1],a


def validate_motion(samples,now):
    if len(samples)<10 or not all(all(math.isfinite(p[k]) for k in ('t','x','y','yaw')) for p in samples):
        raise ValueError('Insufficient or nonfinite Gazebo motion samples')
    if not 0<=now-samples[-1]['t']<1:
        raise ValueError('Gazebo ground truth is stale')
    if any(b['t']<=a['t'] for a,b in zip(samples,samples[1:])):
        raise ValueError('Gazebo samples are not time ordered')
    distance=sum(math.hypot(a['x']-b['x'],a['y']-b['y']) for a,b in zip(samples,samples[1:]))
    if distance<1:
        raise ValueError('Robot did not actually travel at least 1 m')
    return distance


def validate_polygon(points):
    p = [tuple(map(float, xy)) for xy in points]
    if p and p[0] == p[-1]:
        p.pop()
    if len(p) < 3 or any(len(xy)!=2 or not all(map(math.isfinite,xy)) for xy in p):
        raise ValueError('Polygon must contain at least three finite XY vertices')
    if len(set(p)) != len(p):
        raise ValueError('Duplicate polygon vertices')
    crosses = []
    for i,a in enumerate(p):
        b,c = p[(i+1)%len(p)],p[(i+2)%len(p)]
        crosses.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
    if not (all(v>0 for v in crosses) or all(v<0 for v in crosses)):
        raise ValueError('This demo requires a strictly convex polygon in perimeter order')
    # Reject star-shaped self intersections too, including convex-turn pentagrams.
    def cross(a,b,c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    for i,a in enumerate(p):
        b=p[(i+1)%len(p)]
        for j in range(i+2,len(p)):
            if i==0 and j==len(p)-1:
                continue
            c,d=p[j],p[(j+1)%len(p)]
            if cross(a,b,c)*cross(a,b,d)<=0 and cross(c,d,a)*cross(c,d,b)<=0:
                raise ValueError('Self intersecting polygon')
    return p+[p[0]]


def validate_path(poses, bounds, footprint):
    if len(poses)<2:
        raise ValueError('Path must contain at least two poses')
    if len(bounds)!=4 or not all(map(math.isfinite,bounds)) or bounds[0]>=bounds[1] or bounds[2]>=bounds[3]:
        raise ValueError('Invalid allowed-area bounds')
    dense=[poses[0]]
    for a,b in zip(poses,poses[1:]):
        if not all(map(math.isfinite,(*a,*b))):raise ValueError('Nonfinite path pose')
        da=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
        n=max(1,math.ceil(math.dist(a[:2],b[:2])/.05),math.ceil(abs(da)/.02))
        dense.extend((a[0]+(b[0]-a[0])*i/n,a[1]+(b[1]-a[1])*i/n,a[2]+da*i/n) for i in range(1,n+1))
    for x,y,yaw in dense:
        if not all(map(math.isfinite,(x,y,yaw))):
            raise ValueError('Nonfinite path pose')
        c,s=math.cos(yaw),math.sin(yaw)
        for fx,fy in footprint:
            px,py=x+c*fx-s*fy,y+s*fx+c*fy
            if not (bounds[0]<=px<=bounds[1] and bounds[2]<=py<=bounds[3]):
                raise ValueError(f'BG1 footprint outside empty test area at ({x:.2f},{y:.2f},{yaw:.2f})')
    return sum(math.dist(a[:2],b[:2]) for a,b in zip(poses,poses[1:]))


def connector(start,end,spacing=.05):
    if spacing<=0 or not all(map(math.isfinite,(*start,*end,spacing))):
        raise ValueError('Invalid connector arguments')
    dx,dy=end[0]-start[0],end[1]-start[1]
    n=max(1,math.ceil(math.hypot(dx,dy)/spacing))
    yaw=math.atan2(dy,dx)
    return [tuple(start)]+[(start[0]+dx*i/n,start[1]+dy*i/n,yaw) for i in range(n)]+[tuple(end)]


def validate_result(status,code,frame,poses,bounds,footprint):
    if status!=4 or code!=0:
        raise ValueError(f'Coverage planning failed: action={status}, error_code={code}')
    if frame!='map':
        raise ValueError(f'Expected map frame, got {frame!r}')
    return validate_path(poses,bounds,footprint)
