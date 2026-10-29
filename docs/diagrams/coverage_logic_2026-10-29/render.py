"""Deterministic teaching storyboards. Not benchmark output or robot commands."""
from pathlib import Path
import sys
import ctypes as c
import json
import hashlib
import textwrap
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon, PathPatch, FancyArrowPatch
from matplotlib.path import Path as MPath
from matplotlib.font_manager import FontProperties

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT))
from coverage_benchmark import raster_geometry as rg
from coverage_benchmark.grid_connector import GridConnector
from coverage_benchmark.swath_geometry import clip_swath

FONT=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':'Noto Sans CJK JP','svg.fonttype':'none','svg.hashsalt':'bg1-coverage-storyboards','axes.unicode_minus':False})
BG='#f4f7fb'; INK='#19324b'; WALL='#64717c'; BLUE='#1674bd'; GREEN='#23986d'; LIGHT='#d7efe4'
ORANGE='#edb66b'; RED='#d54c47'; PURPLE='#7f65a4'; MUTED='#64758a'
OUTER=[[0,0],[16,0],[16,10],[0,10],[0,0]]
HOLE=[[7,4],[7,6],[9,6],[9,4],[7,4]]
ORIGINAL={'type':'Polygon','coordinates':[OUTER,HOLE]}
D=1.;R=.05
buf=rg._bind('OGR_G_Buffer',rg.ptr,[rg.ptr,c.c_double,c.c_int])
original=rg.from_json(json.dumps(ORIGINAL).encode());offset=buf(original,-D,30)
ptr=rg.export(offset)
SAFE=json.loads(c.string_at(ptr));rg.free(ptr);rg.geometry_close(offset);rg.geometry_close(original)
ALLOWED=np.flipud(rg.rasterize([{'geometry':SAFE}],(200,320),R))
CONNECTOR=GridConnector(ALLOWED,R)
DUBINS=np.loadtxt(HERE/'dubins_example.csv',delimiter=',')
START=[5.8,5.];GOAL=[14.5,5.8]
ASTAR=np.asarray(CONNECTOR.connect(START,GOAL))
inside=(DUBINS[:,0]>7)&(DUBINS[:,0]<9)&(DUBINS[:,1]>4)&(DUBINS[:,1]<6)
assert inside.any() and CONNECTOR.clear(ASTAR)
YS=np.arange(1.8,8.3,.8)
SWATHS=[]
for y in YS:SWATHS.extend(clip_swath([[1.05,float(y)],[14.95,float(y)]],ALLOWED,R))
CENTRES=[]
for y in np.arange(1.5,9.,1.):
 for x in np.arange(1.5,15.,1.):
  xx0,xx1=int((x-.5)/R),int((x+.5)/R)
  yy0,yy1=int((y-.5)/R),int((y+.5)/R)
  if ALLOWED[yy0:yy1,xx0:xx1].all():CENTRES.append((x,y))
A=np.array([[x,1.5] for x in np.arange(1.5,15.,1.)]+[[14.5,2.5]]+
           [[x,2.5] for x in np.arange(13.5,1.,-1.)]+[[1.5,3.5]]+[[x,3.5] for x in np.arange(2.5,6.,1.)])
B=np.array([[10.5,6.5],[11.5,6.5],[12.5,6.5],[13.5,6.5],[14.5,6.5],[14.5,7.5]]+
           [[x,7.5] for x in np.arange(13.5,1.,-1.)])
JOIN=np.asarray(CONNECTOR.connect(A[-1],B[0]))


def text(ax,x,y,s,size=14,color=INK,weight='normal',ha='left',va='center',**kw):
 return ax.text(x,y,s,fontproperties=FONT,fontsize=size,color=color,fontweight=weight,ha=ha,va=va,**kw)


def polygon(ax,geo,face,edge=None,lw=1.5,alpha=1.,ls='-'):
 polys=geo['coordinates'] if geo['type']=='MultiPolygon' else [geo['coordinates']]
 for rings in polys:
  vertices=[];codes=[]
  for index,ring in enumerate(rings):
   points=np.asarray(ring,float)
   area=np.sum(points[:-1,0]*points[1:,1]-points[1:,0]*points[:-1,1])
   if (index==0 and area<0) or (index>0 and area>0):points=points[::-1]
   vertices.extend(points);codes.extend([MPath.MOVETO]+[MPath.LINETO]*(len(points)-2)+[MPath.CLOSEPOLY])
  ax.add_patch(PathPatch(MPath(vertices,codes),facecolor=face,edgecolor=edge or face,
                         lw=lw,alpha=alpha,linestyle=ls,zorder=2))


def base(ax,mode='plain',grid=False):
 ax.set_xlim(-1.2,17.2);ax.set_ylim(-2.3,12.8);ax.set_aspect('equal');ax.axis('off')
 ax.add_patch(Rectangle((-.45,-.45),16.9,10.9,facecolor=WALL,zorder=0))
 polygon(ax,ORIGINAL,'white',WALL,lw=1.7)
 if mode in ('band','safe'):
  polygon(ax,ORIGINAL,ORANGE if mode=='band' else '#e9edf2',WALL)
  polygon(ax,SAFE,LIGHT,GREEN,lw=2.)
 if grid:
  for x in np.arange(0,16.01,.5):ax.plot([x,x],[0,10],color='#c5cdd6',lw=.45,zorder=3)
  for y in np.arange(0,10.01,.5):ax.plot([0,16],[y,y],color='#c5cdd6',lw=.45,zorder=3)
  ax.add_patch(Rectangle((7,4),2,2,facecolor=WALL,zorder=4))


def route(ax,points,color=BLUE,lw=2.6,arrows=False,alpha=1.,ls='-'):
 points=np.asarray(points)
 ax.plot(points[:,0],points[:,1],color=color,lw=lw,alpha=alpha,ls=ls,zorder=7)
 if arrows:
  for a,b in zip(points,points[1:]):
   if np.linalg.norm(b-a)>.65:
    mid=(a+b)/2;v=(b-a)/np.linalg.norm(b-a)*.28
    ax.annotate('',xy=mid+v,xytext=mid-v,arrowprops=dict(arrowstyle='-|>',color=color,lw=1.4),zorder=8)


def robot(ax,x,y,theta=0):
 ax.add_patch(Circle((x,y),.28,facecolor=BLUE,edgecolor='white',lw=1.3,zorder=11))
 t=np.deg2rad(theta);v=np.array([np.cos(t),np.sin(t)])
 ax.annotate('',xy=np.array([x,y])+.7*v,xytext=(x,y),arrowprops=dict(arrowstyle='-|>',color=BLUE,lw=2),zorder=12)


def arrow(ax,a,b,color=MUTED,ls='-',style='-|>',lw=1.7):
 ax.annotate('',xy=b,xytext=a,arrowprops=dict(arrowstyle=style,color=color,lw=lw,linestyle=ls),zorder=13)


def header(ax,n,title,description,color=BLUE):
 ax.add_patch(Circle((-.25,11.55),.44,color=color,zorder=40,clip_on=False))
 text(ax,-.25,11.55,str(n),17,'white',weight='bold',ha='center',zorder=41)
 text(ax,.55,11.55,textwrap.fill(title,width=24),20,weight='bold',zorder=41,linespacing=1.2)
 text(ax,0,-1.05,'\n'.join(textwrap.fill(line,width=34) for line in description.split('\n')),13.5,va='center',linespacing=1.45,zorder=41)


def canvas(title,subtitle,footer,rows=4):
 fig,grid=plt.subplots(rows,2,figsize=(17.5,rows*5.7+1.4),facecolor=BG)
 height=rows*5.7+1.4
 fig.subplots_adjust(left=.035,right=.965,top=1-1.33/height,bottom=.065 if rows==2 else .04,hspace=.12,wspace=.10)
 fig.text(.045,1-.24/height,title,fontproperties=FONT,fontsize=27,fontweight='bold',color=INK,va='top')
 fig.text(.045,1-.90/height,subtitle,fontproperties=FONT,fontsize=14,color=MUTED,va='top')
 axes=[]
 for row in range(rows):
  for step in range(2):axes.append(grid[row,step if row%2==0 else 1-step])
 # Explicit snake arrows: 1→2↓3←4↓5→6↓7←8.
 for index in range(len(axes)-1):
  a,b=axes[index].get_position(),axes[index+1].get_position()
  if index%2==0:
   if a.x0<b.x0:p=(a.x1+.004,(a.y0+a.y1)/2);q=(b.x0-.004,(b.y0+b.y1)/2)
   else:p=(a.x0-.004,(a.y0+a.y1)/2);q=(b.x1+.004,(b.y0+b.y1)/2)
  else:p=((a.x0+a.x1)/2,a.y0-.002);q=((b.x0+b.x1)/2,b.y1+.002)
  fig.add_artist(FancyArrowPatch(p,q,transform=fig.transFigure,arrowstyle='-|>',mutation_scale=19,lw=1.6,color='#a0adbd',zorder=50))
 fig.text(.045,.015,footer,fontproperties=FONT,fontsize=12,color=MUTED,va='bottom')
 return fig,axes


def save(fig,name):
 fig.savefig(HERE/(name+'.png'),dpi=165,facecolor=BG)
 fig.savefig(HERE/(name+'.svg'),facecolor=BG,metadata={'Date':None})
 svg=HERE/(name+'.svg')
 svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines())+'\n')
 plt.close(fig)


def headland():
 fig,axes=canvas('先看懂 Headland：给作业条带留出一圈边带',
                 '从原区域开始看变化。箭头按 1 → 2 ↓ 3 ← 4 阅读；墙与障碍实体始终不动。',
                 '教学几何：16 m × 10 m，障碍 2 m × 2 m，d = 1 m。橙色带不是轨迹，也不代表已经清扫。\n主图扫描间距仅示意；放大剖面中的 d / w 按同一横向比例绘制。',rows=2)
 ax=axes[0];base(ax);header(ax,1,'原区域：哪里原本可以作业？','白色是原本的自由区域；灰色是墙外与障碍。')
 text(ax,2.1,8.2,'自由区域',18);text(ax,8,5,'障碍',16,'white',ha='center')
 robot(ax,3,2)
 ax=axes[1];base(ax,'band');header(ax,2,'做负 buffer：从所有边界后退 d','外墙边向内退；障碍孔周围也后退，因此条带绕开的范围变大。')
 text(ax,2,8.55,'新外边界',15,GREEN);arrow(ax,(4.8,8.5),(4.8,9),GREEN)
 text(ax,10.5,5.1,'新内孔边界',15,GREEN);arrow(ax,(10.5,5.1),(10,5.1),GREEN)
 ax.annotate('',xy=(1,2.6),xytext=(0,2.6),arrowprops=dict(arrowstyle='<->',color=INK,lw=1.5))
 text(ax,.5,3.2,'d',16,ha='center')
 ax.annotate('',xy=(6,4.9),xytext=(7,4.9),arrowprops=dict(arrowstyle='<->',color=INK,lw=1.5))
 text(ax,6.5,4.1,'d',16,ha='center')
 ax=axes[2];base(ax,'band');header(ax,3,'得到条带区：只在绿色区域铺线','绿色区域更小。橙色边带先不铺作业线；窄口还可能被内缩切断。')
 for s in SWATHS:route(ax,s,lw=2.)
 text(ax,.5,.5,'预留边带',13,'#935921');text(ax,10.4,7.8,'作业条带',15,BLUE)
 ax=axes[3];ax.axis('off');ax.set_xlim(-1.2,17.2);ax.set_ylim(-2.3,12.8);ax.set_aspect('equal')
 header(ax,4,'放大一面墙：两个宽度别混淆','d 是预留带宽；w 是作业宽度 / 条带间距。\n橙色仅表示暂不铺条带，不自动等于机器人禁行。\n几何内缩也没有算出一条绕障转弯路线。')
 ax.add_patch(Rectangle((0,2),2,7.3,facecolor=WALL));ax.add_patch(Rectangle((2,2),4,7.3,facecolor=ORANGE))
 ax.add_patch(Rectangle((6,2),9,7.3,facecolor=LIGHT));ax.plot([6,6],[2,9.3],color=GREEN,lw=2,ls='--')
 text(ax,1,8.5,'墙',18,'white',ha='center');text(ax,4,8.5,'预留带',18,ha='center');text(ax,10.5,8.5,'铺作业线的区域',18,ha='center')
 for x in (7.2,9.6,12.0):ax.plot([x,x],[2.6,7.5],color=BLUE,lw=3)
 arrow(ax,(2,1.1),(6,1.1),INK,style='<->');text(ax,4,.4,'d = 1 m',16,ha='center')
 arrow(ax,(7.2,4.2),(9.6,4.2),INK,style='<->');text(ax,8.4,5,'w = 0.6 m',15,ha='center')
 text(ax,7.5,10.1,'条带间距 ≠ 预留带宽',15,GREEN)
 save(fig,'01_headland_explained')


def coarse(ax,visited=None):
 visited=[] if visited is None else visited
 for y in np.arange(1.5,9.,1.):
  for x in np.arange(1.5,15.,1.):
   free=(x,y) in CENTRES
   ax.add_patch(Rectangle((x-.5,y-.5),1,1,facecolor='#cae7dc' if free else '#d9dfe6',
                          edgecolor='#9baaba',lw=.55,alpha=.8,zorder=3))
 if CENTRES:
  p=np.array(CENTRES);ax.scatter(p[:,0],p[:,1],s=13,c=GREEN,zorder=5)
 if len(visited):
  p=np.array(visited);ax.scatter(p[:,0],p[:,1],s=20,c=ORANGE,zorder=9)


def rounded(points):
 p=np.asarray(points);corners=[p[0]]
 for i in range(1,len(p)-1):
  if abs(np.cross(p[i]-p[i-1],p[i+1]-p[i]))>1e-8:corners.append(p[i])
 corners.append(p[-1]);p=np.array(corners);out=[p[0]]
 for a,b,d in zip(p,p[1:],p[2:]):
  u=(b-a)/np.linalg.norm(b-a);v=(d-b)/np.linalg.norm(d-b)
  radius=min(.3,np.linalg.norm(b-a)/4,np.linalg.norm(d-b)/4)
  left=b-radius*u;right=b+radius*v;out.append(left)
  for t in np.linspace(0,1,16):out.append((1-t)**2*left+2*(1-t)*t*b+t*t*right)
 out.append(p[-1]);return np.asarray(out)


def leader():
 fig,axes=canvas('Leader：从自由栅格，一步一步挑出覆盖路线',
                 '当前 zigzag 主调用链。按编号与箭头阅读；同一张地图在每一步发生什么，都画出来。',
                 '教学分镜，不是实测输出。粗格/路径间距为便于看清而放大；当前预览关闭独立岛边路线。')
 ax=axes[0];base(ax,grid=True);header(ax,1,'读入栅格地图，圈定任务区','每个小像素代表自由 / 障碍 / 未知；选区外先当作不可走。')
 ax.add_patch(Rectangle((0,0),16,10,fill=False,edgecolor=BLUE,lw=2.7,ls='--',zorder=8));robot(ax,1.5,1.5)
 text(ax,8,5,'障碍',14,'white',ha='center');text(ax,1,8.8,'任务选区',16,BLUE)
 ax=axes[1];base(ax,'band');header(ax,2,'地图准备：膨胀障碍、提取外轮廓','橙色区域不再允许机器人中心进入；若分成多块，继续取最大外轮廓。')
 text(ax,8,5,'障碍',14,'white',ha='center');text(ax,10.4,8.2,'保留外轮廓',16,GREEN)
 # Small, explicitly separate vignette demonstrates largest-component selection.
 ax.add_patch(Rectangle((10.3,.3),5.2,2.3,facecolor='white',edgecolor='#c6d0dc',zorder=10))
 text(ax,10.6,2.2,'若膨胀后断连',12,MUTED,zorder=11)
 ax.add_patch(Rectangle((10.7,.7),2.3,.8,facecolor=LIGHT,edgecolor=GREEN,zorder=11))
 ax.add_patch(Rectangle((13.8,.7),.7,.6,facecolor='#d9dfe6',zorder=11));text(ax,14.8,1,'×',20,RED,zorder=12)
 ax=axes[2];base(ax,'safe');coarse(ax);header(ax,3,'变成粗格：留下可以覆盖的格子','逐格检查附近障碍；合格格子选一个自由代表点（绿点）。')
 text(ax,10.4,5,'碰障粗格\n不进入候选集',14,MUTED);arrow(ax,(10.2,4.8),(9.5,4.5),MUTED)
 ax=axes[3];base(ax,'safe');coarse(ax,A[:14]);route(ax,A[:14],arrows=True)
 header(ax,4,'先扫当前行：优先沿原方向走','比较左 / 右邻格的分数；走过的格子标为已访问（橙点），避免反复选。')
 robot(ax,7.5,1.5);text(ax,3,8.2,'绿色：还没访问',15,GREEN);text(ax,3,7.3,'橙色：已经访问',15,'#a96f2e')
 ax=axes[4];base(ax,'safe');coarse(ax,A);route(ax,A,lw=2.3)
 header(ax,5,'行内走不动：找最近的未访问格','障碍挡住右侧，左侧又走过了；从剩余候选格中挑一个新目标。')
 ax.scatter([5.5],[4.5],marker='*',s=230,c=GREEN,zorder=12)
 ax.scatter([6.5],[3.5],marker='x',s=90,c=RED,lw=2,zorder=12)
 arrow(ax,(5.5,3.5),(5.5,4.5),MUTED,ls=':');robot(ax,5.5,3.5)
 text(ax,10.4,8.3,'★ 下一个目标\n虚线只是“选中它”\n不是可走直连',14,GREEN)
 ax=axes[5];base(ax,'safe');route(ax,A,lw=1.7);route(ax,B,PURPLE,lw=1.7)
 ax.scatter(A[:,0],A[:,1],s=12,c=MUTED,zorder=9)
 ax.scatter([1.5,14.5,14.5,1.5,1.5,5.5],[1.5,1.5,2.5,2.5,3.5,3.5],s=65,c=ORANGE,zorder=10)
 header(ax,6,'遍历若干步后：分段、删共线点','粗格点换成米制坐标；远跳先断开。\n删去共线冗点，拐角保留；短片段可能过滤。')
 text(ax,10.7,8.4,'后续另一段',15,PURPLE);text(ax,2,4.6,'第一段',15,BLUE)
 ax=axes[6];base(ax,'safe');route(ax,A,MUTED,lw=1.2,ls='--');route(ax,B,MUTED,lw=1.2,ls='--')
 route(ax,rounded(A));route(ax,rounded(B),PURPLE)
 header(ax,7,'Bezier 局部优化：把硬拐角变圆滑','只展示形状变化。曲线圆滑 ≠ 自动避障；之后仍需要地图检查。')
 text(ax,10.4,8.3,'灰虚线：原折线\n彩色线：局部圆滑',14,MUTED)
 ax=axes[7];base(ax,'safe');route(ax,rounded(A));route(ax,rounded(B),BLUE);route(ax,JOIN,'#c47d1a',lw=3.3,arrows=True)
 header(ax,8,'用 A* 拼接各段，输出米制路径','橙线示意地图内的 A* 连接。\n原生连接失败可能跳过片段，因此不保证全图扫完。')
 robot(ax,1.5,1.5);text(ax,10.5,8.2,'橙线：A* 连接',15,'#a46815')
 save(fig,'02_leader_storyboard')


def fields():
 fig,axes=canvas('Fields2Cover：先铺作业条带，再排序和连接',
                 '①–⑦解释当前原生流程；⑧单独标出我们新增的避障适配。仍使用同一张带障碍地图。',
                 '教学分镜，非benchmark输出；间距为可读性放大。实际修改版先做精确净空，不重复执行原生Headland。')
 ax=axes[0];base(ax,grid=True);header(ax,1,'从地图识别自由空间','本项目先读PNG/PGM，再提取自由区域。\nFields2Cover 库实际接收多边形。')
 text(ax,2,8.4,'白：自由',16);text(ax,8,5,'障碍',14,'white',ha='center')
 ax=axes[1];base(ax);polygon(ax,ORIGINAL,'none',BLUE,lw=2.8)
 for ring,col in ((OUTER,BLUE),(HOLE,RED)):
  p=np.array(ring);ax.scatter(p[:,0],p[:,1],s=35,c=col,zorder=10)
 ax.plot([7,7,9,9,7],[4,6,6,4,4],color=RED,lw=3,zorder=7)
 header(ax,2,'变成“外环＋内孔”的多边形','外环围出可作业边界，内孔扣掉障碍；不能只取外接矩形。')
 text(ax,1.5,8.6,'外环',17,BLUE);text(ax,9.8,5.4,'内孔',17,RED)
 ax=axes[2];base(ax,'band');header(ax,3,'Headland：先把条带区缩进去','四周边界往里退，障碍周围也退开；橙色带留着，绿色区才铺条带。')
 arrow(ax,(0,2.3),(1,2.3),INK,style='<->');text(ax,.5,3,'d',16,ha='center')
 text(ax,10.5,7.9,'看专门的 Headland 图\n可看到内缩前后差别',14,GREEN)
 ax=axes[3];base(ax,'band');header(ax,4,'按作业宽度，铺一组平行扫描线','先确定方向，再按间距平移长直线；虚线此时只是待裁剪的几何线。')
 for y in YS:ax.plot([-.3,16.3],[y,y],color=BLUE,lw=1.6,ls='--',zorder=6)
 text(ax,10.4,9.6,'方向固定的平行线',14,BLUE)
 ax=axes[4];base(ax,'band');header(ax,5,'与绿色区域求交，裁成作业条带','跨过障碍的中间部分删掉；一条长扫描线可能被切成两段。')
 for s in SWATHS:
  route(ax,s,lw=2.);p=np.array(s);ax.scatter(p[:,0],p[:,1],s=14,c=BLUE,zorder=8)
 ax.plot([6,10],[5,5],color=RED,lw=2,ls=':',zorder=8);text(ax,8,5,'删',17,'white',ha='center',zorder=10)
 ax=axes[5];base(ax,'safe');header(ax,6,'给条带排序，再交替改变行进方向','这是条带排序，不是BCD区域分解，也没有优化整条路线的绕障总长度。')
 for i,s in enumerate(SWATHS):
  p=np.array(s if i%2==0 else s[::-1]);route(ax,p,lw=1.9,arrows=True)
  text(ax,p[0,0]+.1,p[0,1]+.24,str(i+1),10,BLUE)
 text(ax,10.4,9.6,'编号示意访问顺序',13,MUTED)
 ax=axes[6];base(ax,'band');header(ax,7,'原生：按端点和朝向生成 Dubins 转弯','示例红线由原生库生成。\n转弯器没有读取障碍图，因此连接可能穿障碍。')
 route(ax,[[2,5],START],BLUE,arrows=True);route(ax,[GOAL,[10.2,5.8]],BLUE,arrows=True)
 route(ax,DUBINS,RED,lw=2.8)
 ax.scatter([START[0],GOAL[0]],[START[1],GOAL[1]],s=80,c=BLUE,zorder=11)
 text(ax,8,5,'穿障碍',15,'white',ha='center',zorder=12)
 text(ax,2,8.3,'原生输出仍可能碰撞',16,RED)
 ax=axes[7];base(ax,'safe');header(ax,'8*','本项目新增：检查＋避障连接','原生连接通过检查才保留；否则用A*。整条路线再审计，不合格就失败。',color=GREEN)
 route(ax,[[2,5],START],BLUE,arrows=True);route(ax,[GOAL,[10.2,5.8]],BLUE,arrows=True)
 route(ax,DUBINS,RED,lw=1.1,ls=':',alpha=.4);route(ax,ASTAR,GREEN,lw=3,arrows=True)
 text(ax,1.7,8.3,'绿色：绕障后的连接',16,GREEN)
 text(ax,8.8,1.8,'A* 折线可停车转向\n不承诺 Dubins 最小半径',13,MUTED)
 save(fig,'03_fields2cover_storyboard')


if __name__=='__main__':
 headland();leader();fields()
 metadata=dict(teaching_only=True,not_benchmark_output=True,room=[16,10],obstacle=[7,4,9,6],
               headland_m=D,headland_operation='OGR_G_Buffer(-1)',
               native_dubins=dict(start=START,end=GOAL,start_heading=0,end_heading='pi',min_radius=.6,
                                  points=len(DUBINS),points_inside_original_obstacle=int(inside.sum()),
                                  csv_sha256=hashlib.sha256((HERE/'dubins_example.csv').read_bytes()).hexdigest()),
               drawing_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               safe_polygon=SAFE,
               native_example_source_sha256=hashlib.sha256((HERE/'dubins_example.cpp').read_bytes()).hexdigest())
 (HERE/'geometry_evidence.json').write_text(json.dumps(metadata,indent=2)+'\n')
 print('Rendered 3 teaching storyboards as PNG + SVG; native collision example verified.')
