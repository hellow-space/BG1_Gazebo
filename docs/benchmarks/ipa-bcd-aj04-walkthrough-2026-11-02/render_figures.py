#!/usr/bin/env python3
"""Deterministic figures, with actual native masks and measured trajectories."""
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, label
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap
from matplotlib import patheffects as pe

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT))
D=json.loads((HERE/'evidence/report_data.json').read_text())
T=json.loads((HERE/'evidence/native_trace/trace.json').read_text())
M=np.load(HERE/'evidence/report_masks.npz')
S=D['summary'];P=S['parameters'];R=P['resolution_m'];W=P['width'];H=P['height']
IMAGE=np.asarray(Image.open(ROOT/f'data/maps/imported/{S["scene"]}/{S["scene"]}.png').convert('RGB'))
EXT=(0,W*R,0,H*R);OUT=HERE/'assets';OUT.mkdir(exist_ok=True)
plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False,
 'figure.facecolor':'#f4f7fb','axes.facecolor':'#e2e7ed','text.color':'#142b43',
 'axes.labelcolor':'#53647a','xtick.color':'#53647a','ytick.color':'#53647a',
 'svg.fonttype':'none','svg.hashsalt':'ipa-bcd-aj04-walkthrough'})
BLUE='#1877ba';GREEN='#129266';ORANGE='#e7a026';RED='#d94245';PURPLE='#913cbb';INK='#18334c'
COLORS=[matplotlib.colors.to_hex(plt.get_cmap('turbo')((i*.61803398875)%1)) for i in range(80)]
PATH=np.asarray(D['path'])
NATIVE=np.asarray(Image.open(HERE/'evidence/native_trace/01_rotated_room.png'))
INV=cv2.invertAffineTransform(np.asarray(D['rotation']['R']))
ORIGINAL_LABELS=cv2.warpAffine(M['native_labels'].astype(np.uint16),INV,(W,H),flags=cv2.INTER_NEAREST)


def mask(ax,a,color,alpha=.75,zorder=2):
    rgba=np.zeros((*a.shape,4));rgba[a,:3]=matplotlib.colors.to_rgb(color);rgba[a,3]=alpha
    ax.imshow(rgba,origin='lower',extent=EXT,interpolation='nearest',zorder=zorder)


def base(ax,crop=None,safe=False):
    ax.imshow(IMAGE,extent=EXT,interpolation='nearest',zorder=0)
    if safe:mask(ax,M['f'],BLUE,.22)
    ax.set_aspect('equal');ax.set_xlabel('x / m');ax.set_ylabel('y / m')
    if crop:ax.set_xlim(crop[0],crop[2]);ax.set_ylim(crop[1],crop[3])
    else:ax.set_xlim(EXT[:2]);ax.set_ylim(EXT[2:])
    ax.tick_params(labelsize=8)
    for s in ax.spines.values():s.set_color('#b8c5d4')


def title(ax,value):ax.set_title(value,fontsize=12,pad=12,fontweight='bold')


def route(ax,points,color=BLUE,lw=.8,alpha=1,**kwargs):
    pts=np.asarray(points)
    if len(pts):ax.plot(pts[:,0],pts[:,1],color=color,lw=lw,alpha=alpha,**kwargs)


def poly(ax,c,lw=1,color=None,label_text=False):
    p=np.asarray(c['polygon']);color=color or COLORS[c['id']]
    if len(p)>2:ax.fill(p[:,0],p[:,1],facecolor=color,edgecolor=color,alpha=.38,lw=lw,zorder=3)
    elif len(p):ax.scatter(p[:,0],p[:,1],c=color,s=22,zorder=4)
    if label_text:
        ax.text(*c['center'],c['name'],ha='center',va='center',fontsize=8,zorder=8,
                path_effects=[pe.withStroke(linewidth=2,foreground='white')])


def crop_of(points,pad=.65,min_span=1.2):
    p=np.asarray(points);lo=p.min(axis=0);hi=p.max(axis=0);mid=(lo+hi)/2
    span=np.maximum(hi-lo+pad*2,min_span)
    return [max(0,mid[0]-span[0]/2),max(0,mid[1]-span[1]/2),
            min(W*R,mid[0]+span[0]/2),min(H*R,mid[1]+span[1]/2)]


def done(fig,name,subtitle=None,vector=False):
    if subtitle:fig.suptitle(subtitle,fontsize=15,fontweight='bold',y=1.10,va='bottom')
    fig.savefig(OUT/(name+'.png'),dpi=155,bbox_inches='tight',facecolor=fig.get_facecolor())
    if vector:fig.savefig(OUT/(name+'.svg'),bbox_inches='tight')
    plt.close(fig)


def numbered(ax,cells):
    # Every cell gets a callout; point-sized cells must not disappear in an overview.
    for c in cells:poly(ax,c,lw=.3)
    left=sorted([c for c in cells if c['center'][0]<32],key=lambda c:c['center'][1])
    right=sorted([c for c in cells if c['center'][0]>=32],key=lambda c:c['center'][1])
    for group,xx,ha in [(left,8,'right'),(right,61,'left')]:
        ys=np.linspace(12,80,len(group))
        for c,yy in zip(group,ys):
            ax.annotate(c['name'],xy=c['center'],xytext=(xx,yy),ha=ha,va='center',fontsize=8,
                color=INK,arrowprops=dict(arrowstyle='-',color=COLORS[c['id']],lw=.65),
                bbox=dict(boxstyle='round,pad=.16',facecolor='white',edgecolor=COLORS[c['id']],lw=.6),zorder=9)
    ax.set_xlim(5,66)


def rotation_figure():
    """Compare the same F with matching upward axes; isolate the native row display."""
    affine=np.asarray(D['rotation']['R'],float);linear=affine[:,:2]
    f=M['f'];native=(NATIVE>127)
    reproduced=cv2.warpAffine(f.astype(np.uint8)*255,affine,(NATIVE.shape[1],NATIVE.shape[0]),flags=cv2.INTER_AREA)
    assert np.array_equal((reproduced>127),native)
    assert np.allclose(linear.T@linear,np.eye(2),atol=1e-12)
    assert np.isclose(np.linalg.det(linear),1,atol=1e-12)
    theta=float(np.degrees(np.arctan2(linear[1,0],linear[0,0])))
    assert np.isclose(theta,-S['native_rotation_deg'])
    safe_interior=f&(distance_transform_edt(f)>5)
    yy,xx=np.nonzero(safe_interior);free_points=np.column_stack((xx,yy))
    points=np.array([free_points[np.argmin(np.linalg.norm(free_points-(np.array(target)/R-.5),axis=1))]
                     for target in [(19.,20.),(28.,61.),(51.,60.)]],float)
    rotated=points@linear.T+affine[:,2]
    assert f[points[:,1].astype(int),points[:,0].astype(int)].all()
    assert native[np.rint(rotated[:,1]).astype(int),np.rint(rotated[:,0]).astype(int)].all()
    span=max(max(np.ptp(np.column_stack(np.nonzero(a)),axis=0)) for a in [f,native])+100
    fig,axes=plt.subplots(1,3,figsize=(18,7.3),constrained_layout=True)
    palette=ListedColormap(['#263747','#f4f8fc'])
    def frame(ax,array,upward):
        ax.imshow(array,cmap=palette,vmin=0,vmax=1,origin='lower' if upward else 'upper',interpolation='nearest')
        y,x=np.nonzero(array);cx=(x.min()+x.max())/2;cy=(y.min()+y.max())/2
        ax.set_xlim(cx-span/2,cx+span/2)
        ax.set_ylim(cy-span/2,cy+span/2) if upward else ax.set_ylim(cy+span/2,cy-span/2)
        ax.set_aspect('equal');ax.tick_params(labelsize=8)
        ax.set_xlabel('x / px' if array is f else 'x′ / px')
        ax.set_ylabel('y / px（向上）' if array is f else ('y′ / px（向上）' if upward else '原生行 y′ / px（向下）'))
    frame(axes[0],f,True);frame(axes[1],native,True);frame(axes[2],native,False)
    for x0,y0,x1,y1 in D['hough']['lines']:
        axes[0].plot([x0,x1],[y0,y1],color=ORANGE,lw=.5,alpha=.6)
    title(axes[0],'07 旋转前：实际输入 F\n统一 y 向上；橙线为 Hough 检测')
    title(axes[1],f'08 同一 F：顺时针 {abs(theta):.6f}°\n统一 y′ 向上，可直接与左图比较')
    title(axes[2],'补充：同一输出的原生行号视图\ny′ 向下，显示上发生上下翻转')
    mark_colors=['#ffb743','#ff68ad','#52d5ca']
    for ax,ps in [(axes[0],points),(axes[1],rotated),(axes[2],rotated)]:
        for name,p,color in zip('ABC',ps,mark_colors):
            ax.scatter(*p,s=65,facecolors=color,edgecolors='white',lw=1,zorder=9)
            ax.annotate(name,p,xytext=(8,7),textcoords='offset points',fontsize=13,fontweight='bold',color=color,
                        path_effects=[pe.withStroke(linewidth=2,foreground='#263747')],zorder=10)
    anchor=np.asarray(P['start'])/R-.5;anchor_rotated=linear@anchor+affine[:,2]
    basis=[np.array([150.,0]),np.array([0.,150.])]
    for ax,origin,vectors in [(axes[0],anchor,basis),(axes[1],anchor_rotated,[linear@v for v in basis])]:
        for text,vec,color in zip(['输入 +x','输入 +y'],vectors,[RED,BLUE]):
            endpoint=origin+vec
            assert ax.get_xlim()[0]<endpoint[0]<ax.get_xlim()[1]
            assert ax.get_ylim()[0]<endpoint[1]<ax.get_ylim()[1]
            ax.annotate('',origin+vec,xytext=origin,arrowprops=dict(arrowstyle='-|>',color=color,lw=2),zorder=8)
            ax.annotate(text,origin+vec,xytext=(5,5),textcoords='offset points',fontsize=9,color=color,
                        path_effects=[pe.withStroke(linewidth=2,foreground='white')],zorder=11)
    # Runtime assertions cover the plotted axes, not just a textual claim about them.
    assert axes[0].get_ylim()[0]<axes[0].get_ylim()[1]
    assert axes[1].get_ylim()[0]<axes[1].get_ylim()[1]
    assert axes[2].get_ylim()[0]>axes[2].get_ylim()[1]
    evidence=dict(input='同一 F 起点可行分量',linear=linear.tolist(),translation=affine[:,2].tolist(),
        determinant=float(np.linalg.det(linear)),orthogonality=(linear.T@linear).tolist(),
        angle_y_up_degrees=theta,opencv_angle_degrees=S['native_rotation_deg'],
        x_direction_after=linear[:,0].tolist(),y_direction_after=linear[:,1].tolist(),
        landmark_names=list('ABC'),landmarks_before=points.tolist(),landmarks_after=rotated.tolist(),
        native_mask_reproduced_exactly=True,
        primary_axes_y_up=[True,True],native_supplement_y_down=True,
        plotted_ylim=[list(ax.get_ylim()) for ax in axes],plotted_span_pixels=float(span))
    (HERE/'evidence/rotation_display.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n')
    done(fig,'03_rotation','左、中两图同一 F / 同一像素尺度 / y 均向上｜A、B、C 一一对应；右图单独解释原生显示翻转',vector=True)


def stages():
    print('Rendering process figures',flush=True)
    fig,axes=plt.subplots(1,3,figsize=(15,9),constrained_layout=True)
    base(axes[0]);title(axes[0],'01 原始 PNG：1208 × 1710')
    base(axes[1]);mask(axes[1],M['free'],GREEN,.6);title(axes[1],'02 已知自由像素：507,209 格')
    base(axes[2]);mask(axes[2],M['free']&~M['target'],ORANGE,.85);mask(axes[2],M['target'],BLUE,.55)
    title(axes[2],'03 最大四连通区：470,743 格')
    done(fig,'01_input','输入筛选｜橙色为未进入最大连通区的自由像素')

    target=np.flipud(M['target']);distance=distance_transform_edt(np.pad(target,1))[1:-1,1:-1]
    fig,axes=plt.subplots(1,3,figsize=(15,9),constrained_layout=True)
    base(axes[0]);im=axes[0].imshow(np.ma.masked_where(~M['target'],np.flipud(distance)*R),origin='lower',extent=EXT,cmap='viridis',zorder=2)
    axes[0].scatter(*P['start'],s=55,marker='*',c='red',zorder=6);title(axes[0],'04 EDT 距离与最大净空起点')
    fig.colorbar(im,ax=axes[0],shrink=.45,label='距非目标像素中心 / m')
    base(axes[1]);mask(axes[1],M['target']&~M['navigable'],ORANGE,.7);mask(axes[1],M['navigable'],BLUE,.55)
    title(axes[1],'05 距离 > 20 像素：295,111 格')
    base(axes[2]);mask(axes[2],M['navigable']&~M['f'],RED,.9);mask(axes[2],M['f'],GREEN,.65)
    axes[2].scatter(*P['start'],s=60,marker='*',c='#ffd52a',edgecolors=INK,zorder=7)
    title(axes[2],'06 起点可达 F：278,927 格')
    done(fig,'02_clearance','1 m 净空仅在外层应用一次｜红色 16,184 格与起点断连')

    rotation_figure()

    fig,axes=plt.subplots(1,2,figsize=(14,9),constrained_layout=True)
    base(axes[0],safe=True)
    for e in D['events']:
        for cut in e['cuts_world']:route(axes[0],cut,RED if e['type']=='IN' else PURPLE,1.1)
    title(axes[0],'09 全部实际切割线映回原图')
    axes[1].imshow(NATIVE,cmap='gray',interpolation='nearest')
    for e in D['events']:
        for x0,y,x1,_ in e['cuts']:axes[1].plot([x0,x1],[y,y],color=RED if e['type']=='IN' else PURPLE,lw=1)
    title(axes[1],'10 在旋转图中逐行扫描')
    axes[1].set_xlabel('x′ / px');axes[1].set_ylabel('y′ / px')
    done(fig,'04_all_cuts','66 次候选事件 → 30 次有效切割事件 → 79 段连续边界像素｜红 IN，紫 OUT')

    examples=[next(e for e in D['events'] if e['index']==7),next(e for e in D['events'] if e['index']==6)]
    fig,axes=plt.subplots(1,2,figsize=(14,7),constrained_layout=True)
    for ax,e in zip(axes,examples):
        points=[p for line in e['cuts_world'] for p in line]+e['critical_world']
        base(ax,crop_of(e['critical_world'],pad=1.3,min_span=3),safe=True)
        for cut in e['cuts_world']:route(ax,cut,RED if e['type']=='IN' else PURPLE,2)
        if e['critical_world']:
            pts=np.asarray(e['critical_world']);ax.scatter(pts[:,0],pts[:,1],s=11,c='#ffce19',edgecolors=INK,lw=.2,zorder=9)
        title(ax,f'E{e["index"]:02} / {e["type"]}：片段 {e["before"]} → {e["after"]}，原生行 {e["y"]}')
    done(fig,'05_event_zoom','真实事件放大｜黄点是源码邻域检查通过的黑像素，不等同于独立障碍数量')

    cmap=ListedColormap(['#273141',*COLORS])
    fig,axes=plt.subplots(1,2,figsize=(14,7),constrained_layout=True)
    for ax,name,head in zip(axes,['03_labels_before_merge','04_labels_final'],['11 切割后 floodFill：80 个标签','12 边界像素归属后：仍为 80 个标签']):
        img=np.asarray(Image.open(HERE/f'evidence/native_trace/{name}.png'))
        ax.imshow(img,cmap=cmap,vmin=0,vmax=80,interpolation='nearest');title(ax,head)
        ax.set_xlim(360,1380);ax.set_ylim(1040,170);ax.set_xlabel('x′ / px');ax.set_ylabel('y′ / px')
    done(fig,'06_label_stages','min_cell_area = 0、min_cell_width = 0：本轮没有小 cell 合并；切割线随后分配到邻区')

    fig,ax=plt.subplots(figsize=(12,14),constrained_layout=True);base(ax);numbered(ax,D['cells'])
    title(ax,'80 块原生 BCD 区域｜C00–C79，一块不少')
    done(fig,'07_all_80_cells',vector=True)

    big=D['cells'][38];sweep=T['sweeps'][38]
    fig,axes=plt.subplots(1,3,figsize=(17,7),constrained_layout=True)
    base(axes[0],crop_of(big['polygon']),safe=True);poly(axes[0],big)
    for line in big['grid']:route(axes[0],line,BLUE,.8)
    title(axes[0],'13 C38：局部扫线映回原图')
    local_grid(axes[1],38);title(axes[1],f'14 cell 再旋转 {big["local_angle_deg"]:.5f}°')
    base(axes[2],crop_of(big['polygon']),safe=True);route(axes[2],big['raw_path'],ORANGE,.7)
    if big['raw_path']:
        axes[2].scatter(*big['raw_path'][0],c=GREEN,s=35,zorder=9);axes[2].scatter(*big['raw_path'][-1],c=RED,s=35,zorder=9)
    title(axes[2],f'15 四角距离选择 → 有序路径（{big["raw_points"]} 点）')
    done(fig,'08_cell_sweep','行距 floor(0.6 / √2 / 0.05) = 8 px = 0.40 m；半格 4 px；沿行采样 1 px')

    fig,axes=plt.subplots(1,2,figsize=(14,9),constrained_layout=True)
    for ax in axes:base(ax,safe=True)
    for c in D['cells']:
        route(axes[0],c['raw_path'],COLORS[c['id']],.4,.8)
        axes[1].add_collection(LineCollection(c['safe_pieces'],colors=COLORS[c['id']],linewidths=.5))
    title(axes[0],'16 原生 cell 路径：46,243 个点（未连接各 cell）')
    title(axes[1],'17 在原分辨率 F 上裁剪：39,465 个安全片段')
    done(fig,'09_raw_safe','原生候选长度之和 2321.87 m → 安全片段长度之和 2263.49 m；均不是最终整路长度')

    fig,axes=plt.subplots(1,2,figsize=(14,9),constrained_layout=True)
    for ax in axes:base(ax,safe=True)
    for c in D['cells']:
        if c['visit_order']:
            route(axes[0],PATH[c['first_vertex']:c['last_vertex']+1],COLORS[c['id']],.55)
    for st in D['schedule']:
        point=PATH[st['first_vertex']+1]
        axes[0].text(*point,str(st['step']),fontsize=6,ha='center',zorder=9,
                    path_effects=[pe.withStroke(linewidth=2,foreground='white')])
    for line in D['astar']:route(axes[1],line,PURPLE,1.2,.9)
    axes[1].scatter(*P['start'],s=75,marker='*',c=ORANGE,zorder=9)
    title(axes[0],'18 最终访问顺序：61 个 cell；数字为访问序号')
    title(axes[1],'19 实际写入路线的 209 次 A* 连接')
    done(fig,'10_order_connections','532 次 A* 搜索 = 323 次入口候选评价 + 209 次实际路线连接')

    st=D['schedule'][0]
    fig,(ax,tax)=plt.subplots(1,2,figsize=(15,8),gridspec_kw={'width_ratios':[1.25,1]},constrained_layout=True)
    points=[st['here']]+[c['point'] for c in st['candidates']]
    base(ax,crop_of(points,pad=1),safe=True)
    for c in st['candidates']:
        chosen=c['region']==st['region'] and c['reverse']==st['reverse']
        route(ax,c['route'],GREEN if chosen else '#8c9cae',2 if chosen else .7,.95)
        ax.scatter(*c['point'],c=GREEN if chosen else ORANGE,s=30,zorder=8)
        ax.annotate(f'C{c["region"]:02} {"反" if c["reverse"] else "正"}',c['point'],fontsize=8,xytext=(3,5),textcoords='offset points')
    ax.scatter(*st['here'],c=RED,marker='*',s=85,zorder=9);title(ax,'20 首轮候选真实接近路径；绿色为胜出候选')
    tax.axis('off');rows=[[f'C{c["region"]:02}', '反向' if c['reverse'] else '正向',f'{c["euclidean_m"]:.4f}',f'{c["length_m"]:.4f}'] for c in st['candidates']]
    table=tax.table(cellText=rows,colLabels=['cell','方向','直线距离 / m','安全路线 / m'],loc='center',cellLoc='center',colWidths=[.18,.18,.31,.33])
    table.auto_set_font_size(False);table.set_fontsize(11);table.scale(1,2)
    title(tax,'21 先按直线距离选 8 个，再比较真实安全距离')
    done(fig,'11_scheduler_first','本轮胜出 C38 正向，接近长度 7.343333 m｜其余 60 轮的完整候选表见 HTML')

    chosen=sorted(enumerate(D['astar_metadata']),key=lambda x:x[1]['length_m']-x[1]['direct_m'],reverse=True)[:3]
    fig,axes=plt.subplots(1,3,figsize=(17,7),constrained_layout=True)
    for ax,(index,meta) in zip(axes,chosen):
        line=D['astar'][index];base(ax,crop_of(line,pad=.7),safe=True)
        route(ax,[line[0],line[-1]],RED,1.4,ls='--');route(ax,line,PURPLE,2)
        ax.scatter(*line[0],s=35,c=GREEN,zorder=9);ax.scatter(*line[-1],s=35,c=RED,zorder=9)
        title(ax,f'C{meta["region"]:02} 接近：直线 {meta["direct_m"]:.2f} m\n实际安全连接 {meta["length_m"]:.2f} m')
    done(fig,'12_astar_zoom','红虚线仅表示待连接端点间的直线｜紫色才是写入最终路线的 A* 路径')

    fig,axes=plt.subplots(1,2,figsize=(14,9),constrained_layout=True)
    base(axes[0]);route(axes[0],PATH,BLUE,.45)
    axes[0].scatter(*PATH[0],s=70,c=GREEN,marker='*',zorder=9);axes[0].scatter(*PATH[-1],s=40,c=RED,zorder=9)
    title(axes[0],'22 最终连续路线：2493.437449 m')
    base(axes[1]);mask(axes[1],M['free']&~M['covered'],ORANGE,.7);mask(axes[1],M['free']&M['covered'],GREEN,.72)
    mask(axes[1],M['f']&~M['covered'],RED,1.)
    title(axes[1],'23 0.6 m 圆盘扫掠：全自由区覆盖 59.384790%')
    done(fig,'13_final','绿：已覆盖自由格；橙：未覆盖自由格；红：F 内未覆盖格｜路线包含全部接近、绕行和重复')

    residual=M['f']&~M['covered'];lab,n=label(residual);sizes=np.bincount(lab.ravel());sizes[0]=0
    ids=np.argsort(sizes)[-4:][::-1]
    fig,axes=plt.subplots(1,4,figsize=(18,5.5),constrained_layout=True)
    for ax,index in zip(axes,ids):
        yy,xx=np.nonzero(lab==index);pts=np.column_stack((xx+.5,yy+.5))*R
        base(ax,crop_of(pts,pad=.4,min_span=1),safe=True);mask(ax,residual,RED,1)
        route(ax,PATH,BLUE,.8)
        title(ax,f'残余连通片 {index}：{len(xx)} 格\n{len(xx)*R*R:.4f} m²')
    done(fig,'14_residual_zoom',f'F 内尚余 147 格 / {147*R*R:.4f} m²，分为 {n} 个四连通小片｜没有调用 Local 补扫')


def local_grid(ax,index):
    sweep=T['sweeps'][index];p=np.asarray(sweep['polygon_rotated'],np.int32)
    image=np.zeros((p[:,1].max()+10,p[:,0].max()+10),np.uint8)
    cv2.drawContours(image,[p],-1,255,cv2.FILLED)
    shape=sweep['rotated_shape'];rot=cv2.warpAffine(image,np.asarray(sweep['R_cell']),tuple(shape),flags=cv2.INTER_AREA)
    rot=(rot>127).astype(np.uint8)
    assert int(rot.sum())==sweep['rotated_white_pixels']
    ax.imshow(rot,cmap=ListedColormap(['#d5dde7','#ffffff']),vmin=0,vmax=1,interpolation='nearest')
    points=[]
    for line in sweep['grid']:
        for key in ['upper','lower']:
            if line[key]:
                q=np.asarray(line[key]);ax.plot(q[:,0],q[:,1],color=BLUE,lw=.9,marker='.',markersize=1.4);points.extend(line[key])
    if points:
        q=np.asarray(points);lo=q.min(axis=0)-5;hi=q.max(axis=0)+5
    else:
        q=cv2.transform(p.astype(np.float32)[None],np.asarray(sweep['R_cell']))[0];lo=q.min(axis=0)-4;hi=q.max(axis=0)+4
        ax.scatter(q[:,0],q[:,1],marker='s',s=90,facecolors='none',edgecolors=RED,lw=1)
        ax.text(.5,.95,'红框：原始像素中心投影\n重采样后二值图没有白格',transform=ax.transAxes,ha='center',va='top',fontsize=9,color=RED)
    ax.set_xlim(lo[0],hi[0]);ax.set_ylim(hi[1],lo[1]);ax.set_aspect('equal')
    ax.set_xlabel('cell 局部列 / px');ax.set_ylabel('cell 局部行 / px');ax.tick_params(labelsize=8)
    if max(hi-lo)<25:
        ax.set_xticks(np.arange(math.ceil(lo[0]),math.floor(hi[0])+1),minor=True)
        ax.set_yticks(np.arange(math.ceil(lo[1]),math.floor(hi[1])+1),minor=True)
        ax.grid(which='minor',color='#aab9c9',lw=.4)


def cards():
    folder=OUT/'cells';folder.mkdir(exist_ok=True)
    for c in D['cells']:
        i=c['id'];fig,axes=plt.subplots(1,3,figsize=(15,5.5),constrained_layout=True)
        crop=crop_of(c['polygon'],pad=.35,min_span=.8)
        base(axes[0],crop,safe=True);poly(axes[0],c,lw=1.4)
        centre=c['center'];axes[0].scatter(*centre,s=45,facecolors='none',edgecolors=RED,lw=1,zorder=8)
        title(axes[0],f'{c["name"]} 原图位置｜原生标签 {c["label"]}\n面积 {c["native_area_pixels"]} 格 = {c["area_m2"]:.4f} m²')
        local_grid(axes[1],i);title(axes[1],f'局部旋转 {c["local_angle_deg"]:.4f}°\n{c["grid_rows"]} 条网格线 / {c["grid_points"]} 个网格点')
        base(axes[2],crop,safe=True);route(axes[2],c['raw_path'],'#9ca7b2',1.4)
        axes[2].add_collection(LineCollection(c['safe_pieces'],colors=ORANGE,linewidths=1.6,zorder=5))
        if c['visit_order']:
            route(axes[2],PATH[c['first_vertex']:c['last_vertex']+1],BLUE,.8,zorder=6)
            for s in D['astar_metadata']:
                if s['region']==i:route(axes[2],PATH[s['first_vertex']:s['last_vertex']+1],PURPLE,1.4,zorder=7)
        title(axes[2],f'原生 {c["raw_points"]} 点 → 安全 {c["safe_piece_count"]} 段\n'+(f'第 {c["visit_order"]} 个访问 / A* {c["astar_sections"]} 次' if c['visit_order'] else c['status']))
        fig.suptitle(f'{c["name"]}｜灰：原生候选；橙：安全片段；蓝：本区最终路线；紫：A*（接近可超出当前局部框）',fontsize=13,y=1.04)
        fig.savefig(folder/f'C{i:02}.png',dpi=130,bbox_inches='tight');plt.close(fig)
        if (i+1)%10==0:print(f'  cells {i+1}/80',flush=True)
    for page in range(10):
        fig,axes=plt.subplots(2,4,figsize=(16,9),constrained_layout=True)
        for ax,c in zip(axes.ravel(),D['cells'][page*8:(page+1)*8]):
            base(ax,crop_of(c['polygon'],pad=.4,min_span=1),safe=True);poly(ax,c,lw=1)
            route(ax,c['raw_path'],BLUE,.65)
            title(ax,f'{c["name"]}｜{c["area_m2"]:.4f} m²\n'+(f'访问 #{c["visit_order"]}' if c['visit_order'] else '未单独访问'))
        done(fig,f'atlas_{page+1:02}',f'全部分区放大索引 {page+1}/10｜C{page*8:02}–C{page*8+7:02}（每框比例不同）')


def web_assets():
    Image.fromarray(IMAGE).save(OUT/'map.png')
    for name,mask_value,color in [('f',M['f'],BLUE),('covered',M['free']&M['covered'],GREEN),
                                 ('residual',M['free']&~M['covered'],ORANGE),('f_residual',M['f']&~M['covered'],RED)]:
        rgba=np.zeros((*mask_value.shape,4),np.uint8);rgba[mask_value,:3]=np.array(matplotlib.colors.to_rgb(color))*255;rgba[mask_value,3]=150
        Image.fromarray(np.flipud(rgba)).save(OUT/(name+'.png'))
    np.savez_compressed(HERE/'evidence/display_labels.npz',labels_original=ORIGINAL_LABELS)
    rgba=np.zeros((H,W,4),np.uint8)
    for i in range(80):rgba[ORIGINAL_LABELS==i+1,:3]=np.array(matplotlib.colors.to_rgb(COLORS[i]))*255;rgba[ORIGINAL_LABELS==i+1,3]=150
    Image.fromarray(np.flipud(rgba)).save(OUT/'native_cells.png')


if __name__=='__main__':
    web_assets();stages();cards()
