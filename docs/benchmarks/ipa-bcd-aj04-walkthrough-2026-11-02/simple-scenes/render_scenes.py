#!/usr/bin/env python3
"""Draw four teachable cases directly from IPA native trace and label images."""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt

HERE=Path(__file__).resolve().parent
EVIDENCE=HERE/'evidence';ASSETS=HERE/'assets';ASSETS.mkdir(exist_ok=True)
BG='#f4f7fb';INK='#18334c';BLACK='#25394a';BLUE='#1381bc';YELLOW='#ffda40';RED='#e04b4b'
PALETTE=['#d7eafd','#c9eddf','#ffe6b8','#e4d8f4']
plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False,
 'figure.facecolor':BG,'text.color':INK,'svg.fonttype':'none','svg.hashsalt':'ipa-classic-scenes'})


def read(name):
    root=EVIDENCE/name
    return dict(mask=np.asarray(Image.open(root/'input.png')),
        final=np.asarray(Image.open(root/'04_labels_final.png')),
        cut=np.asarray(Image.open(root/'02_cuts.png')),
        trace=json.loads((root/'trace.json').read_text()))


def draw_grid(ax, scene, *, scan=None, points=(), event_cuts=(), final=False, failed=None, check=None):
    mask=scene['mask'];labels=scene['final'];h,w=mask.shape
    if final:
        ax.imshow(labels,cmap=ListedColormap([BLACK]+PALETTE),vmin=0,vmax=4,origin='upper',interpolation='nearest')
    else:
        ax.imshow(mask==255,cmap=ListedColormap([BLACK,'white']),vmin=0,vmax=1,origin='upper',interpolation='nearest')
    # The background grid is intentional: each square is one input pixel.
    ax.vlines(np.arange(-.5,w,.99999999999),-.5,h-.5,colors='#8594a0',alpha=.24,lw=.28,zorder=2)
    ax.hlines(np.arange(-.5,h,.99999999999),-.5,w-.5,colors='#8594a0',alpha=.24,lw=.28,zorder=2)
    if scan is not None:
        ax.annotate('',xy=(1,21),xytext=(1,2),
                    arrowprops=dict(arrowstyle='-|>',color=BLUE,lw=1.5),zorder=5)
        for row in (scan if isinstance(scan,list) else [scan]):
            ax.plot([1,28],[row,row],color=BLUE,lw=2,ls=(0,(5,3)),zorder=5)
    for e in event_cuts:
        for x0,y,x1,_ in e['cuts']:
            for x in range(x0,x1+1):ax.add_patch(Rectangle((x-.5,y-.5),1,1,facecolor=RED,edgecolor='none',zorder=4))
    if points:
        p=np.asarray(points);ax.scatter(p[:,0],p[:,1],s=25,c=YELLOW,edgecolors=BLACK,lw=.35,zorder=6)
    if failed:
        x,y=failed
        ax.add_patch(Rectangle((x-1.5,y-1.5),3,1,fill=False,edgecolor=BLUE,lw=2,zorder=7))
        ax.scatter([x],[y],s=120,facecolors='none',edgecolors=BLUE,lw=2,zorder=7)
    if check:
        x,neighbor_y=check
        ax.add_patch(Rectangle((x-1.5,neighbor_y-.5),3,1,fill=False,edgecolor=BLUE,lw=2,zorder=7))
    if final:
        edges=[]
        for y in range(h):
            for x in range(w):
                a=int(labels[y,x])
                if not a:continue
                if x+1<w and labels[y,x+1] and labels[y,x+1]!=a:edges.append([(x+.5,y-.5),(x+.5,y+.5)])
                if y+1<h and labels[y+1,x] and labels[y+1,x]!=a:edges.append([(x-.5,y+.5),(x+.5,y+.5)])
        ax.add_collection(LineCollection(edges,colors=BLACK,linewidths=1.8,zorder=8))
        for label_id in range(1,int(labels.max())+1):
            zone=labels==label_id;distance=distance_transform_edt(zone)
            y,x=np.unravel_index(np.argmax(distance),distance.shape)
            ax.text(x,y,chr(64+label_id),ha='center',va='center',fontsize=19,fontweight='bold',color=INK,zorder=9,
                    bbox=dict(boxstyle='round,pad=.15',facecolor='white',edgecolor='none',alpha=.84))
    ax.set(xlim=(-.5,w-.5),ylim=(h-.5,-.5),aspect='equal');ax.set_xticks([]);ax.set_yticks([])
    for spine in ax.spines.values():spine.set_color('#b5c5d0')


def frame(ax,scene,title,caption,**kwargs):
    draw_grid(ax,scene,**kwargs)
    ax.set_title(title,fontsize=15,fontweight='bold',loc='left',pad=8)
    ax.text(.5,-.075,caption,ha='center',va='top',fontsize=11,transform=ax.transAxes,linespacing=1.45)


def save(fig,name):
    for ext in ['png','svg']:fig.savefig(ASSETS/f'{name}.{ext}',dpi=155,facecolor=BG,bbox_inches='tight')
    plt.close(fig)


def save_panels(name,scene,panels):
    # Individual frames keep each step readable on a narrow browser or phone.
    for i,(heading,caption,kwargs) in enumerate(panels,1):
        fig,ax=plt.subplots(figsize=(5.2,5.3))
        fig.subplots_adjust(left=.06,right=.98,top=.86,bottom=.16)
        frame(ax,scene,heading,caption,**kwargs)
        fig.savefig(ASSETS/f'{name}-{i:02}.png',dpi=145,facecolor=BG,bbox_inches='tight',pad_inches=.08)
        plt.close(fig)


def simple(name,title,subtitle,panels):
    scene=read(name);fig,axes=plt.subplots(1,4,figsize=(18,5.8))
    fig.subplots_adjust(left=.025,right=.985,top=.73,bottom=.17,wspace=.15)
    fig.text(.03,.96,title,fontsize=23,fontweight='bold',va='top')
    fig.text(.03,.88,subtitle,fontsize=13,va='top')
    for ax,(heading,caption,kwargs) in zip(axes,panels):frame(ax,scene,heading,caption,**kwargs)
    fig.text(.03,.05,'蓝虚线：扫描行（不是机器人路线）　白：可通行　深色：不可通行　彩色：最终区域',fontsize=11)
    save(fig,name)
    save_panels(name,scene,panels)


def two_events(name,title,subtitle,panels):
    scene=read(name);ev=scene['trace']['events']
    fig,axes=plt.subplots(2,3,figsize=(16,10.4))
    fig.subplots_adjust(left=.045,right=.975,top=.83,bottom=.11,hspace=.42,wspace=.15)
    fig.text(.05,.975,title,fontsize=23,fontweight='bold',va='top')
    fig.text(.05,.916,subtitle,fontsize=13,va='top')
    steps=panels(ev)
    for ax,(heading,caption,kwargs) in zip(axes.flat,steps):frame(ax,scene,heading,caption,**kwargs)
    fig.text(.05,.035,'蓝虚线：扫描行　蓝框：检查三个格　黄点：检查通过的黑格　红格：临时边界　彩色：最终区域　黑细线：区域交界',fontsize=10)
    save(fig,name)
    save_panels(name,scene,steps)


def main():
    (EVIDENCE/'figures.json').unlink(missing_ok=True)
    source=json.loads((EVIDENCE/'verification.json').read_text());assert source['passed']
    simple('empty','场景 1｜空房间：一直只有一段','没有障碍，也没有内部切割，最终 1 个区域。',[
        ('① 输入','白色是能走的区域',{}),
        ('② 逐行扫描','上、中、下都是 1 段',dict(scan=[5,11,18])),
        ('③ 扫出房间','1 → 0；检查后没有画界',dict(scan=21)),
        ('④ 编号结果','整体连通：1 块区域 A',dict(final=True))])
    two_events('island','场景 2｜中央有障碍：从 1 段分成 2 段，再汇合','先在障碍开头切，再在障碍末尾切：最终上、左、右、下共 4 区。',lambda e:[
        ('① 输入','中间有一个黑色障碍',{}),
        ('② 扫到障碍：1 → 2','IN：黄点的上方三格全白',dict(scan=8,points=e[0]['critical_points'],check=(12,7))),
        ('③ 从黄点画界','在当前行向两边标红',dict(scan=8,points=e[0]['critical_points'],event_cuts=e[:1])),
        ('④ 扫过障碍：2 → 1','OUT：黄点的下方三格全白',dict(scan=14,points=e[1]['critical_points'],event_cuts=e[:1],check=(12,14))),
        ('⑤ 再画一条分界','OUT 回到上一行标红',dict(scan=14,points=e[1]['critical_points'],event_cuts=e[:2])),
        ('⑥ 编号结果','临时边界归区，最终 4 区',dict(final=True))])
    two_events('meeting','场景 3｜两条入口后来汇合：有 IN 不一定画界','先出现第二条白段，但检查失败；两条入口汇合时 OUT 才画界。最终 3 区。',lambda e:[
        ('① 输入','两条入口在下面相连',{}),
        ('② 新白段出现：1 → 2','IN：进入检查',dict(scan=7)),
        ('③ 检查失败，不画界','蓝框示例：上方三格不是全白',dict(scan=7,failed=(9,7))),
        ('④ 两段汇合：2 → 1','OUT：黄点的下方三格全白',dict(scan=14,points=e[1]['critical_points'],check=(9,14))),
        ('⑤ 补画临时边界','在上一行，左右白格标红',dict(scan=14,points=e[1]['critical_points'],event_cuts=e[:2])),
        ('⑥ 编号结果','左入口、右入口、大厅：3 区',dict(final=True))])
    simple('notch','场景 4｜侧墙凸起：通道变窄，但白段仍是 1 段','宽度变化不等于分叉；这个例子没有内部切割，最终仍是 1 区。',[
        ('① 输入','障碍与右侧墙连在一起',{}),
        ('② 上方扫描','白色有 1 段',dict(scan=5)),
        ('③ 狭窄处扫描','还是 1 段，不产生内部分界',dict(scan=10)),
        ('④ 编号结果','扫出房间后无新增界：1 区',dict(final=True))])
    artifact={}
    for name in source['results']:
        artifact[name]={ext:hashlib.sha256((ASSETS/f'{name}.{ext}').read_bytes()).hexdigest() for ext in ['png','svg']}
        artifact[name]['frames']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(ASSETS.glob(f'{name}-??.png'))}
    (EVIDENCE/'figures.json').write_text(json.dumps(dict(passed=True,assets_sha256=artifact,
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),ensure_ascii=False,indent=2)+'\n')
    print('Rendered 4 native-traced visual explanations (PNG and SVG)')


if __name__=='__main__':main()
