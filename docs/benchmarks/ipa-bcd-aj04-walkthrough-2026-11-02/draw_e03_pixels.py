#!/usr/bin/env python3
"""Draw exact E03 native pixels, with before/after borders and explicit coordinate grids."""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle, Patch
from matplotlib.lines import Line2D
import numpy as np
from PIL import Image

HERE=Path(__file__).resolve().parent
OUT=HERE/'assets'
BG='#f4f7fb';INK='#18334c';BLACK='#293b4c';RED='#dd4b4b';YELLOW='#ffe136';DASH='#b38b00';BLUE='#147db3'
plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False,'text.color':INK,
    'svg.fonttype':'none','svg.hashsalt':'e03-exact-pixels'})
SOURCE=HERE/'evidence/native_trace/01_rotated_room.png'
TRACE=HERE/'evidence/native_trace/trace.json'
native=np.asarray(Image.open(SOURCE))
trace=json.loads(TRACE.read_text());event=trace['events'][2]
assert (event['type'],event['y'],event['before'],event['after'])==('OUT',258,3,2)
assert not any(e['cuts'] for e in trace['events'][:2])
white=native==255
cp=np.asarray(event['critical_points'],int)
cut=np.zeros_like(white)
for x0,y,x1,_ in event['cuts']:cut[y,x0:x1+1]=True


def audit():
    # Independently derive eligible black pixels and apply the upstream OUT loop.
    current=258;previous=257;first=int(np.flatnonzero(white[previous])[0])
    found=[(x,previous) for x in range(first+1,white.shape[1])
           if not white[previous,x] and white[current,max(0,x-1):min(white.shape[1],x+2)].all()]
    assert np.array_equal(found,cp)
    replay=np.zeros_like(white)
    for x,y in found:
        for direction in [-1,1]:
            xx=x+direction
            while 0<=xx<white.shape[1]:
                if not white[y,xx]:break
                if white[y-1,xx]:replay[y,xx]=True
                xx+=direction
    assert np.array_equal(replay,cut)
    assert len(cp)==196 and cut.sum()==66
    assert event['cuts']==[[930,257,993,257],[1311,257,1312,257]]
    assert white[257,994:1000].all() and not white[256,994:1000].any()
    assert white[257,1310] and not white[256,1310]
    return dict(passed=True,event='E03',critical_pixels=len(cp),border_pixels=int(cut.sum()),
        border_runs=event['cuts'],independent_local_event_replay=True,
        source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        trace_sha256=hashlib.sha256(TRACE.read_bytes()).hexdigest())


def save(fig,name):
    for ext in ['png','svg']:fig.savefig(OUT/f'{name}.{ext}',dpi=170,facecolor=BG)
    plt.close(fig)


def legend(fig,y):
    items=[Patch(facecolor='white',edgecolor='#9baeba',label='白格：可行'),Patch(facecolor=BLACK,label='黑格：不可行'),
        Line2D([],[],color=BLACK,marker='o',markerfacecolor=YELLOW,markersize=8,ls='none',label='黄点：检查通过的黑格'),
        Patch(facecolor=RED,label='红格：本次新增边界（原为白格）'),
        Line2D([],[],color=DASH,ls='--',label='黄虚线：当前扫描行 258')]
    fig.legend(handles=items,loc='lower center',bbox_to_anchor=(.5,y),ncol=3,fontsize=11,frameon=False)


def overview():
    fig=plt.figure(figsize=(16,8.6),facecolor=BG)
    fig.text(.055,.94,'E03 全貌：黄点不是切线，红格才是新增边界',fontsize=23,weight='bold')
    fig.text(.055,.885,'真实原生像素｜x 向右，行号 y 向下；纵向行高已放大。所有标记都保留真实坐标。',fontsize=12)
    ax=fig.add_axes([.115,.35,.79,.42])
    values=white[256:260,925:1318].astype(int);values[cut[256:260,925:1318]]=2
    ax.imshow(values,cmap=ListedColormap([BLACK,'white',RED]),vmin=0,vmax=2,origin='upper',
        extent=(924.5,1317.5,259.5,255.5),interpolation='nearest',aspect='auto')
    ax.set(xlim=(924.5,1317.5),ylim=(259.5,255.5),xlabel='原生 x / px')
    ax.set_yticks(range(256,260),[f'y = {y}' for y in range(256,260)])
    ax.set_xticks([930,993,1080,1181,1288,1312]);ax.tick_params(length=0,labelsize=10,pad=8)
    for y in np.arange(255.5,260,1):ax.axhline(y,color='#9baeba',lw=.6)
    ax.axhline(258,color=DASH,ls=(0,(6,5)),lw=1.8,zorder=5)
    ax.scatter(cp[:,0],cp[:,1],s=9,facecolors=YELLOW,edgecolors='none',zorder=6)
    for lo,hi,label in [(930,993,'64 个红格'),(1311,1312,'2 个红格')]:
        ax.annotate(label,((lo+hi)/2,257),xytext=(955 if lo==930 else 1280,255.12),
            color=RED,fontsize=13,ha='center',arrowprops=dict(arrowstyle='->',color=RED,lw=1.7),annotation_clip=False,zorder=8)
    for lo,hi,name in [(988,1005,'A'),(1305,1316,'B')]:
        ax.add_patch(Rectangle((lo-.5,255.5),hi-lo+1,4,fill=False,edgecolor=BLUE,lw=2,zorder=7))
        ax.text((lo+hi)/2,259.83,f'{name}：下图逐格放大',color=BLUE,ha='center',va='top',fontsize=11)
    fig.text(.115,.255,'256 行：2 段 → 257 行：3 段 → 258 行：2 段，触发 OUT；边界回写在 257 行。',fontsize=14,weight='bold')
    fig.text(.115,.195,'196 个黄点位于黑格；只新增 66 个红格，分成 64 格和 2 格两段。红格是临时分区标记。',fontsize=12)
    legend(fig,.045)
    save(fig,'18_e03_pixel_overview')


def grid(ax,x0,x1,after,title):
    y0=255;y1=259
    values=white[y0:y1+1,x0:x1+1].astype(int)
    if after:values[cut[y0:y1+1,x0:x1+1]]=2
    ax.imshow(values,cmap=ListedColormap([BLACK,'white',RED]),vmin=0,vmax=2,origin='upper',
        extent=(x0-.5,x1+.5,y1+.5,y0-.5),interpolation='nearest')
    ax.set(xlim=(x0-.5,x1+.5),ylim=(y1+.5,y0-.5),aspect='equal')
    ax.set_xticks(range(x0,x1+1));ax.set_yticks(range(y0,y1+1))
    ax.xaxis.tick_top();ax.tick_params(axis='x',length=0,pad=5,labelsize=9,rotation=45)
    ax.tick_params(axis='y',length=0,pad=7,labelsize=10)
    ax.set_xticks(np.arange(x0-.5,x1+1,1),minor=True);ax.set_yticks(np.arange(y0-.5,y1+1,1),minor=True)
    ax.grid(which='minor',color='#a0b1be',lw=.8);ax.tick_params(which='minor',length=0)
    ax.set_title(title,loc='left',fontsize=15,weight='bold',pad=51)
    points=cp[(cp[:,0]>=x0)&(cp[:,0]<=x1)]
    ax.scatter(points[:,0],points[:,1],s=55,facecolors=YELLOW,edgecolors='none',zorder=6)
    ax.axhline(258,color=DASH,ls=(0,(5,4)),lw=1.5,zorder=5)
    if after:
        for x in np.flatnonzero(cut[257,x0:x1+1])+x0:
            ax.text(x,257,'界',color='white',ha='center',va='center',fontsize=10,zorder=7)
    sample=1000 if x0<1100 else 1309
    ax.add_patch(Rectangle((sample-1.5,257.5),3,1,fill=False,edgecolor=BLUE,lw=2.5,zorder=7))
    # Every plotted cell is one actual pixel, without anisotropic stretching.
    assert float(ax.get_aspect())==1


def details():
    fig=plt.figure(figsize=(17,11),facecolor=BG)
    fig.text(.055,.955,'E03 逐格放大：每个正方形就是 1 个真实像素',fontsize=23,weight='bold')
    fig.text(.055,.908,'上下对照同一批像素：上排尚未写边界，下排叠加 E03 的新增边界；蓝框是某个黄点下方的三格检查。',fontsize=12)
    locations=[(.07,.60,.51,.17),(.66,.60,.30,.17),(.07,.30,.51,.17),(.66,.30,.30,.17)]
    specs=[(988,1005,False,'A / 切割前：长边界的右端'),(1305,1316,False,'B / 切割前：2 格短边界附近'),
           (988,1005,True,'A / 切割后：988～993 变红'),(1305,1316,True,'B / 切割后：1311、1312 变红')]
    for location,spec in zip(locations,specs):grid(fig.add_axes(location),*spec)
    fig.text(.07,.575,'黄点 (1000,257) 下方三格全白 → OUT 检查通过。',fontsize=12,color=BLUE)
    fig.text(.66,.575,'黄点 (1309,257) 下方三格全白。',fontsize=12,color=BLUE)
    fig.text(.07,.23,'994～999 保持白色：它们在 256 行是黑格，\n不满足“本格和上一行都白”的画线条件。',fontsize=12,linespacing=1.7)
    fig.text(.66,.23,'1310 保持白色：上一行是黑格。\n1311、1312 两行都白，所以变红。',fontsize=12,linespacing=1.7)
    fig.text(.07,.125,'A 只截取了长边界的右端；完整范围是 x = 930～993，共 64 格。红格与黄点都在 257 行，扫描线在 258 行。',fontsize=12,weight='bold')
    legend(fig,.025)
    save(fig,'19_e03_pixel_before_after')


if __name__=='__main__':
    result=audit();overview();details()
    result['figures']=['18_e03_pixel_overview','19_e03_pixel_before_after']
    result['generator_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (HERE/'evidence/e03_pixel_verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False))
