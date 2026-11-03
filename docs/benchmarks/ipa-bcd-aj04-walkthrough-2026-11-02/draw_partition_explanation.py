#!/usr/bin/env python3
"""Teaching schematic plus exact native scan rows; no planner execution."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from PIL import Image
from scipy.ndimage import label

HERE=Path(__file__).resolve().parent
OUT=HERE/'assets'
INK='#18334c';BLACK='#293b4c';BLUE='#147db3';RED='#dd4b4b';GOLD='#e7a026'
plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False,
                     'text.color':INK,'svg.fonttype':'none','svg.hashsalt':'bcd-explanation'})


def save(fig,name):
    for suffix in ['png','svg']:
        fig.savefig(OUT/f'{name}.{suffix}',dpi=160,facecolor=fig.get_facecolor())
    plt.close(fig)


def room(ax):
    ax.set(xlim=(0,12),ylim=(12,0),aspect='equal');ax.axis('off')
    ax.add_patch(Rectangle((1,1),8,10,facecolor='white',edgecolor=BLACK,lw=3,zorder=2))
    ax.add_patch(Rectangle((4,4),2,4,facecolor=BLACK,zorder=4))
    ax.text(5,6,'障\n碍',ha='center',va='center',color='white',fontsize=14,zorder=5)


def cuts(ax):
    for y in [4,8]:
        for lo,hi in [(1,4),(6,9)]:ax.plot([lo,hi],[y,y],color=RED,lw=3,zorder=6)


def teaching():
    fig,axes=plt.subplots(1,3,figsize=(16,8.7),facecolor='#f4f7fb')
    fig.subplots_adjust(left=.03,right=.99,top=.79,bottom=.20,wspace=.06)
    fig.text(.04,.94,'BCD 怎么把一个房间切成几个区域？',fontsize=25,weight='bold')
    fig.text(.04,.885,'教学示意｜三个画面是同一个房间；白色可通行，深色为墙或障碍；扫描线向下移动。',fontsize=13)
    for ax in axes:room(ax)
    for ax,text in zip(axes,['① 看扫描线上的白色段数','② 确认临界点，再画分界线','③ 用分界线隔开，再编号']):
        ax.set_title(text,fontsize=16,weight='bold',loc='left',pad=18)
    ax=axes[0]
    for y,parts,n in [(2.3,[(1,9)],1),(6,[(1,4),(6,9)],2),(9.7,[(1,9)],1)]:
        for lo,hi in parts:ax.plot([lo+.08,hi-.08],[y,y],color=BLUE,lw=5,zorder=6)
        ax.text(9.35,y,f'{n} 段',fontsize=13,va='center',color=BLUE)
    ax.annotate('',(.35,10.8),xytext=(.35,1.2),arrowprops=dict(arrowstyle='->',color=BLUE,lw=2))
    ax.text(2.5,3.65,'1 → 2：IN',fontsize=12,color=RED,ha='center',zorder=8,
            bbox=dict(facecolor='white',edgecolor='none',pad=2))
    ax.text(2.5,8.6,'2 → 1：OUT',fontsize=12,color=RED,ha='center',zorder=8,
            bbox=dict(facecolor='white',edgecolor='none',pad=2))
    ax.text(5,11.75,'宽窄变化、一直两段：继续延伸',ha='center',fontsize=12)
    ax=axes[1];cuts(ax)
    for y,text,ty in [(4,'障碍开始：检查 IN 临界点',2.6),(8,'障碍结束：检查 OUT 临界点',9.6)]:
        ax.scatter(4,y,s=85,c=GOLD,edgecolors='white',lw=1.2,zorder=9)
        ax.annotate(text,(4,y),xytext=(5,ty),ha='center',fontsize=11,
                    arrowprops=dict(arrowstyle='->',color=GOLD,lw=1.5),zorder=8)
    ax.text(5,11.75,'红线向左右延伸，遇到障碍或墙停止',ha='center',fontsize=11)
    ax=axes[2]
    regions=[((1,1),8,3,'A','#d8eafa',(5,2.5)),((1,4),3,4,'B','#d4eee5',(2.5,6)),
             ((6,4),3,4,'C','#fbe9c8',(7.5,6)),((1,8),8,3,'D','#e8ddf5',(5,9.5))]
    for xy,w,h,name,color,center in regions:
        ax.add_patch(Rectangle(xy,w,h,facecolor=color,zorder=3))
        ax.text(*center,name,fontsize=23,weight='bold',ha='center',va='center',zorder=8)
    cuts(ax)
    ax.text(5,11.75,'四连通填充 → 这个示意图有 4 区',ha='center',fontsize=12)
    fig.text(.04,.135,'关键：段数变化只触发候选检查；满足临界点条件，才真正画线。',fontsize=18,weight='bold',color=RED)
    fig.text(.04,.073,'分区线是临时的编号边界，机器人仍可跨区通行。这里的 4 区是教学示意；AJ04 的实际结果是 80 区。',fontsize=12)
    # Independent connectivity check on the pictured rectangle/obstacle/cuts.
    mask=np.ones((100,80),bool);mask[30:70,30:50]=False
    mask[30,:]=False;mask[69,:]=False
    _,count=label(mask);assert count==4
    save(fig,'15_partition_logic')
    return count


def runs(row):
    edges=np.diff(np.r_[False,row,False].astype(int))
    return list(zip(np.flatnonzero(edges==1).tolist(),(np.flatnonzero(edges==-1)-1).tolist()))


def actual_rows():
    native=np.asarray(Image.open(HERE/'evidence/native_trace/01_rotated_room.png'))==255
    trace=json.loads((HERE/'evidence/native_trace/trace.json').read_text())
    rows={y:runs(native[y]) for y in range(255,259)}
    assert [len(v) for v in rows.values()]==[1,2,3,2]
    assert rows[256][-1]==(1311,1312)
    assert [(e['type'],e['before'],e['after'],len(e['cuts'])) for e in trace['events'][:3]]==[
        ('IN',1,2,0),('IN',2,3,0),('OUT',3,2,2)]
    fig=plt.figure(figsize=(16,8),facecolor='#f4f7fb')
    fig.text(.05,.935,'E01 → E02 → E03：真实像素发生了什么？',fontsize=24,weight='bold')
    fig.text(.05,.88,'AJ04 原生旋转图｜横向 x 保持像素比例；仅把扫描行的高度放大，方便看清。',fontsize=13)
    ax=fig.add_axes([.12,.32,.71,.49]);ax.set(xlim=(920,1325),ylim=(4.4,-.6))
    ax.set_yticks([0,1.25,2.5,3.75],['y = 255','y = 256','y = 257','y = 258'])
    ax.set_xticks([930,1000,1100,1181,1288,1312]);ax.set_xlabel('原生旋转图 x / px',fontsize=11)
    ax.tick_params(axis='both',length=0,pad=9,labelsize=11)
    for s in ax.spines.values():s.set_visible(False)
    ax.set_facecolor('#f4f7fb')
    descriptions=['初始：1 段','E01：1 → 2\nIN，未画线','E02：2 → 3\nIN，未画线','E03：3 → 2\nOUT，画出 2 条线']
    for i,((y,segments),desc) in enumerate(zip(rows.items(),descriptions)):
        cy=i*1.25
        ax.add_patch(Rectangle((925,cy-.29),393,.58,color=BLACK,zorder=1))
        for lo,hi in segments:
            ax.add_patch(Rectangle((lo-.5,cy-.25),hi-lo+1,.5,facecolor='white',edgecolor='#bdcbd5',lw=.6,zorder=3))
        ax.text(1332,cy,desc,va='center',fontsize=12,color=INK,clip_on=False)
    # Highlight actual new runs, preserving their real horizontal widths.
    for lo,hi,cy in [(1311,1312,1.25),(1181,1288,2.5)]:
        ax.add_patch(Rectangle((lo-.5,cy-.25),hi-lo+1,.5,facecolor='#ffd377',edgecolor=GOLD,lw=1,zorder=4))
    ax.annotate('新出现：仅 2 像素！',(1311.5,1.25),xytext=(1145,.65),fontsize=12,color='#a36500',
                arrowprops=dict(arrowstyle='->',color=GOLD,lw=1.5),zorder=8)
    for source,target in [((1234,2.78),(1234,3.47)),((1311,2.78),(1295,3.47))]:
        ax.annotate('',target,xytext=source,arrowprops=dict(arrowstyle='->',color=BLUE,lw=2),zorder=6)
    fig.text(.12,.205,'E03：右边两段在下一行接起来了 → 段数减少 → OUT。',fontsize=17,weight='bold',color=BLUE)
    fig.text(.12,.13,'E01 / E02：白段数增加，但临界点检查未通过，所以没有实际切割。',fontsize=14)
    fig.text(.12,.065,'金色 = 新出现的白段；蓝箭头 = 段之间的接续关系，非机器人路线。E03 的切割记录在上一行 y = 257。',fontsize=11)
    save(fig,'16_e01_e03_rows')
    return dict(rows={str(y):v for y,v in rows.items()},events=[dict(type=e['type'],before=e['before'],after=e['after'],
        cuts=e['cuts'],critical_points=len(e['critical_points'])) for e in trace['events'][:3]])


def critical_pixels():
    native=np.asarray(Image.open(HERE/'evidence/native_trace/01_rotated_room.png'))==255
    assert not native[256,999:1002].any()
    assert native[258,999:1002].all()
    assert native[257,999] and not native[257,1000]
    fig,axes=plt.subplots(1,2,figsize=(14,6.2),facecolor='#f4f7fb')
    fig.subplots_adjust(left=.12,right=.95,top=.70,bottom=.28,wspace=.5)
    fig.text(.055,.92,'临界点检查：盯住同一个黑格 x = 1000',fontsize=23,weight='bold')
    fig.text(.055,.84,'真实像素放大｜候选黑格位于 y = 257；图中小圆圈标出它，上下三格决定检查结果。',fontsize=12)
    for ax,ys,heading,caption in zip(axes,[(256,257),(257,258)],
        ['E02 / IN：看候选黑格的上一行','E03 / OUT：看候选黑格的下一行'],
        ['上面三格是黑色 → 不满足 → 此处不切','下面三格全是白色 → 满足 → 此处触发切割']):
        ax.set(xlim=(-.7,3.7),ylim=(2.2,-.65),aspect='equal');ax.axis('off')
        ax.set_title(heading,fontsize=15,weight='bold',pad=15)
        for j,y in enumerate(ys):
            ax.text(-.15,j+.5,f'y={y}',ha='right',va='center',fontsize=12)
            for i,x in enumerate(range(999,1002)):
                ax.add_patch(Rectangle((i,j),1,1,facecolor='white' if native[y,x] else BLACK,edgecolor='#9baeba',lw=1.5))
                if j==0:ax.text(i+.5,-.15,str(x),ha='center',fontsize=11)
        jy=ys.index(257)
        ax.scatter(1.5,jy+.5,s=170,facecolors='none',edgecolors=GOLD,lw=3,zorder=5)
        ax.text(1.5,2.35,caption,ha='center',va='top',fontsize=12,color=RED if jy==1 else BLUE)
    fig.text(.055,.105,'同一块黑色边缘：出现段数变化后，还必须检查局部像素；不能只凭 IN / OUT 就认定已经分区。',fontsize=13,weight='bold')
    save(fig,'17_critical_pixel_check')


if __name__=='__main__':
    count=teaching();facts=actual_rows();critical_pixels()
    facts.update(teaching_cells=count,teaching_is_not_aj04=True)
    (HERE/'evidence/partition_explanation.json').write_text(json.dumps(facts,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(facts,ensure_ascii=False))
