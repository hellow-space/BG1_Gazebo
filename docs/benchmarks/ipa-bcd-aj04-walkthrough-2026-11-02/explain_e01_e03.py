#!/usr/bin/env python3
"""Audit every E01--E03 black-pixel test and border ray; render teaching insets."""
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from markdown_it import MarkdownIt
import numpy as np
from PIL import Image

HERE=Path(__file__).resolve().parent
SOURCE=HERE/'evidence/native_trace/01_rotated_room.png'
TRACE=HERE/'evidence/native_trace/trace.json'
NATIVE=np.asarray(Image.open(SOURCE))
WHITE=NATIVE==255
EVENTS=json.loads(TRACE.read_text())['events'][:3]
BG='#f4f7fb';INK='#18334c';BLACK='#293b4c';RED='#dd4b4b';GOLD='#e7a026'
plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False,
    'text.color':INK,'svg.fonttype':'none','svg.hashsalt':'e01-e03-complete'})


def runs(xs):
    values=sorted(set(int(x) for x in xs))
    result=[]
    for x in values:
        if result and x==result[-1][1]+1:result[-1][1]=x
        else:result.append([x,x])
    return result


def audit():
    cell=NATIVE.copy();checks=[];rays=[];summaries=[]
    for index,event in enumerate(EVENTS,1):
        current=event['y'];sy=current if event['type']=='IN' else current-1
        ny=current-1 if event['type']=='IN' else current
        first=int(np.flatnonzero(WHITE[sy])[0]);points=[];before=cell.copy();black=[]
        for x in range(first+1,WHITE.shape[1]):
            if WHITE[sy,x]:continue
            triple=[int(NATIVE[ny,np.clip(x+dx,0,WHITE.shape[1]-1)]) for dx in [-1,0,1]]
            passed=all(v==255 for v in triple);black.append(x)
            checks.append(dict(event=f'E{index:02}',trigger_row=current,test_row=sy,neighbor_row=ny,
                x=x,test_value=int(NATIVE[sy,x]),neighbor_left=triple[0],neighbor_center=triple[1],neighbor_right=triple[2],passed=passed))
            if not passed:continue
            points.append([x,sy])
            reference=current-1 if event['type']=='IN' else max(0,current-2)
            for direction in [-1,1]:
                xx=x+direction;marked=[];skipped=[];already=[]
                while 0<=xx<WHITE.shape[1]:
                    value=cell[sy,xx]
                    if value==0:break
                    if value==255 and cell[reference,xx]==255:
                        cell[sy,xx]=127;marked.append(xx)
                    elif value==127:already.append(xx)
                    else:skipped.append(xx)
                    xx+=direction
                rays.append(dict(event=f'E{index:02}',critical_x=x,row=sy,direction='left' if direction<0 else 'right',
                    reference_row=reference,marked=runs(marked),skipped_white=runs(skipped),already_border=runs(already),
                    stop_x=xx,stop_reason='black' if 0<=xx<WHITE.shape[1] else 'image_edge'))
        assert points==event['critical_points']
        expected=np.zeros_like(WHITE)
        for lo,y,hi,_ in event['cuts']:expected[y,lo:hi+1]=True
        actual=(cell!=before);assert np.array_equal(actual,expected)
        summaries.append(dict(event=f'E{index:02}',trigger_row=current,type=event['type'],before=event['before'],after=event['after'],
            test_row=sy,neighbor_row=ny,checked_black_ranges=runs(black),checked_black_pixels=len(black),
            passed_ranges=runs([p[0] for p in points]),passed_pixels=len(points),border_runs=event['cuts'],new_border_pixels=int(actual.sum())))
    assert [s['checked_black_pixels'] for s in summaries]==[727,612,612]
    assert [s['passed_pixels'] for s in summaries]==[0,0,196]
    assert [s['new_border_pixels'] for s in summaries]==[0,0,66]
    active=[r for r in rays if r['marked'] or r['skipped_white'] or r['already_border']]
    assert [(r['critical_x'],r['direction']) for r in active]==[(1000,'left'),(1180,'right'),(1289,'left'),(1309,'right')]
    assert not any(r['already_border'] for r in rays)
    for name,records in [('e01_e03_all_pixel_checks.csv',checks),('e01_e03_all_border_rays.csv',rays)]:
        with (HERE/'evidence'/name).open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(records[0]));writer.writeheader()
            writer.writerows({k:json.dumps(v,ensure_ascii=False) if isinstance(v,list) else v for k,v in row.items()} for row in records)
    result=dict(passed=True,all_checks=len(checks),all_border_rays=len(rays),events=summaries,active_rays=active,
        immediate_black_stop_rays=sum(not r['marked'] and not r['skipped_white'] and not r['already_border'] for r in rays),
        no_duplicate_border_writes=True,native_critical_points_equal=True,native_border_pixels_equal=True,
        native_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),trace_sha256=hashlib.sha256(TRACE.read_bytes()).hexdigest(),
        generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        narrative_sha256=hashlib.sha256((HERE/'e01-e03-walkthrough.md').read_bytes()).hexdigest())
    (HERE/'evidence/e01_e03_step_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return result


def draw_failed_checks():
    fig,axes=plt.subplots(2,3,figsize=(15,9.5),facecolor=BG)
    fig.subplots_adjust(left=.07,right=.98,top=.76,bottom=.13,wspace=.35,hspace=.9)
    fig.text(.05,.94,'E01、E02：把每段黑色的起点逐格拿出来看',fontsize=23,weight='bold')
    fig.text(.05,.87,'IN 需要：圆圈里的当前格是黑色，且上方三个格全部为白色；红框标出导致失败的上一行黑格。',fontsize=12)
    samples=[(256,994,'E01：左侧白段之后'),(256,1313,'E01：2 像素白段之后'),None,
             (257,1000,'E02：左侧白段之后'),(257,1289,'E02：新增中间白段之后'),(257,1313,'E02：右侧白段之后')]
    for ax,sample in zip(axes.flat,samples):
        ax.axis('off')
        if sample is None:
            ax.text(.05,.82,'图中是各段的首个黑格。\n代码还检查该段后面的每个黑格。',transform=ax.transAxes,fontsize=13,linespacing=1.8)
            ax.text(.05,.25,'E01：727 次检查，全部失败。\nE02：612 次检查，全部失败。\n完整逐格结果保存在 CSV。',transform=ax.transAxes,fontsize=13,color=RED,linespacing=1.8)
            continue
        y,x,title=sample
        ax.set(xlim=(-.85,3.1),ylim=(2.15,-.55),aspect='equal')
        ax.set_title(title,fontsize=13,weight='bold',pad=18)
        for j,yy in enumerate([y-1,y]):
            ax.text(-.12,j+.5,f'y={yy}',ha='right',va='center',fontsize=11)
            for i,xx in enumerate(range(x-1,x+2)):
                white=bool(WHITE[yy,xx]);color='white' if white else BLACK
                ax.add_patch(Rectangle((i,j),1,1,facecolor=color,edgecolor='#9baeba',lw=1))
                ax.text(i+.5,j+.5,'白' if white else '黑',color=INK if white else 'white',ha='center',va='center',fontsize=12)
                if j==0:
                    ax.text(i+.5,-.12,str(xx),ha='center',fontsize=10)
                    if not white:ax.add_patch(Rectangle((i+.05,j+.05),.9,.9,fill=False,edgecolor=RED,lw=2))
        ax.scatter(1.5,1.5,s=850,facecolors='none',edgecolors=GOLD,lw=2.5,zorder=5)
    fig.text(.05,.045,'注意：旧白段继续存在，并不是 IN 临界点。上一行只有 2 个连续白格，也容纳不了“三格全白”的检查窗口。',fontsize=13,weight='bold')
    for suffix in ['png','svg']:fig.savefig(HERE/'assets'/f'20_e01_e02_failed_checks.{suffix}',dpi=165,facecolor=BG)
    plt.close(fig)


def write_html():
    source=HERE/'e01-e03-walkthrough.md'
    md=MarkdownIt('commonmark',{'html':False}).enable('table')
    body=md.render(source.read_text())
    style='''body{margin:0;background:#f4f7fb;color:#18334c;font:17px/1.8 system-ui,sans-serif}main{max-width:1120px;margin:auto;padding:40px 30px 90px;background:white}h1{font-size:32px}h2{margin-top:50px;border-top:2px solid #dce6ee;padding-top:25px}h3{margin-top:30px}a{color:#147db3}img{width:100%;height:auto;border:1px solid #dce6ee}table{border-collapse:collapse;display:block;overflow:auto;max-width:100%;font-size:15px}td,th{border:1px solid #cbd9e3;padding:8px 12px;white-space:nowrap}th{background:#eef5fa}pre{background:#f1f5f8;padding:20px;overflow:auto;font:15px/1.8 monospace}code{color:#12577c}strong{color:#132f48}li{margin:8px 0}'''
    page='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>E01～E03 逐像素完整拆解</title><style>'+style+'</style><main>'+body+'</main></html>\n'
    (HERE/'e01-e03-walkthrough.html').write_text(page)


if __name__=='__main__':
    result=audit();draw_failed_checks();write_html()
    print(json.dumps({k:result[k] for k in ['passed','all_checks','all_border_rays','immediate_black_stop_rays','native_critical_points_equal','native_border_pixels_equal']},ensure_ascii=False))
