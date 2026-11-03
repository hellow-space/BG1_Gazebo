#!/usr/bin/env python3
"""Assemble an offline HTML report plus a readable Markdown counterpart."""
import json
import hashlib
from pathlib import Path
import re
import matplotlib
from bs4 import BeautifulSoup, NavigableString

HERE=Path(__file__).resolve().parent


def main():
    data=json.loads((HERE/'evidence/report_data.json').read_text())
    trace=json.loads((HERE/'evidence/native_trace/trace.json').read_text())
    data['colors']=[matplotlib.colors.to_hex(matplotlib.colormaps['turbo']((i*.61803398875)%1)) for i in range(80)]
    data['native_centers']=[s['center_rotated'] for s in trace['sweeps']]
    (HERE/'report-data.js').write_text('window.IPA_REPORT='+json.dumps(data,ensure_ascii=False,separators=(',',':'),allow_nan=False)+';\n')
    html=(HERE/'report_template.html').read_text()
    atlas='\n'.join(f'<details class="atlas"><summary>索引 {i+1:02}/10 · C{i*8:02}–C{i*8+7:02}</summary><figure><a href="assets/atlas_{i+1:02}.png" target="_blank"><img loading="lazy" src="assets/atlas_{i+1:02}.png" alt="C{i*8:02}至C{i*8+7:02}全部区域放大"></a></figure></details>' for i in range(10))
    links='\n'.join(f'<a href="assets/cells/C{i:02}.png" target="_blank">C{i:02} 三联放大</a>' for i in range(80))
    html=html.replace('@@ATLAS@@',atlas).replace('@@CELL_LINKS@@',links)
    # A regenerated report must not keep executing a browser-cached older viewer.
    viewer_version=hashlib.sha256((HERE/'report.js').read_bytes()).hexdigest()[:12]
    html=html.replace('src="report.js"',f'src="report.js?v={viewer_version}"')
    rotation_version=hashlib.sha256((HERE/'assets/03_rotation.png').read_bytes()).hexdigest()[:12]
    html=html.replace('src="assets/03_rotation.png"',f'src="assets/03_rotation.png?v={rotation_version}"')
    (HERE/'report.html').write_text(html)
    # Convert the same narrative, not a separately maintained second explanation.
    soup=BeautifulSoup(html,'html.parser')
    for node in soup.select('aside,script,footer,.toolbar,.panel,.step,.eyebrow,.stats,.hint,#load-status,#cell-info,#raw-info,#corner-details'):
        node.decompose()
    cells=soup.find(id='cells-table')
    cells.append(BeautifulSoup(''.join('<tr>'+''.join(f'<td>{value}</td>' for value in [
        c['name'],c['label'],f'{c["area_m2"]:.4f}',c['native_area_pixels'],c['grid_rows'],c['raw_points'],
        c['safe_piece_count'],c['visit_order'] or '—',c['astar_sections'],c['status']])+'</tr>' for c in data['cells']),'html.parser'))
    events=soup.find(id='events-table')
    events.append(BeautifulSoup(''.join('<tr>'+''.join(f'<td>{value}</td>' for value in [
        f'E{e["index"]:02}',e['type'],e['y'],f'{e["before"]} → {e["after"]}',len(e['critical_points']),len(e['cuts'])])+'</tr>' for e in data['events']),'html.parser'))
    soup.find(id='rotation-matrix').string='R =\n'+ '\n'.join(' '.join(f'{v:.12f}' for v in row) for row in data['rotation']['R'])
    soup.find(id='schedule-table').append(BeautifulSoup('<table><tr><th>访问 #</th><th>区域</th><th>方向</th><th>接近 m</th><th>含接近长度 m</th></tr>'+''.join(
        f'<tr><td>{s["step"]}</td><td>C{s["region"]:02}</td><td>{"反向" if s["reverse"] else "正向"}</td><td>{data["cells"][s["region"]]["approach_length_m"]:.6f}</td><td>{data["cells"][s["region"]]["final_length_m"]:.6f}</td></tr>' for s in data['schedule'])+'</table>','html.parser'))

    def md(node):
        if isinstance(node,NavigableString):return re.sub(r'\s+',' ',str(node))
        name=node.name
        if name=='figure':
            image=node.find('img');caption=node.find('figcaption')
            return f'\n\n![{image.get("alt","")}]({image["src"]})\n\n'+(caption.get_text(' ',strip=True)+'\n\n' if caption else '')
        if name=='table':
            rows=[[cell.get_text(' ',strip=True).replace('|','/').replace('\n',' ') for cell in tr.find_all(['th','td'],recursive=False)] for tr in node.find_all('tr')]
            rows=[row for row in rows if row]
            if not rows:return ''
            return '\n\n'+'\n'.join(['| '+' | '.join(rows[0])+' |','| '+' | '.join('---' for _ in rows[0])+' |',*['| '+' | '.join(row)+' |' for row in rows[1:]]])+'\n\n'
        if name=='pre':return '\n\n```text\n'+node.get_text().strip()+'\n```\n\n'
        text=''.join(md(child) for child in node.children)
        if name in ('h1','h2','h3'):return '\n\n'+'#'*int(name[1])+' '+text.strip()+'\n\n'
        if name in ('p','section','header','div'):return '\n\n'+text.strip()+'\n\n' if text.strip() else ''
        if name=='summary':return '\n\n### '+text.strip()+'\n\n'
        if name=='li':return '\n'+str(list(node.parent.find_all('li',recursive=False)).index(node)+1)+'. '+text.strip()+'\n'
        if name in ('ol','ul','details'):return '\n\n'+text.strip()+'\n\n'
        if name in ('b','strong'):return '**'+text+'**'
        if name=='code':return '`'+text+'`'
        if name=='br':return ' '
        if name=='a':return '['+text+']('+node['href']+')' if text.strip() else ''
        return text
    intro='优先打开 [交互报告 report.html](report.html)：逐块缩放 80 区、逐次回放 66 个事件与 61 轮调度。下文为相同正文的静态阅读版。\n\n'
    body=md(soup.main)
    body=re.sub(r'\n[ \t]+\n','\n\n',body)
    body=re.sub(r'\n{3,}','\n\n',body)
    body=body.replace('# IPA BCD 在 AJ04 上 究竟怎样划分、怎样规划','# AJ04 · IPA BCD 全流程图解\n\n'+intro,1)
    (HERE/'README.md').write_text(body.strip()+'\n')
    print('Wrote report.html, README.md and report-data.js')


if __name__=='__main__':main()
