#!/usr/bin/env python3
"""Validate report geometry accounting, complete indexes, local resources and hashes."""
from collections import Counter
from html.parser import HTMLParser
import hashlib
import json
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote, urlsplit

import numpy as np
import cv2
from PIL import Image

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


class Links(HTMLParser):
    def __init__(self):super().__init__();self.urls=[];self.ids=[]
    def handle_starttag(self,tag,attrs):
        attrs=dict(attrs)
        if 'id' in attrs:self.ids.append(attrs['id'])
        for key in ('href','src'):
            if key in attrs:self.urls.append(attrs[key])


def verify_rotation(data):
    affine=np.asarray(data['rotation']['R']);linear=affine[:,:2]
    assert np.allclose(linear.T@linear,np.eye(2),atol=1e-12)
    assert np.isclose(np.linalg.det(linear),1,atol=1e-12)
    theta=float(np.degrees(np.arctan2(linear[1,0],linear[0,0])))
    assert np.isclose(theta,-data['summary']['native_rotation_deg'])
    with np.load(HERE/'evidence/report_masks.npz') as masks:f=masks['f']
    native=np.asarray(Image.open(HERE/'evidence/native_trace/01_rotated_room.png'))>127
    rebuilt=cv2.warpAffine(f.astype(np.uint8)*255,affine,(native.shape[1],native.shape[0]),flags=cv2.INTER_AREA)>127
    assert np.array_equal(native,rebuilt)
    display=json.loads((HERE/'evidence/rotation_display.json').read_text())
    assert np.array_equal(display['linear'],linear)
    assert np.array_equal(display['translation'],affine[:,2])
    before=np.asarray(display['landmarks_before']);after=np.asarray(display['landmarks_after'])
    assert before.shape==after.shape==(3,2)
    assert np.allclose(before@linear.T+affine[:,2],after)
    assert f[before[:,1].astype(int),before[:,0].astype(int)].all()
    pixels=np.rint(after).astype(int);assert native[pixels[:,1],pixels[:,0]].all()
    limits=np.asarray(display['plotted_ylim'])
    assert (limits[:2,1]>limits[:2,0]).all() and limits[2,1]<limits[2,0]
    assert np.allclose(np.abs(limits[:,1]-limits[:,0]),display['plotted_span_pixels'])
    return dict(native_pixels_reproduced_exactly=True,determinant=float(np.linalg.det(linear)),
                angle_y_up_degrees=theta,matching_landmarks=3,primary_axes_y_up=True)


def main():
    data=json.loads((HERE/'evidence/report_data.json').read_text());s=data['summary']
    trace=json.loads((HERE/'evidence/native_trace/trace.json').read_text())
    assert len(data['cells'])==80 and [c['id'] for c in data['cells']]==list(range(80))
    assert trace['before_merge_count']==trace['final_label_count']==80
    assert len(data['events'])==66 and sum(bool(e['cuts']) for e in data['events'])==30
    assert sum(len(e['cuts']) for e in data['events'])==79
    assert Counter(e['type'] for e in data['events'])=={'IN':31,'OUT':35}
    assert len(data['schedule'])==61 and len({e['region'] for e in data['schedule']})==61
    assert len(data['path'])==79225 and sum(c['raw_points'] for c in data['cells'])==46243
    assert sum(c['safe_piece_count'] for c in data['cells'])==39465
    assert sum(c['astar_sections'] for c in data['cells'])==209
    assert s['astar_candidate_searches']==323 and s['all_astar_searches']==532
    assert len(s['empty_raw'])==6 and len(s['singleton_cells'])==9 and len(s['short_clipped_cells'])==4
    assert all(data['cells'][i]['rotated_white_pixels']==0 for i in s['empty_raw'])
    assert np.isclose(sum(s['path_length_parts_m'].values()),s['metrics']['path_length_m'],rtol=0,atol=1e-8)
    # The event log must account for every actual temporary border pixel, no conceptual cuts added.
    cut_map=np.asarray(Image.open(HERE/'evidence/native_trace/02_cuts.png'))
    borders=(cut_map!=0)&(cut_map!=255);reconstructed=np.zeros_like(borders)
    for event in trace['events']:
        for x0,y,x1,_ in event['cuts']:reconstructed[y,x0:x1+1]=True
    assert np.array_equal(reconstructed,borders)
    # The trace must match both serialized bytes and all parsed native values.
    trace_check=json.loads((HERE/'evidence/native_trace/verification.json').read_text())
    assert trace_check['native_output_equal']
    assert trace_check['historical_output_sha256']==trace_check['trace_output_sha256']==sha(HERE/'evidence/native_trace/output.json')
    current=json.loads((HERE/'evidence/report_verification.json').read_text())
    assert all(current[k] is True for k in ('historical_source_hashes_match','all_80_cells_match_native_labels',
        'all_61_scheduler_choices_replayed','all_78930_targets_replayed','final_metrics_equal_saved',
        'strict_closed_cell_audit','independent_pixel_box_audit','covered_mask_replayed'))
    for path,digest in current['source_files_sha256'].items():assert sha(ROOT/path)==digest,path
    for path,digest in current['evidence_sha256'].items():assert sha(HERE/path)==digest,path
    template=(HERE/'report_template.html').read_text()
    html=(HERE/'report.html').read_text();assert '@@' not in html
    parser=Links();parser.feed(html)
    assert len(parser.ids)==len(set(parser.ids)),'Duplicate HTML IDs'
    for url in parser.urls:
        parts=urlsplit(url)
        if parts.scheme or parts.netloc:raise AssertionError('Unexpected online dependency '+url)
        if parts.path:assert (HERE/unquote(parts.path)).resolve().is_file(),url
        if parts.fragment and not parts.path:assert parts.fragment in parser.ids,url
    js=(HERE/'report.js').read_text()
    for element in re.findall(r"\$\('([^']+)'\)",js):assert element in parser.ids,element
    for i in range(80):assert f'assets/cells/C{i:02}.png' in html
    expected=[*(f'assets/cells/C{i:02}.png' for i in range(80)),*(f'assets/atlas_{i:02}.png' for i in range(1,11))]
    for p in expected:
        with Image.open(HERE/p) as im:im.verify()
    for p in (HERE/'assets').glob('*.png'):
        with Image.open(p) as im:im.verify()
    datajs=(HERE/'report-data.js').read_text();web=json.loads(datajs.removeprefix('window.IPA_REPORT=').removesuffix(';\n'))
    for key in data:assert web[key]==data[key],key
    assert len(web['colors'])==len(web['native_centers'])==80
    subprocess.run(['node','--check',str(HERE/'report.js')],check=True)
    rotation_check=verify_rotation(data)
    browser_path=HERE/'evidence/browser_qa.json'
    browser_checked=False
    browser_scope='完整 HTML 与交互脚本版本匹配。'
    if browser_path.exists():
        browser=json.loads(browser_path.read_text())
        assert browser['passed']
        assert browser['viewer_sha256']==sha(HERE/'report.js')
        if browser['html_sha256']!=sha(HERE/'report.html'):
            # Preserve the original browser certificate. A static rotation-section correction
            # does not imply a new browser run; require every byte outside that section unchanged.
            previous=HERE/'evidence/rotation_before.html'
            assert browser['html_sha256']==sha(previous)
            pattern=r'<section id="rotation">.*?</section>'
            old,count_old=re.subn(pattern,'<section id="rotation"></section>',previous.read_text(),flags=re.S)
            new,count_new=re.subn(pattern,'<section id="rotation"></section>',html,flags=re.S)
            assert count_old==count_new==1 and old==new,'Changes outside the independently checked rotation section'
            browser_scope='沿用原 Chrome 交互验收；本次只更新旋转静态章节，章节外 HTML 逐字相同、脚本 SHA 相同。旋转图另作几何/像素和目视复核，未重跑浏览器。'
        assert browser['test_script_sha256']==sha(HERE/'browser_qa.mjs')
        assert browser['selection_checks']==dict(cells=80,events=66,orders=61,filters=dict(visited=61,missing=19,all=80))
        assert len(browser['numeric_checks'])==10 and len(browser['playback_checks'])==17
        assert all(browser['visual_checks'].values())
        assert browser['console_errors_in_fresh_postfix_tab']==[]
        for p,digest in browser['screenshots'].items():
            assert sha(HERE/p)==digest,p
            with Image.open(HERE/p) as image:image.verify()
        subprocess.run(['node','--check',str(HERE/'browser_qa.mjs')],check=True)
        browser_checked=True
    files=[p for p in HERE.rglob('*') if p.is_file() and p.name!='verification.json' and '__pycache__' not in p.parts]
    # Include native verification.json explicitly; only exclude the root output certificate itself.
    native_verification=HERE/'evidence/native_trace/verification.json'
    if native_verification not in files:files.append(native_verification)
    verification=dict(passed=True, native_cells=80,scheduled_cells=61,event_rows=66,effective_cut_events=30,
        actual_border_pixels=int(borders.sum()),all_static_cell_cards=80,atlas_pages=10,
        native_output_byte_identical=True,geometry_and_schedule_replay=True,
        local_resource_links_checked=len(parser.urls),javascript_syntax_checked=True,
        motion_executed=False,browser_interactions_verified=browser_checked,
        rotation_display_verified=rotation_check,browser_qa_scope=browser_scope,
        browser_qa='实际 Chrome 交互证据见 evidence/browser_qa.json；本命令校验其版本与截图哈希。' if browser_checked else '尚无匹配的浏览器交互证据。',
        sha256={str(p.relative_to(HERE)):sha(p) for p in sorted(files)})
    (HERE/'verification.json').write_text(json.dumps(verification,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in verification.items() if k!='sha256'},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
