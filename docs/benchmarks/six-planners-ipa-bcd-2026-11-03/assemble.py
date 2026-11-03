"""Assemble fresh records into a common table, plots and a browser report."""
import csv
from datetime import datetime, timezone
import html
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from coverage_benchmark.map_io import load_map
from coverage_benchmark.runner import plot_scene
from coverage_benchmark.provenance import file_sha256
from rerun import sources,save
OUT=Path(__file__).resolve().parent
LABELS={'leader':'Leader','ipa_bcd':'IPA BCD＋安全连接','stc':'Spiral-STC','f2c_safe':'Fields2Cover＋安全连接','ethz':'ETHZ BCD＋GTSP','ipa_safe':'IPA Local Energy＋A*'}


def length(m):return '—' if m['path_length_m'] is None else f"{m['path_length_m']:,.2f} m"

def verdict(rec):
    if not rec['planner_success']:return '规划失败'
    return '通过' if rec['metrics']['path_valid'] else '不通过（名义覆盖率）'


def main():
    mf=json.loads((OUT/'manifest.json').read_text());assert mf['planning_completed']
    assert mf['implementation_sha256']==sources()
    records=[json.loads((OUT/p).read_text()) for p in mf['records']];assert len(records)==18
    p=mf['parameters'];rows=[]
    for rec in records:
        rows.append(dict(scene=rec['scene'],algorithm=rec['algorithm'],**{k:rec['metrics'][k] for k in ('coverage_percent','path_length_m','path_valid')}))
    with (OUT/'summary.csv').open('w',newline='') as file:
        w=csv.DictWriter(file,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    text='''# 六算法全量重跑：BCD 已替换为 IPA BCD

2026-11-03，原三图 × 当前六算法共 **18 组，全部真实重新调用规划器**，没有读取旧路线代替本次运行。旧 Richey BCD、BCD_LOCAL、HYBRID_LOCAL 不参与本轮。仅启动隔离规划服务，不运行 Gazebo、导航执行或机器人运动。

**网页入口：[report.html](report.html)** · [三项指标 CSV](summary.csv) · [运行/源码/原生证据指纹](manifest.json) · [独立复核](verification.json)

## 三项指标

覆盖分母为原图全部已知自由像素；失败返回空路线，长度记缺失。路径不通过安全检查时，扫掠率只表示几何名义值，不能视为可执行覆盖。

| 地图 | 算法／完整路线 | 覆盖率 | 完整返回路径长度 | 共同栅格几何有效性 |
| --- | --- | ---: | ---: | --- |
'''
    for rec in records:
        m=rec['metrics'];text+=f"| {rec['scene'].split('_')[0]} | [{LABELS[rec['algorithm']]}]({rec['scene']}_{rec['algorithm']}.json) | {m['coverage_percent']:.4f}% | {length(m)} | {verdict(rec)} |\n"
    text+='\n## 同图完整轨迹与扫掠\n\n上排：完整返回路径，蓝色通过共同净空检查、红色不通过，绿点为返回起点。下排：工具圆盘扫掠与原图自由像素的交集。各列统一坐标、范围与像素尺度。图可单独打开缩放。\n'
    for scene in p['scenes']:
        recs=[r for r in records if r['scene']==scene]
        m=load_map(ROOT/'data/maps/imported'/scene/(scene+'.png'),p['resolution'],p['clearance'])
        print('PLOT',scene,flush=True)
        plot_scene(ROOT/'data/maps/imported'/scene/(scene+'.png'),recs,m,OUT/(scene+'_comparison.png'),p['tool_width'])
        text+=f'\n### {scene}\n\n![六算法路径与扫掠]({scene}_comparison.png)\n'
    text+='''
## 输入与实现边界

- 同三张原 PNG、阈值、0.05 m/像素（内部与既有管线一致转换为 float32）、公共请求起点、0.6 m 作业宽度、1 m 中心净空。原图全体自由像素为分母；净空外、起点不可达或内部预处理排除的区域仍计未覆盖。
- IPA BCD 直接调用固定 IPA 的分解及单 cell 扫线；项目层裁剪安全候选、按安全连接距离贪心调度并连接。不是旧 Richey BCD。这个适配关闭额外半格腐蚀，不调用 Local、不做补扫。
- IPA Local Energy 使用原有算法 6 适配，保留原生额外半格内缩和访问顺序；项目层保留所有目标并加入起点接近/安全 A*。它与 BCD 的内部有效范围不同，不能只比较路线长短。
- Fields2Cover 使用原生排序扫线，项目层进行净空裁剪及安全连接；原生转弯失败时明确保留错误和已成功的原生有序扫线，不将失败曲线冒充成功。
- ETHZ 使用 0.2 m 保守全自由分块、原生 BCD/扫描/可见图/GK-MA；返回包含公共起点接近和回程的闭合路线。分块舍弃数量保存在每图 JSON。
- Leader 和 STC 使用原适配；Leader 服务返回的净空临界段按共同指标如实判定。STC 保守粗化并从最近可用粗格开始，可能失败或存在起点偏移；没有补造起点连接。下面列出实际起点偏移，不能把各算法的路径长度视为相同起终点问题的最优解。
- 长度计算包含返回路径全部相邻点连接、绕行和重复；已扫面积算并集。不保证全图覆盖、曲率/动力学可执行、动态避障或真机能力。

## 实际运行和失败细节

| 地图 | 算法 | 实际调用耗时（含适配，不含统一绘图） | 返回起点距请求起点 | 原图穿障段 | 共同净空违规段 |
| --- | --- | ---: | ---: | ---: | ---: |
'''
    for rec in records:
        diag=rec['metrics']['diagnostics'];offset=np.linalg.norm(np.asarray(rec['path'][0])-rec['input']['start']) if rec['path'] else None
        text+=f"| {rec['scene'].split('_')[0]} | {LABELS[rec['algorithm']]} | {rec['adapter_elapsed_seconds']:.2f} s | {f'{offset:.4f} m' if offset is not None else '—'} | {diag.get('collision_segments','—')} | {diag.get('invalid_segments','—')} |\n"
    for rec in records:
        if not rec['planner_success'] or not rec['metrics']['path_valid']:
            text+=f"\n- **{rec['scene'].split('_')[0]} / {LABELS[rec['algorithm']]}**：{rec['message']}；指标诊断：`{json.dumps(rec['metrics']['diagnostics'],ensure_ascii=False)}`。\n"
        if rec['algorithm']=='f2c_safe' and rec['details'].get('native_worker_error'):
            text+=f"\n- **{rec['scene'].split('_')[0]} / F2C 原生转弯状态**：{rec['details']['native_worker_error']}；最终路线是否有效以主表为准。\n"
    text+='''
## 复现与验证

在项目根目录、相同依赖和本地地图存在时：

```bash
# 规划脚本只接受全新输出目录，避免覆盖本次或历史结果。
# 另复制本目录三个 .py 脚本到 docs/benchmarks/<新目录>/ 后执行：
python3 docs/benchmarks/<新目录>/rerun.py
python3 docs/benchmarks/<新目录>/assemble.py
python3 docs/benchmarks/<新目录>/verify.py
```

独立复核重新检查输入/源码/依赖及全部结果/证据 SHA，重算三项指标与独立点到折线覆盖见证；对 IPA BCD 重放原生 cell 扫线、调度与连接，对 Local 重放全部目标和连接，对 F2C 重放原生条带及连接，对 ETHZ 核对完整原生闭合路径；Leader 对照服务响应、STC 确定性重放。失败保留，不要求所有规划器成功或所有返回路线有效才保存证书。适用静态栅格几何范围。

`evidence/<地图>/<算法>/` 保存本轮输入、原生输出及日志，JSON 中的原生路径指向这些文件。证据含本机绝对路径，异机应另目录重新运行。历史报告未改写。
'''
    (OUT/'README.md').write_text(text)
    # A small standalone HTML view with links to full-size images and raw records.
    parts=['<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>六算法重跑：IPA BCD</title><style>body{font:16px/1.6 system-ui;max-width:1550px;margin:32px auto;padding:0 20px;color:#183246;background:#f5f8fb}table{border-collapse:collapse;width:100%;background:white}td,th{border:1px solid #cbd5df;padding:8px;text-align:left}img{width:100%;background:white}a{color:#1764b3}.bad{color:#b02b2b}section{margin:32px 0}code{background:#e4edf4;padding:2px 5px}</style><h1>六算法全量重跑：IPA BCD 替换旧 BCD</h1><p>2026-11-03 · 三图 × 六算法 · 全部重新调用，共 18 组 · 0.05 m / 0.6 m 作业宽 / 1 m 净空 · 未执行运动</p><p><a href="README.md">完整说明</a> · <a href="summary.csv">指标 CSV</a> · <a href="manifest.json">来源与证据</a> · <a href="verification.json">独立验证</a></p><p>无效路径的覆盖率仅为名义扫掠值。各算法内部范围、粗化及回程约束不同；详见完整说明。</p>']
    for scene in p['scenes']:
        parts.append('<section><h2>'+scene+'</h2><table><tr><th>算法 / 完整路线 JSON</th><th>覆盖率</th><th>路径长度</th><th>共同几何有效性</th></tr>')
        for rec in [r for r in records if r['scene']==scene]:
            m=rec['metrics'];name=rec['algorithm'];cls='' if m['path_valid'] else ' class="bad"'
            parts.append(f'<tr{cls}><td><a href="{scene}_{name}.json">{html.escape(LABELS[name])}</a></td><td>{m["coverage_percent"]:.4f}%</td><td>{length(m)}</td><td>{verdict(rec)}</td></tr>')
        parts.append(f'</table><p>点击图打开原尺寸，浏览器可继续放大。</p><a href="{scene}_comparison.png"><img src="{scene}_comparison.png" alt="{scene} 六算法轨迹和覆盖"></a></section>')
    parts.append('<p>原生日志在 evidence/ 下。旧 Richey BCD 和两个已删除混合变体未参与。</p></html>')
    (OUT/'report.html').write_text(''.join(parts))
    mf['artifacts_sha256']={str(f.relative_to(OUT)):file_sha256(f) for f in OUT.rglob('*') if f.is_file() and f.name not in ('manifest.json','verification.json') and '__pycache__' not in f.parts}
    mf.update(completed=True,report_finished_utc=datetime.now(timezone.utc).isoformat());save(OUT/'manifest.json',mf)
    print('ASSEMBLED_18_FRESH_RECORDS',flush=True)


if __name__=='__main__':main()
