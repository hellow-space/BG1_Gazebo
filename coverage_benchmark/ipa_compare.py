"""Run IPA Local Energy+A*, compare against all five pinned historical runs."""
import argparse
import csv
from datetime import datetime,timezone
import json
from pathlib import Path
import shutil
import subprocess
import uuid
import numpy as np
import yaml
from coverage_benchmark.adapters.ipa import plan
from coverage_benchmark.compare_saved import load_sources,THREE,F2C
from coverage_benchmark.ethz_compare import DEFAULT_OUTPUT as FIVE
from coverage_benchmark.map_io import load_map
from coverage_benchmark.metrics import evaluate_path
from coverage_benchmark.models import PlanningInput
from coverage_benchmark.provenance import file_sha256
from coverage_benchmark.runner import plot_scene

DEFAULT_OUTPUT=Path('docs/benchmarks/six-planners-2026-10-30')
DISPLAY={'leader':'Leader','bcd':'Richey BCD','stc':'Spiral-STC','f2c_safe':'Fields2Cover + A*','ethz':'ETHZ BCD + GTSP','ipa_safe':'IPA Local Energy + A*'}
SOURCE_FILES=('coverage_benchmark/adapters/ipa.py','coverage_benchmark/adapters/ipa_build.py',
 'coverage_benchmark/ipa_connection.py','coverage_benchmark/ipa_audit.py','coverage_benchmark/ipa_compare.py',
 'coverage_benchmark/verify_ipa.py','coverage_benchmark/grid_connector.py','coverage_benchmark/swath_geometry.py',
 'coverage_benchmark/adapters/f2c_safe.py','coverage_benchmark/map_io.py','coverage_benchmark/models.py',
 'coverage_benchmark/metrics.py','coverage_benchmark/runner.py','coverage_benchmark/provenance.py',
 'coverage_benchmark/compare_saved.py','coverage_benchmark/ethz_compare.py','coverage_benchmark/configs/three_maps.yaml')


def load_previous(root):
    scenes,old,sources=load_sources(root)
    common=json.loads((root/THREE/'manifest.json').read_text())
    five=json.loads((root/FIVE/'manifest.json').read_text())
    if not five.get('completed') or five['parameters']!=common['parameters'] or five['maps']!=common['maps']:
        raise ValueError('Five-planner historical input mismatch or incomplete evidence')
    for name,digest in five['source_sha256'].items():
        if file_sha256(root/name)!=digest:raise ValueError('Historical source changed: '+name)
    sources[str(FIVE/'manifest.json')]=file_sha256(root/FIVE/'manifest.json')
    for scene in scenes:
        p=FIVE/(scene+'_ethz.json')
        if file_sha256(root/p)!=five['artifacts_sha256'][p.name]:raise ValueError('Historical ETHZ output changed')
        record=json.loads((root/p).read_text())
        if record['scene']!=scene or record['algorithm']!='ethz':raise ValueError('Historical record identity mismatch')
        old.append(record);sources[str(p)]=file_sha256(root/p)
    return scenes,old,sources,common


def start_run(output,manifest):
    (output/'verification.json').unlink(missing_ok=True)
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')


def invalidate_failure(output,error):
    """Never leave a previous PASS certificate after a rejected replay/attempt."""
    (output/'verification.json').unlink(missing_ok=True)
    p=output/'manifest.json'
    if p.exists():
        try:manifest=json.loads(p.read_text())
        except ValueError:return
        manifest.update(completed=False,last_validation_error=str(error))
        p.write_text(json.dumps(manifest,indent=2)+'\n')


def source_path(scene,name):
    if name=='ipa_safe':return DEFAULT_OUTPUT/(scene+'_ipa_safe.json')
    if name=='ethz':return FIVE/(scene+'_ethz.json')
    if name=='f2c_safe':return F2C/(scene+'_safe.json')
    return THREE/(scene+'_'+name+'.json')


def report(root,output,records):
    import os
    text='''# 六算法同图对比：新增 IPA Grid Local Energy＋A*

2026-10-30。IPA在原三张PNG实际运行，另外五种复用历史完整路线并统一复算；只规划，不启动仿真或运动。

## 三项指标

| 场景 | 算法／完整路线 | 覆盖率 | 完整路径长度 | 栅格几何有效性 |
| --- | --- | ---: | ---: | --- |
'''
    for record in records:
        name=record['algorithm'];scene=record['scene'];m=record['metrics']
        link=(scene+'_ipa_safe.json') if name=='ipa_safe' else os.path.relpath(root/source_path(scene,name),output)
        short=scene.split('_')[0].replace('A11123','').replace('A111241','')
        length='—（无路径）' if m['path_length_m'] is None else f"{m['path_length_m']:,.2f} m"
        verdict='通过' if m['path_valid'] else ('失败' if m['path_length_m'] is None else '未通过')
        if name=='bcd' and m['path_length_m'] is not None:verdict+='（原生内存缺陷）'
        text+=f"| {short} | [{DISPLAY[name]}]({link}) | {m['coverage_percent']:.2f}% | {length} | {verdict} |\n"
    text+='\n[CSV](summary.csv) · [输入/源码/依赖指纹](manifest.json) · [独立复核](verification.json)\n\n## 同图完整轨迹与扫掠\n'
    for scene in dict.fromkeys(r['scene'] for r in records):
        text+=f'\n### {scene}.png\n\n![六算法完整全局轨迹与扫掠]({scene}_comparison.png)\n'
    text+='''
## 同一输入与安全处理

- 原图、阈值、float32的0.05 m分辨率、公共起点、0.6 m作业宽度与1 m中心净空与旧实验逐项核对；所有路线均以原图全部自由像素为覆盖分母，只列三项指标。
- IPA固定算法6的原生房间旋转、能量评分和完整目标顺序；输入为公共起点的四邻接净空分量。上游半格内缩保留，因此有额外内部预处理，不宣称各实现内部有效范围相同。
- 覆盖半径0.3 m；原生格距为0.3×sqrt(2) m，并在内部按像素取整。输入像素索引通过半格平移映射到共同像素中心，无地图裁剪/缩放或输出点移动。
- 自有连接层从公共起点开始，按原顺序保留每一个原生目标。直线检查通过则保留，否则在共同原分辨率净空图上无切角A*；所有连接和起点接近计长，不删坏段，不要求回起点。
- 全路通过原图/1 m共同净空审计和禁止触碰障碍格边角的闭格审计；独立复核再用解析线段/阻塞像素方框相交检查。不满足任何一项即整路失败，原生数据仍保留。
- ETHZ仍保留其0.2 m保守分块和闭合回程规则；BCD额外内缩、STC粗化及Leader历史净空问题不改写。此为固定实现的端到端比较，长度需连同覆盖率/范围/起终点读取。
- 仅保证已知栅格模型里的完整路线几何安全；不等同全图完成、动态避障、曲率/动力学或真机能力。

## 原生Local Energy诊断

以下仅用于揭示原生访问序列，未进入六算法主排名；没有把无效线段删除后冒充原生成功。

| 场景 | 原生目标数 | 原生直接连线覆盖率 | 原生直接连线长度 | 原生直接连线有效性 | A*连接数 |
| --- | ---: | ---: | ---: | --- | ---: |
'''
    for r in records:
        if r['algorithm']!='ipa_safe':continue
        m=r['details']['raw_metrics'];d=r['details'];length=m['path_length_m']
        text+=f"| {r['scene'].split('_')[0]} | {len(d.get('native_path',[]))} | {m['coverage_percent']:.2f}% | {length:.2f} m | {'通过' if m['path_valid'] else '未通过'} | {d.get('connections',{}).get('astar_connections',0)} |\n" if length is not None else f"| {r['scene']} | 0 | 0% | — | 失败 | 0 |\n"
    text+='''
## 可溯源与复现

固定上游 `ipa320/ipa_coverage_planning@986c18384ed884dadd3bc857cd0c47c13b7d4716`；只读原文件，薄ROS消息/日志兼容与明确拒绝FOV模式的接口位于自有 `ipa_native/`。manifest绑定原图、共同栅格、实际编译源码/头、编译器、可执行文件、动态库和逐段来源。上游许可声明含学术/非商业用途限制，保留原声明。

```bash
git submodule update --init third_party/ipa_coverage_planning
python3 -m coverage_benchmark.adapters.ipa_build
python3 -m coverage_benchmark.ipa_compare --output docs/benchmarks/six-planners-local
python3 -m coverage_benchmark.verify_ipa --output docs/benchmarks/six-planners-local
```

复现限制：已指纹化项目源码、选定上游头、编译器、实际二进制和当前解析的动态库；OpenCV/Eigen等系统开发头未逐个独立指纹化，跨机器重建仍需核对开发环境。

首次构建需要g++、pkg-config、OpenCV4和Eigen3开发头；仍需本地三张PNG及既有五算法证据。`--reuse`只复算已保存IPA，不重跑规划；另设output保留本次基线。
'''
    (output/'README.md').write_text(text)


def _run(root,runtime,output,reuse=False):
    output.mkdir(parents=True,exist_ok=True)
    scenes,old,sources,common=load_previous(root)
    config=yaml.safe_load((root/'coverage_benchmark/configs/three_maps.yaml').read_text())
    if config!=common['parameters']:raise ValueError('Current benchmark parameters differ from historical inputs')
    implementations={n:file_sha256(root/n) for n in SOURCE_FILES}
    implementations.update({str(p.relative_to(root)):file_sha256(p) for p in (root/'coverage_benchmark/adapters/ipa_native').rglob('*') if p.is_file()})
    if reuse:
        manifest=json.loads((output/'manifest.json').read_text())
        if not manifest['completed'] or manifest['implementation_sha256']!=implementations or manifest['source_sha256']!=sources:
            raise ValueError('Incompatible saved IPA evidence')
        if manifest['parameters']!=config or manifest['maps']!=common['maps']:raise ValueError('Saved input mismatch')
    else:
        manifest=dict(run_id=uuid.uuid4().hex,started_utc=datetime.now(timezone.utc).isoformat(),completed=False,
                      completed_scenes=[],parameters=config,maps={},source_sha256=sources,
                      implementation_sha256=implementations,artifacts_sha256={},old_planners_rerun=False,
                      ipa_planner_rerun=True,motion_executed=False,
                      base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip())
        start_run(output,manifest)
    records=[];rows=[]
    for scene in scenes:
        image=runtime/'data/maps/imported'/scene/(scene+'.png')
        data=load_map(image,config['resolution'],config['clearance'])
        if data.metadata!=common['maps'][scene]:raise ValueError('Map metadata mismatch: '+scene)
        manifest['maps'][scene]=data.metadata;name=scene+'_ipa_safe.json'
        if reuse:
            if file_sha256(output/name)!=manifest['artifacts_sha256'][name]:raise ValueError('IPA result fingerprint changed')
            record=json.loads((output/name).read_text())
        else:
            print('Planning native IPA Local Energy + A*: '+scene,flush=True)
            request=PlanningInput(scene,data.target,data.navigable,data.resolution,data.start,config['tool_width'],config['clearance'],root,runtime)
            work=root/'coverage_benchmark/.work/ipa'/manifest['run_id']/scene
            result=plan(request,work)
            raw=result.details.get('native_path',[])
            result.details['raw_metrics']=evaluate_path(raw,data.free,data.navigable,data.resolution,config['tool_width'],bool(raw))
            metric=evaluate_path(result.path,data.free,data.navigable,data.resolution,config['tool_width'],result.success)
            if result.success and not metric['path_valid']:raise ValueError('IPA safety contract violated')
            record=dict(scene=scene,algorithm='ipa_safe',run_id=manifest['run_id'],path=result.path,planner_success=result.success,message=result.message,details=result.details,metrics=metric)
            (output/name).write_text(json.dumps(record,separators=(',',':'),allow_nan=False)+'\n')
            for artifact in ('ipa_input.json','ipa_input.png','ipa_output.txt','ipa_native.log'):
                dest=output/(scene+'_'+artifact);dest.unlink(missing_ok=True)
                if (work/artifact).exists():shutil.copyfile(work/artifact,dest)
            print(json.dumps(dict(scene=scene,**metric)),flush=True)
        group=[r for r in old if r['scene']==scene]+[record]
        for r in group:
            metric=evaluate_path(r['path'],data.free,data.navigable,data.resolution,config['tool_width'],r['planner_success'])
            if metric!=r['metrics']:raise ValueError('Metric mismatch: '+scene+'/'+r['algorithm'])
            rows.append(dict(scene=scene,algorithm=r['algorithm'],**{k:metric[k] for k in ('coverage_percent','path_length_m','path_valid')}));records.append(r)
        plot_scene(image,[dict(r,algorithm=DISPLAY[r['algorithm']]) for r in group],data,output/(scene+'_comparison.png'),config['tool_width'])
        if scene not in manifest['completed_scenes']:manifest['completed_scenes'].append(scene)
        for p in output.glob(scene+'_*'):
            if p.is_file():manifest['artifacts_sha256'][p.name]=file_sha256(p)
        (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    with (output/'summary.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=['scene','algorithm','coverage_percent','path_length_m','path_valid'],lineterminator='\n');w.writeheader();w.writerows(rows)
    report(root,output,records)
    for name in ('summary.csv','README.md'):manifest['artifacts_sha256'][name]=file_sha256(output/name)
    manifest.update(completed=True,finished_utc=datetime.now(timezone.utc).isoformat())
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return rows


def run(root,runtime,output,reuse=False):
    (output/'verification.json').unlink(missing_ok=True)
    try:return _run(root,runtime,output,reuse)
    except BaseException as error:
        invalidate_failure(output,error)
        raise


if __name__=='__main__':
    root=Path(__file__).resolve().parents[1];parser=argparse.ArgumentParser()
    parser.add_argument('--runtime-root',type=Path,default=root);parser.add_argument('--output',type=Path,default=root/DEFAULT_OUTPUT)
    parser.add_argument('--reuse',action='store_true');args=parser.parse_args()
    run(root,args.runtime_root.resolve(),args.output.resolve(),args.reuse)
