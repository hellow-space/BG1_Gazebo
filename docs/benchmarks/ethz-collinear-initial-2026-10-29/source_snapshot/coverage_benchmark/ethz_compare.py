"""Run ETHZ only, combine its evidence with the four existing planner runs."""
import argparse
import csv
from datetime import datetime,timezone
import json
from pathlib import Path
import shutil
import subprocess
import uuid
import yaml
from coverage_benchmark.adapters.ethz import plan
from coverage_benchmark.compare_saved import load_sources,THREE,LABELS as OLD_LABELS
from coverage_benchmark.map_io import load_map
from coverage_benchmark.metrics import evaluate_path
from coverage_benchmark.models import PlanningInput
from coverage_benchmark.provenance import file_sha256
from coverage_benchmark.runner import plot_scene

LABELS={**OLD_LABELS,'ethz':'ETHZ Polygon Coverage（BCD＋GTSP）'}
DISPLAY={'leader':'Leader','bcd':'Richey BCD','stc':'Spiral-STC','f2c_safe':'Fields2Cover + A*','ethz':'ETHZ BCD + GTSP'}
DEFAULT_OUTPUT=Path('docs/benchmarks/five-planners-2026-10-29')
SOURCE_FILES=('coverage_benchmark/adapters/ethz.py','coverage_benchmark/adapters/ethz_build.py',
              'coverage_benchmark/ethz_compare.py','coverage_benchmark/verify_ethz.py',
              'coverage_benchmark/map_io.py','coverage_benchmark/models.py',
              'coverage_benchmark/raster_geometry.py','coverage_benchmark/adapters/f2c_safe.py',
              'coverage_benchmark/metrics.py','coverage_benchmark/runner.py','coverage_benchmark/provenance.py')


def report(output,records):
    text='''# 五算法同图对比：新增 ETHZ Polygon Coverage Planning

2026-10-29。ETHZ 在相同三张PNG上实际运行；另外四种实现复用已有完整路线，并用同一指标重新核算。本次只规划，不启动Gazebo或运动。

## 三项指标

| 场景 | 算法／完整路线 | 覆盖率 | 路径长度 | 栅格几何有效性 |
| --- | --- | ---: | ---: | --- |
'''
    for record in records:
        scene=record['scene'];name=record['algorithm'];metric=record['metrics']
        if name=='ethz':link=scene+'_ethz.json'
        elif name=='f2c_safe':link='../f2c-obstacle-2026-10-28/'+scene+'_safe.json'
        else:link='../three-planners-2026-10-27/'+scene+'_'+name+'.json'
        short=scene.split('_')[0].replace('A11123','').replace('A111241','')
        length='—（无路径）' if metric['path_length_m'] is None else f"{metric['path_length_m']:,.2f} m"
        verdict='通过' if metric['path_valid'] else ('失败' if metric['path_length_m'] is None else '未通过')
        if name=='bcd' and metric['path_length_m'] is not None:verdict+='（原生内存缺陷）'
        text+=f"| {short} | [{LABELS[name]}]({link}) | {metric['coverage_percent']:.2f}% | {length} | {verdict} |\n"
    text+='''
[三指标CSV](summary.csv) · [输入、版本和实现指纹](manifest.json) · [独立复算](verification.json)

## 同图五列轨迹与扫掠

每张图上排是完整路线，下排是0.6 m作业宽度扫掠并集。蓝线通过公共净空栅格审计，红线未通过；绿点是返回起点。所有列比例一致，失败保留。
'''
    for scene in dict.fromkeys(r['scene'] for r in records):
        text+=f'\n### {scene}.png\n\n![五算法轨迹与覆盖]({scene}_comparison.png)\n'
    text+='''
## ETHZ 的适配与实际运行

初次开启`offset_polygons`时三图在原生分解邻接检查失败；[初次记录与源码快照](../ethz-offset-initial-2026-10-29/README.md)保留。当前关闭该可选预处理，与官方ROS配置一致。

- 上游：[ethz-asl/polygon_coverage_planning](https://github.com/ethz-asl/polygon_coverage_planning)，固定提交 `e4e977e5a52de72665f6f5f82e5770cf092068e1`。核心使用原生BCD区域分解、各cell候选往复扫描、可见图最短连接、GK-MA求解GTSP；不是用我们自己的A*或排序代替上游。
- 自有入口只做地图/路径I/O和运行控制，独立编译核心；ROS 1日志、断言、包路径由薄兼容头提供。CGAL 5.6需要在生成的offset源副本中显式指定原有Epeck输出核；上游文件不改。
- 输入为公共起点的1 m净空可达分量，保留障碍孔。像素边界向可行区内部微退1微米，避免原生可见图沿边界走时落入半开栅格的障碍侧；这一偏置作用在输入多边形，未移动、裁剪或修复输出路线。所有像素中心归属保持一致。
- 扫描间距0.6 m，Line传感器模型、零重叠、欧氏距离目标；BCD、全部可扫方向、`wall_distance=0`（输入已处理净空）、`offset_polygons=false`（沿用官方ROS示例，不做额外cell边界偏置）。起点和终点均为公共起点，因此含原生接近及回程；其它实现的起终点处理见下文。
- 原生进程每图限180秒、3 GiB地址空间；完整输入、多边形、原生输出和日志分别保留。不成功就记失败，不删除坏段制造有效路径。
'''
    for r in records:
        if r['algorithm']!='ethz':continue
        d=r['details'];scene=r['scene'];native=d.get('native',{})
        message='原生完整路线已返回' if r['planner_success'] else ('原生求解超时' if d.get('timed_out') else f"原生失败，退出码{d.get('native_returncode')}")
        text+=f"\n- **{scene}**：{message}。输入顶点{d.get('input_vertices')}、孔洞{d.get('input_holes')}，原生分解cell数{native.get('decomposition_cells','未完成')}，耗时{d.get('native_elapsed_seconds',0):.3f}秒。"
        for suffix,label in (('_ethz_input.json','输入多边形'),('_ethz_native.log','原生日志'),('_ethz_output.txt','原生路径')):
            if (output/(scene+suffix)).exists():text+=f' [{label}]({scene+suffix})'
        text+='\n'
    text+='''
## 共同口径与比较限制

- 同三张PNG、同阈值、0.05 m/像素和零原点假设；三图全图自由像素分母507,209 / 645,425 / 453,369。净空可能切断通道，原图其余区域仍计为未覆盖。
- 作业宽度0.6 m、中心净空1 m；所有相邻路径点间连接、重复段和回程均计长，扫掠只算并集。无效路径仍显示名义扫掠率，必须连同有效性读取。
- Leader、Richey BCD、Spiral-STC、Fields2Cover修改版来自历史真实运行，未重跑；Leader有少量精确净空临界段，BCD有已确认原生内存越界。旧结果不因本次对比改写。
- 各实现内部范围和起终点不同：BCD额外内缩0.55 m、STC保守粗化；F2C修改版和ETHZ使用相同起点净空分量。F2C修改版含起点接近但不要求回起点，ETHZ本次为闭合路线。路径长度不能脱离覆盖率与此差异直接排名。
- 这是固定实现与参数的端到端比较。通过栅格几何不代表全图覆盖、连续footprint、曲率/动力学可执行或真机能力；没有执行机器人运动。

## 依赖与可溯源

CGAL/Mono的Ubuntu deb包在本地 `.build/ethz/deps` 解包，不修改系统安装；manifest记录包及实际二进制指纹。GK-MA源码下载自上游指定地址，校验原始MD5后原样应用ETHZ仓库附带补丁。仓库采用GPL-3.0-or-later；依赖GK-MA另有限非商业用途条款，见[上游许可说明](https://github.com/ethz-asl/polygon_coverage_planning#licensing)。

```bash
git submodule update --init third_party/polygon_coverage_planning
python3 -m coverage_benchmark.adapters.ethz_build
python3 -m pytest coverage_benchmark/tests/test_ethz.py -q
python3 -m coverage_benchmark.ethz_compare
python3 -m coverage_benchmark.verify_ethz
```

首次构建需网络、apt下载工具、clang++、g++、CMake、Boost/Eigen开发头及原有Python/GDAL环境。加 `--output` 可另存实验；`--reuse` 只核验并汇总已保存ETHZ路线，不重跑。历史来源：[四算法总览](../four-planners-2026-10-28/README.md)。
'''
    (output/'README.md').write_text(text)


def copy_run_artifacts(source,output,scene):
    """Replace only this scene's managed native evidence, never unrelated files."""
    for artifact in ('ethz_input.json','ethz_input.txt','ethz_output.txt','ethz_native.log'):
        destination=output/(scene+'_'+artifact)
        destination.unlink(missing_ok=True)
        if (source/artifact).exists():shutil.copyfile(source/artifact,destination)


def run(root,runtime,output,reuse=False):
    output.mkdir(parents=True,exist_ok=True)
    scenes,old,sources=load_sources(root)
    old_manifest=json.loads((root/THREE/'manifest.json').read_text())
    config=yaml.safe_load((root/'coverage_benchmark/configs/three_maps.yaml').read_text())
    if config!=old_manifest['parameters']:raise ValueError('Current benchmark parameters differ from historical runs')
    if reuse:
        manifest=json.loads((output/'manifest.json').read_text())
        if manifest['parameters']!=config or manifest['maps']!=old_manifest['maps']:raise ValueError('Incompatible ETHZ saved inputs')
        for name,digest in manifest['implementation_sha256'].items():
            if file_sha256(root/name)!=digest:raise ValueError('ETHZ implementation changed: '+name)
    else:
        manifest=dict(run_id=uuid.uuid4().hex,started_utc=datetime.now(timezone.utc).isoformat(),completed=False,
                      parameters=config,maps={},completed_scenes=[],ethz_planner_rerun=True,
                      old_planners_rerun=False,motion_executed=False,
                      base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
                      implementation_sha256={n:file_sha256(root/n) for n in SOURCE_FILES},
                      artifacts_sha256={},source_sha256=sources)
    rows=[];records=[]
    for scene in scenes:
        image=runtime/'data/maps/imported'/scene/(scene+'.png')
        data=load_map(image,config['resolution'],config['clearance'])
        if data.metadata!=old_manifest['maps'][scene]:raise ValueError('Map metadata changed: '+scene)
        manifest['maps'][scene]=data.metadata
        name=scene+'_ethz.json'
        if reuse:
            if file_sha256(output/name)!=manifest['artifacts_sha256'][name]:raise ValueError('ETHZ result fingerprint changed')
            ethz=json.loads((output/name).read_text())
        else:
            print('Planning native ETHZ: '+scene,flush=True)
            request=PlanningInput(scene,data.target,data.navigable,data.resolution,data.start,
                                  config['tool_width'],config['clearance'],root,runtime)
            work=root/'coverage_benchmark/.work/ethz'/manifest['run_id']/scene
            result=plan(request,work)
            metric=evaluate_path(result.path,data.free,data.navigable,data.resolution,config['tool_width'],result.success)
            ethz=dict(run_id=manifest['run_id'],scene=scene,algorithm='ethz',path=result.path,
                      planner_success=result.success,message=result.message,details=result.details,metrics=metric)
            (output/name).write_text(json.dumps(ethz,separators=(',',':'),allow_nan=False)+'\n')
            copy_run_artifacts(work,output,scene)
            print(json.dumps(dict(scene=scene,algorithm='ethz',**metric)),flush=True)
        group=[r for r in old if r['scene']==scene]+[ethz]
        for record in group:
            actual=evaluate_path(record['path'],data.free,data.navigable,data.resolution,config['tool_width'],record['planner_success'])
            if actual!=record['metrics']:raise ValueError('Saved metric mismatch: '+scene+'/'+record['algorithm'])
            records.append(record)
            rows.append(dict(scene=scene,algorithm=record['algorithm'],**{k:actual[k] for k in ('coverage_percent','path_length_m','path_valid')}))
        plot_scene(image,[dict(r,algorithm=DISPLAY[r['algorithm']]) for r in group],data,
                   output/(scene+'_comparison.png'),config['tool_width'])
        if scene not in manifest['completed_scenes']:manifest['completed_scenes'].append(scene)
        for p in output.glob(scene+'_*'):
            if p.is_file():manifest['artifacts_sha256'][p.name]=file_sha256(p)
        (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    with (output/'summary.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['scene','algorithm','coverage_percent','path_length_m','path_valid'],lineterminator='\n')
        writer.writeheader();writer.writerows(rows)
    report(output,records)
    for name in ('summary.csv','README.md'):manifest['artifacts_sha256'][name]=file_sha256(output/name)
    manifest.update(completed=True,finished_utc=datetime.now(timezone.utc).isoformat())
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return rows


if __name__=='__main__':
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser()
    parser.add_argument('--runtime-root',type=Path,default=root)
    parser.add_argument('--output',type=Path,default=root/DEFAULT_OUTPUT)
    parser.add_argument('--reuse',action='store_true')
    args=parser.parse_args()
    run(root,args.runtime_root.resolve(),args.output.resolve(),args.reuse)
