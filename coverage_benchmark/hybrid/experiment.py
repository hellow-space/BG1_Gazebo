"""IPA BCD/Local comparison, durable native evidence, independent replay."""
import argparse
import csv
from datetime import datetime,timezone
import json
from pathlib import Path
import uuid
import numpy as np
from PIL import Image
from scipy.ndimage import label
from coverage_benchmark.hybrid.planner import compare,partition,candidates
from coverage_benchmark.hybrid.build import build,verify_build
from coverage_benchmark.map_io import load_map
from coverage_benchmark.models import PlanningInput
from coverage_benchmark.metrics import evaluate_path,sweep_mask,path_length
from coverage_benchmark.ipa_connection import strict_path_valid
from coverage_benchmark.ipa_audit import path_box_audit
from coverage_benchmark.provenance import file_sha256,array_fingerprint
from coverage_benchmark.runner import plot_scene

MODES=('ipa_bcd','ipa_local')
DEFAULT_SCENES=('A11123AJ04_231127_094529','A11123AC22_231213_163036','A111241C03_240410_144535')
DEFAULT_OUTPUT='docs/benchmarks/ipa-native-local'
NAMES=dict(ipa_bcd='IPA BCD',ipa_local='IPA Local Energy')
HELPERS=('map_io.py','metrics.py','models.py','grid_connector.py','swath_geometry.py','ipa_connection.py','ipa_audit.py','provenance.py','runner.py','adapters/ipa.py','adapters/ipa_build.py')


def sources(root):
    paths=[*sorted((root/'coverage_benchmark/hybrid').rglob('*.py')),*sorted(p for p in (root/'coverage_benchmark/hybrid/native').rglob('*') if p.is_file()),*(root/'coverage_benchmark'/p for p in HELPERS)]
    return {str(p.relative_to(root)):file_sha256(p) for p in paths}


def save(path,value,compact=False):path.write_text(json.dumps(value,ensure_ascii=False,indent=None if compact else 2,separators=(',',':') if compact else None,allow_nan=False)+'\n')


def invalidate(out,error):
    (out/'verification.json').unlink(missing_ok=True)
    p=out/'manifest.json'
    if p.exists():
        value=json.loads(p.read_text());value.update(completed=False,validation_error=str(error));save(p,value)


def partition_plot(source,m,labels,destination,regions):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    image=np.asarray(Image.open(source).convert('L'));r=m.resolution;h,w=image.shape
    fig,axes=plt.subplots(1,2,figsize=(16,8),constrained_layout=True)
    kinds={x['id']:x['kind'] for x in regions};rgba=np.zeros((*labels.shape,4))
    for rid,kind in kinds.items():rgba[labels==rid]=[.1,.45,.95,.7] if kind=='bcd' else [1.,.5,.05,.7]
    for ax in axes:ax.imshow(image,cmap='gray',vmin=0,vmax=255,extent=(0,w*r,0,h*r));ax.set_aspect('equal');ax.set_xlabel('x (m)');ax.set_ylabel('y (m)')
    axes[0].imshow(rgba,origin='lower',extent=(0,w*r,0,h*r));axes[0].set_title('Blue: native IPA BCD cells | Orange: raster gaps')
    masked=np.ma.masked_where(labels<0,labels);axes[1].imshow(masked,origin='lower',extent=(0,w*r,0,h*r),cmap='tab20',alpha=.8)
    axes[1].set_title('Task regions; gaps retained; no artificial navigation walls')
    fig.savefig(destination,dpi=150);plt.close(fig)


def report(out,records):
    text='# IPA BCD / Local Energy 对照\n\n只运行纯 IPA BCD 和纯 IPA Local Energy；参数以 manifest 为准。混合变体已移除。\n\n| 场景 | 算法 | 覆盖率 | 完整路径长度 | 栅格几何有效性 |\n| --- | --- | ---: | ---: | --- |\n'
    for rec in records:
        m=rec['metrics'];length='—' if m['path_length_m'] is None else f"{m['path_length_m']:,.2f} m"
        text+=f"| {rec['scene'].split('_')[0]} | [{NAMES[rec['algorithm']]}]({rec['scene']}_{rec['algorithm']}.json) | {m['coverage_percent']:.2f}% | {length} | {'通过' if m['path_valid'] else '失败'} |\n"
    text+='\n覆盖分母为原图全部已知自由格。路线包含起点接近与所有跨区连接；未运行仿真或运动。BCD 采用安全距离贪心调度，不保证全局最优；Local 不做补扫。\n\n## 轨迹与分区图\n\n'
    for scene in dict.fromkeys(x['scene'] for x in records):text+=f"### {scene}\n\n![完整轨迹及扫掠]({scene}_comparison.png)\n\n![区域与算法分配]({scene}_partition.png)\n\n"
    text+='复现命令见[模块说明](../../../coverage_benchmark/hybrid/README.md)。[manifest](manifest.json)固定源码、构建/运行库、原图、输入、全部证据；[verification](verification.json)逐项独立重放。失败会撤销旧验证证书。\n'
    (out/'README.md').write_text(text)


def run(root,maps,out,scenes=DEFAULT_SCENES,resolution=.05,width=.6,clearance=1.,plots=True):
    root=Path(root).resolve();maps=Path(maps).resolve();out=Path(out).resolve();out.mkdir(parents=True,exist_ok=True)
    (out/'verification.json').unlink(missing_ok=True)
    manifest=dict(run_id=uuid.uuid4().hex,created_utc=datetime.now(timezone.utc).isoformat(),completed=False,
                  scenes=list(scenes),parameters=dict(resolution=resolution,tool_width=width,clearance=clearance),source_sha256=sources(root),maps={},motion_executed=False)
    save(out/'manifest.json',manifest)
    try:
        _,build_info=build(root);manifest['build']=build_info;records=[]
        for scene in scenes:
            print('Planning '+scene,flush=True);source=maps/scene/(scene+'.png');m=load_map(source,resolution,clearance);manifest['maps'][scene]=m.metadata
            work=out/'evidence'/manifest['run_id']/scene
            request=PlanningInput(scene,m.free,m.navigable,m.resolution,m.start,width,clearance,root,work)
            results=compare(request,progress=lambda mode:print('  '+mode,flush=True));scene_records=[]
            decomp=results['ipa_bcd'].details['decomposition'];labels=np.load(work/'partition.npz')['labels']
            for name,result in results.items():
                # The generated output.json is the sole raw kernel output, no duplicated geometry.
                result.details['decomposition']={key:decomp[key] for key in ('diagnostics','evidence')}
                for call in result.details.get('native_calls',[]):
                    call.pop('build',None);call.pop('path',None)
                metric=evaluate_path(result.path,m.free,m.navigable,m.resolution,width,result.success)
                if result.success and (not result.details['strict_valid'] or not result.details['box_audit']['passed']):raise ValueError('False success')
                rec=dict(scene=scene,algorithm=name,input=m.metadata,planner_success=result.success,message=result.message,path=result.path,details=result.details,metrics=metric)
                save(out/(scene+'_'+name+'.json'),rec,compact=True);records.append(rec);scene_records.append(rec)
                print('  '+name+' '+str(metric),flush=True)
            if plots:
                plot_scene(source,scene_records,m,out/(scene+'_comparison.png'),width)
                partition_plot(source,m,labels,out/(scene+'_partition.png'),results['ipa_bcd'].details['partition'])
            save(out/'manifest.json',manifest)
        with (out/'metrics.csv').open('w') as file:
            writer=csv.writer(file);writer.writerow(['scene','algorithm','coverage_percent','path_length_m','path_valid'])
            for rec in records:writer.writerow([rec['scene'],rec['algorithm'],*(rec['metrics'][k] for k in ('coverage_percent','path_length_m','path_valid'))])
        report(out,records)
        if sources(root)!=manifest['source_sha256']:raise ValueError('Source changed during run')
        files=[p for p in out.rglob('*') if p.is_file() and p.name not in ('manifest.json','verification.json') and (p.parent==out or manifest['run_id'] in p.parts)]
        manifest['artifacts_sha256']={str(p.relative_to(out)):file_sha256(p) for p in files};manifest['completed']=True;save(out/'manifest.json',manifest)
        return verify(root,maps,out)
    except BaseException as error:invalidate(out,error);raise


def require(condition,message):
    if not condition:raise ValueError(message)


def _output(evidence):
    return json.loads(next(Path(p) for p in evidence['files_sha256'] if Path(p).name=='output.json').read_text())


def _mask(evidence,name):
    return np.asarray(Image.open(evidence['request'][name]).convert('L'))==255


def verify(root,maps,out):
    root=Path(root).resolve();maps=Path(maps).resolve();out=Path(out).resolve()
    try:
        manifest=json.loads((out/'manifest.json').read_text());require(manifest['completed'],'Incomplete run')
        require(manifest['source_sha256']==sources(root),'Source hash mismatch')
        require(verify_build(manifest['build'],root),'Build/runtime mismatch')
        for name,digest in manifest['artifacts_sha256'].items():require(file_sha256(out/name)==digest,'Artifact changed: '+name)
        p=manifest['parameters'];count=0;witnesses=0;rows=[]
        for scene in manifest['scenes']:
            m=load_map(maps/scene/(scene+'.png'),p['resolution'],p['clearance']);require(m.metadata==manifest['maps'][scene],'Input mismatch')
            allowed=np.flipud(m.navigable);labs,_=label(allowed);x,y=np.floor(np.asarray(m.start)/m.resolution).astype(int);f=labs==labs[y,x]
            for mode in MODES:
                rec=json.loads((out/(scene+'_'+mode+'.json')).read_text());require(rec['input']==m.metadata,'Per-record input mismatch')
                metric=evaluate_path(rec['path'],m.free,m.navigable,m.resolution,p['tool_width'],rec['planner_success']);require(metric==rec['metrics'],'Metric mismatch')
                rows.append([scene,mode,metric['coverage_percent'],metric['path_length_m'],metric['path_valid']]);d=rec['details'];count+=1
                all_evidence=[d['decomposition']['evidence'],*(call['evidence'] for call in d.get('native_calls',[]))]
                if 'native_failure' in d:all_evidence.append(d['native_failure']['evidence'])
                for evidence in all_evidence:
                    for name,digest in evidence['files_sha256'].items():require(file_sha256(Path(name))==digest,'Native evidence changed: '+name)
                    require(np.array_equal(_mask(evidence,'allowed'),f),'Native F mismatch')
                    for name,digest in evidence['masks_sha256'].items():require(array_fingerprint(_mask(evidence,name))==digest,'Native mask hash mismatch')
                decomposition=_output(d['decomposition']['evidence']);regions,partition_labels=partition(f,decomposition['cells'],m.resolution,p['tool_width'])
                saved=np.load(d['partition_file']);require(np.array_equal(saved['labels'],partition_labels) and np.array_equal(saved['f'],f),'Partition mismatch')
                if not rec['planner_success']:require(not rec['path'],'Failure returned path');continue
                path=np.asarray(rec['path']);require(np.array_equal(path[0],m.start),'Start approach missing')
                require(strict_path_valid(path,f,m.resolution) and path_box_audit(path,f,m.resolution),'Unsafe final path')
                covered=sweep_mask(path,f.shape,m.resolution,p['tool_width']);require(int((f&~covered).sum())==d['residual_pixels'],'Residual mismatch')
                require(int((np.flipud(m.free)&~covered).sum())==d['residual_free_pixels'],'Original free residual mismatch')
                previous=0
                for section in d['sections']:
                    require(section['first_vertex']==previous,'Unaccounted final path section')
                    previous=section['last_vertex']
                    a,b=section['first_vertex'],section['last_vertex'];require(0<=a<=b<len(path),'Invalid section range')
                require(previous==len(path)-1,'Final path suffix has no source')
                # Rebuild candidates and replay every target in order for each source group.
                pieces=[candidates(cell,f,m.resolution)[0] for cell in decomposition['cells']]
                for item in d['schedule']:
                    if item['kernel']!='bcd':continue
                    ids=[item['region']]
                    values=[v for index in ids for v in pieces[index]]
                    if item['reverse']:values=[v[::-1] for v in values[::-1]]
                    endpoints=[v for pair in values for v in pair]
                    sections=[s for s in d['sections'] if s['source'].get('phase')=='main' and s['source'].get('kernel')=='ipa_bcd' and s['source'].get('region')==item['region']]
                    require(len(endpoints)==len(sections),'BCD target count mismatch')
                    require(all(np.allclose(path[s['last_vertex']],v,rtol=0,atol=1e-12) for s,v in zip(sections,endpoints)),'BCD target replay mismatch')
                for call in d['native_calls']:
                    e=call['evidence'];ctx=call['context'];prefix=path[:ctx['vertex']+1];before=sweep_mask(prefix,f.shape,m.resolution,p['tool_width'])
                    require(np.array_equal(_mask(e,'covered'),before),'C inheritance mismatch')
                    task=_mask(e,'task')
                    require(ctx['phase']=='main' and ctx['region']=='all','Unexpected Local phase')
                    require(np.array_equal(task,f),'R mismatch')
                    require(e['request']['target_policy']=='centres','Local target policy mismatch')
                    raw=_output(e)['path'];sections=[s for s in d['sections'] if s['source'].get('kernel')=='ipa_local' and s['source'].get('phase')=='main']
                    require(len(raw)==len(sections),'Local target count mismatch')
                    require(all(np.allclose(path[s['last_vertex']],v,rtol=0,atol=1e-12) for s,v in zip(sections,raw)),'Local target replay mismatch')
                # Independent point-to-polyline distance witnesses, not the sweep implementation.
                yy,xx=np.nonzero(np.flipud(m.free));indices=np.linspace(0,len(xx)-1,min(256,len(xx)),dtype=int)
                a=path[:-1];delta=path[1:]-a;norm=np.sum(delta*delta,axis=1)
                for index in indices:
                    point=np.array([xx[index]+.5,yy[index]+.5])*m.resolution
                    t=np.divide(np.sum((point-a)*delta,axis=1),norm,out=np.zeros_like(norm),where=norm>0).clip(0,1)
                    dist2=np.min(np.sum((point-a-t[:,None]*delta)**2,axis=1))
                    require(bool(covered[yy[index],xx[index]])==bool(dist2<=(p['tool_width']/2)**2+1e-12),'Independent sweep witness mismatch');witnesses+=1
        with (out/'metrics.csv').open() as file:csv_rows=list(csv.reader(file))[1:]
        require([[('' if v is None else str(v)) for v in row] for row in rows]==csv_rows,'CSV mismatch')
        certificate=dict(passed=True,run_id=manifest['run_id'],records=count,independent_coverage_witnesses=witnesses,
                         strict_and_analytic_box_audits=True,native_target_replay=True,frc_replay=True,motion_executed=False)
        save(out/'verification.json',certificate);return certificate
    except BaseException as error:invalidate(out,error);raise


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',default=DEFAULT_OUTPUT);parser.add_argument('--maps-root',default='data/maps/imported');parser.add_argument('--verify',action='store_true');args=parser.parse_args()
    result=verify(Path.cwd(),args.maps_root,args.output) if args.verify else run(Path.cwd(),args.maps_root,args.output)
    print(json.dumps(result,ensure_ascii=False))

if __name__=='__main__':main()
