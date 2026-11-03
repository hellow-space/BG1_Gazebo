#!/usr/bin/env python3
"""Run the actual instrumented IPA decomposition on four small teaching masks."""
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

import numpy as np
from PIL import Image

HERE=Path(__file__).resolve().parent
REPORT=HERE.parent
ROOT=HERE.parents[3]
OUT=HERE/'evidence'
OUT.mkdir(exist_ok=True)


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def compile_native():
    verification=json.loads((REPORT/'evidence/native_trace/verification.json').read_text())
    for path,digest in verification['generated_sha256'].items():
        assert sha(Path(path))==digest, f'instrumented native source changed: {path}'
    cmd=shlex.split(verification['compile_command'])
    main=next(x for x in cmd if x.endswith('/main.cpp'))
    source=HERE/'native_demo.cpp'
    work=ROOT/'coverage_benchmark/.build/ipa_bcd_simple_scenes'
    work.mkdir(parents=True,exist_ok=True)
    binary=work/'native_demo'
    cmd=[str(source) if x==main else x for x in cmd]
    cmd[cmd.index('-o')+1]=str(binary)
    result=subprocess.run(cmd,text=True,capture_output=True,timeout=180)
    (OUT/'compile.log').write_text(result.stdout+result.stderr)
    if result.returncode:raise RuntimeError(result.stderr[-9000:])
    return binary,cmd,verification


def masks():
    room=np.zeros((24,30),np.uint8);room[3:21,3:27]=255
    island=room.copy();island[8:14,12:18]=0
    meeting=np.zeros_like(room);meeting[3:21,3:9]=255;meeting[7:21,18:27]=255;meeting[14:21,3:27]=255
    notch=room.copy();notch[8:14,18:27]=0
    return {'empty':room,'island':island,'meeting':meeting,'notch':notch}


def run():
    (OUT/'verification.json').unlink(missing_ok=True)
    binary,command,source=compile_native()
    expected={
        'empty':([(21,'OUT',1,0,0,0)],1),
        'island':([(8,'IN',1,2,6,2),(14,'OUT',2,1,6,2),(21,'OUT',1,0,0,0)],4),
        'meeting':([(7,'IN',1,2,0,0),(14,'OUT',2,1,9,2),(21,'OUT',1,0,0,0)],3),
        'notch':([(21,'OUT',1,0,0,0)],1),
    }
    result={}
    for name,mask in masks().items():
        path=OUT/name;path.mkdir(exist_ok=True)
        Image.fromarray(mask).save(path/'input.png')
        process=subprocess.run([str(binary),str(path/'input.png'),str(path)],text=True,capture_output=True,timeout=30)
        (path/'native.log').write_text(process.stdout+process.stderr)
        if process.returncode:raise RuntimeError(f'{name}: {process.stderr[-4000:]}')
        trace=json.loads((path/'trace.json').read_text())
        observed=[(e['y'],e['type'],e['before'],e['after'],len(e['critical_points']),len(e['cuts'])) for e in trace['events']]
        before=np.asarray(Image.open(path/'03_labels_before_merge.png'))
        final=np.asarray(Image.open(path/'04_labels_final.png'))
        cuts=np.asarray(Image.open(path/'02_cuts.png'))
        rebuilt=np.zeros_like(mask,dtype=bool)
        for e in trace['events']:
            for x0,y,x1,_ in e['cuts']:rebuilt[y,x0:x1+1]=True
        assert np.array_equal(rebuilt,cuts==25),name
        assert trace['before_merge_count']==trace['final_label_count']==trace['polygon_count']==len(np.unique(final[final>0]))
        result[name]=dict(events=observed,cells=trace['final_label_count'],free_pixels=int((mask==255).sum()),
                          boundary_pixels=int(rebuilt.sum()),cut_pixels_covered=bool((final[rebuilt]>0).all()),
                          input_sha256=sha(path/'input.png'),trace_sha256=sha(path/'trace.json'))
        print(name,json.dumps(result[name],ensure_ascii=False),flush=True)
        if (observed,result[name]['cells'])!=expected[name]:
            raise AssertionError(f'{name}: actual {(observed,result[name]["cells"])} differs from expected {expected[name]}')
        assert np.array_equal(mask==255,(final>0)),name
    verification=dict(passed=True,results=result,native_original_commit=source['upstream_commit'],
        native_source_trace_sha256=source['trace_script_sha256'],cpp_sha256=sha(HERE/'native_demo.cpp'),
        build_script_sha256=sha(Path(__file__)),binary_sha256=sha(binary),
        compile_command=shlex.join(command),independent_masks=True,global_rotation_applied=False,
        note='教学输入，直接调用带追踪的 IPA 分区函数；固定扫描方向，面积和宽度合并阈值均为0。')
    (OUT/'verification.json').write_text(json.dumps(verification,ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__':run()
