"""Headless process interface; arrays use lower-left raster row order."""
import json
import math
import os
from pathlib import Path
import subprocess
import time
import numpy as np
from PIL import Image
from coverage_benchmark.hybrid.build import build
from coverage_benchmark.adapters.ipa import run_native
from coverage_benchmark.provenance import file_sha256,array_fingerprint


class NativeFailure(RuntimeError):
    def __init__(self,message,evidence,build_info):
        super().__init__(message)
        self.details=dict(reason=message,evidence=evidence,build=build_info)


def _run(mode,f,resolution,width,start,root,work,task=None,covered=None,target_policy='centres'):
    f=np.asarray(f,bool)
    if f.ndim!=2 or not f.any() or not math.isfinite(resolution) or resolution<=0 or not math.isfinite(width) or width<=0 or width/math.sqrt(2)/resolution<1:raise ValueError('Invalid native map/resolution/width')
    if np.asarray(start).shape!=(2,) or not np.isfinite(start).all():raise ValueError('Invalid native start')
    if target_policy not in ('centres','disk_gain'):raise ValueError('Unknown target policy')
    if task is not None:
        task=np.asarray(task,bool);covered=np.asarray(covered,bool)
        if task.shape!=f.shape or covered.shape!=f.shape:raise ValueError('F/R/C shape mismatch')
        if target_policy=='centres' and np.any(task&~f):raise ValueError('Centre task must be within F')
    work=Path(work).resolve();work.mkdir(parents=True,exist_ok=True);exe,info=build(root)
    config=dict(mode=mode,resolution=resolution,tool_width=width,start=list(start),target_policy=target_policy)
    masks={'allowed':f}
    if task is not None:masks.update(task=task,covered=covered)
    for name,mask in masks.items():
        p=work/(name+'.png');Image.fromarray(np.asarray(mask,dtype='uint8')*255).save(p);config[name]=str(p)
    request=work/'request.json';output=work/'output.json';log=work/'native.log';output.unlink(missing_ok=True)
    request.write_text(json.dumps(config,indent=2)+'\n');started=time.monotonic();error=None;result=None
    try:
        code=run_native([str(exe),str(request),str(output)],dict(os.environ),log,180)
        if code:raise RuntimeError('Native IPA exited '+str(code)+': '+log.read_text(errors='replace')[-2000:])
        result=json.loads(output.read_text())
    except subprocess.TimeoutExpired as exception:error='Native IPA timeout: '+str(exception.timeout)+' seconds'
    except (OSError,ValueError,RuntimeError) as exception:error=str(exception)
    finally:
        files=[request,output,log,*(work/(n+'.png') for n in masks)]
        evidence=dict(mode=mode,seconds=time.monotonic()-started,request=config,
            masks_sha256={name:array_fingerprint(mask) for name,mask in masks.items()},
            files_sha256={str(p):file_sha256(p) for p in files if p.exists()})
    if error:raise NativeFailure(error,evidence,info)
    result['build']=info;result['evidence']=evidence;return result


def decompose(f,resolution,width,start,root,work):return _run('decompose',f,resolution,width,start,root,work)


def local(f,task,covered,resolution,width,start,root,work,target_policy='centres'):
    return _run('local',f,resolution,width,start,root,work,task,covered,target_policy)
