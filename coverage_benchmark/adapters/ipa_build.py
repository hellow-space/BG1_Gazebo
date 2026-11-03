"""Compile pinned IPA Local Energy footprint core with explicit ROS-only shims."""
import fcntl
import hashlib
import json
from pathlib import Path
import re
import shlex
import shutil
import subprocess

UPSTREAM_COMMIT='986c18384ed884dadd3bc857cd0c47c13b7d4716'
RELATIVE='ipa_room_exploration/common/'
UPSTREAM_FILES=tuple(RELATIVE+x for x in ('src/energy_functional_explorator.cpp','src/room_rotator.cpp',
    'include/ipa_room_exploration/energy_functional_explorator.h','include/ipa_room_exploration/room_rotator.h',
    'include/ipa_room_exploration/grid.h','include/ipa_room_exploration/histogram.h',
    'include/ipa_room_exploration/meanshift2d.h'))


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def runtime_fingerprints(executable):
    output=subprocess.check_output(['ldd',str(executable)],text=True)
    if 'not found' in output:raise RuntimeError('Native runtime dependency is missing')
    libraries=set(re.findall(r'(?:=>\s+)?(/[^\s()]+)',output))
    if not libraries:raise RuntimeError('No runtime dependency bindings resolved')
    return {str(Path(p).resolve()):sha(Path(p).resolve()) for p in sorted(libraries)}


def cache_valid(info,identity):
    if info.get('identity')!=identity:return False
    try:
        hashes_match=all(sha(Path(p))==digest for group in ('artifacts_sha256','runtime_sha256')
                         for p,digest in info[group].items())
        if not hashes_match:return False
        if 'upstream_commit' in identity:
            return runtime_fingerprints(info['executable'])==info['runtime_sha256']
        return True
    except (OSError,KeyError,RuntimeError,subprocess.SubprocessError):return False


def build(root):
    root=Path(root).resolve();up=root/'third_party/ipa_coverage_planning'
    commit=subprocess.check_output(['git','-C',str(up),'rev-parse','HEAD'],text=True).strip()
    if commit!=UPSTREAM_COMMIT or subprocess.check_output(['git','-C',str(up),'status','--porcelain']).strip():
        raise ValueError('IPA upstream differs from pinned clean commit')
    own=root/'coverage_benchmark/adapters/ipa_native'
    files=[*(up/p for p in UPSTREAM_FILES),*sorted(p for p in own.rglob('*') if p.is_file()),Path(__file__).resolve()]
    compiler=Path(shutil.which('g++')).resolve()
    flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','opencv4'],text=True))
    identity=dict(upstream_commit=commit,source_sha256={str(p.relative_to(root)):sha(p) for p in files},
                  compiler=str(compiler),compiler_sha256=sha(compiler),opencv_flags=flags,
                  mode='native footprint; unchanged scoring/rotation/half-grid erosion; ROS messages/log/FOV shims only')
    key=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()[:20]
    directory=root/'coverage_benchmark/.build/ipa'/key;directory.mkdir(parents=True,exist_ok=True)
    executable=directory/'ipa_headless';infofile=directory/'build.json'
    with (directory/'.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        if infofile.exists():
            info=json.loads(infofile.read_text())
            if cache_valid(info,identity):return executable,info
        command=[str(compiler),'-std=c++14','-O2','-I/usr/include/eigen3','-I'+str(own/'compat'),
                 '-I'+str(up/(RELATIVE+'include')),'-include','limits',str(own/'main.cpp'),
                 str(up/UPSTREAM_FILES[0]),str(up/UPSTREAM_FILES[1]),'-o',str(executable),*flags]
        result=subprocess.run(command,capture_output=True,text=True,timeout=180)
        (directory/'compile.log').write_text(result.stdout+result.stderr)
        if result.returncode:raise RuntimeError('IPA compile failed: '+result.stderr[-12000:])
        info=dict(identity=identity,upstream_commit=commit,compile_command=shlex.join(command),executable=str(executable),
                  artifacts_sha256={str(executable):sha(executable)},
                  runtime_sha256=runtime_fingerprints(executable))
        infofile.write_text(json.dumps(info,indent=2)+'\n')
        return executable,info


if __name__=='__main__':
    executable,info=build(Path(__file__).resolve().parents[2]);print(executable);print(info['upstream_commit'])
