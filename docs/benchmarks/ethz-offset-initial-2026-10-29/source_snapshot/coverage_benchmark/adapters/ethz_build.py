"""Build the pinned ETHZ core and original patched GK-MA, without ROS 1."""
from pathlib import Path
import fcntl
import hashlib
import json
import os
import re
import subprocess
import urllib.request
import zipfile

GK_URL='https://www.cs.nott.ac.uk/~pszdk/gtsp_ma_source_codes.zip'
GK_MD5='765fad8e3746fa3dd9b81be0afb34d35'  # Upstream ExternalProject checksum.
PACKAGES=('libcgal-dev','libgmp-dev','libmpfr-dev','libgmpxx4ldbl',
          'libmono-2.0-dev','libmonosgen-2.0-dev','libmonosgen-2.0-1','mono-runtime-sgen',
          'mono-runtime-common','mono-mcs','libmono-corlib4.5-dll',
          'libmono-corlib4.5-cil','libmono-system-core4.0-cil',
          'libmono-system-xml4.0-cil','libmono-system4.0-cil',
          'libmono-microsoft-csharp4.0-cil','libmono-security4.0-cil',
          'libmono-system-security4.0-cil','libmono-system-drawing4.0-cil')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def environment(base):
    base=Path(base).resolve();usr=base/'deps/usr'
    return dict(os.environ,ETHZ_BUILD_ROOT=str(base),
                MONO_PATH=str(usr/'lib/mono/4.5'),MONO_CFG_DIR=str(base/'deps/etc'),
                LD_LIBRARY_PATH=':'.join(map(str,(base/'devel/lib',usr/'lib',usr/'lib/x86_64-linux-gnu'))),
                MONO_THREADS_SUSPEND='preemptive',MONO_GC_PARAMS='max-heap-size=512m')


def prepare_dependencies(base):
    """Use signed apt repository metadata; extract .deb files locally, no sudo."""
    base=Path(base).resolve();debs=base/'debs';deps=base/'deps'
    debs.mkdir(parents=True,exist_ok=True);deps.mkdir(exist_ok=True)
    manifest=base/'dependencies.json'
    if manifest.exists():
        data=json.loads(manifest.read_text())
        if data.get('packages')==list(PACKAGES) and all((debs/name).exists() and sha(debs/name)==digest for name,digest in data['deb_sha256'].items()):
            return data
    if not all(any(debs.glob(name+'_*.deb')) for name in PACKAGES):
        subprocess.run(['apt-get','download',*PACKAGES],cwd=debs,check=True)
    metadata={}
    for package in sorted(debs.glob('*.deb')):
        subprocess.run(['dpkg-deb','-x',str(package),str(deps)],check=True)
        metadata[package.name]=sha(package)
    data=dict(packages=list(PACKAGES),deb_sha256=metadata,repository='Ubuntu Noble apt-configured repositories',installation='local extraction only')
    manifest.write_text(json.dumps(data,indent=2)+'\n')
    return data


def build(root):
    root=Path(root).resolve();up=root/'third_party/polygon_coverage_planning'
    base=root/'coverage_benchmark/.build/ethz';base.mkdir(parents=True,exist_ok=True)
    with (base/'build.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        if subprocess.check_output(['git','-C',str(up),'status','--porcelain']).strip():
            raise ValueError('ETHZ upstream contains uncommitted changes')
        commit=subprocess.check_output(['git','-C',str(up),'rev-parse','HEAD'],text=True).strip()
        own=Path(__file__).parent/'ethz_native'
        source_hashes={str(p.relative_to(root)):sha(p) for p in sorted(own.rglob('*')) if p.is_file()}
        source_hashes[str(Path(__file__).relative_to(root))]=sha(__file__)
        identity=dict(upstream_commit=commit,build_source_sha256=source_hashes)
        executable=base/'cmake/ethz_planner';stamp=base/'build.json'
        if stamp.exists() and executable.exists():
            info=json.loads(stamp.read_text())
            if info['identity']==identity and all(sha(base/name)==digest for name,digest in info['artifacts_sha256'].items()):
                return executable,info
        dependency_info=prepare_dependencies(base)
        archive=base/'gtsp_ma_source_codes.zip'
        if not archive.exists():
            with urllib.request.urlopen(GK_URL,timeout=60) as response:archive.write_bytes(response.read())
        if hashlib.md5(archive.read_bytes()).hexdigest()!=GK_MD5:
            raise ValueError('GK-MA archive differs from upstream checksum')
        generated=base/'gk_ma'
        if not (generated/'.patched').exists():
            generated.mkdir(exist_ok=True)
            with zipfile.ZipFile(archive) as z:
                for name in z.namelist():
                    if not (generated/name).resolve().is_relative_to(generated.resolve()):
                        raise ValueError('Unsafe GK-MA archive member')
                z.extractall(generated)
            patches=up/'polygon_coverage_solvers'
            text=(patches/'CMakeLists.txt').read_text()
            with (base/'patch.log').open('w') as log:
                for target,patch in re.findall(r'COMMAND patch (\S+) \$\{CMAKE_CURRENT_SOURCE_DIR\}/patches/(\S+)',text):
                    subprocess.run(['patch','--batch',target,str(patches/'patches'/patch)],cwd=generated,
                                   stdout=log,stderr=subprocess.STDOUT,check=True)
            (generated/'.patched').touch()
        lib=base/'devel/lib';lib.mkdir(parents=True,exist_ok=True)
        env=environment(base);usr=base/'deps/usr'
        with (base/'build.log').open('w') as log:
            def run(command,cwd=None):
                subprocess.run(list(map(str,command)),cwd=cwd,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
            run([usr/'bin/mono-sgen',usr/'lib/mono/4.5/mcs.exe','-unsafe','-platform:anycpu',
                 '-r:System.Drawing','-out:'+str(lib/'GkMa.exe'),'-recurse:'+str(generated/'GkMa/*.cs')])
            cpp=[generated/'NativeHelper'/name for name in ('ClusterOptimisation.cpp','ImprovementManager.cpp','Insert.cpp','KOpt.cpp','NativeHelper.cpp','Swap.cpp')]
            run(['g++','-shared','-fPIC','-O1',*cpp,'-o',lib/'libnative_helper.so'])
            run(['cmake','-S',own,'-B',base/'cmake','-DCMAKE_BUILD_TYPE=Release',
                 '-DCMAKE_CXX_COMPILER=clang++','-DCMAKE_PREFIX_PATH='+str(usr),
                 '-DCMAKE_INCLUDE_PATH='+str(usr/'include/x86_64-linux-gnu')])
            run(['cmake','--build',base/'cmake','-j1'])
        artifacts={str(p.relative_to(base)):sha(p) for p in (executable,lib/'GkMa.exe',lib/'libnative_helper.so')}
        info=dict(identity=identity,dependencies=dependency_info,artifacts_sha256=artifacts,
                  runtime_sha256={str(p.relative_to(base)):sha(p) for p in (usr/'lib/libmonosgen-2.0.so.1.0.0',usr/'lib/libmono-native.so.0.0.0',usr/'lib/mono/4.5/mscorlib.dll')},
                  gk_ma=dict(url=GK_URL,upstream_md5=GK_MD5,sha256=sha(archive),
                             patches='unchanged upstream polygon_coverage_solvers/patches'),
                  cgal='5.6 (Ubuntu Noble)',compiler=subprocess.check_output(['clang++','--version'],text=True).splitlines()[0])
        stamp.write_text(json.dumps(info,indent=2)+'\n')
        return executable,info


if __name__=='__main__':
    print(build(Path(__file__).resolve().parents[2])[0])
