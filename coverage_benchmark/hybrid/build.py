"""Generated IPA copies: expose existing BCD, add independent F/R/C Local state."""
import fcntl
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess
from coverage_benchmark.adapters.ipa_build import UPSTREAM_COMMIT,sha,runtime_fingerprints,cache_valid

PATCHES=[
 'Hybrid-only copies: disable native extra half-grid erosion; F already encodes safety.',
 'Local adds target_ flag, initializes visited from C, retains original E and neighbor/global selection.',
 'Local targets obey R while original obstacle/neighbor state obeys F; keep original room rotation.',
 'Cleanup: disk-gain targets may revisit covered centres to sweep residual pixels outside F; raw C retained in evidence.',
 'Expose protected native BCD decomposition and per-cell sweep using a subclass; unused TSP interfaces throw.',
]


def replace_once(text,old,new):
    if text.count(old)!=1:raise ValueError('Unexpected pinned IPA source for patch: '+old[:80])
    return text.replace(old,new,1)


def generated_local(text):
    text='#include "context.hpp"\n'+text
    text=replace_once(text,'room_rotation.rotateRoom(room_map, rotated_room_map, R, bbox);',
                     'room_rotation.rotateRoom(room_map, rotated_room_map, R, bbox);\n cv::invertAffineTransform(R, hybrid_inverse);')
    text=replace_once(text,'cv::erode(rotated_room_map, inflated_rotated_room_map, cv::Mat(), cv::Point(-1, -1), half_grid_spacing_as_int);',
                     'rotated_room_map.copyTo(inflated_rotated_room_map);')
    text=replace_once(text,'GridGenerator::completeCellTest(inflated_rotated_room_map, current_node.center_, grid_spacing_as_int) == true',
                     'GridGenerator::completeCellTest(inflated_rotated_room_map, current_node.center_, grid_spacing_as_int) == true && hybrid_mask(hybrid_allowed,current_node.center_)')
    text=replace_once(text,'current_node.visited_ = false;','current_node.visited_ = hybrid_mask(hybrid_covered,current_node.center_);')
    text=replace_once(text,'current_row.push_back(current_node);',
     '''current_node.target_ = !current_node.obstacle_ && hybrid_mask(hybrid_task,current_node.center_);
        if (!current_node.obstacle_) { ++hybrid_free_nodes;
          if(current_node.visited_)++hybrid_seed_nodes;
          if(current_node.target_ && !current_node.visited_)++hybrid_target_nodes;
          if(!current_node.target_ && current_node.visited_)++hybrid_outside_visited;
        }
        current_row.push_back(current_node);''')
    text=text.replace('nodes[row][column].obstacle_==false)',
                      'nodes[row][column].obstacle_==false && nodes[row][column].target_ && !nodes[row][column].visited_)')
    text=text.replace('(*neighbor)->obstacle_ == false && (*neighbor)->visited_ == false',
                      '(*neighbor)->obstacle_ == false && (*neighbor)->visited_ == false && (*neighbor)->target_')
    text=text.replace('nodes[row][col].obstacle_==false && nodes[row][col].visited_==false',
                      'nodes[row][col].obstacle_==false && nodes[row][col].visited_==false && nodes[row][col].target_')
    return text


def check_upstream(root):
    up=Path(root)/'third_party/ipa_coverage_planning'
    if subprocess.check_output(['git','-C',str(up),'rev-parse','HEAD'],text=True).strip()!=UPSTREAM_COMMIT or subprocess.check_output(['git','-C',str(up),'status','--porcelain']).strip():raise ValueError('IPA upstream must remain pinned and clean')


def verify_build(info,root):
    """Re-read original inputs, compiler, generated artifacts and current bindings."""
    try:
        check_upstream(root);identity=info['identity']
        if any(sha(Path(root)/p)!=digest for p,digest in identity['source_sha256'].items()):return False
        if sha(Path(identity['compiler']))!=identity['compiler_sha256']:return False
        return cache_valid(info,identity)
    except (OSError,ValueError,KeyError,subprocess.SubprocessError):return False


def build(root):
    root=Path(root).resolve();up=root/'third_party/ipa_coverage_planning';own=root/'coverage_benchmark/hybrid/native'
    check_upstream(root)
    room=up/'ipa_room_exploration/common';nav=up/'ipa_building_navigation/common'
    sources=[room/'src'/n for n in ('energy_functional_explorator.cpp','boustrophedon_explorator.cpp','room_rotator.cpp','meanshift2d.cpp')]
    sources += [nav/'src'/n for n in ('A_star_pathplanner.cpp','node.cpp')]
    inputs=[*sources,*sorted((room/'include').rglob('*.h')),*sorted((nav/'include').rglob('*.h')),
            *sorted(p for p in own.rglob('*') if p.is_file()),*sorted(p for p in (root/'coverage_benchmark/adapters/ipa_native/compat').rglob('*') if p.is_file()),Path(__file__).resolve()]
    compiler=Path(shutil.which('g++')).resolve();flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','opencv4'],text=True))
    identity=dict(upstream_commit=UPSTREAM_COMMIT,source_sha256={str(p.relative_to(root)):sha(p) for p in inputs},
                  compiler=str(compiler),compiler_sha256=sha(compiler),flags=flags,patches=PATCHES)
    key=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()[:20]
    directory=root/'coverage_benchmark/.build/hybrid'/key;directory.mkdir(parents=True,exist_ok=True)
    exe=directory/'hybrid_native';record=directory/'build.json'
    with (directory/'.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        if record.exists():
            info=json.loads(record.read_text())
            if cache_valid(info,identity):return exe,info
        include=directory/'include/ipa_room_exploration';include.mkdir(parents=True,exist_ok=True)
        header=(room/'include/ipa_room_exploration/energy_functional_explorator.h').read_text()
        header=replace_once(header,'bool visited_;','bool visited_;\n bool target_=false;')
        (include/'energy_functional_explorator.h').write_text(header)
        generated=[]
        for p in sources:
            text=p.read_text()
            if p.name=='energy_functional_explorator.cpp':text=generated_local(text)
            if p.name=='boustrophedon_explorator.cpp':
                text=replace_once(text,'cv::erode(room_map, inflated_room_map, cv::Mat(), cv::Point(-1, -1), half_grid_spacing_as_int+grid_obstacle_offset);','room_map.copyTo(inflated_room_map);')
            dest=directory/p.name;dest.write_text(text);generated.append(dest)
        command=[str(compiler),'-std=c++14','-O2','-include',str(own/'context.hpp'),
                 '-I'+str(directory/'include'),'-I'+str(own),'-I'+str(own/'compat'),
                 '-I'+str(root/'coverage_benchmark/adapters/ipa_native/compat'),'-I/usr/include/eigen3',
                 '-I'+str(room/'include'),'-I'+str(nav/'include'),str(own/'main.cpp'),*map(str,generated),'-o',str(exe),*flags]
        result=subprocess.run(command,capture_output=True,text=True,timeout=180)
        (directory/'compile.log').write_text(result.stdout+result.stderr)
        if result.returncode:raise RuntimeError('Hybrid native compilation failed: '+result.stderr[-14000:])
        info=dict(identity=identity,upstream_commit=UPSTREAM_COMMIT,executable=str(exe),compile_command=shlex.join(command),
                  artifacts_sha256={str(p):sha(p) for p in [exe,*generated,include/'energy_functional_explorator.h']},
                  runtime_sha256=runtime_fingerprints(exe))
        record.write_text(json.dumps(info,indent=2)+'\n');return exe,info
