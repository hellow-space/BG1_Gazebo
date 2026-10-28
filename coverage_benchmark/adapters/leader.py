"""Invoke the existing Leader service in its own overlay and ROS domain."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import numpy as np
from coverage_benchmark.models import PlanningResult


def plan(request, work_dir):
    work_dir = Path(work_dir).resolve()
    work_dir.mkdir(parents=True, exist_ok=True)
    executable = request.runtime_root/'leader_coverage_ws/build/bg1_leader_coverage/planner_server'
    try:
        config = dict(executable=str(executable), resolution=request.resolution,
                      start=request.start, tool_width=request.tool_width, clearance=request.clearance)
        (work_dir/'leader_input.json').write_text(json.dumps(config)+'\n')
        np.savez_compressed(work_dir/'leader_input.npz', target=request.free)
        worker = Path(__file__).with_name('leader_worker.py')
        env = dict(os.environ, ROS_DOMAIN_ID='101', ROS_LOG_DIR=str(work_dir/'ros_logs'))
        command = ['bash','--noprofile','--norc','-c',
                   'source "$1"; source "$2"; source "$3"; exec python3 "$4" "$5"',
                   'leader-benchmark', '/opt/ros/jazzy/setup.bash',
                   str(request.runtime_root/'ros_ws/install/setup.bash'),
                   str(request.runtime_root/'leader_coverage_ws/install/setup.bash'), str(worker), str(work_dir)]
        with (work_dir/'leader_worker.log').open('w') as log:
            process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                process.wait(timeout=180)
            except BaseException:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                raise
        if process.returncode:
            raise RuntimeError(f'Leader worker exited {process.returncode}; see {work_dir}/leader_worker.log')
        data = json.loads((work_dir/'leader_output.json').read_text())
        data['details']['executable_sha256'] = hashlib.sha256(executable.read_bytes()).hexdigest()
        data['details']['ros_domain_id'] = 101
        return PlanningResult(**data)
    except Exception as error:
        return PlanningResult(message=f'{type(error).__name__}: {error}')
