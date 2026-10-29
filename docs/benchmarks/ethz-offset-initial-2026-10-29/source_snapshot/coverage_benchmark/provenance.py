"""Content fingerprints for maps, native dependencies and custom planning stages."""
import hashlib
import json
import subprocess
import numpy as np

SOURCE_FILES=(
    'coverage_benchmark/models.py','coverage_benchmark/map_io.py','coverage_benchmark/metrics.py',
    'coverage_benchmark/raster_geometry.py','coverage_benchmark/swath_geometry.py',
    'coverage_benchmark/grid_connector.py','coverage_benchmark/obstacle_connector.py',
    'coverage_benchmark/adapters/f2c_safe.py','coverage_benchmark/adapters/f2c_worker.py',
    'coverage_benchmark/f2c_compare.py','coverage_benchmark/runner.py',
    'coverage_benchmark/provenance.py','coverage_benchmark/configs/three_maps.yaml',
)


def file_sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def array_fingerprint(values):
    values=np.ascontiguousarray(values)
    header=json.dumps(dict(shape=list(values.shape),dtype=values.dtype.str),sort_keys=True).encode()
    return hashlib.sha256(header+b'\0'+values.tobytes()).hexdigest()


def implementation_fingerprints(root):
    return {name:file_sha256(root/name) for name in SOURCE_FILES}


def native_fingerprints(runtime):
    result={}
    for name in ('Fields2Cover','opennav_coverage'):
        directory=runtime/'third_party'/name
        commit=subprocess.check_output(['git','-C',str(directory),'rev-parse','HEAD'],text=True).strip()
        if subprocess.check_output(['git','-C',str(directory),'status','--porcelain'],text=True).strip():
            raise ValueError(f'{name} upstream contains uncommitted modifications')
        result[name+'_commit']=commit
    for relative in ('coverage_ws/build/opennav_coverage/opennav_coverage',
                     'coverage_ws/install/fields2cover/lib/libFields2Cover.so',
                     'coverage_ws/src/bg1_coverage_demo/config/coverage.yaml'):
        result[relative]=file_sha256(runtime/relative)
    return result
