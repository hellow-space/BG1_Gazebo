from pathlib import Path
import importlib
import numpy as np


def module():
    assert (Path(__file__).parents[1]/'provenance.py').exists(), 'Provenance module missing'
    return importlib.import_module('coverage_benchmark.provenance')


def test_array_fingerprint_includes_shape_and_preserves_boolean_semantics():
    p=module()
    values=np.array([[True,False],[False,True]])
    assert p.array_fingerprint(values)==p.array_fingerprint(values.copy())
    assert p.array_fingerprint(values)!=p.array_fingerprint(values.reshape(1,4))
    changed=values.copy();changed[0,0]=False
    assert p.array_fingerprint(values)!=p.array_fingerprint(changed)


def test_file_fingerprint_changes_with_source_content(tmp_path):
    p=module();source=tmp_path/'source.py';source.write_text('version one\n')
    first=p.file_sha256(source)
    source.write_text('version two\n')
    assert first!=p.file_sha256(source)
