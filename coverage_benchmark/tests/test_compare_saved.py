import importlib
from pathlib import Path
import copy
import json
import pytest

ROOT=Path(__file__).resolve().parents[2]


def module():
    assert (ROOT/'coverage_benchmark/compare_saved.py').exists(),'Saved comparison module missing'
    return importlib.import_module('coverage_benchmark.compare_saved')


def test_aggregate_selects_safe_f2c_and_preserves_all_failures():
    scenes,records,sources=module().load_sources(ROOT)
    assert len(scenes)==3 and len(records)==12 and len(sources)==14
    for scene in scenes:
        group=[record for record in records if record['scene']==scene]
        assert [record['algorithm'] for record in group]==['leader','bcd','stc','f2c_safe']
    assert sum(record['metrics']['path_length_m'] is None for record in records)==4
    assert not any(record['algorithm']=='f2c_native' for record in records)
    assert all('path' in record for record in records)


def test_incompatible_image_or_planning_parameters_rejected():
    m=module()
    first=json.loads((ROOT/'docs/benchmarks/three-planners-2026-10-27/manifest.json').read_text())
    second=json.loads((ROOT/'docs/benchmarks/f2c-obstacle-2026-10-28/manifest.json').read_text())
    m.validate_manifests(first,second)
    changed=copy.deepcopy(second);changed['parameters']['clearance']=.5
    with pytest.raises(ValueError,match='parameter'):m.validate_manifests(first,changed)
    changed=copy.deepcopy(second);scene=next(iter(changed['maps']));changed['maps'][scene]['source_sha256']='wrong'
    with pytest.raises(ValueError,match='map'):m.validate_manifests(first,changed)
