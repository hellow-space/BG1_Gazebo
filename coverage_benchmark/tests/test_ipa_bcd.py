"""Current BCD entry runs IPA and works without the retired Richey dependency."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from PIL import Image
import yaml

ROOT=Path(__file__).resolve().parents[2]


def test_benchmark_runs_and_verifies_real_ipa_bcd_without_richey_or_leader(tmp_path):
    from coverage_benchmark.runner import run
    from coverage_benchmark.verify import verify
    assert importlib.util.find_spec('coverage_benchmark.adapters.ipa_bcd'), 'IPA BCD adapter missing'
    scene='room';folder=tmp_path/'data/maps/imported'/scene;folder.mkdir(parents=True)
    f=np.zeros((70,130),bool);f[5:65,5:125]=True;f[25:50,45:65]=False
    Image.fromarray(np.flipud(f).astype('uint8')*255).save(folder/(scene+'.png'))
    (folder/(scene+'.pgm')).write_bytes(b'P5\n# 0.05 m/pixel companion metadata\n130 70\n255\n'+np.flipud(f).astype('uint8').tobytes())
    config=tmp_path/'config.yaml';config.write_text(yaml.safe_dump(dict(resolution=.1,tool_width=.6,clearance=0.,scenes=[scene])))
    out=tmp_path/'out';rows=run(ROOT,tmp_path,out,config,algorithm_filter=['ipa_bcd'])
    assert len(rows)==1 and rows[0]['algorithm']=='ipa_bcd' and rows[0]['path_valid']
    record=json.loads((out/'room_ipa_bcd.json').read_text())
    assert record['planner_success'] and record['details']['kernel_calls']['bcd']>0
    assert record['details']['kernel_calls']['local']==0
    assert record['details']['decomposition']['build']['upstream_commit']=='986c18384ed884dadd3bc857cd0c47c13b7d4716'
    assert not (out/'room_bcd.json').exists()
    assert verify(ROOT,tmp_path,out)['passed']


def test_cli_offers_ipa_bcd_and_rejects_retired_bcd():
    help_result=subprocess.run([sys.executable,'-m','coverage_benchmark','--help'],cwd=ROOT,capture_output=True,text=True)
    assert help_result.returncode==0 and 'ipa_bcd' in help_result.stdout
    result=subprocess.run([sys.executable,'-m','coverage_benchmark','--algorithm','bcd'],cwd=ROOT,capture_output=True,text=True)
    assert result.returncode==2 and 'invalid choice' in result.stderr
