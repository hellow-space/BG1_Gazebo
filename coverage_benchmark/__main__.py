import argparse
from pathlib import Path
from coverage_benchmark.runner import run, ALGORITHMS, DEFAULT_OUTPUT

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description='Leader / IPA BCD / Spiral-STC planning-only benchmark')
parser.add_argument('--runtime-root',type=Path,default=root)
parser.add_argument('--output',type=Path,default=root/DEFAULT_OUTPUT)
parser.add_argument('--config',type=Path,default=root/'coverage_benchmark/configs/three_maps.yaml')
parser.add_argument('--scene',action='append')
parser.add_argument('--algorithm',action='append',choices=ALGORITHMS)
args = parser.parse_args()
run(root,args.runtime_root,args.output,args.config,args.scene,args.algorithm)
