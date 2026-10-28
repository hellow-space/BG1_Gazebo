import argparse
from pathlib import Path
from coverage_benchmark.runner import run

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description='Leader / BCD / Spiral-STC planning-only benchmark')
parser.add_argument('--runtime-root',type=Path,default=root)
parser.add_argument('--output',type=Path,default=root/'docs/benchmarks/three-planners-2026-10-27')
parser.add_argument('--config',type=Path,default=root/'coverage_benchmark/configs/three_maps.yaml')
parser.add_argument('--scene',action='append')
parser.add_argument('--algorithm',action='append',choices=['leader','bcd','stc'])
args = parser.parse_args()
run(root,args.runtime_root,args.output,args.config,args.scene,args.algorithm)
