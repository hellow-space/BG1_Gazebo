# Third-party source provenance

| Directory | Source | Use |
| --- | --- | --- |
| `leader/coverage_path_planner/` | Provided by the user's leader; original ROS 1 source snapshot | Read by the ROS 2 adapter generator in `leader_coverage_ws` |
| `opennav_coverage/` | Git submodule: open-navigation/opennav_coverage, Jazzy | Built by `coverage_ws/build.sh` |
| `Fields2Cover/` | Git submodule: Fields2Cover/Fields2Cover, v1.2.1-devel | Built by `coverage_ws/build.sh` |
| `python_robotics/` | Git submodule: AtsushiSakai/PythonRobotics | Native Spiral-STC offline benchmark via `coverage_benchmark/adapters/stc.py` |
| `polygon_coverage_planning/` | Git submodule: ethz-asl/polygon_coverage_planning | Native BCD, visibility graph and GK-MA via `coverage_benchmark/adapters/ethz.py` |
| `ipa_coverage_planning/` | Git submodule: ipa320/ipa_coverage_planning, pinned noetic_dev commit | Native Grid Local Energy (algorithm 6) via `coverage_benchmark/adapters/ipa.py`; separate safe A* connections |

Richey `bcd_planner/` was removed at the user's request. Its saved benchmark results remain historical evidence; the old adapter/build scripts were also removed; the current benchmark uses `coverage_benchmark/adapters/ipa_bcd.py` instead. IPA upstream algorithms remain intact; project variants `bcd_local` and `hybrid_local` were removed from the offline entry points.

All five submodule commits remain pinned in the parent repository. The
opennav/Fields2Cover section names in `.gitmodules` retain historical identifiers;
the actual checkout paths are under `third_party/`.

```bash
git submodule update --init --recursive
```

BG1 integration, launch files and execution controls remain in their own
workspaces. Leader source files are preserved without edits; generated ROS 2
copies live only in `leader_coverage_ws/build/`. Moving source directories
requires regenerating affected CMake caches and installed links.

Offline benchmark adapters and generated compatibility copies are maintained
separately in `coverage_benchmark/`; upstream files remain unchanged. See the
[module instructions](../coverage_benchmark/README.md) and
[five-planner report](../docs/benchmarks/five-planners-2026-10-29/README.md).
Original licenses remain with each dependency; GK-MA additionally restricts
use to non-commercial purposes as documented by the ETHZ repository.

IPA package metadata states academic/non-commercial licensing and directs commercial users to Fraunhofer IPA; original source headers and declarations are retained. The [IPA design](../docs/benchmarks/ipa-local-energy-design.md) and [six-planner report](../docs/benchmarks/six-planners-2026-10-30/README.md) describe the unchanged upstream core and our safety layer.
