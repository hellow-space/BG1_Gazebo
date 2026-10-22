# Third-party source provenance

| Directory | Source | Use |
| --- | --- | --- |
| `leader/coverage_path_planner/` | Provided by the user's leader; original ROS 1 source snapshot | Read by the ROS 2 adapter generator in `leader_coverage_ws` |
| `opennav_coverage/` | Git submodule: open-navigation/opennav_coverage, Jazzy | Built by `coverage_ws/build.sh` |
| `Fields2Cover/` | Git submodule: Fields2Cover/Fields2Cover, v1.2.1-devel | Built by `coverage_ws/build.sh` |

The two submodule commits remain pinned in the parent repository. Their
submodule section names in `.gitmodules` retain the historical identifiers;
the actual checkout paths are under `third_party/`.

```bash
git submodule update --init --recursive
```

BG1 integration, launch files and execution controls remain in their own
workspaces. Leader source files are preserved without edits; generated ROS 2
copies live only in `leader_coverage_ws/build/`. Moving source directories
requires regenerating affected CMake caches and installed links.
