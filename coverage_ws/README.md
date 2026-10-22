# BG1 coverage demo — isolated Jazzy overlay

This workspace deploys `open-navigation/opennav_coverage` on ROS 2 Jazzy
with Fields2Cover `v1.2.1-devel`. It reads the existing BG1 model and RPP
configuration from `../ros_ws`; none of those files are modified.
Exact upstream commits are in [docs/versions.json](docs/versions.json).

## Start and test

Run these commands from `/home/jimmy/Desktop/BG1_Gazebo`.
Close any existing BG1 simulation first. The startup script detects the
standard BG1 launch commands and rejects a second instance.

```bash
# Terminal 1: Gazebo + RViz + coverage server + current RPP controller.
# The robot remains stopped until an execution command is sent.
bash coverage_ws/run.sh
```

```bash
# Terminal 2: compute and display the path; DOES NOT command motion.
bash coverage_ws/task.sh

# Execute: approach the first path point, then follow the coverage path.
bash coverage_ws/task.sh --execute
```

The default region is a 6 × 4 m rectangle. RViz shows the yellow boundary
and green coverage path. Fields2Cover generates 7 swaths and approximately
35.7 m of connected path. The headland is reserved for turns; this demo
does not separately cover the reserved border or certify full cleaning
coverage. Forward-only Dubins turns can extend beyond the input polygon;
the larger empty test area is explicitly checked against BG1's footprint.

Press **Ctrl+C in the task terminal** to cancel motion. To stop the entire
coverage simulation, including Gazebo and RViz:

```bash
bash coverage_ws/stop.sh
```

Do not run keyboard teleoperation, another task client, or the original
navigation demo concurrently. If a cancellation transport failure requires
the fallback to deactivate the controller, restart the coverage demo.

Headless launch for repeatable tests:

```bash
bash coverage_ws/run.sh gui:=false headless:=true rviz:=false
```

Task output defaults to `/tmp/bg1_coverage_result.json`; select a separate
file with `--output /path/to/result.json`. It includes planning/action
status, the path, real Gazebo trajectory, travel and final position error.
Logs are under `/tmp/bg1_coverage_logs`.

## Configuration and module boundaries

- `src/bg1_coverage_demo/config/task.yaml`: polygon, empty test-area bounds,
  and timeouts. Edit this default file to keep the preview boundary aligned
  with the task; the optional CLI `--task-config` override does not change
  the persistent preview node's default boundary.
- `src/bg1_coverage_demo/config/coverage.yaml`: width, headland, swath order,
  and turning settings. Restart the demo after changing it.
- `launch/demo.launch.py`: isolated bringup; includes the existing
  `bg1_gazebo/sim.launch.py` and reads `bg1_navigation/config/nav2.yaml`.
- `scripts/coverage_task.py`: `ComputeCoveragePath` and `FollowPath` client.
- `scripts/preview.py`: retains the last path for RViz without commanding
  velocity.
- `scripts/sim_localization.py`: **simulation-only** localization using
  Gazebo poses to correct `map → odom` drift. The wheel `/odom` and model
  remain unchanged. This is not AMCL, SLAM, or a real-robot localization
  solution. Its ground-truth source is also used to record physical motion.

The 0.6 m operation width is a demonstration assumption, not the measured
cleaning-tool width. The maximum body width remains 0.76 m and footprint
comes from the existing description package. RPP retains the original
0.3 m/s limit and lidar collision checking. This demo validates a convex
region in an empty scene, not obstacle-aware coverage planning.

An initial static `map → odom` experiment completed in wheel odometry but
missed the actual endpoint by 4.63 m due to accumulated odometry error.
That failed evidence is retained in `docs/verification/initial_static_tf_failure.json`.
The separate simulation localization node was added to isolate coverage
planning/tracking from that localization limitation.

## Build and dependencies

The BG1 adapter lives in this workspace. Third-party sources live in
`../third_party/opennav_coverage` and `../third_party/Fields2Cover`; generated
build/install/log trees are local and ignored by Git.
Build only this overlay:

```bash
bash coverage_ws/build.sh
```

The build selects only `fields2cover`, `opennav_coverage_msgs`,
`opennav_coverage`, and `bg1_coverage_demo`. Upstream BT, navigator, row
coverage, and TurtleBot demo packages are left unbuilt. Fields2Cover
downloads pinned CMake FetchContent dependencies on its first configure.
The machine already supplies ROS Jazzy/Nav2, GDAL development libraries,
Eigen, GEOS, TBB, Python development/NumPy/Matplotlib and C++ build tools.
No system ROS package or underlay source patch was needed.

For a fresh source checkout, use the repository URLs, branches and commit
IDs in `docs/versions.json`; preserve the pinned Fields2Cover 1.2.1 branch
rather than substituting Fields2Cover 2.x. See the upstream
[Jazzy installation instructions](https://github.com/open-navigation/opennav_coverage/tree/jazzy).

Run the adapter's tests with the overlay loaded:

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
source coverage_ws/install/setup.bash
python3 -m pytest coverage_ws/src/bg1_coverage_demo/test -q
```

Historical integrity snapshot from the initial 2026-10-19 integration
(before later mapping/scenario work; not a current regression gate):

```bash
sha256sum --check coverage_ws/docs/ros_ws_before.sha256
```

Runtime evidence and the final trajectory plot live in `docs/verification`.
Verified on 2026-10-19: 7 swaths / 35.71 m planned, FollowPath succeeded,
actual final position error 0.188 m and yaw error 0.0366 rad. The run,
including approach and cleanup, took 163.25 s. Adapter tests: 21 source
cases / 23 colcon aggregate, all passed; original 31 source tests and
39 protected-file checks also passed.
Removing the coverage overlay after stopping its processes restores the
original project layout; the original command remains
`bash ros_ws/navigation_demo.sh` in a fresh terminal.
