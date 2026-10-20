# BG1 Gazebo navigation and coverage

Lightweight BG1 Pro simulation for ROS 2 Jazzy and Gazebo Harmonic.
The project includes a differential-drive model, Nav2/RPP point-to-point
navigation, and an independent Fields2Cover-based coverage demonstration.

- [Navigation workspace and build instructions](ros_ws/README.md)
- [Coverage overlay and usage](coverage_ws/README.md)
- [Coverage planning/execution evidence](coverage_ws/docs/verification/summary.json)

## Clone

Clone with `git clone --recurse-submodules <repository-url>`, or initialize
the pinned third-party dependencies after cloning:

```bash
git submodule update --init --recursive
```

The submodules pin `opennav_coverage`'s Jazzy branch and Fields2Cover's
`v1.2.1-devel` branch. Build the underlay first using `ros_ws/README.md`,
then build the coverage overlay:

```bash
bash coverage_ws/build.sh
```

## Run

From this repository's root, choose one simulation at a time:

```bash
# Point-to-point Nav2 navigation
bash ros_ws/navigation_demo.sh

# Independent coverage demo
bash coverage_ws/run.sh
```

In another terminal, request coverage planning or motion:

```bash
bash coverage_ws/task.sh             # Plan and visualize only
bash coverage_ws/task.sh --execute   # Approach and follow the coverage path
bash coverage_ws/stop.sh             # Stop coverage Gazebo/RViz and nodes
```

The coverage demo uses simulator ground-truth localization to isolate
coverage planning and path tracking from wheel-odometry drift. It does
not validate real-robot localization or obstacle-aware cleaning coverage.
Build/install/log trees are generated locally and excluded from Git.
