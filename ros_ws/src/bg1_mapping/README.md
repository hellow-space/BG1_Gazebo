# BG1 laser mapping

Standalone ROS 2 Jazzy SLAM Toolbox online asynchronous mapping, using the
existing BG1 lidar/odometry and Gazebo obstacle world. No AMCL, static map
server, navigation controller or simulator ground-truth localization is
started. SLAM Toolbox is the sole `map -> odom` TF source.

Data flow: `/scan` + `odom -> base_footprint -> nav_lidar_link` transforms
feed laser scan matching, pose-graph optimization and loop closure.
Outputs are `/map` (5 cm occupancy grid) and `map -> odom` localization.
No IMU is required for this minimal planar simulation. The previous
wheel-odometry drift makes real scan-based correction particularly relevant.

Build using the workspace's colcon command, then from the repository root:

```bash
bash ros_ws/mapping.sh
```

This opens Gazebo, RViz and a terminal titled **BG1 Mapping Keyboard**.
Focus the keyboard terminal. `i` forward, `,` reverse, `j` left turn,
`l` right turn, `k` stop, `Ctrl+C` stop and exit teleoperation. Releasing a
key does not stop motion. Initial speeds are 0.15 m/s and 0.30 rad/s.

Drive slowly around the perimeter and view both sides of internal obstacles.
Return near the starting area to give loop closure a chance to align the map.
An initial map at rest contains only the first scan and is not a completed map.
Laser range and angle resolution, scan matching and wheel slip can affect
the final map; inspect wall alignment after the loop.

Stop the robot with `k`, then save in another terminal:

```bash
# Automatic timestamped session directory and prefix under data/maps/slam/
bash ros_ws/save_map.sh

# Or select a fresh prefix without a file extension
bash ros_ws/save_map.sh data/maps/slam/my_room/my_room
```

The helper saves `.pgm` + `.yaml` for Nav2/AMCL and `.posegraph` + `.data`
for resuming SLAM. It refuses existing output prefixes. These files serve
different purposes; a serialized pose graph is not an occupancy-map YAML.
Original navigation still loads its original demo map until explicitly
configured to use the newly saved YAML.

Stop the complete mapping session, including its keyboard node:

```bash
bash ros_ws/stop_mapping.sh
```

Without a GUI/keyboard terminal:

```bash
bash ros_ws/mapping.sh gui:=false headless:=true rviz:=false teleop:=false
```

Do not run another BG1 simulation, AMCL or the coverage demo simultaneously.
Start guards detect standard BG1 launches. Other launch wrappers should
likewise be used only after the current session is stopped.

Configuration is `config/slam.yaml`, based on the installed Jazzy async
defaults with 0.1 m/0.1 rad scan-keyframe thresholds, 2 s map updates and
20 m maximum range. Model, bridge and existing Nav2 parameters are unchanged.

2026-10-20 startup verification: SLAM active, 360 finite scan points in
nav_lidar_link, nonempty map in map frame, live map->odom transforms,
mapping_keyboard discovered, no AMCL/coverage localization nodes present.
Map saver and graph serialization both exercised to /tmp only; user has
not yet completed a manual mapping route at this verification checkpoint.

Upstream: https://github.com/SteveMacenski/slam_toolbox/tree/jazzy

## Larger worlds

Use `bash ros_ws/mapping.sh scene:=warehouse` (36 × 28 m) or
`bash ros_ws/mapping.sh scene:=office` (32 × 26 m). These select only the
Gazebo world and wider RViz view; SLAM starts without a reference map.
Default remains `scene:=demo`. See [scene details](../bg1_scenarios/README.md).
