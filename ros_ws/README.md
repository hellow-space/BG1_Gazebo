# BG1 Pro Gazebo Navigation Model

This workspace provides a lightweight PUDU BG1 Pro model for ROS 2 Jazzy and Gazebo Harmonic. It is intended for navigation integration, footprint checks, topic wiring, and simple motion tests. The model uses two rear drive wheels, one front caster, a box collision shell, a rear squeegee envelope, and an equivalent planar lidar.

The former ROS 1 reference project and its ZIP archive have been removed after checking that the BG1 packages are self-contained. Runtime dependencies are supplied by the system ROS 2 installation.

Project layout:

- `src/bg1_description`: model, geometry, materials, and model tests.
- `src/bg1_gazebo`: launch files, worlds, bridge, and runtime checks.
- `src/bg1_navigation`: Nav2 parameters, matching demo world/map, RViz, and goal verification.
- `src/bg1_mapping`: standalone SLAM Toolbox mapping and keyboard teleoperation.
- `src/bg1_scenarios`: larger warehouse/office worlds, matching reference maps and scene selection.
- `docs`: design history, preview images, and verification records.
- `keyboard_control.sh`: keyboard teleoperation entry point.
- `build` and `install`: local generated files created by the build command below; excluded from Git and absent from a fresh clone. Regenerate both together when needed.

The user manual is stored in `../docs/manuals/`. Historical design documents describe the original workspace layout; their references to the removed ROS 1 tree are provenance, not dependencies.

## Model scope

- Manual body envelope without squeegee: `1.120 x 0.660 x 1.303 m`.
- Manual maximum envelope with squeegee: `1.195 x 0.760 x 1.303 m`.
- Empty model mass: `348.5 kg`.
- Command limit: `1.0 m/s` linear and `0.8 rad/s` angular.
- Navigation interfaces: `/cmd_vel`, `/odom`, `/scan`, `/joint_states`, `/tf`, `/tf_static`, and `/clock`.
- `primitive` appearance uses boxes and cylinders. `wrapped` uses a 1172-triangle rounded Collada shell, layered tank cover, dark chassis, sensor-panel cues, and small side decals. Both modes share the same physical model.

Wheel track, wheel size, caster placement, inertia distribution, friction, torque, lidar pose, and angular-speed limit are engineering baselines because the user manual does not publish them. Edit them in `bg1_description/config/bg1_geometry.yaml` and `bg1_description/config/bg1_physics.yaml`. The maximum Nav2 footprint is in `bg1_description/config/nav2_footprint.yaml`.

## Build

From the `BG1_Gazebo` directory:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base ros_ws/log build \
  --base-paths ros_ws/src \
  --build-base ros_ws/build \
  --install-base ros_ws/install \
  --symlink-install
source ros_ws/install/setup.bash
```

## Run

Use a writable ROS log directory in this environment:

```bash
mkdir -p /tmp/bg1_ros_logs
export ROS_LOG_DIR=/tmp/bg1_ros_logs
```

Launch the wrapped model with the GUI:

```bash
ros2 launch bg1_gazebo sim.launch.py appearance:=wrapped gui:=true
```

Launch headless:

```bash
ros2 launch bg1_gazebo sim.launch.py appearance:=wrapped gui:=false headless:=true
```

Select `appearance:=primitive` to remove the visual mesh and texture without changing physics or sensor behavior.

Send a short direct velocity command from another sourced terminal:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

## Verify the runtime

### Nav2 navigation demo

Close any earlier simulation and keyboard controller, then from `BG1_Gazebo`:

```bash
bash ros_ws/navigation_demo.sh
```

This starts Gazebo, RViz, a static map, AMCL localization, NavFn planning,
Regulated Pure Pursuit control, and velocity smoothing. The robot starts at
`(0, 0, 0)` with an initialized AMCL pose. Navigation speed is capped at
`0.3 m/s`. The rectangular costmap footprint includes the rear squeegee.
The TF chain is `map -> odom -> base_footprint -> base_link -> nav_lidar_link`.

Use RViz's **Nav2 Goal** tool: click a free location and drag to set the
desired heading. Keep enough space for the full robot to turn. To test a
fixed goal on the other side of the central obstacle, in a sourced terminal:

```bash
export ROS_LOG_DIR=/tmp/bg1_ros_logs
ros2 run bg1_navigation demo_goal.py --x 4.8 --y 0 --yaw 0
```

The script checks valid laser returns, sends a NavigateToPose action, and
compares the result against independent Gazebo world poses. It writes
`/tmp/bg1_nav2_result.json`. Do not send other goals or keyboard commands
during that check. Headless launch:

```bash
bash ros_ws/navigation_demo.sh gui:=false headless:=true rviz:=false
```

Verified on 2026-10-16: outbound and return actions both succeeded in
28.6 s / 33.5 s, with Gazebo position errors of 0.161 m / 0.163 m.
See `docs/previews/bg1_nav2_trajectory.png` and
`docs/verification/nav2_summary.json`. These results cover this static
scene and the two tested goals; tight passages and moving obstacles have
not been validated.

The demo uses a known synthetic map, not online SLAM. To change the scene,
edit `src/bg1_navigation/config/demo_scene.yaml` and regenerate both world
and occupancy map with:

```bash
python3 ros_ws/src/bg1_navigation/scripts/build_demo_scene.py \
  --package ros_ws/src/bg1_navigation \
  --base-world ros_ws/src/bg1_gazebo/worlds/bg1_flat.sdf
```

### Keyboard control

With Gazebo running, open a separate terminal from `BG1_Gazebo` and run:

```bash
bash ros_ws/keyboard_control.sh
```

Focus that terminal (not the Gazebo window). Keys: `i` forward, `,` reverse,
`j` left turn, `l` right turn, `k` stop, `Ctrl+C` stop and exit.
Releasing a key does **not** stop the robot; press `k` explicitly.
Initial speeds are `0.2 m/s` and `0.4 rad/s`.

Navigation mode lifts the simplified rigid squeegee by `20 mm` so it does
not drag on the floor. Wheel collision names are deliberately omitted to
preserve contact parameters through the installed SDFormat converter.
The passive caster uses a provisional friction coefficient of `0.05`.
Ground-truth keyboard demonstration evidence is saved in
`docs/verification/keyboard_motion.json`.

### Automated interface check

With the headless simulation running, execute:

```bash
export ROS_LOG_DIR=/tmp/bg1_ros_logs
ros2 run bg1_gazebo verify_runtime.py
```

The verifier checks message freshness, a two-second stationary window, forward travel, positive rotation, stopping, and the `1.0 m/s` speed ceiling. It writes evidence to `/tmp/bg1_gazebo_verification/runtime_evidence.json` and publishes zero velocity before exit.

Run static and package tests with:

```bash
python3 -m pytest ros_ws/src/bg1_description/test ros_ws/src/bg1_gazebo/test ros_ws/src/bg1_navigation/test -q
colcon --log-base ros_ws/log test \
  --base-paths ros_ws/src \
  --build-base ros_ws/build \
  --install-base ros_ws/install \
  --event-handlers console_direct+
colcon test-result --test-result-base ros_ws/build --verbose
```

## Threshold scene

`bg1_thresholds.sdf` contains separate `20 mm` and `25 mm` bars at `y=-1 m` and `y=+1 m`. Launch it with the matching world name and choose a lane through `spawn_y`:

```bash
ros2 launch bg1_gazebo sim.launch.py \
  world:=$(ros2 pkg prefix bg1_gazebo)/share/bg1_gazebo/worlds/bg1_thresholds.sdf \
  world_name:=bg1_thresholds spawn_y:=-1.0
```

This scene checks only the simplified simulation geometry and contact settings. It does not certify obstacle-crossing performance of the real machine.

## Regenerate the visual wrap

After changing authoritative geometry or the label generator:

```bash
python3 ros_ws/src/bg1_description/tools/build_visual_assets.py \
  --geometry ros_ws/src/bg1_description/config/bg1_geometry.yaml \
  --output-root ros_ws/src/bg1_description
```

The generated `meshes/manifest.json` records the geometry hash, asset hashes, body dimensions, and triangle count.

The texture is an atlas: its bottom strip holds solid surface colors, while its upper area supplies the side labels. Preserve this layout when replacing the PNG. Styling follows the manual's outline drawings approximately; it is a lightweight visual interpretation.

## Larger scenes and SLAM

See [scene layouts, screenshots and regeneration](src/bg1_scenarios/README.md).
`warehouse` is 36 × 28 m with rack aisles, pallets and columns; `office` is
32 × 26 m with sixteen interconnected furnished rooms. Both spawn at origin.

```bash
bash ros_ws/mapping.sh scene:=warehouse
# Or: bash ros_ws/mapping.sh scene:=office
bash ros_ws/save_map.sh
bash ros_ws/stop_mapping.sh

# Start a separate navigation session with a generated reference map
bash ros_ws/navigation_demo.sh scene:=office
# Or: bash ros_ws/navigation_demo.sh scene:=warehouse
```

Run one session at a time. Default scene remains `demo`. SLAM explores an
unknown map; navigation uses the selected scene's known map. Office uses a
scene-local NavFn Dijkstra override; original Nav2/RPP settings are retained
for demo and warehouse. Complex scenes are not wired into the separate
empty-region coverage demonstration.
