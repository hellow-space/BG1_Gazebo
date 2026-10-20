# BG1 Pro Navigation Gazebo Model Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a lightweight BG1 Pro three-wheel navigation model under the workspace root that runs with ROS 2 Jazzy and Gazebo Harmonic and exposes ROS 2 velocity, odometry, TF, joint-state, clock, and laser-scan interfaces.

**Architecture:** A single parameterized URDF/Xacro is the robot source for both `robot_state_publisher` and Gazebo's URDF-to-SDF conversion. The model uses Gazebo Harmonic native DiffDrive, JointStatePublisher, PosePublisher, and GPU lidar systems; `ros_gz_bridge` exposes their topics to ROS 2. Appearance is switchable between primitive visuals and a generated low-polygon wrapped Collada mesh while collision, inertia, joints, sensors, and plugins remain identical.

**Tech Stack:** ROS 2 Jazzy, ament/colcon, Xacro, Gazebo Harmonic 8, SDFormat 14, `ros_gz_sim`, `ros_gz_bridge`, Python 3, PyYAML, Pillow, pytest/ament tests.

**Spec:** `docs/superpowers/specs/2026-10-16-bg1-navigation-gazebo-model-design.md`

## Global Constraints

- New implementation files live under root-level `ros_ws/src`; `simulation_construct/` remains unchanged and is only a reference.
- Manual body envelope without squeegee: `1.120 x 0.660 x 1.303 m`.
- Manual maximum envelope with squeegee: `1.195 x 0.760 x 1.303 m`.
- Total empty-model mass: `348.5 kg` across all physical links.
- Maximum navigation command speed baseline: `1.0 m/s`; do not enforce `0.2 m/s` as a minimum command.
- Routine floor-obstacle test: `20 mm`; published upper-bound test: `25 mm`.
- Initial wheel track `0.62 m`, radius `0.12 m`, and width `0.08 m` are `ESTIMATED_BASELINE` values, not vehicle measurements.
- Initial axle-relative envelope is front `+0.840 m`, rear `-0.355 m`, left/right `+/-0.380 m`; body itself spans `-0.280 m` to `+0.840 m`.
- Default runtime is ROS 2 Jazzy with Gazebo Harmonic; no ROS 1 or Gazebo Classic runtime dependency may enter the new packages.
- Runtime commands set `ROS_LOG_DIR=/tmp/bg1_ros_logs` because the environment's default `$HOME/.ros/log` is read-only.
- The workspace root has no usable Git metadata, so task checkpoints use test/build evidence instead of commits.

## Review Focus

- Geometry drift: tests must compare the Xacro body, squeegee, wheel joints, and Nav2 footprint against the same YAML values.
- Mass drift: tests must sum all expanded URDF link masses and require `348.5 kg` within `1e-6 kg`.
- Topic mismatch: launch-contract and runtime checks must discover the Gazebo topics and prove the ROS names `/cmd_vel`, `/odom`, `/scan`, `/joint_states`, `/tf`, and `/clock` are present where applicable.
- Appearance contamination: primitive and wrapped expansions must have identical nonvisual XML signatures.
- Invalid provisional geometry: tests must reject or flag wheel/caster placement outside the maximum envelope and verify the spawned robot remains upright and stationary before motion commands.

---

### Task 1: ROS 2 Package Skeleton and Authoritative Configuration

**Files:**
- Create: `ros_ws/src/bg1_description/package.xml`
- Create: `ros_ws/src/bg1_description/CMakeLists.txt`
- Create: `ros_ws/src/bg1_description/config/bg1_geometry.yaml`
- Create: `ros_ws/src/bg1_description/config/bg1_physics.yaml`
- Create: `ros_ws/src/bg1_description/config/nav2_footprint.yaml`
- Create: `ros_ws/src/bg1_description/test/test_configuration.py`
- Create: `ros_ws/src/bg1_gazebo/package.xml`
- Create: `ros_ws/src/bg1_gazebo/CMakeLists.txt`
- Create: `ros_ws/README.md`

**Interfaces:**
- Consumes: Manual dimensions from the approved spec and provisional wheel values from `simulation_construct/ros_ws/src/cleanbot_description/config/vehicle_geometry.yaml`.
- Produces: `bg1_geometry.yaml` keys used by Xacro and tests; `bg1_physics.yaml` keys used by inertial/contact definitions; a Nav2 footprint polygon using the axle-relative maximum envelope.

- [x] **Step 1: Write the failing configuration tests**

Add tests named:

```python
def test_manual_dimensions_and_limits_are_authoritative(): ...
def test_axle_relative_envelope_closes_exactly(): ...
def test_nav2_footprint_matches_maximum_envelope(): ...
def test_provisional_wheels_and_caster_fit_inside_envelope(): ...
def test_obsolete_model_values_are_absent(): ...
```

Assertions must pin the exact values in Global Constraints. `test_obsolete_model_values_are_absent` recursively scans the new `bg1_description` and `bg1_gazebo` source files and rejects numeric model claims of `344 kg` and `70 mm`; it must not reject explanatory test text or the approved spec outside the packages.

- [x] **Step 2: Run the test to verify it fails**

Run:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest ros_ws/src/bg1_description/test/test_configuration.py -q
```

Expected: FAIL because the two packages and YAML files do not yet exist.

- [x] **Step 3: Create the ament packages and configuration**

Use `ament_cmake` packages. Install `config`, `urdf`, `meshes`, `materials`, and `launch` from `bg1_description`; install `config`, `launch`, `worlds`, and `scripts` from `bg1_gazebo`. Add test dependencies for `ament_cmake_pytest` and runtime dependencies for Xacro, robot-state-publisher, Gazebo integration, bridge messages, and launch.

Define these geometry fields with units in their names:

```text
body_length_m, body_width_m, overall_height_m
front_extent_m, body_rear_extent_m, rear_extent_m, half_width_m
body_bottom_z_m, squeegee_height_m
wheel_track_m, wheel_radius_m, wheel_width_m
caster_x_m, caster_radius_m, caster_trail_m
nav_lidar_x_m, nav_lidar_z_m, nav_lidar_min_m, nav_lidar_max_m
max_linear_mps, max_angular_rps, parameter_status
```

Use `body_bottom_z_m: 0.120`, `caster_x_m: 0.550`, `caster_radius_m: 0.080`, `caster_trail_m: 0.060`, `nav_lidar_x_m: 0.780`, `nav_lidar_z_m: 0.280`, scan range `0.12-20.0 m`, and provisional `max_angular_rps: 0.8`.

Physics assigns `3.0 kg` to each drive wheel, `1.0 kg` to the caster swivel, `0.8 kg` to the caster wheel, and derives the body mass as `340.7 kg`. Mark mass from the manual and all inertia/contact/drive parameters with separate provenance comments.

- [x] **Step 4: Run the configuration tests**

Run the Step 2 command.

Expected: all configuration tests PASS.

- [x] **Step 5: Build the empty packages as a checkpoint**

Run:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base ros_ws/log build --base-paths ros_ws/src --build-base ros_ws/build --install-base ros_ws/install --symlink-install
```

Expected: both packages are discovered and the build exits `0`.

---

### Task 2: Parameterized Physical Model and Gazebo Systems

**Files:**
- Create: `ros_ws/src/bg1_description/urdf/bg1.urdf.xacro`
- Create: `ros_ws/src/bg1_description/launch/description.launch.py`
- Create: `ros_ws/src/bg1_description/test/test_model_contract.py`
- Modify: `ros_ws/src/bg1_description/CMakeLists.txt`

**Interfaces:**
- Consumes: `bg1_geometry.yaml` and `bg1_physics.yaml` from Task 1.
- Produces: `/robot_description`, links `base_footprint`, `base_link`, `left_drive_wheel_link`, `right_drive_wheel_link`, `caster_swivel_link`, `caster_wheel_link`, and `nav_lidar_link`; joints with matching names; Gazebo Harmonic systems for drive, joint state, pose, and lidar.

- [x] **Step 1: Write failing model-contract tests**

Add tests named:

```python
def test_required_tree_and_joint_axes(): ...
def test_body_and_squeegee_collision_envelopes(): ...
def test_total_mass_is_348_5_kg(): ...
def test_inertias_are_positive_and_symmetric(): ...
def test_diff_drive_matches_joint_geometry(): ...
def test_harmonic_plugins_and_gpu_lidar_contract(): ...
def test_xacro_converts_to_sdf_14(): ...
```

The conversion test runs `xacro` to a temporary URDF, then `gz sdf -p` and checks for a valid SDF model. It must reject Classic plugin filenames such as `libgazebo_ros_diff_drive.so`.

- [x] **Step 2: Run the tests to verify they fail**

Run:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest ros_ws/src/bg1_description/test/test_model_contract.py -q
```

Expected: FAIL because the Xacro model is absent.

- [x] **Step 3: Implement the physical Xacro**

Load both YAML files once and derive body height, center, squeegee size/center, wheel positions, body mass, and primitive inertias in Xacro expressions. Use simple box, cylinder, and sphere/caster collisions.

Add native Harmonic systems:

```text
gz-sim-diff-drive-system / gz::sim::systems::DiffDrive
gz-sim-joint-state-publisher-system / gz::sim::systems::JointStatePublisher
gz-sim-pose-publisher-system / gz::sim::systems::PosePublisher
```

Configure DiffDrive with the two drive joints, `0.62 m` separation, `0.12 m` radius, 50 Hz odometry, the model-scoped command/odometry/TF topics, odometry frame `odom`, and child frame `base_link`. Set `max_linear_velocity` to `1.0`, `min_linear_velocity` to `-1.0`, `max_angular_velocity` to `0.8`, and `min_angular_velocity` to `-0.8`; these fields are supported by the installed Gazebo Harmonic DiffDrive system.

Add a `gpu_lidar` sensor to `nav_lidar_link`, 360 samples, 10 Hz, `-pi` to `+pi`, configured range, and topic `scan`. Contact/friction values come only from `bg1_physics.yaml`.

- [x] **Step 4: Implement the description launch file**

`description.launch.py` declares `appearance` (`primitive` or `wrapped`), resolves the installed package paths, invokes Xacro, and starts `robot_state_publisher` with `use_sim_time` and `/robot_description`.

- [x] **Step 5: Run model tests and build**

Run:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest ros_ws/src/bg1_description/test/test_configuration.py ros_ws/src/bg1_description/test/test_model_contract.py -q
colcon --log-base ros_ws/log build --base-paths ros_ws/src --build-base ros_ws/build --install-base ros_ws/install --symlink-install
```

Expected: all tests PASS and the build exits `0`.

---

### Task 3: Low-Polygon BG1 Wrap Assets

**Files:**
- Create: `ros_ws/src/bg1_description/tools/mesh_primitives.py`
- Create: `ros_ws/src/bg1_description/tools/build_visual_assets.py`
- Create: `ros_ws/src/bg1_description/materials/textures/bg1_label.png` (generated)
- Create: `ros_ws/src/bg1_description/meshes/bg1_body.dae` (generated)
- Create: `ros_ws/src/bg1_description/meshes/manifest.json` (generated)
- Create: `ros_ws/src/bg1_description/test/test_appearance_contract.py`
- Modify: `ros_ws/src/bg1_description/urdf/bg1.urdf.xacro`
- Modify: `ros_ws/src/bg1_description/CMakeLists.txt`

**Interfaces:**
- Consumes: authoritative geometry YAML and the reusable Collada-writing approach from `simulation_construct/ros_ws/src/cleanbot_appearance/tools/mesh_primitives.py`.
- Produces: deterministic BG1 visual assets, `appearance:=wrapped`, and a manifest recording geometry-source hash, triangle count, and asset hashes.

- [x] **Step 1: Write failing appearance tests**

Add tests named:

```python
def test_asset_build_is_deterministic(): ...
def test_wrapped_mesh_uses_authoritative_geometry(): ...
def test_mesh_triangle_budget_is_below_1500(): ...
def test_wrap_texture_is_present_and_legible_size(): ...
def test_primitive_and_wrapped_nonvisual_signatures_match(): ...
```

The final test removes only `<visual>` elements and Gazebo display material elements before comparing expanded XML trees.

- [x] **Step 2: Run the tests to verify they fail**

Run:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest ros_ws/src/bg1_description/test/test_appearance_contract.py -q
```

Expected: FAIL because no asset builder or generated assets exist.

- [x] **Step 3: Implement deterministic asset generation**

Adapt only the small mesh writer needed for a rounded rectangular shell, squeegee visual, lidar cap, and textured quads. Generate a white/dark-gray shell and a replaceable `1024 x 320` PNG carrying simple `PUDU` and `BG1 Pro` identification. Do not copy the existing brush or high-polygon tire generation.

The asset builder accepts:

```text
--geometry <path-to-bg1_geometry.yaml>
--output-root <bg1_description-package-root>
```

It writes assets atomically and records hashes in `manifest.json`.

- [x] **Step 4: Add the wrapped Xacro branch**

For `appearance:=wrapped`, use `package://bg1_description/meshes/bg1_body.dae` for body visuals while retaining primitive drive-wheel and caster visuals. For `appearance:=primitive`, retain the simple body and squeegee boxes. No conditional may wrap collision, inertial, joint, sensor, or plugin elements.

- [x] **Step 5: Generate assets and run appearance/model tests**

Run:

```bash
python3 ros_ws/src/bg1_description/tools/build_visual_assets.py --geometry ros_ws/src/bg1_description/config/bg1_geometry.yaml --output-root ros_ws/src/bg1_description
source /opt/ros/jazzy/setup.bash
python3 -m pytest ros_ws/src/bg1_description/test -q
```

Expected: generated files exist, triangle count is below `1500`, all description tests PASS.

---

### Task 4: Gazebo World, Spawn, and ROS 2 Bridges

**Files:**
- Create: `ros_ws/src/bg1_gazebo/worlds/bg1_flat.sdf`
- Create: `ros_ws/src/bg1_gazebo/launch/sim.launch.py`
- Create: `ros_ws/src/bg1_gazebo/config/bridge_topics.yaml`
- Create: `ros_ws/src/bg1_gazebo/test/test_launch_contract.py`
- Modify: `ros_ws/src/bg1_gazebo/CMakeLists.txt`

**Interfaces:**
- Consumes: `/robot_description` and the installed `ros_gz_sim` / `ros_gz_bridge` packages.
- Produces: Gazebo world `bg1_flat`; spawned model `bg1_pro`; ROS 2 topics `/cmd_vel`, `/odom`, `/scan`, `/joint_states`, `/tf`, `/tf_static`, and `/clock` as supported by discovered Gazebo topics.

- [x] **Step 1: Write failing launch-contract tests**

Add tests named:

```python
def test_world_loads_required_harmonic_systems(): ...
def test_launch_uses_installed_model_and_headless_switch(): ...
def test_bridge_contract_contains_navigation_topics(): ...
def test_no_ros1_or_classic_dependencies_are_declared(): ...
```

The bridge test pins message pairs and directions for command, odometry, scan, clock, joint states, and pose/TF. Each `bridge_topics.yaml` entry uses the installed `ros_gz_bridge` schema:

```yaml
- ros_topic_name: "cmd_vel"
  gz_topic_name: "/model/bg1_pro/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
```

Use the corresponding `GZ_TO_ROS` direction and message types for odometry (`nav_msgs/msg/Odometry` / `gz.msgs.Odometry`), scan (`sensor_msgs/msg/LaserScan` / `gz.msgs.LaserScan`), clock (`rosgraph_msgs/msg/Clock` / `gz.msgs.Clock`), joint state (`sensor_msgs/msg/JointState` / `gz.msgs.Model`), and TF (`tf2_msgs/msg/TFMessage` / `gz.msgs.Pose_V`). The final scoped joint-state and TF topic names are discovered with `gz topic -l` during runtime verification and then pinned in the test.

- [x] **Step 2: Run the tests to verify they fail**

Run:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest ros_ws/src/bg1_gazebo/test/test_launch_contract.py -q
```

Expected: FAIL because the world, launch, and bridge configuration are absent.

- [x] **Step 3: Implement the validation world**

Create an SDF 1.10 world with Physics, UserCommands, SceneBroadcaster, Sensors using Ogre2, a sun, and a `100 x 100 m` ground plane. Add no navigation obstacles in this task; stable chassis behavior is isolated first.

- [x] **Step 4: Implement simulation launch**

`sim.launch.py` declares `gui`, `headless`, `paused`, `appearance`, spawn pose, and `world` arguments. It:

1. Sets `GZ_SIM_RESOURCE_PATH` to installed package shares.
2. Includes `ros_gz_sim/launch/gz_sim.launch.py`.
3. Starts `robot_state_publisher` from the same expanded Xacro.
4. Spawns `bg1_pro` from `/robot_description` using `ros_gz_sim create`.
5. Starts `ros_gz_bridge parameter_bridge` from `bridge_topics.yaml`.

Use topic direction ROS-to-Gazebo for `/cmd_vel` and Gazebo-to-ROS for odometry, scan, clock, joint state, and pose/TF.

- [x] **Step 5: Run launch-contract tests and rebuild**

Run:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest ros_ws/src/bg1_gazebo/test/test_launch_contract.py -q
colcon --log-base ros_ws/log build --base-paths ros_ws/src --build-base ros_ws/build --install-base ros_ws/install --symlink-install
```

Expected: tests PASS and both packages build.

---

### Task 5: Headless Runtime Verification and Operator Documentation

**Files:**
- Create: `ros_ws/src/bg1_gazebo/scripts/verify_runtime.py`
- Create: `ros_ws/src/bg1_gazebo/test/test_runtime_helpers.py`
- Create: `ros_ws/src/bg1_gazebo/worlds/bg1_thresholds.sdf`
- Modify: `ros_ws/README.md`
- Modify: `docs/superpowers/specs/2026-10-16-bg1-navigation-gazebo-model-design.md` only if runtime discovery requires a documented topic-name correction.

**Interfaces:**
- Consumes: installed workspace, `sim.launch.py`, ROS 2 bridge topics, and Gazebo model pose services/topics.
- Produces: repeatable headless verification for spawn stability, forward motion, rotation, stopping, topic availability, speed ceiling, and 20/25 mm threshold scenes.

- [x] **Step 1: Write failing helper tests**

Add tests named:

```python
def test_stationary_window_rejects_drift_or_tip(): ...
def test_forward_motion_requires_positive_x_progress(): ...
def test_rotation_requires_positive_yaw_change(): ...
def test_stop_window_requires_near_zero_twist(): ...
def test_speed_ceiling_rejects_overspeed(): ...
```

Use pure functions over timestamped pose/odometry samples so failures are deterministic and do not launch Gazebo inside unit tests.

- [x] **Step 2: Run the helper tests to verify they fail**

Run:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest ros_ws/src/bg1_gazebo/test/test_runtime_helpers.py -q
```

Expected: FAIL because the verifier is absent.

- [x] **Step 3: Implement the runtime verifier**

`verify_runtime.py` subscribes to `/odom`, `/scan`, `/joint_states`, `/tf`, and `/clock`, publishes bounded commands on `/cmd_vel`, and records JSON evidence under `/tmp/bg1_gazebo_verification`. It performs, in order:

1. Topic and message freshness check.
2. Two-second stationary stability check.
3. Low-speed forward command and positive-X displacement check.
4. Positive angular command and yaw-change check.
5. Zero-command stop check.
6. `1.0 m/s` command ceiling check.

It must always publish zero velocity before exiting, including exceptions and timeouts.

- [x] **Step 4: Add threshold world and documentation**

Create separate `20 mm` and `25 mm` low obstacles in `bg1_thresholds.sdf`; document that this world checks the simplified model and does not certify real hardware. Document build, headless launch, GUI launch, direct teleoperation topic, runtime verification, appearance selection, and where provisional measurements live.

- [x] **Step 5: Run complete static verification**

Run:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest ros_ws/src/bg1_description/test ros_ws/src/bg1_gazebo/test -q
colcon --log-base ros_ws/log build --base-paths ros_ws/src --build-base ros_ws/build --install-base ros_ws/install --symlink-install
colcon --log-base ros_ws/log test --base-paths ros_ws/src --build-base ros_ws/build --install-base ros_ws/install --event-handlers console_direct+
colcon test-result --test-result-base ros_ws/build --verbose
```

Expected: all source tests and ament tests PASS; build and test-result commands exit `0`.

- [x] **Step 6: Run headless Gazebo verification**

Run the launch in one process group with writable logging:

```bash
mkdir -p /tmp/bg1_ros_logs
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
ROS_LOG_DIR=/tmp/bg1_ros_logs ros2 launch bg1_gazebo sim.launch.py gui:=false headless:=true appearance:=wrapped
```

Then run:

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
ROS_LOG_DIR=/tmp/bg1_ros_logs ros2 run bg1_gazebo verify_runtime.py
```

Expected: verifier exits `0`, writes JSON evidence, and reports fresh navigation topics, stable spawn, forward motion, positive rotation, stop, and speed compliance. Stop the launch process cleanly after verification.

- [x] **Step 7: Inspect the rendered model once**

Launch with `gui:=true`, capture one screenshot of the wrapped model, and visually check body proportions, wheel placement, caster placement, lidar position, wrap orientation, clipping, and ground contact. Fix only observed model defects, then repeat affected static and runtime checks.


## Execution status — 2026-10-16

Implementation and listed command checks completed. The checklist was synchronized after the GUI appearance revision. Final shell: 1172 triangles; actual Gazebo render: `docs/previews/bg1_wrapped_gazebo.png`. Source tests: 26 passed; colcon aggregate: 31 tests, zero failures. Runtime command checks passed with evidence in `/tmp/bg1_gazebo_verification/runtime_evidence.json`.

Validation limits: motion/stationary checks use wheel odometry, so they do not independently verify chassis tilt, ground-truth displacement, or wheel slip. The 20/25 mm threshold scene is created and SDF-validated; obstacle crossing has not been measured. Full Nav2 mapping/localization/path-following integration is outside this model-construction implementation. Independent chassis-pose validation remains an open acceptance limitation from the review focus.

