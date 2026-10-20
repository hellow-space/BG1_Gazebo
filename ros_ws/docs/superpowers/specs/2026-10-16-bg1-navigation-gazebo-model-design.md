# BG1 Pro Navigation Gazebo Model Design

## Objective

Build a lightweight Gazebo model of the PUDU BG1 Pro under the workspace root. The model is intended for two-dimensional navigation, collision checking, command/odometry integration, and visual identification. It is not intended to reproduce the cleaning mechanism, detailed sensor fusion, or calibrated vehicle dynamics.

The extracted `simulation_construct/` project remains a reusable reference. The new BG1 files live outside it so the source project and its existing regression baselines remain intact.

## Source of Truth

The product dimensions and operating limits come from `BG1_Pro_User_Manual_用户手册-CN_0506.pdf`:

- Body envelope without squeegee: `1.120 x 0.660 x 1.303 m`.
- Maximum envelope with squeegee: `1.195 x 0.760 x 1.303 m`.
- Empty vehicle mass: approximately `348.5 kg`.
- Commanded operating speed ceiling: `1.0 m/s`.
- Minimum physical passage width: `0.85 m`.
- Recommended floor protrusion: less than `20 mm`.
- Published maximum obstacle height: `25 mm`.

Wheel radius, drive-wheel track, axle position, caster position, center of mass, and angular speed are not specified in the manual. Their initial values are simulation assumptions and must be kept in one configuration file with an explicit `ESTIMATED_BASELINE` status.

The older `simulation_construct` documents contain a `344 kg` mass and a `70 mm` obstacle reference. Those values do not define the new model.

## Selected Architecture

Create a root-level ROS 2 Jazzy ament workspace targeting Gazebo Harmonic:

```text
BG1_Gazebo/
└── ros_ws/
    └── src/
        ├── bg1_description/
        └── bg1_gazebo/
```

`bg1_description` owns geometry, physics parameters, URDF/Xacro, materials, textures, and model contract tests. `bg1_gazebo` owns Gazebo launch files and a small validation world. This separation keeps the model usable from other navigation stacks without importing the extracted project's planning implementation.

The model reuses the proven three-wheel differential-drive pattern from `simulation_construct/ros_ws/src/cleanbot_description`: two driven rear wheels, one freely swiveling front caster, joint-state publication, velocity commands, odometry, and a planar laser scan. Code and assets will be adapted into BG1-named ROS 2 packages rather than making the new project depend on the extracted ROS 1 directory.

Gazebo Harmonic is the runtime source of physics, wheel joint states, odometry, poses, and laser scans. `ros_gz_bridge` exposes the required Gazebo Transport topics to ROS 2. The extracted Gazebo Classic plugin XML is a behavioral reference only and is not copied into the new model.

## Coordinate Convention

- `x`: forward.
- `y`: left.
- `z`: upward.
- `base_link`: ground projection of the rear drive-axle midpoint.
- `base_footprint`: ground-level navigation frame fixed to `base_link`.

The axle-relative front and rear extents will be derived from the configured axle location. They must not be assumed symmetric around `base_link`.

## Physical Model

The minimum physical tree is:

```text
base_footprint
└── base_link
    ├── left_drive_wheel_link
    ├── right_drive_wheel_link
    ├── caster_swivel_link
    │   └── caster_wheel_link
    └── nav_lidar_link
```

The visible body is one low-detail shell. The collision geometry is deliberately simpler:

1. A main body box matching the `1.120 x 0.660 m` body footprint.
2. A low rear squeegee box extending the collision and navigation envelope to `1.195 x 0.760 m`.
3. Cylinders for the two drive wheels.
4. A simple caster collision attached through swivel and rolling joints.

Cleaning brushes, tanks, pipes, screens, cameras, and decorative sensors are omitted from collision and inertia. Small exterior details may be included in the body texture only.

The sum of all link masses must be `348.5 kg`. Wheel and caster masses remain assumptions; the body mass is calculated as the remainder. Body inertia is calculated from the simplified box distribution and marked estimated. The model does not simulate water transfer, sloshing, cleaning forces, or payload-dependent center-of-mass changes.

## Provisional Wheel Geometry

For the first navigation model, use the extracted model's values as explicit placeholders:

- Drive-wheel center track: `0.62 m`.
- Drive-wheel radius: `0.12 m`.
- Drive-wheel width: `0.08 m`.

The caster location and body offset will be rescaled to fit the BG1 body envelope while keeping the front caster ahead of the drive axle. All values remain centralized in `bg1_geometry.yaml` so measured values can replace them without editing Xacro or launch files.

## Drive and Odometry

Use Gazebo Harmonic's `gz-sim-diff-drive-system` with:

- Gazebo command topic: `/model/bg1_pro/cmd_vel`.
- Gazebo odometry topic: `/model/bg1_pro/odometry`.
- Odometry frame: `odom`.
- Robot base frame: `base_link`.
- Published maximum linear speed enforced by the surrounding controller: `1.0 m/s`.

The ROS 2 launch layer bridges and remaps these to `/cmd_vel` and `/odom`. It also bridges Gazebo joint state, pose/TF, clock, and laser topics needed by Nav2 and RViz.

The manual's `0.2 m/s` lower operating value is not enforced as a minimum command because navigation controllers need slower commands for docking, final approach, and stable stopping.

Wheel torque, acceleration, friction, and maximum angular speed are simulation parameters, not claimed hardware measurements. The first model prioritizes stable navigation over dynamic fidelity.

## Navigation Sensor

Use one 360-degree two-dimensional Gazebo GPU lidar that is bridged to ROS 2 `/scan`. It is an equivalent navigation sensor representing the usable planar output of the real robot's 3D LiDAR, VSLAM, and multi-sensor system.

Do not simulate RGBD cameras, multiple lidars, VSLAM, or sensor fusion in the first version. Sensor pose, update rate, range, and minimum distance live in `bg1_geometry.yaml`. The visual shell may show a simple lidar cap without creating another sensor.

## Footprint

Generate the navigation footprint from the axle-relative maximum envelope, including the squeegee. The target physical width is `0.760 m` and target overall length is `1.195 m`.

The `0.85 m` passage width is an extreme physical-clearance test. Passing through it is not guaranteed when a navigation stack adds inflation or safety margins. Validation must distinguish:

- Physical collision clearance.
- Costmap footprint clearance.
- Planner clearance with configured inflation.

## Appearance and Wrap

Create a lightweight body mesh or six-face box mesh with stable UV coordinates. It should use far fewer triangles than the existing YUNJIE appearance assets. The initial appearance is a neutral white and dark-gray BG1 shell with replaceable side, front, and rear wrap textures.

The first texture set may use simple `PUDU` and `BG1 Pro` identification. Formal artwork can replace the PNG textures later without modifying collision, inertia, joints, or sensor definitions.

Visual replacement must preserve a nonvisual model signature so applying a wrap cannot change physics, joints, plugins, or sensors.

## Reuse Boundary

Reuse or adapt from `simulation_construct`:

- Three-wheel link and joint structure.
- Differential-drive and joint-state Gazebo plugin configuration.
- Geometry-through-YAML pattern.
- Visual-only overlay contract.
- COLLADA generation helpers where they reduce implementation work.
- Headless Xacro/SDF and model-contract testing techniques.

Do not copy into the new project:

- Company planning source or prebuilt libraries.
- Existing high-polygon YUNJIE meshes.
- Cleaning-operation behavior trees and cleaning mechanisms.
- Water-payload experiments.
- Historical `344 kg` and `70 mm` assumptions.

The new packages target ROS 2 Jazzy and Nav2. They do not directly compose with the extracted ROS 1 company navigation binaries. Reusing that planning stack later requires a separate ROS 1 environment or an explicit ROS 1/ROS 2 bridge project, neither of which is part of this model task.

## Configuration Files

`bg1_description/config/bg1_geometry.yaml` is the only source for dimensions, link positions, wheel geometry, sensor pose, and speed limits.

`bg1_description/config/bg1_physics.yaml` is the only source for masses, inertias, friction, contact, torque, and wheel acceleration.

Known manual values and estimated simulation values must be labeled separately in comments and documentation.

## Validation

Static validation must prove:

- Xacro expands successfully.
- Gazebo converts the expanded URDF to SDF.
- Required links, joints, plugins, frames, and topics exist.
- Total mass is `348.5 kg` within floating-point tolerance.
- Main body and maximum collision envelopes match their configured values.
- The drive plugin uses the same track and diameter as the wheel joints.
- Applying the wrap changes only visual elements.
- No obsolete `344 kg` or `70 mm` value enters the new package.

Gazebo validation must prove:

- The model spawns without errors.
- It remains stable while stationary.
- Positive linear commands move it forward.
- Positive angular commands rotate it in the expected direction.
- Zero command stops it.
- `/odom`, TF, joint states, and `/scan` are published.
- The command path respects the `1.0 m/s` ceiling used by the navigation integration.
- It accepts ROS 2 velocity commands through `ros_gz_bridge` and exposes the topics required for later Nav2 integration.
- A `20 mm` obstacle is the routine threshold test and `25 mm` is treated as the published upper-bound test.

## Out of Scope

- Photorealistic body geometry.
- Detailed underside and cleaning assembly.
- Cleaning performance or fluid simulation.
- Calibrated tire, motor, suspension, center-of-mass, or inertia behavior.
- Multiple physical lidars, RGBD cameras, and VSLAM.
- Charging, automatic filling, draining, and docking mechanisms.
- Modifications to the company navigation stack.

## Delivery Criteria

The first version is complete when the root-level packages build, the static contract tests pass, the model spawns and drives stably in Gazebo, navigation topics are available, the conservative BG1 footprint is used, and the replaceable wrap is visible without affecting physics.
