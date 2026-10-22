# Two larger BG1 scenes

Scope: extend the existing world/map generation and launch flow with two
static environments; keep demo as default and preserve robot physics and
SLAM and original demo Nav2 parameters. Warehouse: 36 x 28 m with repeated rack aisles,
cross aisles, staging obstacles and columns. Office: 32 x 26 m with a
central cross corridor, sixteen interconnected rooms, 2.2 m doors and
corner furniture. Both spawn at (0,0,0).

Implementation: independent bg1_scenarios resource/helper package, YAML
layouts, deterministic SDF/PGM/map-YAML generation and overview images.
Existing mapping/navigation launches gain scene:=demo|warehouse|office.
Navigation uses the generated reference map; SLAM loads only the world.
Coverage remains the existing empty-rectangle demonstration because its
planner has not been integrated with obstacle-aware region decomposition.

Validation: generator tests first; SDF/map agreement; connected free space
and reachable goal probes with conservative radius >= BG1 circumradius
plus footprint padding; preserved original tests; build and runtime smoke
checks for both worlds. Save images and reproducible commands. Do not
commit or push unrelated existing mapping files or the user's saved map.


## Verification, 2026-10-21

- Five ROS packages built successfully; 37 source pytest cases pass and
  colcon reports 45 tests, zero errors/failures/skips. Both worlds pass
  `gz sdf -k`.
- Conservative connected-space checks use the padded BG1 footprint's
  circumradius plus half-cell diagonal, not a point-robot approximation.
- Real Gazebo screenshots are in `bg1_scenarios/previews/*_gazebo.png`.
- Nav2 runtime: warehouse 6/6 and office 20/20 ComputePathToPose probes
  succeed from the origin, with live map, scan and TF. These checks do not
  command motion or establish end-to-end navigation success.
- Both scenes start SLAM Toolbox with laser, nonempty map and map-to-base
  TF available; no AMCL or coverage simulator localization node is present.
  This is stationary startup validation, not a completed mapping route.
- Runtime reports: `../verification/scenes/{warehouse,office}_{navigation,mapping}.json`.

Office NavFn A* reproducibly reported a legal potential but failed to
extract a path to (-12.5,-5). Changing only use_astar=false made every
probe succeed. The persistent fix is a scene-specific planner override
in config/office_navigation.yaml; original nav2.yaml and RPP are unchanged.
Demo and warehouse retain A*. A fresh office launch passed all 20 probes.

The map generator uses free_thresh=0.196 so background pixel value 205 is
classified unknown rather than free. The reference maps and Gazebo
collision geometry are generated together. Layout-generation tools are
source-only; users regenerate with the documented repository-relative command.
