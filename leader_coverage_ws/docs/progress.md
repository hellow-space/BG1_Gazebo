# Leader coverage ROS 2 adapter — 2026-10-21

User approved independent ROS 2 preview, own interfaces without robot_msg,
warehouse scene and RViz selection. Original leader package remains in root.
Work performed in leader_coverage_ws; no source replacement, commits or push.

Task 1 complete: own PlanCoverage service and generated-source C++ adapter.
Two packages build. Tests reject missing map, bad polygon/frame/settings,
unknown start, out-of-map vertices and partial occupancy; both source patterns
plan around an island. All 21 recorded source-file hashes match originals.

Task 2 complete: Publish Point polygon selection, close on first vertex,
plan/reset/undo/demo/pattern controls, retained markers/paths/status.
Real discovered service without a responding executor verifies request reset,
late-response isolation and steady-clock timeout recovery.

Task 3 complete: build/run/region/stop scripts, scoped navigation include,
separate RViz view, README and actual warehouse integration evidence.
Latest overlay colcon: 12 tests, zero errors/failures/skips. Existing workspace
and pure selection tests: 40 pytest cases pass. Python compilation, shell
syntax and git diff whitespace checks pass. A fresh reviewer rechecked both
P2 fixes and found no remaining Important issues.

Runtime evidence: docs/verification/warehouse.json and warehouse.png.
Direct requests: zigzag 2087 poses / 111.194 m; backshaped 2564 poses /
154.668 m. Real /clicked_point closure produced 2125 poses. Invalid service
requests rejected; reset clears retained path. Differences in path count
reflect live TF start vs explicit (0,0), not a switch of algorithms.
Final scene is open with demo selection and zigzag preview, robot stationary.
No robot motion or full-coverage-rate claim is made.

Resolved integration findings (RED -> GREEN): rclpy Node.services is a
read-only property (renamed command_services); nested rviz=false launch
argument leaked into parent (scoped GroupAction); occupancy 50 did not seed
upstream obstacle inflation (normalize all nonfree cells to100); pending
service response loss blocked UI indefinitely (future cancellation/token and
steady clock timeout). Original unused smoothPathInt missing return remains
an upstream compile warning. Startup RViz shader warning was observed, but
final map and path rendering were visually confirmed.

Manual motion extension (user approved 2026-10-21): added motion.sh start,
stop/status, central coverage_motion node and foreground CLI. Nav2 approach
then RPP FollowPath uses existing configuration. Original user selection was
captured before testing; exact polygon restored after test (path regenerated
from current robot position). Full route remains available, robot stopped.

Motion tests: original leader-generated 120-pose prefix including a turn
completed approach+follow; actual Gazebo travel5.8696m, endpoint error0.2065m.
External stop while following, selector undo during approach, and CLI SIGINT
all terminate; each subsequent2-second stationary window has0m displacement.
See docs/verification/motion/runtime.json and original_selection.json.
A fresh reviewer identified CLI exceptions escaping without cancellation;
RED/GREEN tests now cover lost start acknowledgement, monitor timeout and
explicit duplicate rejection. Real ROS ActionServer protocol tests cover
late acceptance/stop, selection change, rejection and duplicate start;3 pass.
Review recheck found no remaining important issues. No full86m route or
complete cleaning-coverage claim is made.
Final validation: overlay colcon23 tests, zero errors/failures/skips; existing
workspace plus pure selection40 pytest cases pass; source/shell/whitespace
checks pass. Live controller desired_linear_vel remains0.3. Live retained
polygon coordinates match the saved user polygon exactly; motion phase is
stopped with regenerated1460-point path ready for explicit start.
