# Leader coverage planner ROS 2 preview

Approved in chat 2026-10-21: preserve leader-provided root-level
coverage_path_planner, adapt in an independent workspace, own interfaces
because robot_msg is unavailable; use warehouse and RViz polygon selection;
preview only, no robot execution.

Build-generated ROS 2 translation of the existing C++ planner and task-mask
helper, keeping algorithm bodies and original source hashes. Replace ROS1
message/time/logging/costmap interfaces with ROS2 equivalents. Drop unused
robot_msg assessment interfaces and device/region JSON loading in the preview
map initializer; use live /map and the user's selected polygon instead.

Service /leader_coverage/plan accepts map-frame PolygonStamped, start point,
pattern (0 zigzag,1 backshaped), stripe spacing and clearance in metres;
returns status, error and nav_msgs paths. Validate polygons, known free map,
positive settings and generated path segments. Rejected paths must not appear
as valid results. Conservative circular clearance 1m is a preview baseline;
no claim of controller feasibility or complete coverage.

Python UI collects /clicked_point, visualizes vertices and polygon, closes
within 0.35m of first vertex with at least three vertices, then requests
planning. Explicit plan/reset/undo services support scripting; a mode helper
chooses pattern. Single request at a time. Display status and retain paths.
Launch includes original warehouse Nav2 stack unchanged and a dedicated RViz
config. No cmd_vel or navigation execution is sent by this overlay.

## Adaptation decisions and limits

Original files stay read-only; translation changes transport/logging/types.
The preview map initializer omits proprietary device/region loading and
unused robot_msg assessment functions, and fixes an original single-pose
yaw iterator edge case. Display-only custom cost constants are assigned in
the compatibility header. No traversal/connection/smoothing algorithm is
substituted. The original unused smoothPathInt function has a missing-return
compiler warning; it is not called by this preview.

All nonzero/unknown map cells are normalized to lethal obstacles before the
source helper's inflation (partial occupancy must receive the same clearance).
Optional island tracing is disabled. Coverage path points and interpolated
segments are validated; boundary contour is visualization only. Original
coarse cells/connection policy can omit narrow or disconnected subareas:
preview generation is not proof of full coverage.

UI pending requests have a steady-clock timeout and cancel/invalidate on
selection changes. Late responses cannot replace a newer selection or clear
its pending state. Cancellation drops the client response; ROS services do
not stop computation already executing in the server.

## Manual execution extension — approved 2026-10-21

User subsequently approved manual start/stop and simulation motion testing.
Default planning stays motionless. A separate coverage_motion node snapshots
the current valid map path, NavigatesToPose to its first pose, then sends
FollowPath to existing controller FollowPath / goal_checker / progress_checker.
Original RPP and robot configuration remain unchanged, linear limit0.3m/s.

Motion CLI runs in foreground, Ctrl+C/stop cancels; resetting/replacing the
selected path cancels centrally. A second start is rejected. Start/result
acceptance races retain the goal until cancellation/termination; status loss
in the CLI triggers stop/wait even if start acknowledgement was lost. Explicit
rejection of a duplicate start leaves the already running task alone. Lost
cancellation acknowledgement triggers lifecycle deactivation fallback and
requires a simulation restart rather than silently allowing a new start.
Approach can leave the cleaning polygon using the existing navigable map.
