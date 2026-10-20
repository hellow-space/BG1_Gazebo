# Coverage deployment ledger

Execution: inline, explicitly requested by user after design approval. Parent has no Git history; preserve upstream commits and protected-file hashes instead of creating a worktree. All new code and dependencies reside in coverage_ws.

Task 1 complete: cloned opennav_coverage jazzy at 194250501889ce8896c5ee8b80a4d1317750be0e and Fields2Cover v1.2.1-devel at 87a91dbd9a7e762b15706b5337f7828f02a584ee. Minimal dependency closure built successfully; no upstream patches or system installs were required.

Task 2 complete: invalid polygon, result status/frame, nonfinite pose, rotated footprint and connector checks added RED→GREEN. Review strengthened rotation sweeps, raw quaternion validation and time-ordered/recent actual motion checks. Full suite currently includes action failure tests.

Task 3 runtime: planning succeeded with 647 poses, 7 swaths, 35.7077 m, approximately 2–6 ms simulation planning time. Plan-only probe observed zero /cmd_vel messages and zero odometry drift. Interrupted execution was canceled by controller_server and stopped; cancellation evidence retained.

Task 3: Ruling: static map→odom assumption failed after long coverage route (4.6299 m actual error despite action SUCCEEDED). Added isolated simulation ground-truth localization rather than changing physics or existing navigation files. Cost if wrong: this demo cannot establish real-world localization; documented and requires AMCL/SLAM for deployment outside simulation. Original failed evidence retained.

Compatibility fixes: protobuf JSON encodes Gazebo timestamp seconds as strings; normalize explicitly (regression test). Upstream Path poses omit individual frame IDs; accept inheritance from the validated map-level header while rejecting contradictory frames (regression test).

Final reviewer identified five important issues: cleanup short-circuit, untracked goal acknowledgement, rotation sweeps, raw quaternion validation and stationary/stale success evidence. All addressed with failure tests. Ctrl+C/SIGTERM handlers preserve the ROS context until cancellation/stop/reaping. Tests retain pending futures, always attempt stop/reap, and attempt controller deactivation when cancellation transport fails.

Final: minor (deferred): --task-config custom polygon does not update the preview node's default boundary; README directs users to edit the shared default task.yaml. Cost: preview boundary mismatch if advanced override is used.

Task 3 complete: corrected full run returned FollowPath SUCCEEDED. Coverage path 35.7077 m / 7 swaths; approach plus execution plus cleanup 163.246 s. Actual recorded travel 36.9805 m, 1,581 ground-truth samples, final error 0.18833 m and yaw error 0.03656 rad. The run starts from the prior interrupted pose, so it additionally verifies map-frame approach after odometry drift rather than assuming a reset origin.

Task 4 complete: adapter source suite 21 cases; colcon aggregate 23 tests, 0 errors/failures/skips. Original ros_ws source suite 31 cases passed. SHA256 of all 39 protected files matches. Both upstream git status outputs are clean. Duplicate launch rejected, plan-only zero command/drift probe passed, interrupt cancellation confirmed and final 0.5 s actual drift zero. Stop helper was exercised during restarts and reaped Gazebo descendants.

Artifacts: docs/verification/plan.json, plan_only_safety.json, cancellation.json, cancellation_summary.json, execution.json, summary.json, coverage_trajectory.png, underlay_integrity.txt. Final Gazebo/RViz remain open with the last coverage path and stopped robot.

Final review: fresh reviewer completed; five important findings addressed in one fix pass with RED→GREEN tests. Minor action-test coverage comment addressed by added lifecycle tests and runtime interruption/plan-only probes; custom preview boundary override remains documented and deferred. No existing framework files were changed.
