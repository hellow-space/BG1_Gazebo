# BG1 Coverage Implementation Plan

> Execution: inline using executing-plans. User explicitly requested execution after approving the design. No additional approval round; no parent Git repository is available.

Goal: deploy the Jazzy coverage server in an isolated overlay and verify planning plus BG1/RPP execution.
Spec: 2026-10-19-bg1-coverage-design.md.
Architecture: read-only ros_ws underlay; minimal upstream build; separate bg1_coverage_demo launch and action client.

## Constraints and review focus

- Never edit ros_ws source or existing entry points; compare SHA256 manifests before/after.
- Pin coverage jazzy and Fields2Cover v1.2.1-devel commits; preserve upstream code.
- Reject invalid polygons, nonfinite paths, failed actions and footprint excursions outside the empty test area.
- Planning alone never commands motion; execution has bounded waits, cancellation and explicit stop.
- Check independent Gazebo pose, not only odometry; distinguish demo static TF from actual localization.

## Tasks

1. [x] Snapshot protected files; fetch upstream branches; inspect APIs and build minimal dependency closure. Files: vendor sources, versions.json, build.sh. Verify: colcon builds fields2cover, opennav_coverage_msgs, opennav_coverage.
2. [x] Implement bg1_coverage_demo pure geometry validation and tests first. Interfaces: validate_polygon(points), validate_path(poses, bounds, footprint), connector(start,end). Test invalid polygon, nonfinite path, rotated footprint boundary violation, bounded connector. Verify failing tests then green suite.
3. [x] Add isolated launch, server config, RViz and explicit planning/execution client. Reuse base model launch and RPP config. Client computes coverage, validates it, publishes path/region, optionally approaches start and follows path, saves status/ground truth, stops on failure. Verify planning produces multiple swaths and no cmd_vel, then complete a real Gazebo coverage action.
4. [x] Document build/start/plan/execute/stop; run tests and protected-file hashes; save evidence and inspect trajectory. Review action cleanup, process conflicts, coordinate frames and configuration isolation.

## Verification commands

- bash coverage_ws/build.sh
- python3 -m pytest coverage_ws/src/bg1_coverage_demo/test -q
- bash coverage_ws/run.sh gui:=false rviz:=false
- bash coverage_ws/task.sh --output coverage_ws/docs/verification/plan.json
- bash coverage_ws/task.sh --execute --output coverage_ws/docs/verification/execution.json
- sha256sum --check coverage_ws/docs/ros_ws_before.sha256

No source commits/worktrees: parent is not a Git repository. Use upstream repository commits and a local execution ledger instead. Final review will inspect the full adapter and runtime evidence.
