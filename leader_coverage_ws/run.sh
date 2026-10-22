#!/usr/bin/env bash
set -e
if pgrep -f '^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch (bg1_coverage_demo|bg1_navigation|bg1_gazebo|bg1_mapping|bg1_leader_coverage) ' >/dev/null; then
  echo 'A BG1 simulation is already running. Stop it before starting the leader coverage preview.' >&2
  exit 1
fi
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace_dir/../ros_ws/install/setup.bash"
source "$workspace_dir/install/setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/bg1_leader_coverage_logs}"
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch bg1_leader_coverage demo.launch.py "$@"
