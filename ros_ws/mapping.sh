#!/usr/bin/env bash
set -e
if pgrep -f '^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch (bg1_coverage_demo|bg1_navigation|bg1_gazebo|bg1_mapping) ' >/dev/null; then
  echo 'A BG1 simulation is already running. Stop it before starting mapping.' >&2
  exit 1
fi
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace_dir/install/setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/bg1_mapping_logs}"
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch bg1_mapping mapping.launch.py "$@"
