#!/usr/bin/env bash
set -e
if pgrep -f '^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch (bg1_coverage_demo|bg1_navigation|bg1_gazebo) ' >/dev/null; then
  echo 'A BG1 simulation is already running. Stop it before starting the coverage demo.' >&2
  exit 1
fi
coverage_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$coverage_dir/../ros_ws/install/setup.bash"
source "$coverage_dir/install/setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/bg1_coverage_logs}"
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch bg1_coverage_demo demo.launch.py "$@"
