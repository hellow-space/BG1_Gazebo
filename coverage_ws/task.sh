#!/usr/bin/env bash
set -e
coverage_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$coverage_dir/../ros_ws/install/setup.bash"
source "$coverage_dir/install/setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/bg1_coverage_logs}"
mkdir -p "$ROS_LOG_DIR"
exec ros2 run bg1_coverage_demo coverage_task.py "$@"
