#!/usr/bin/env bash
set -e
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace_dir/../ros_ws/install/setup.bash"
source "$workspace_dir/install/setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/bg1_leader_coverage_logs}"
mkdir -p "$ROS_LOG_DIR"
exec ros2 run bg1_leader_coverage motion_command.py "$@"
