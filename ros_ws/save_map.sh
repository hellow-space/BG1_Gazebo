#!/usr/bin/env bash
set -e
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace_dir/install/setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/bg1_mapping_logs}"
mkdir -p "$ROS_LOG_DIR"
if [ "$#" -eq 0 ]; then
  map_name="bg1_$(date +%Y%m%d_%H%M%S)"
  exec ros2 run bg1_mapping save_map.py "$workspace_dir/../data/maps/slam/$map_name/$map_name"
fi
exec ros2 run bg1_mapping save_map.py "$@"
