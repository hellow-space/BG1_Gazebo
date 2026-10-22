#!/usr/bin/env bash
set -e
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace_dir/../ros_ws/install/setup.bash"
colcon --log-base "$workspace_dir/log" build --base-paths "$workspace_dir/src" \
 --build-base "$workspace_dir/build" --install-base "$workspace_dir/install" --symlink-install \
 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
 -DLEADER_SOURCE="$workspace_dir/../third_party/leader/coverage_path_planner"
