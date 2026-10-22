#!/usr/bin/env bash
set -e
coverage_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$coverage_dir/../ros_ws/install/setup.bash"
colcon --log-base "$coverage_dir/log" build --base-paths "$coverage_dir/src" \
  "$coverage_dir/../third_party/opennav_coverage" "$coverage_dir/../third_party/Fields2Cover" \
  --build-base "$coverage_dir/build" --install-base "$coverage_dir/install" \
  --packages-up-to bg1_coverage_demo --symlink-install --parallel-workers 2 "$@" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DBUILD_TESTS=OFF -DBUILD_TUTORIALS=OFF -DBUILD_PYTHON=OFF
