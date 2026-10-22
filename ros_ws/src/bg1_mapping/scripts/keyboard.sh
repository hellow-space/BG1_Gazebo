#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/bg1_mapping_logs}"
mkdir -p "$ROS_LOG_DIR"
printf '\nBG1 MAPPING: i=forward, comma=reverse, j=left, l=right, k=STOP.\n'
printf 'Focus THIS terminal. Releasing a key does not stop; press k.\n'
printf 'Start speed: 0.15 m/s, turn: 0.30 rad/s. Ctrl+C stops and exits.\n\n'
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r __node:=mapping_keyboard -p speed:=0.15 -p turn:=0.30
