#!/usr/bin/env bash
# Run in a terminal while bg1_gazebo sim.launch.py is running.
set -e
source /opt/ros/jazzy/setup.bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/bg1_ros_logs}"
mkdir -p "$ROS_LOG_DIR"
printf '\nBG1 keyboard: i=forward, comma=reverse, j=left, l=right, k=STOP.\n'
printf 'Commands persist after releasing a key. Press k to stop; Ctrl+C exits.\n\n'
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.4
