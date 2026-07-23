#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/josemsotov/robot_ws/install/setup.bash
ros2 param set /robot_follower max_linear_vel 0.10
ros2 param set /robot_follower max_angular_vel 0.18
ros2 param set /robot_follower min_linear_vel 0.04
ros2 param set /robot_follower velocity_smoothing_alpha 0.20
ros2 param get /robot_follower max_linear_vel
ros2 param get /robot_follower max_angular_vel
ros2 param get /robot_follower min_linear_vel
ros2 param get /robot_follower velocity_smoothing_alpha
