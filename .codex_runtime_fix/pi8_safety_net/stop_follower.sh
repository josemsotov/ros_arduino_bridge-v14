#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/josemsotov/robot_ws/install/setup.bash
ros2 topic pub --rate 5 --times 3 /follower/enable std_msgs/msg/Bool '{data: false}'
ros2 topic pub --rate 10 --times 5 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
