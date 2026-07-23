#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/josemsotov/robot_ws/install/setup.bash
ros2 topic pub --once /follower/enable std_msgs/msg/Bool '{data: true}'
