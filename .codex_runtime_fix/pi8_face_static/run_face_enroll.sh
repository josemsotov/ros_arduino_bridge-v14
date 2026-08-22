#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/josemsotov/robot_ws/install/setup.bash
ros2 topic pub --rate 1 --times 3 \
  /gesture/command std_msgs/msg/String \
  '{data: FACE_STATIC_ENROLL}'
