#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /home/josemsotov/robot_ws/install/setup.bash

for i in 1 2 3 4 5 6 7 8; do
  timeout 2 ros2 topic echo --once /arduino/raw_rx std_msgs/msg/String || true
done
