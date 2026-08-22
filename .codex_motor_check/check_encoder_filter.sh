#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /home/josemsotov/robot_ws/install/setup.bash

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" || true
sleep 1

for i in 1 2 3 4 5; do
  echo "SAMPLE $i"
  timeout 3 ros2 topic echo --once /encoder_counts std_msgs/msg/String || true
  sleep 1
done

echo "MOTOR"
timeout 3 ros2 topic echo --once /motor_status std_msgs/msg/String || true
