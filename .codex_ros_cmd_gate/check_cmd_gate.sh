#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /home/josemsotov/robot_ws/install/setup.bash

echo "CMDVEL publishers"
ros2 topic info -v /cmd_vel | sed -n '1,80p'

echo "No spontaneous /cmd_vel for 4 seconds"
timeout 4 ros2 topic echo /cmd_vel geometry_msgs/msg/Twist || true

echo "Send tiny manual command through API"
curl -fsS --max-time 5 \
  -X POST http://127.0.0.1:8080/api/cmd_vel \
  -H 'Content-Type: application/json' \
  -d '{"linear":0.01,"angular":0.03}'
echo

echo "State"
curl -fsS --max-time 5 http://127.0.0.1:8080/api/state | python3 -m json.tool | sed -n '1,80p'
