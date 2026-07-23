#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /home/josemsotov/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

echo "NODES"
ros2 daemon stop || true
sleep 1
ros2 node list --no-daemon | sort

echo
echo "CMDVEL"
ros2 topic info -v /cmd_vel

echo
echo "STATE"
curl -fsS --max-time 8 http://127.0.0.1:8080/api/state |
  python3 -m json.tool |
  sed -n '1,70p'

echo
echo "NO SPONTANEOUS CMDVEL FOR 4S"
timeout 4 ros2 topic echo /cmd_vel geometry_msgs/msg/Twist || true
