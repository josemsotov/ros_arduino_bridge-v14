#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/josemsotov/robot_ws/install/setup.bash
for attempt in 1 2 3 4 5; do
  if ros2 node list | grep -qx '/robot_follower'; then
    exec bash /tmp/set_suspended_test_limits.sh
  fi
  sleep 2
done
echo "robot_follower node not discovered" >&2
ros2 node list
exit 1
