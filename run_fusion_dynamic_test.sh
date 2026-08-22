#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/josemsotov/robot_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export AMENT_PREFIX_PATH=/home/josemsotov/ros_zenoh_overlay/root/opt/ros/jazzy:/home/josemsotov/robot_ws/install:/opt/ros/jazzy
export LD_LIBRARY_PATH=/home/josemsotov/ros_zenoh_overlay/root/opt/ros/jazzy/lib:/home/josemsotov/robot_ws/install:/home/josemsotov/ros_zenoh_overlay/root/opt/ros/jazzy/opt/zenoh_cpp_vendor/lib:/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/aarch64-linux-gnu
bridge_pid=''
stop_robot() {
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
}
cleanup() {
  stop_robot
  if [ -n "$bridge_pid" ]; then kill "$bridge_pid" >/dev/null 2>&1 || true; wait "$bridge_pid" >/dev/null 2>&1 || true; fi
  systemctl --user start robot-follower.service
}
trap cleanup EXIT INT TERM
systemctl --user stop robot-follower.service
ros2 run arduino_bridge_ros2 arduino_node >/tmp/fusion_bridge.log 2>&1 & bridge_pid=$!
sleep 5
rm -f /tmp/fusion_dynamic.log /tmp/counts_dynamic.log
timeout 18 ros2 topic echo /encoder_fusion/status > /tmp/fusion_dynamic.log & fusion_pid=$!
timeout 18 ros2 topic echo /encoder_counts > /tmp/counts_dynamic.log & counts_pid=$!
sleep 2
for velocity in 0.08 0.15 0.25; do
  timeout 3 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: ${velocity}}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  stop_robot
  sleep 1
done
stop_robot
wait "$fusion_pid" || true
wait "$counts_pid" || true
echo FUSION_SUMMARY
grep 'data:' /tmp/fusion_dynamic.log | sort | uniq -c || true
echo COUNTS_TAIL
grep 'data:' /tmp/counts_dynamic.log | tail -n 8 || true