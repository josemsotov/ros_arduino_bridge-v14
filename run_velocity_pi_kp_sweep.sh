#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/josemsotov/robot_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export AMENT_PREFIX_PATH=/home/josemsotov/ros_zenoh_overlay/root/opt/ros/jazzy:/home/josemsotov/robot_ws/install:/opt/ros/jazzy
export LD_LIBRARY_PATH=/home/josemsotov/ros_zenoh_overlay/root/opt/ros/jazzy/lib:/home/josemsotov/robot_ws/install:/home/josemsotov/ros_zenoh_overlay/root/opt/ros/jazzy/opt/zenoh_cpp_vendor/lib:/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/aarch64-linux-gnu
bridge_pid=''
send_raw() { ros2 topic pub --once /arduino/raw_command std_msgs/msg/String "{data: '$1'}" >/dev/null 2>&1 || true; }
stop_robot() { ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true; }
cleanup() { stop_robot; send_raw 'k off'; if [ -n "$bridge_pid" ]; then kill -- "-$bridge_pid" >/dev/null 2>&1 || true; fi; sleep 2; systemctl --user start robot-follower.service; }
trap cleanup EXIT INT TERM
systemctl --user stop robot-follower.service
setsid ros2 run arduino_bridge_ros2 arduino_node >/tmp/velocity_pi_sweep_bridge.log 2>&1 & bridge_pid=$!
sleep 5
for kp in 0.25 0.40 0.60; do
  send_raw "k ${kp} 0.0"; send_raw 'k on'
  timeout 6 ros2 topic echo /motor_status > "/tmp/velocity_pi_kp_${kp}.log" & echo_pid=$!
  sleep 1
  timeout 4 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  stop_robot; wait "$echo_pid" || true; sleep 1
done
stop_robot; send_raw 'k off'