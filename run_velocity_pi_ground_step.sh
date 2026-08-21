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
setsid ros2 run arduino_bridge_ros2 arduino_node >/tmp/ground_pi_bridge.log 2>&1 & bridge_pid=$!
sleep 5
send_raw 'r'; send_raw 'k 0.25 0.0'; send_raw 'k on'
timeout 5 ros2 topic echo /motor_status >/tmp/ground_pi_motor.log & mpid=$!
timeout 5 ros2 topic echo /encoder_counts >/tmp/ground_pi_counts.log & cpid=$!
timeout 5 ros2 topic echo /encoder_fusion/status >/tmp/ground_pi_fusion.log & fpid=$!
sleep 1
timeout 1 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.06}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
stop_robot
wait "$mpid" || true; wait "$cpid" || true; wait "$fpid" || true
stop_robot; send_raw 'k off'
echo MOTOR; grep 'data:' /tmp/ground_pi_motor.log || true
echo COUNTS; grep 'data:' /tmp/ground_pi_counts.log | tail -n 5 || true
echo FUSION; grep 'data:' /tmp/ground_pi_fusion.log | sort | uniq -c || true