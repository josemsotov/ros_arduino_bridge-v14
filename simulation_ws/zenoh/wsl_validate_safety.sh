#!/usr/bin/env bash
set -eo pipefail
overlay=/home/robotdev/ros_zenoh_overlay/root/opt/ros/jazzy
. /opt/ros/jazzy/setup.bash
export AMENT_PREFIX_PATH="$overlay:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$overlay/lib:$overlay/opt/zenoh_cpp_vendor/lib:${LD_LIBRARY_PATH:-}"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=10
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.40.74:7447"]'

echo ALL_SENSOR_TOPICS
ros2 topic list | grep -Ei 'gps|imu|mpu|sensor|arduino|motor|odom' | sort
echo GPS_STATUS
timeout 5 ros2 topic echo /gps/status --once || true
echo ARDUINO_RX
timeout 5 ros2 topic echo /arduino/raw_rx --once || true
echo MOTOR_STATUS
timeout 5 ros2 topic echo /motor_status --once || true
echo WEB_NODE
ros2 node list | grep robot_operator_web || true
echo STADIA_STATE
timeout 5 ros2 topic echo /stadia/state --once || true
