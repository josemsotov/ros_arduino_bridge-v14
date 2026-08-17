#!/usr/bin/env bash
set -eo pipefail
overlay=/home/robotdev/ros_zenoh_overlay/root/opt/ros/jazzy
. /opt/ros/jazzy/setup.bash
export AMENT_PREFIX_PATH="$overlay:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$overlay/lib:$overlay/opt/zenoh_cpp_vendor/lib:${LD_LIBRARY_PATH:-}"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=10
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.40.74:7447"]'

echo NODES
ros2 node list | sort
echo SENSOR_TOPICS
ros2 topic list | grep -E '^/(camera|kinect|scan|gps|imu|arduino)' | sort
echo CMD_VEL
timeout 5 ros2 topic echo /cmd_vel --once
echo POINT_RATE
timeout 7 ros2 topic hz /camera/depth/points --window 6 || true
echo LIDAR_RATE
timeout 5 ros2 topic hz /scan --window 6 || true
