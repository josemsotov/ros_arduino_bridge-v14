#!/usr/bin/env bash
set -eo pipefail

overlay=/home/josemsotov/ros_zenoh_overlay/root/opt/ros/jazzy
. /opt/ros/jazzy/setup.bash
. /home/josemsotov/robot_ws/install/setup.bash
export AMENT_PREFIX_PATH="$overlay:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$overlay/lib:$overlay/opt/zenoh_cpp_vendor/lib:${LD_LIBRARY_PATH:-}"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=10
exec /home/josemsotov/robot_ws/install/lib/kinect_ros2/kinect_node --ros-args -r __node:=kinect_node_zenoh
