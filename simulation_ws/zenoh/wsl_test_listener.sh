#!/usr/bin/env bash
set -eo pipefail
overlay=/home/robotdev/ros_zenoh_overlay/root/opt/ros/jazzy
. /opt/ros/jazzy/setup.bash
export AMENT_PREFIX_PATH="$overlay:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$overlay/lib:$overlay/opt/zenoh_cpp_vendor/lib:${LD_LIBRARY_PATH:-}"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=10
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.40.74:7447"]'
exec ros2 run demo_nodes_cpp listener --ros-args -r chatter:=/zenoh_test
