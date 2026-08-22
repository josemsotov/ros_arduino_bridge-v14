#!/usr/bin/env bash
set -e
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
source /opt/ros/jazzy/setup.bash
source /home/robotdev/smart_trolley_sim_ws/install/setup.bash

setsid ros2 launch follower_sim sim_follower.launch.py \
  headless:=true rviz:=false enable_follower:=false localization:=false \
  >/tmp/smart_trolley_model_validation.log 2>&1 &
launch_pid=$!
cleanup() {
  kill -INT -- "-$launch_pid" 2>/dev/null || true
  sleep 2
  kill -TERM -- "-$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT

sleep 10
echo '--- MODELS ---'
timeout 8 gz model --list
echo '--- TROLLEY POSE ---'
timeout 8 gz model -m trolley -p
echo '--- TROLLEY LINKS ---'
timeout 8 gz model -m trolley -l
