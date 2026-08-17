#!/usr/bin/env bash
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
source /opt/ros/jazzy/setup.bash
source /home/robotdev/smart_trolley_sim_ws/install/setup.bash
set -u

log_file=/tmp/smart_trolley_sim_smoke.log
setsid ros2 launch follower_sim sim_follower.launch.py headless:=true rviz:=false enable_follower:=false >"$log_file" 2>&1 &
launch_pid=$!

cleanup() {
  kill -INT -- "-$launch_pid" 2>/dev/null || true
  sleep 3
  kill -TERM -- "-$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT

sleep 12
echo '--- ACTIVE TOPICS ---'
ros2 topic list | sort
echo '--- EKF CONTROLLED MOTION ---'
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
timeout 15 python3 "$script_dir/ekf_motion_test.py"
echo '--- SENSOR RATES ---'
timeout 4 ros2 topic hz /scan || true
timeout 4 ros2 topic hz /imu/data_raw || true
echo '--- LAUNCH ERRORS ---'
grep -E 'ERROR|Error|Traceback' "$log_file" || true
