#!/usr/bin/env bash
set -eo pipefail

backend="${1:-d3d12}"
workspace="${2:-/home/robotdev/smart_trolley_sim_ws}"
timeout_s="${3:-600}"

source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"
set -u

case "${backend}" in
  d3d12)
    export GALLIUM_DRIVER=d3d12
    export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
    unset LIBGL_ALWAYS_SOFTWARE || true
    ;;
  software)
    export LIBGL_ALWAYS_SOFTWARE=1
    export GALLIUM_DRIVER=llvmpipe
    ;;
  *)
    echo "Unknown backend: ${backend}" >&2
    exit 2
    ;;
esac

cleanup() {
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  sleep 2
  kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

setsid ros2 launch follower_sim nav2_golf_demo.launch.py >"/tmp/trolley_nav2_${backend}.log" 2>&1 &
launch_pid=$!

for _ in $(seq 1 90); do
  if ros2 action info /navigate_to_pose 2>/dev/null | grep -q 'Action servers: 1'; then
    break
  fi
  sleep 1
done

if ! ros2 action info /navigate_to_pose 2>/dev/null | grep -q 'Action servers: 1'; then
  echo "Nav2 action server did not become ready" >&2
  exit 3
fi

python3 "$(dirname "$0")/benchmark_nav2.py" --timeout "${timeout_s}"
gz model -m trolley -p
