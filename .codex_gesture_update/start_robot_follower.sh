#!/usr/bin/env bash
set -euo pipefail

LOCK_FILE="/tmp/robot_follower.lock"
LOG_FILE="/home/josemsotov/robot_ws/log/robot_follower_service.log"
PARAMS_FILE="/home/josemsotov/robot_ws/src/robot_follower/config/follower_params.yaml"
ENV_FILE="/home/josemsotov/robot_ws/config/robot_follower.env"

mkdir -p "$(dirname "$LOG_FILE")"
exec > >(tee -a "$LOG_FILE") 2>&1

exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "$(date --iso-8601=seconds) robot_follower already running; refusing duplicate start"
  exit 20
fi

set +u
source /opt/ros/jazzy/setup.bash
source /home/josemsotov/robot_ws/install/setup.bash
set -u

if [ -f "$ENV_FILE" ]; then
  set -a
  # shellcheck source=/dev/null
  source "$ENV_FILE"
  set +a
fi

ENABLE_ARDUINO="${ENABLE_ARDUINO:-true}"
ENABLE_LIDAR="${ENABLE_LIDAR:-true}"
ENABLE_KINECT="${ENABLE_KINECT:-true}"
ENABLE_GESTURE="${ENABLE_GESTURE:-true}"
ENABLE_FOLLOWER="${ENABLE_FOLLOWER:-true}"
USE_KINECT="${USE_KINECT:-true}"

echo "$(date --iso-8601=seconds) starting robot_follower launch"
exec ros2 launch robot_follower follower.launch.py \
  params_file:="$PARAMS_FILE" \
  enable_arduino:="$ENABLE_ARDUINO" \
  enable_lidar:="$ENABLE_LIDAR" \
  enable_kinect:="$ENABLE_KINECT" \
  enable_gesture:="$ENABLE_GESTURE" \
  enable_follower:="$ENABLE_FOLLOWER" \
  use_kinect:="$USE_KINECT" \
  "$@"
