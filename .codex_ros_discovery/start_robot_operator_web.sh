#!/usr/bin/env bash
set -euo pipefail

LOCK_FILE="/tmp/robot_operator_web.lock"
LOG_FILE="/home/josemsotov/robot_ws/log/robot_operator_web.log"
ENV_FILE="/home/josemsotov/robot_ws/config/robot_follower.env"
HOST="${ROBOT_OPERATOR_HOST:-0.0.0.0}"
PORT="${ROBOT_OPERATOR_PORT:-8080}"

mkdir -p "$(dirname "$LOG_FILE")"
exec > >(tee -a "$LOG_FILE") 2>&1

exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "$(date --iso-8601=seconds) robot_operator_web already running; refusing duplicate start"
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

echo "$(date --iso-8601=seconds) starting robot_operator_web on ${HOST}:${PORT}"
exec ros2 run robot_operator_web web_server --host "$HOST" --port "$PORT"
