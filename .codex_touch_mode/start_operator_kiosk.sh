#!/usr/bin/env bash
set -euo pipefail

BASE_URL="${SMART_TROLLEY_TOUCH_BASE_URL:-http://127.0.0.1:8080}"
URL="${SMART_TROLLEY_TOUCH_URL:-$BASE_URL/static/touch.html}"
LOG="/home/josemsotov/SMART_TROLLEY_INSTALLATION/operator_kiosk.log"

mkdir -p "$(dirname "$LOG")"
exec >>"$LOG" 2>&1

echo "$(date --iso-8601=seconds) starting touch kiosk for $URL"

for _ in $(seq 1 90); do
  if curl -fsS "$BASE_URL/api/state" >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

export MOZ_ENABLE_WAYLAND=1
if pgrep -u "$(id -u)" -f "/firefox/firefox" >/dev/null 2>&1; then
  echo "$(date --iso-8601=seconds) Firefox already running; opening operator window"
  exec firefox --new-window "$URL"
fi

exec firefox --kiosk "$URL"
