#!/usr/bin/env bash
set -eo pipefail

for name in arduino_node follower_node web_server; do
  echo "== $name =="
  pids="$(pgrep -f "$name" || true)"
  if [ -z "$pids" ]; then
    echo "not running"
    continue
  fi
  for pid in $pids; do
    echo "PID $pid"
    tr '\0' '\n' < "/proc/$pid/environ" |
      grep -E '^(ROS|RMW|CYCLONE|FAST|AMENT|COLCON|PYTHONPATH)' |
      sort || true
  done
done
