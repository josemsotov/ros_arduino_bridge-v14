#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/josemsotov/SMART_TROLLEY_INSTALLATION"
BACKUP_DIR="$ROOT/backups"
TS="$(date +%Y%m%d_%H%M%S)"
DEST="$BACKUP_DIR/SMART_TROLLEY_PI_INSTALLATION_$TS.tar.gz"

mkdir -p "$BACKUP_DIR"

tar -czf "$DEST" \
  -C /home/josemsotov \
  robot_ws/src \
  robot_ws/config \
  robot_ws/scripts \
  .config/systemd/user/robot-follower.service \
  .config/systemd/user/robot-operator-web.service \
  SMART_TROLLEY_INSTALLATION/README_INSTALLATION.md \
  SMART_TROLLEY_INSTALLATION/RESTORE_STEPS.md \
  SMART_TROLLEY_INSTALLATION/scripts/backup_smart_trolley_pi.sh

sha256sum "$DEST" > "$DEST.sha256"
echo "$DEST"
