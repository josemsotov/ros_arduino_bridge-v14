#!/usr/bin/env bash
set -euo pipefail

CONFIG="/boot/firmware/config.txt"
BACKUP="/boot/firmware/config.txt.backup-screen"
MARKER="# Built-in DSI touchscreen"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Run with sudo: sudo bash $0" >&2
  exit 1
fi

if [[ ! -f "$CONFIG" ]]; then
  echo "Config file not found: $CONFIG" >&2
  exit 1
fi

cp -a "$CONFIG" "$BACKUP"

if grep -q "dtoverlay=vc4-kms-dsi-7inch" "$CONFIG"; then
  echo "DSI 7-inch overlay is already present."
else
  {
    echo ""
    echo "$MARKER"
    echo "display_auto_detect=1"
    echo "dtoverlay=vc4-kms-dsi-7inch"
  } >> "$CONFIG"
  echo "Added built-in DSI touchscreen overlay."
fi

echo "Backup saved to: $BACKUP"
echo "Reboot required: sudo reboot"
