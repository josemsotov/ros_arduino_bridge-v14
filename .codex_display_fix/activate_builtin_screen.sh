#!/usr/bin/env bash
set -euo pipefail

CONFIG="/boot/firmware/config.txt"
STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP="/boot/firmware/config.txt.backup-built-in-${STAMP}"

if [ "$(id -u)" -ne 0 ]; then
  echo "Run with sudo: sudo $0"
  exit 1
fi

cp "$CONFIG" "$BACKUP"

if ! grep -q '^display_auto_detect=1' "$CONFIG"; then
  printf '\n# Built-in DSI touchscreen\n' >> "$CONFIG"
  printf 'display_auto_detect=1\n' >> "$CONFIG"
fi

if ! grep -q '^dtoverlay=vc4-kms-dsi-7inch' "$CONFIG"; then
  printf '\n# Built-in DSI touchscreen\n' >> "$CONFIG"
  printf 'dtoverlay=vc4-kms-dsi-7inch\n' >> "$CONFIG"
fi

echo "Backed up $CONFIG to $BACKUP"
echo "Built-in DSI display config is present."
echo "Rebooting now so the kernel can detect DSI..."
systemctl reboot
