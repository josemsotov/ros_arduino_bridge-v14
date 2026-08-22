#!/usr/bin/env bash
set -euo pipefail

CONFIG="/boot/firmware/config.txt"
STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP="/boot/firmware/config.txt.backup-ili9881-7inch-${STAMP}"

if [ "$(id -u)" -ne 0 ]; then
  echo "Run with sudo: sudo $0"
  exit 1
fi

cp "$CONFIG" "$BACKUP"

if grep -q '^dtoverlay=vc4-kms-dsi-' "$CONFIG"; then
  sed -i 's/^dtoverlay=vc4-kms-dsi-.*/dtoverlay=vc4-kms-dsi-ili9881-7inch/' "$CONFIG"
else
  printf '\n# Built-in DSI touchscreen\n' >> "$CONFIG"
  printf 'dtoverlay=vc4-kms-dsi-ili9881-7inch\n' >> "$CONFIG"
fi

if ! grep -q '^display_auto_detect=1' "$CONFIG"; then
  printf 'display_auto_detect=1\n' >> "$CONFIG"
fi

echo "Backed up $CONFIG to $BACKUP"
echo "Set built-in touchscreen overlay to vc4-kms-dsi-ili9881-7inch."
echo "Rebooting now..."
systemctl reboot
