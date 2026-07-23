#!/usr/bin/env bash
set -euo pipefail

export XDG_RUNTIME_DIR=/run/user/1000
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus

serial="$(
  gdbus call --session \
    --dest org.gnome.Mutter.DisplayConfig \
    --object-path /org/gnome/Mutter/DisplayConfig \
    --method org.gnome.Mutter.DisplayConfig.GetCurrentState |
  sed -E 's/^\(uint32 ([0-9]+),.*/\1/'
)"

if [ -f "$HOME/.config/monitors.xml" ] && [ ! -f "$HOME/.config/monitors.xml.backup-smart-trolley-display" ]; then
  cp "$HOME/.config/monitors.xml" "$HOME/.config/monitors.xml.backup-smart-trolley-display"
fi

gdbus call --session \
  --dest org.gnome.Mutter.DisplayConfig \
  --object-path /org/gnome/Mutter/DisplayConfig \
  --method org.gnome.Mutter.DisplayConfig.ApplyMonitorsConfig \
  "$serial" \
  2 \
  "[(0, 0, 1.0, 2, true, [('DSI-2', '720x1280@60.038', {})]), (720, 200, 1.0, 0, false, [('HDMI-2', '1920x1080@60.000', {})]), (2640, 200, 1.0, 0, false, [('HDMI-1', '1920x1080@60.000', {})])]" \
  "{}" >/dev/null
