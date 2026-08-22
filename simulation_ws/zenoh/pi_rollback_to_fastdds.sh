#!/usr/bin/env bash
set -eo pipefail

systemctl --user disable --now smart-trolley-zenoh-router.service || true
rm -f /home/josemsotov/.config/systemd/user/robot-follower.service.d/zenoh.conf
rm -f /home/josemsotov/.config/systemd/user/robot-operator-web.service.d/zenoh.conf
systemctl --user daemon-reload
systemctl --user restart robot-follower.service robot-operator-web.service

echo "Rollback completado: robot-follower y robot-operator-web usan Fast DDS."
