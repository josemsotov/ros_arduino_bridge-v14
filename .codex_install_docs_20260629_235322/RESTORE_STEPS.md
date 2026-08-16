# Smart Trolley Restore Notes

1. Restore the archive under `/home/josemsotov`.
2. Confirm files exist:
   - `/home/josemsotov/robot_ws/src`
   - `/home/josemsotov/robot_ws/config`
   - `/home/josemsotov/robot_ws/scripts`
   - `/home/josemsotov/.config/systemd/user/robot-follower.service`
   - `/home/josemsotov/.config/systemd/user/robot-operator-web.service`
3. Rebuild ROS packages:

```bash
cd /home/josemsotov/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --merge-install --symlink-install
```

4. Reload and enable services:

```bash
systemctl --user daemon-reload
systemctl --user enable robot-follower.service robot-operator-web.service
systemctl --user restart robot-follower.service robot-operator-web.service
```

5. Open the operator UI:

```text
http://192.168.40.32:8080
```
