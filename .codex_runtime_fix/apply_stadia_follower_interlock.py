#!/usr/bin/env python3
"""Make every follower activation pause Stadia, regardless of its source."""

from pathlib import Path
import shutil
import time


path = Path(
    "/home/josemsotov/robot_ws/src/"
    "arduino_bridge_ros2/arduino_bridge_ros2/stadia_node.py"
)
text = path.read_text()

old_subscription = """        self.create_subscription(String, '/stadia/control', self._control_cb, 5)
"""
new_subscription = """        self.create_subscription(String, '/stadia/control', self._control_cb, 5)
        self.create_subscription(Bool, '/follower/enable', self._follower_enable_cb, 5)
"""

old_callback = """    def _control_cb(self, msg: String):
"""
new_callback = """    def _follower_enable_cb(self, msg: Bool):
        # Any subsystem may enable follower (web, gesture, tests). Pause Stadia
        # immediately so two /cmd_vel publishers can never compete.
        if bool(msg.data) and self.control_mode == 'stadia':
            self.control_mode = 'follower'
            self._send_stop()
            self._publish_status('connected')
            self.get_logger().warn(
                'Follower activation detected; Stadia output interlocked'
            )

    def _control_cb(self, msg: String):
"""

for old, new, label in (
    (old_subscription, new_subscription, "follower subscription"),
    (old_callback, new_callback, "interlock callback"),
):
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one match, found {count}")
    text = text.replace(old, new, 1)

stamp = time.strftime("%Y%m%d_%H%M%S")
backup = path.with_name(f"{path.name}.before_follower_interlock_{stamp}")
shutil.copy2(path, backup)
path.write_text(text)
print(f"patched {path}")
print(f"backup  {backup}")
