#!/usr/bin/env python3
"""Apply the Smart Trolley cmd_vel arbitration and watchdog hotfix."""

from pathlib import Path
import shutil
import time


ROOT = Path("/home/josemsotov/robot_ws/src")
STADIA = ROOT / "arduino_bridge_ros2/arduino_bridge_ros2/stadia_node.py"
ARDUINO = ROOT / "arduino_bridge_ros2/arduino_bridge_ros2/arduino_node.py"
WEB = ROOT / "robot_operator_web/robot_operator_web/web_server.py"
STAMP = time.strftime("%Y%m%d_%H%M%S")


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one match, found {count}")
    return text.replace(old, new, 1)


def patch(path: Path, edits: list[tuple[str, str, str]]) -> None:
    original = path.read_text()
    updated = original
    for old, new, label in edits:
        updated = replace_once(updated, old, new, label)
    backup = path.with_name(f"{path.name}.before_cmd_vel_failsafe_{STAMP}")
    shutil.copy2(path, backup)
    path.write_text(updated)
    print(f"patched {path}")
    print(f"backup  {backup}")


patch(
    STADIA,
    [
        (
            """                self._dev = None
                self.smooth_lin = 0.0
                self.smooth_ang = 0.0
                self._publish_status('disconnected')
""",
            """                # A lost controller must actively overwrite the last non-zero command.
                self._send_stop()
                self.axis.clear()
                self._dev = None
                self._publish_status('disconnected')
""",
            "stadia disconnect stop",
        ),
        (
            """    def _set_stadia_mode(self):
        self.control_mode = 'stadia'
        self.pub_enable.publish(Bool(data=False))
        self._send_stop()
""",
            """    def _set_stadia_mode(self):
        self.control_mode = 'stadia'
        self.pub_enable.publish(Bool(data=False))
        # Refresh the physical stick position so an old cached axis value
        # cannot restart motion when returning from FOLLOWER or IDLE.
        if self._dev is not None:
            try:
                from evdev import ecodes
                self.axis[ecodes.ABS_X] = self._dev.absinfo(ecodes.ABS_X).value
                self.axis[ecodes.ABS_Y] = self._dev.absinfo(ecodes.ABS_Y).value
            except Exception as exc:
                self.get_logger().warn(f'No se pudieron refrescar ejes Stadia: {exc}')
                self.axis.clear()
        self._send_stop()
""",
            "stadia mode axis refresh",
        ),
    ],
)

patch(
    ARDUINO,
    [
        (
            """        self.declare_parameter('ppr',        45)      # pulsos por revolución
""",
            """        self.declare_parameter('ppr',        45)      # pulsos por revolución
        self.declare_parameter('cmd_timeout', 0.50)  # seconds without /cmd_vel
""",
            "arduino timeout parameter",
        ),
        (
            """        self.ppr        = self.get_parameter('ppr').value
""",
            """        self.ppr        = self.get_parameter('ppr').value
        self.cmd_timeout = max(0.10, float(self.get_parameter('cmd_timeout').value))
""",
            "arduino timeout value",
        ),
        (
            """        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
""",
            """        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.last_cmd_time = time.monotonic()
        self.cmd_watchdog_stopped = True
""",
            "arduino watchdog state",
        ),
        (
            """        self.create_timer(0.05, self.request_encoders)
""",
            """        self.create_timer(0.05, self.request_encoders)
        self.create_timer(0.10, self.cmd_watchdog_cb)
""",
            "arduino watchdog timer",
        ),
        (
            """        self.last_cmd_linear = v
        self.last_cmd_angular = w
        self._send(f'v {v:.4f} {w:.4f}')

    def raw_command_cb(self, msg: String):
""",
            """        self.last_cmd_linear = v
        self.last_cmd_angular = w
        self.last_cmd_time = time.monotonic()
        self.cmd_watchdog_stopped = abs(v) < 1e-6 and abs(w) < 1e-6
        self._send(f'v {v:.4f} {w:.4f}')

    def cmd_watchdog_cb(self):
        age = time.monotonic() - self.last_cmd_time
        if age <= self.cmd_timeout or self.cmd_watchdog_stopped:
            return
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.cmd_watchdog_stopped = True
        self._send('v 0.0 0.0')
        self.get_logger().warn(
            f'cmd_vel timeout after {age:.2f}s; motors stopped'
        )

    def raw_command_cb(self, msg: String):
""",
            "arduino watchdog callback",
        ),
    ],
)

patch(
    WEB,
    [
        (
            """        self.robot_mode: str = "IDLE"  # Exclusive mode: IDLE | FOLLOWER | STADIA | GESTURE
""",
            """        self.robot_mode: str = "STADIA"  # Default until another exclusive mode is selected
""",
            "web default mode",
        ),
        (
            """        self.pub_enable = self.create_publisher(Bool, "/follower/enable", 10)
        self.pub_raw = self.create_publisher(String, "/arduino/raw_command", 10)
""",
            """        self.pub_enable = self.create_publisher(Bool, "/follower/enable", 10)
        self.pub_stadia = self.create_publisher(String, "/stadia/control", 10)
        self.pub_raw = self.create_publisher(String, "/arduino/raw_command", 10)
""",
            "web stadia publisher",
        ),
        (
            """        if mode == "FOLLOWER":
            self.pub_enable.publish(Bool(data=True))
        else:
            self.pub_enable.publish(Bool(data=False))
            self.publish_stop()
""",
            """        if mode == "FOLLOWER":
            # Pause Stadia first so it cannot overwrite follower /cmd_vel.
            self.pub_stadia.publish(String(data="FOLLOWER"))
            self.publish_stop()
            self.pub_enable.publish(Bool(data=True))
        elif mode == "STADIA":
            self.pub_enable.publish(Bool(data=False))
            self.publish_stop()
            self.pub_stadia.publish(String(data="STADIA"))
        else:
            self.pub_enable.publish(Bool(data=False))
            self.publish_stop()
            self.pub_stadia.publish(String(data="OFF"))
""",
            "web exclusive mode",
        ),
        (
            """        enabled = bool(payload.get("enabled", False))
        node.publish_enable(enabled)
        with state.lock:
            state.robot_mode = "FOLLOWER" if enabled else "IDLE"
        return {"ok": True, "enabled": enabled}
""",
            """        enabled = bool(payload.get("enabled", False))
        node.set_robot_mode("FOLLOWER" if enabled else "IDLE")
        return {"ok": True, "enabled": enabled}
""",
            "web follower endpoint arbitration",
        ),
        (
            """        node.publish_enable(False)
        node.publish_stop()
        with state.lock:
            state.robot_mode = "IDLE"
        return {"ok": True}
""",
            """        node.set_robot_mode("IDLE")
        return {"ok": True}
""",
            "web stop endpoint arbitration",
        ),
        (
            """        node.publish_enable(False)
        node.publish_stop()
        asyncio.create_task(close_local_kiosk())
""",
            """        node.set_robot_mode("IDLE")
        asyncio.create_task(close_local_kiosk())
""",
            "web kiosk stop arbitration",
        ),
    ],
)

print("cmd_vel failsafe patch complete")
