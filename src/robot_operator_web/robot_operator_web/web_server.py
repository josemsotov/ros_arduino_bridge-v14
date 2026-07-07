#!/usr/bin/env python3
import argparse
import asyncio
import json
import math
import os
import re
import subprocess
import threading
import time
import urllib.parse
import urllib.request
import uuid
from collections import deque
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, NavSatFix
from std_msgs.msg import Bool, String
import uvicorn


PRINTABLE = re.compile(r"^[ -~]{1,96}$")
NOMINATIM_URL = "https://nominatim.openstreetmap.org/search"
ARDUINO_PORT = "/dev/serial/by-id/usb-Arduino_Srl_Arduino_Mega_85438333036351A040D0-if00"
ARDUINO_BRIDGE_DIR = "/home/josemsotov/robot_ws/src/arduino_bridge"
ROS_SETUP = "source /opt/ros/jazzy/setup.bash; source /home/josemsotov/robot_ws/install/setup.bash 2>/dev/null || true"
FACE_ID_STORE = Path(
    os.environ.get("SMART_TROLLEY_FACE_ID_STORE", str(Path.home() / "robot_ws" / "face_id_profiles.json"))
)


LOCAL_GOLF_COURSES: list[dict[str, Any]] = [
    {
        "id": "dorado-beach-east",
        "name": "Dorado Beach East Course",
        "display_name": "Dorado Beach East Course, Dorado",
        "city": "Dorado",
        "country": "Puerto Rico",
        "lat": 18.4747,
        "lon": -66.4362,
        "source": "local",
        "hole": {"number": 1, "par": 4, "yardage": 389, "front": 375, "center": 389, "back": 403},
    },
    {
        "id": "tpc-dorado-sugarcane",
        "name": "TPC Dorado Beach Sugarcane",
        "display_name": "TPC Dorado Beach Sugarcane Course, Dorado",
        "city": "Dorado",
        "country": "Puerto Rico",
        "lat": 18.4731,
        "lon": -66.4394,
        "source": "local",
        "hole": {"number": 1, "par": 4, "yardage": 410, "front": 396, "center": 410, "back": 424},
    },
    {
        "id": "bahia-beach",
        "name": "Bahia Beach Resort Golf Club",
        "display_name": "Bahia Beach Resort Golf Club, Rio Grande",
        "city": "Rio Grande",
        "country": "Puerto Rico",
        "lat": 18.4081,
        "lon": -65.7989,
        "source": "local",
        "hole": {"number": 1, "par": 4, "yardage": 390, "front": 376, "center": 390, "back": 404},
    },
    {
        "id": "coco-beach",
        "name": "Coco Beach Golf Club",
        "display_name": "Coco Beach Golf Club, Rio Grande",
        "city": "Rio Grande",
        "country": "Puerto Rico",
        "lat": 18.4191,
        "lon": -65.7864,
        "source": "local",
        "hole": {"number": 1, "par": 4, "yardage": 405, "front": 391, "center": 405, "back": 419},
    },
    {
        "id": "royal-isabela",
        "name": "Royal Isabela",
        "display_name": "Royal Isabela, Isabela",
        "city": "Isabela",
        "country": "Puerto Rico",
        "lat": 18.4997,
        "lon": -67.0524,
        "source": "local",
        "hole": {"number": 1, "par": 4, "yardage": 395, "front": 381, "center": 395, "back": 409},
    },
    {
        "id": "palmas-flamboyan",
        "name": "Palmas Athletic Club Flamboyan",
        "display_name": "Palmas Athletic Club Flamboyan Course, Humacao",
        "city": "Humacao",
        "country": "Puerto Rico",
        "lat": 18.0864,
        "lon": -65.7978,
        "source": "local",
        "hole": {"number": 1, "par": 4, "yardage": 400, "front": 386, "center": 400, "back": 414},
    },
]


TEST_DEFINITIONS: dict[str, dict[str, Any]] = {
    "peripherals_status": {
        "label": "Perifericos",
        "description": "Servicios, USB/serial, mando, ROS, GPS, LiDAR y camara. No mueve motores.",
        "kind": "command",
        "command": (
            "echo '== SERVICES =='; "
            "systemctl --user --no-pager --plain status robot-follower.service robot-operator-web.service | sed -n '1,80p' || true; "
            "echo; echo '== USB =='; lsusb || true; "
            "echo; echo '== SERIAL =='; ls -l /dev/serial/by-id /dev/ttyACM* /dev/ttyUSB* /dev/ttyAMA* 2>/dev/null || true; "
            "echo; echo '== INPUT / GAMEPAD =='; ls -l /dev/input/js* /dev/input/event* 2>/dev/null || true; "
            "grep -i -A5 -B2 'stadia\\|gamepad\\|controller\\|google' /proc/bus/input/devices || true; "
            f"echo; echo '== ROS TOPICS =='; {ROS_SETUP}; ros2 topic list | sort; "
            "echo; echo '== GPS =='; timeout 4 ros2 topic echo /gps/status --once || true; "
            "echo; echo '== LIDAR =='; timeout 4 ros2 topic echo /scan --once >/tmp/smart_trolley_scan_once.txt && echo 'scan: OK' || echo 'scan: NO DATA'; "
            "echo; echo '== CAMERA RGB =='; timeout 4 ros2 topic echo /camera/rgb/image_raw --once >/tmp/smart_trolley_rgb_once.txt && echo 'camera_rgb: OK' || echo 'camera_rgb: NO DATA'; "
            "echo; echo '== ARDUINO RAW =='; timeout 4 ros2 topic echo /arduino/raw_rx --once || true"
        ),
    },
    "balance_status": {
        "label": "Balance / MPU status",
        "description": "Envia hb stat al Arduino. No mueve motores.",
        "kind": "raw",
        "command": "hb stat",
    },
    "balance_calibrate": {
        "label": "Calibrar balance",
        "description": "Envia hb cal. Robot quieto y nivelado.",
        "kind": "raw",
        "command": "hb cal",
    },
    "characterize_motors": {
        "label": "Caracterizacion motores",
        "description": "Barre PWM 10..80 y compara PPS de ambos motores.",
        "kind": "command",
        "danger": True,
        "command": (
            "systemctl --user stop robot-follower.service; "
            f"cd {ARDUINO_BRIDGE_DIR}; "
            "python3 characterize_motors.py; "
            "status=$?; systemctl --user start robot-follower.service; exit $status"
        ),
    },
    "hall_diagnostic": {
        "label": "Sensores Hall",
        "description": "Ejecuta prueba P5 de Hall del test_control_suite.",
        "kind": "command",
        "danger": True,
        "command": (
            "systemctl --user stop robot-follower.service; "
            f"cd {ARDUINO_BRIDGE_DIR}; "
            f"printf '\\n5\\nq\\nq\\n' | python3 test_control_suite.py {ARDUINO_PORT}; "
            "status=$?; systemctl --user start robot-follower.service; exit $status"
        ),
    },
    "mpu_diagnostic": {
        "label": "MPU pitch/gyro",
        "description": "Ejecuta prueba P7 de MPU del test_control_suite.",
        "kind": "command",
        "command": (
            "systemctl --user stop robot-follower.service; "
            f"cd {ARDUINO_BRIDGE_DIR}; "
            f"printf '\\n7\\nq\\nq\\n' | python3 test_control_suite.py {ARDUINO_PORT}; "
            "status=$?; systemctl --user start robot-follower.service; exit $status"
        ),
    },
    "pin_status": {
        "label": "Pines / sensores",
        "description": "Ejecuta prueba P10 de estado de pines.",
        "kind": "command",
        "command": (
            "systemctl --user stop robot-follower.service; "
            f"cd {ARDUINO_BRIDGE_DIR}; "
            f"printf '\\n10\\nq\\nq\\n' | python3 test_control_suite.py {ARDUINO_PORT}; "
            "status=$?; systemctl --user start robot-follower.service; exit $status"
        ),
    },
    "gps_status": {
        "label": "GPS fisico",
        "description": "Pide estado GPS al Arduino y revisa /gps/status.",
        "kind": "raw",
        "command": "GPS_STATUS",
    },
    "right_motor": {
        "label": "Motor derecho",
        "description": "Diagnostico dedicado del motor derecho.",
        "kind": "command",
        "danger": True,
        "command": (
            "systemctl --user stop robot-follower.service; "
            f"cd {ARDUINO_BRIDGE_DIR}; "
            f"python3 diag_motor_der.py {ARDUINO_PORT}; "
            "status=$?; systemctl --user start robot-follower.service; exit $status"
        ),
    },
    "lidar_status": {
        "label": "LiDAR",
        "description": "Mide frecuencia del topico /scan.",
        "kind": "command",
        "command": f"{ROS_SETUP}; timeout 6 ros2 topic hz /scan",
    },
    "touch_health_check": {
        "label": "Verificar tactil",
        "description": "Comprueba GT911, DSI-2, kiosk y API. Sin movimiento.",
        "kind": "command",
        "command": "/home/josemsotov/SMART_TROLLEY_INSTALLATION/scripts/touch_health_check.sh",
    },
    "repair_touchscreen": {
        "label": "Reparar tactil",
        "description": "Fuerza rebind del driver Goodix GT911 si el tactil no responde.",
        "kind": "command",
        "command": "sudo /home/josemsotov/SMART_TROLLEY_INSTALLATION/scripts/repair_touchscreen.sh",
    },
    "camera_status": {
        "label": "Camara RGB",
        "description": "Mide frecuencia del topico /camera/rgb/image_raw.",
        "kind": "command",
        "command": f"{ROS_SETUP}; timeout 6 ros2 topic hz /camera/rgb/image_raw",
    },
}


def default_golf_state() -> dict[str, Any]:
    return {
        "course": None,
        "hole": {
            "number": 1,
            "par": 4,
            "yardage": 380,
            "front": None,
            "center": 380,
            "back": None,
            "green": None,
        },
        "position": None,
        "last_shot": None,
        "updated_at": time.time(),
    }


def now_s() -> float:
    return time.monotonic()


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def apply_deadband(value: float, threshold: float) -> float:
    return 0.0 if abs(value) < threshold else value


def stamp_age(stamp: float | None) -> float | None:
    if stamp is None:
        return None
    return round(now_s() - stamp, 3)


def haversine_yards(a: dict[str, Any], b: dict[str, Any]) -> float | None:
    try:
        lat1 = math.radians(float(a["lat"]))
        lon1 = math.radians(float(a["lon"]))
        lat2 = math.radians(float(b["lat"]))
        lon2 = math.radians(float(b["lon"]))
    except (KeyError, TypeError, ValueError):
        return None
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    meters = 6371000.0 * 2 * math.atan2(math.sqrt(h), math.sqrt(1 - h))
    return meters * 1.0936133


def golf_snapshot(golf: dict[str, Any]) -> dict[str, Any]:
    out = json.loads(json.dumps(golf))
    hole = out.get("hole") or {}
    position = out.get("position")
    green = hole.get("green")
    gps_yards = haversine_yards(position, green) if position and green else None
    if gps_yards is not None:
        center = round(gps_yards)
        spread = max(8, int(round(float(hole.get("green_depth") or 28) / 2)))
        hole["center"] = center
        hole["front"] = max(0, center - spread)
        hole["back"] = center + spread
    out["hole"] = hole
    out["distance_source"] = "gps" if gps_yards is not None else "manual"
    return out


def local_golf_courses(query: str = "") -> list[dict[str, Any]]:
    term = query.strip().lower()
    courses = json.loads(json.dumps(LOCAL_GOLF_COURSES))
    if not term:
        return courses
    matches = []
    for course in courses:
        haystack = " ".join(
            str(course.get(key) or "") for key in ("name", "display_name", "city", "country")
        ).lower()
        if term in haystack:
            matches.append(course)
    return matches


def search_golf_courses(query: str, lat: float | None = None, lon: float | None = None) -> list[dict[str, Any]]:
    params: dict[str, Any] = {
        "format": "jsonv2",
        "limit": 8,
        "addressdetails": 1,
        "extratags": 1,
        "q": f"{query} golf course",
    }
    if lat is not None and lon is not None:
        delta = 0.35
        params["viewbox"] = f"{lon - delta},{lat + delta},{lon + delta},{lat - delta}"
        params["bounded"] = 0
    url = f"{NOMINATIM_URL}?{urllib.parse.urlencode(params)}"
    req = urllib.request.Request(
        url,
        headers={"User-Agent": "SmartTrolleyGolfPanel/0.1 (local robot operator)"},
    )
    with urllib.request.urlopen(req, timeout=6) as res:
        payload = json.loads(res.read().decode("utf-8"))
    results = []
    for item in payload:
        try:
            lat_v = float(item["lat"])
            lon_v = float(item["lon"])
        except (KeyError, TypeError, ValueError):
            continue
        address = item.get("address") or {}
        results.append(
            {
                "name": item.get("name") or item.get("display_name", "Golf course").split(",", 1)[0],
                "display_name": item.get("display_name", ""),
                "lat": lat_v,
                "lon": lon_v,
                "city": address.get("city") or address.get("town") or address.get("village") or "",
                "country": address.get("country") or "",
                "source": "openstreetmap",
            }
        )
    return results


def parse_key_values(line: str) -> dict[str, Any]:
    out: dict[str, Any] = {"raw": line}
    for token in line.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        try:
            if value.replace(".", "", 1).replace("-", "", 1).isdigit():
                out[key] = float(value) if "." in value else int(value)
            else:
                out[key] = value
        except ValueError:
            out[key] = value
    return out


def parse_json_or_raw(line: str) -> dict[str, Any]:
    try:
        payload = json.loads(line)
        return payload if isinstance(payload, dict) else {"raw": line}
    except json.JSONDecodeError:
        return {"raw": line}


def load_face_id_profile() -> dict[str, Any]:
    try:
        if FACE_ID_STORE.exists():
            payload = json.loads(FACE_ID_STORE.read_text(encoding="utf-8"))
            return payload if isinstance(payload, dict) else {}
    except Exception:
        return {}
    return {}


def save_face_id_profile(profile: dict[str, Any]) -> None:
    FACE_ID_STORE.parent.mkdir(parents=True, exist_ok=True)
    FACE_ID_STORE.write_text(json.dumps(profile, indent=2, sort_keys=True), encoding="utf-8")


class SharedState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.motor_status: dict[str, Any] = {}
        self.encoder_counts = ""
        self.odom: dict[str, Any] = {}
        self.follower_debug: dict[str, Any] = {}
        self.follower_state: dict[str, Any] = {}
        self.gesture_status: dict[str, Any] = {}
        self.scan: dict[str, Any] = {}
        self.gps_status: dict[str, Any] = {}
        self.golf: dict[str, Any] = default_golf_state()
        self.test_runs: dict[str, dict[str, Any]] = {}
        self.raw_rx = deque(maxlen=80)
        self.rgb_jpeg: bytes | None = None
        self.depth_jpeg: bytes | None = None
        self.rgb_stamp: float | None = None
        self.depth_stamp: float | None = None
        self.stamps: dict[str, float] = {}
        self.last_cmd: dict[str, float] = {"linear": 0.0, "angular": 0.0}
        self.follower_enabled = False
        self.manual_control_active = False
        self.active_clients = 0
        self.last_camera_request = 0.0
        self.face_id: dict[str, Any] = load_face_id_profile()

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "motor_status": dict(self.motor_status),
                "encoder_counts": self.encoder_counts,
                "odom": dict(self.odom),
                "follower_debug": dict(self.follower_debug),
                "follower_state": dict(self.follower_state),
                "gesture_status": dict(self.gesture_status),
                "scan": dict(self.scan),
                "gps_status": dict(self.gps_status),
                "golf": golf_snapshot(self.golf),
                "test_runs": dict(self.test_runs),
                "raw_rx": list(self.raw_rx),
                "last_cmd": dict(self.last_cmd),
                "follower_enabled": self.follower_enabled,
                "manual_control_active": self.manual_control_active,
                "face_id": dict(self.face_id),
                "ages": {name: stamp_age(stamp) for name, stamp in self.stamps.items()},
                "camera": {
                    "rgb_age": stamp_age(self.rgb_stamp),
                    "depth_age": stamp_age(self.depth_stamp),
                    "rgb_ready": self.rgb_jpeg is not None,
                    "depth_ready": self.depth_jpeg is not None,
                },
            }


class OperatorNode(Node):
    def __init__(self, state: SharedState) -> None:
        super().__init__("robot_operator_web")
        self.state = state
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_enable = self.create_publisher(Bool, "/follower/enable", 10)
        self.pub_raw = self.create_publisher(String, "/arduino/raw_command", 10)
        self.pub_gesture_command = self.create_publisher(String, "/gesture/command", 10)
        self._stadia_proc = None
        self.create_subscription(String, "/stadia/control", self._stadia_ctrl_cb, 5)
        self.create_subscription(String, "/motor_status", self.motor_status_cb, 10)
        self.create_subscription(String, "/arduino/raw_rx", self.raw_rx_cb, 10)
        self.create_subscription(String, "/encoder_counts", self.encoder_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(Twist, "/follower/debug", self.follower_debug_cb, 10)
        self.create_subscription(String, "/follower/state", self.follower_state_cb, 10)
        self.create_subscription(String, "/gesture/status", self.gesture_status_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_subscription(NavSatFix, "/fix", self.fix_cb, 10)
        self.create_subscription(String, "/gps/status", self.gps_status_cb, 10)
        self.create_subscription(Image, "/camera/rgb/image_raw", self.rgb_cb, 2)
        self.create_subscription(Image, "/camera/depth/image_raw", self.depth_cb, 2)
        self._last_rgb_encode = 0.0
        self._last_depth_encode = 0.0
        self._last_manual_takeover = 0.0

    def publish_cmd(self, linear: float, angular: float, *, source: str = "manual") -> None:
        msg = Twist()
        msg.linear.x = clamp(apply_deadband(float(linear), 0.02), -0.7, 0.7)
        msg.angular.z = clamp(apply_deadband(float(angular), 0.05), -1.5, 1.5)
        moving = abs(msg.linear.x) > 0.0 or abs(msg.angular.z) > 0.0
        if source == "manual" and moving:
            self._manual_takeover()
        self.pub_cmd.publish(msg)
        with self.state.lock:
            self.state.last_cmd = {"linear": msg.linear.x, "angular": msg.angular.z}
            self.state.manual_control_active = moving

    def publish_stop(self) -> None:
        self.publish_cmd(0.0, 0.0, source="stop")
        with self.state.lock:
            self.state.manual_control_active = False

    def publish_enable(self, enabled: bool) -> None:
        self.pub_enable.publish(Bool(data=bool(enabled)))
        if not enabled:
            self.publish_stop()
        with self.state.lock:
            self.state.follower_enabled = bool(enabled)
            if enabled:
                self.state.manual_control_active = False

    def publish_identity_command(self, command: str) -> None:
        payload = {
            "command": command,
            "source": "touch_face_id",
            "stamp": time.time(),
        }
        self.pub_gesture_command.publish(String(data=json.dumps(payload)))

    def _manual_takeover(self) -> None:
        now = now_s()
        if now - self._last_manual_takeover < 0.25:
            return
        self._last_manual_takeover = now
        self.pub_enable.publish(Bool(data=False))
        with self.state.lock:
            self.state.follower_enabled = False

    def publish_raw_command(self, command: str) -> None:
        cmd = command.strip()
        if "\n" in cmd or "\r" in cmd or not PRINTABLE.match(cmd):
            raise ValueError("Command must be one printable line, max 96 characters")
        self.pub_raw.publish(String(data=cmd))

    def motor_status_cb(self, msg: String) -> None:
        with self.state.lock:
            self.state.motor_status = parse_key_values(msg.data)
            self.state.stamps["motor_status"] = now_s()

    def raw_rx_cb(self, msg: String) -> None:
        with self.state.lock:
            self.state.raw_rx.append(msg.data)
            self.state.stamps["arduino_raw_rx"] = now_s()

    def encoder_cb(self, msg: String) -> None:
        with self.state.lock:
            self.state.encoder_counts = msg.data
            self.state.stamps["encoder_counts"] = now_s()

    def odom_cb(self, msg: Odometry) -> None:
        with self.state.lock:
            self.state.odom = {
                "x": round(msg.pose.pose.position.x, 3),
                "y": round(msg.pose.pose.position.y, 3),
                "linear": round(msg.twist.twist.linear.x, 3),
                "angular": round(msg.twist.twist.angular.z, 3),
            }
            self.state.stamps["odom"] = now_s()

    def follower_debug_cb(self, msg: Twist) -> None:
        with self.state.lock:
            self.state.follower_debug = {
                "target_dist": round(msg.linear.x, 3),
                "target_angle": round(msg.angular.z, 3),
            }
            self.state.stamps["follower_debug"] = now_s()

    def follower_state_cb(self, msg: String) -> None:
        with self.state.lock:
            self.state.follower_state = parse_json_or_raw(msg.data)
            self.state.stamps["follower_state"] = now_s()

    def _stadia_ctrl_cb(self, msg: String) -> None:
        if msg.data == 'ON':
            self._stadia_start()
        elif msg.data == 'OFF':
            self._stadia_stop()

    def _stadia_start(self) -> None:
        import subprocess, os
        if self._stadia_proc and self._stadia_proc.poll() is None:
            return
        script = os.path.expanduser('~/robot_ws/src/arduino_bridge_ros2/stadia_pi.py')
        self._stadia_proc = subprocess.Popen(
            ['python3', script],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.get_logger().info('Stadia started')

    def _stadia_stop(self) -> None:
        import subprocess
        if self._stadia_proc and self._stadia_proc.poll() is None:
            self._stadia_proc.terminate()
        subprocess.run(['pkill', '-f', 'stadia_pi.py'], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._stadia_proc = None
        self.get_logger().info('Stadia stopped')

    def gesture_status_cb(self, msg: String) -> None:
        with self.state.lock:
            self.state.gesture_status = parse_json_or_raw(msg.data)
            self.state.stamps["gesture_status"] = now_s()

    def gps_status_cb(self, msg: String) -> None:
        with self.state.lock:
            self.state.gps_status = parse_key_values(msg.data)
            self.state.stamps["gps_status"] = now_s()

    def fix_cb(self, msg: NavSatFix) -> None:
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        if msg.status.status < 0:
            return
        accuracy = 0.0
        if msg.position_covariance and msg.position_covariance[0] > 0:
            accuracy = math.sqrt(float(msg.position_covariance[0]))
        position = {
            "lat": round(float(msg.latitude), 7),
            "lon": round(float(msg.longitude), 7),
            "accuracy": round(float(accuracy), 2),
            "source": "robot_gps",
            "timestamp": time.time(),
        }
        with self.state.lock:
            self.state.gps_status.update(position)
            self.state.gps_status["fix"] = 1
            self.state.golf["position"] = position
            self.state.golf["updated_at"] = time.time()
            self.state.stamps["fix"] = now_s()

    def scan_cb(self, msg: LaserScan) -> None:
        valid = []
        front = []
        step = max(1, len(msg.ranges) // 180)
        points = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min or r >= msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            valid.append(r)
            if abs(angle) <= math.radians(60):
                front.append(r)
            if i % step == 0:
                points.append([round(angle, 3), round(float(r), 3)])
        with self.state.lock:
            self.state.scan = {
                "valid_count": len(valid),
                "front_min": round(min(front), 3) if front else None,
                "min": round(min(valid), 3) if valid else None,
                "max": round(max(valid), 3) if valid else None,
                "points": points[:240],
            }
            self.state.stamps["scan"] = now_s()

    def rgb_cb(self, msg: Image) -> None:
        t = now_s()
        with self.state.lock:
            active = (self.state.active_clients > 0) or (t - self.state.last_camera_request < 3.0)
        if not active:
            return

        if t - self._last_rgb_encode < 0.18:
            return
        self._last_rgb_encode = t
        jpeg = encode_rgb(msg)
        if jpeg:
            with self.state.lock:
                self.state.rgb_jpeg = jpeg
                self.state.rgb_stamp = t
                self.state.stamps["camera_rgb"] = t

    def depth_cb(self, msg: Image) -> None:
        t = now_s()
        with self.state.lock:
            active = (self.state.active_clients > 0) or (t - self.state.last_camera_request < 3.0)
        if not active:
            return

        if t - self._last_depth_encode < 0.18:
            return
        self._last_depth_encode = t
        jpeg = encode_depth(msg)
        if jpeg:
            with self.state.lock:
                self.state.depth_jpeg = jpeg
                self.state.depth_stamp = t
                self.state.stamps["camera_depth"] = t


def encode_rgb(msg: Image) -> bytes | None:
    try:
        if msg.encoding.lower() == "rgb8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif msg.encoding.lower() == "bgr8":
            bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        elif msg.encoding.lower() == "mono8":
            mono = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            bgr = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        else:
            return None
        if bgr.shape[1] > 640:
            scale = 640 / bgr.shape[1]
            bgr = cv2.resize(bgr, (640, int(bgr.shape[0] * scale)))
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 72])
        return buf.tobytes() if ok else None
    except Exception:
        return None


def encode_depth(msg: Image) -> bytes | None:
    try:
        if msg.encoding.lower() not in ("16uc1", "mono16"):
            return None
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        clipped = np.clip(depth, 300, 4000)
        norm = ((clipped - 300) * (255.0 / 3700.0)).astype(np.uint8)
        colored = cv2.applyColorMap(255 - norm, cv2.COLORMAP_TURBO)
        if colored.shape[1] > 640:
            scale = 640 / colored.shape[1]
            colored = cv2.resize(colored, (640, int(colored.shape[0] * scale)))
        ok, buf = cv2.imencode(".jpg", colored, [int(cv2.IMWRITE_JPEG_QUALITY), 72])
        return buf.tobytes() if ok else None
    except Exception:
        return None


def start_test_run(state: SharedState, test_id: str, command: str) -> dict[str, Any]:
    run_id = uuid.uuid4().hex[:10]
    log_path = f"/tmp/smart_trolley_test_{run_id}.log"
    run = {
        "id": run_id,
        "test_id": test_id,
        "label": TEST_DEFINITIONS[test_id]["label"],
        "status": "running",
        "started_at": time.time(),
        "finished_at": None,
        "returncode": None,
        "log_path": log_path,
    }
    with state.lock:
        state.test_runs[run_id] = dict(run)

    def worker() -> None:
        with open(log_path, "w", encoding="utf-8", errors="replace") as log:
            log.write(f"$ {command}\n\n")
            log.flush()
            proc = subprocess.Popen(
                ["bash", "-lc", command],
                stdout=log,
                stderr=subprocess.STDOUT,
                text=True,
            )
            rc = proc.wait()
        with state.lock:
            current = state.test_runs.get(run_id, dict(run))
            current["status"] = "done" if rc == 0 else "failed"
            current["finished_at"] = time.time()
            current["returncode"] = rc
            state.test_runs[run_id] = current

    threading.Thread(target=worker, daemon=True).start()
    return run


def read_test_log(run: dict[str, Any], max_bytes: int = 12000) -> str:
    path = run.get("log_path")
    if not path or not os.path.exists(path):
        return ""
    with open(path, "rb") as fh:
        try:
            fh.seek(max(0, os.path.getsize(path) - max_bytes))
        except OSError:
            pass
        data = fh.read()
    return data.decode("utf-8", errors="replace")


def create_app(node: OperatorNode, state: SharedState) -> FastAPI:
    share = get_package_share_directory("robot_operator_web")
    static_dir = os.path.join(share, "static")
    app = FastAPI(title="Smart Trolley Operator")
    app.mount("/static", StaticFiles(directory=static_dir, follow_symlink=True), name="static")

    async def close_local_kiosk() -> None:
        await asyncio.sleep(0.4)
        subprocess.run(
            ["pkill", "-TERM", "-u", str(os.getuid()), "-f", "firefox"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    @app.get("/")
    def index() -> FileResponse:
        return FileResponse(os.path.join(static_dir, "index.html"))

    @app.get("/api/state")
    def api_state() -> dict[str, Any]:
        return state.snapshot()

    @app.post("/api/follower")
    async def api_follower(payload: dict[str, Any]) -> dict[str, Any]:
        enabled = bool(payload.get("enabled", False))
        node.publish_enable(enabled)
        return {"ok": True, "enabled": enabled}

    @app.get("/api/identity")
    def api_identity() -> dict[str, Any]:
        with state.lock:
            return {
                "ok": True,
                "face_id": dict(state.face_id),
                "follower_state": dict(state.follower_state),
                "camera": {
                    "rgb_ready": state.rgb_jpeg is not None,
                    "rgb_age": stamp_age(state.rgb_stamp),
                },
            }

    @app.post("/api/identity/enroll")
    async def api_identity_enroll(payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name") or "Jugador").strip()[:40] or "Jugador"
        node.publish_identity_command("FACE_ID_ENROLL")
        with state.lock:
            state.face_id = {
                **dict(state.face_id),
                "name": name,
                "status": "capturing",
                "requested_at": time.time(),
            }
            face_id = dict(state.face_id)
        return {"ok": True, "face_id": face_id}

    @app.post("/api/identity/save")
    async def api_identity_save(payload: dict[str, Any]) -> dict[str, Any]:
        name = str(payload.get("name") or "").strip()[:40]
        with state.lock:
            follower_state = dict(state.follower_state)
            current_name = name or str(state.face_id.get("name") or "Jugador")

        profile = {
            "name": current_name,
            "status": str(follower_state.get("identity_status") or "unknown"),
            "verified": bool(follower_state.get("identity_verified", False)),
            "score": follower_state.get("identity_score"),
            "visual": str(follower_state.get("identity_description") or ""),
            "updated_at": time.time(),
        }
        if not profile["visual"]:
            raise HTTPException(status_code=409, detail="Face ID visual profile not ready")
        save_face_id_profile(profile)
        with state.lock:
            state.face_id = dict(profile)
        return {"ok": True, "face_id": profile}

    @app.post("/api/identity/clear")
    async def api_identity_clear() -> dict[str, Any]:
        node.publish_identity_command("FACE_ID_CLEAR")
        profile = {"status": "cleared", "updated_at": time.time()}
        save_face_id_profile(profile)
        with state.lock:
            state.face_id = dict(profile)
        return {"ok": True, "face_id": profile}

    @app.post("/api/cmd_vel")
    async def api_cmd_vel(payload: dict[str, Any]) -> dict[str, Any]:
        linear = float(payload.get("linear", 0.0))
        angular = float(payload.get("angular", 0.0))
        node.publish_cmd(linear, angular)
        return {"ok": True, "linear": linear, "angular": angular}

    @app.post("/api/stop")
    async def api_stop() -> dict[str, Any]:
        node.publish_enable(False)
        node.publish_stop()
        return {"ok": True}


    @app.post("/api/stadia/start")
    async def api_stadia_start() -> dict[str, Any]:
        node._stadia_start()
        return {"ok": True, "stadia": "running"}

    @app.post("/api/stadia/stop")
    async def api_stadia_stop() -> dict[str, Any]:
        node._stadia_stop()
        return {"ok": True, "stadia": "stopped"}


    @app.post("/api/voice/start")
    async def api_voice_start() -> dict[str, Any]:
        import subprocess, os
        script = os.path.expanduser('~/robot_ws/src/arduino_bridge_ros2/voice_control.py')
        subprocess.Popen(['python3', script, '--server'],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        import time; time.sleep(0.4)
        try:
            import urllib.request
            urllib.request.urlopen('http://127.0.0.1:8765/start', data=b'', timeout=2)
        except Exception: pass
        return {"ok": True, "voice": "started"}

    @app.post("/api/voice/stop")
    async def api_voice_stop() -> dict[str, Any]:
        try:
            import urllib.request
            urllib.request.urlopen('http://127.0.0.1:8765/stop', data=b'', timeout=2)
        except Exception: pass
        return {"ok": True, "voice": "stopped"}

    @app.post("/api/position")
    async def api_position(payload: dict[str, Any]) -> dict[str, Any]:
        import math as _math, threading, time as _time
        dist = float(payload.get("distance_m", 0.0))
        speed = float(payload.get("speed_ms", 0.3))
        if abs(dist) < 0.01:
            return {"ok": False, "error": "distance_too_small"}
        dist_per_pulse = _math.pi * 0.27 / 45  # diameter=0.27m, PPR=45
        pulses_target = abs(dist) / dist_per_pulse
        direction = 1.0 if dist > 0 else -1.0

        def _run() -> None:
            start_enc: tuple | None = None
            deadline = _time.monotonic() + abs(dist) / max(abs(speed), 0.05) + 5.0
            node.publish_cmd(direction * min(abs(speed), 0.5), 0.0, source="position_ctrl")
            while _time.monotonic() < deadline:
                _time.sleep(0.05)
                with state.lock:
                    lines = list(state.raw_rx)
                for line in reversed(lines):
                    if isinstance(line, str) and line.startswith("e "):
                        parts = line.split()
                        if len(parts) >= 3:
                            try:
                                l, r = int(parts[1]), int(parts[2])
                                if start_enc is None:
                                    start_enc = (l, r); break
                                pulses = (abs(l - start_enc[0]) + abs(r - start_enc[1])) / 2
                                if pulses >= pulses_target:
                                    node.publish_stop(); return
                            except ValueError:
                                pass
                        break
            node.publish_stop()

        threading.Thread(target=_run, daemon=True).start()
        return {"ok": True, "distance_m": dist, "speed_ms": speed,
                "est_pulses": round(pulses_target)}

    @app.post("/api/kiosk/exit")
    async def api_kiosk_exit(request: Request) -> dict[str, Any]:
        client_host = request.client.host if request.client else ""
        if client_host not in ("127.0.0.1", "::1"):
            raise HTTPException(status_code=403, detail="Kiosk exit is only allowed locally")
        node.publish_enable(False)
        node.publish_stop()
        asyncio.create_task(close_local_kiosk())
        return {"ok": True}

    @app.post("/api/raw_command")
    async def api_raw_command(payload: dict[str, Any]) -> dict[str, Any]:
        try:
            command = str(payload.get("command", ""))
            node.publish_raw_command(command)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc))
        return {"ok": True, "command": command.strip()}

    @app.get("/api/tests/list")
    def api_tests_list() -> dict[str, Any]:
        tests = []
        for test_id, info in TEST_DEFINITIONS.items():
            tests.append(
                {
                    "id": test_id,
                    "label": info["label"],
                    "description": info["description"],
                    "danger": bool(info.get("danger", False)),
                }
            )
        return {"ok": True, "tests": tests}

    @app.post("/api/tests/run")
    async def api_tests_run(payload: dict[str, Any]) -> dict[str, Any]:
        test_id = str(payload.get("id", "")).strip()
        if test_id not in TEST_DEFINITIONS:
            raise HTTPException(status_code=404, detail="Unknown test")
        definition = TEST_DEFINITIONS[test_id]
        if definition["kind"] == "raw":
            run_id = uuid.uuid4().hex[:10]
            log_path = f"/tmp/smart_trolley_test_{run_id}.log"
            command = str(definition["command"])
            try:
                node.publish_raw_command(command)
                status = "done"
                log_text = f"Sent raw Arduino command: {command}\nCheck /arduino/raw_rx or the live state for the response.\n"
                returncode = 0
            except ValueError as exc:
                status = "failed"
                log_text = str(exc)
                returncode = 1
            with open(log_path, "w", encoding="utf-8", errors="replace") as log:
                log.write(log_text)
            run = {
                "id": run_id,
                "test_id": test_id,
                "label": definition["label"],
                "status": status,
                "started_at": time.time(),
                "finished_at": time.time(),
                "returncode": returncode,
                "log_path": log_path,
            }
            with state.lock:
                state.test_runs[run_id] = dict(run)
            return {"ok": returncode == 0, "run": run}

        run = start_test_run(state, test_id, str(definition["command"]))
        return {"ok": True, "run": run}

    @app.get("/api/tests/run/{run_id}")
    def api_tests_run_status(run_id: str) -> dict[str, Any]:
        with state.lock:
            run = dict(state.test_runs.get(run_id, {}))
            raw_rx = list(state.raw_rx)[-24:]
        if not run:
            raise HTTPException(status_code=404, detail="Run not found")
        return {"ok": True, "run": run, "log": read_test_log(run), "raw_rx": raw_rx}

    @app.get("/api/golf/state")
    def api_golf_state() -> dict[str, Any]:
        with state.lock:
            return {"ok": True, "golf": golf_snapshot(state.golf)}

    @app.post("/api/golf/search")
    async def api_golf_search(payload: dict[str, Any]) -> dict[str, Any]:
        query = str(payload.get("query", "")).strip()
        if len(query) < 2:
            return {"ok": True, "results": []}
        lat = payload.get("lat")
        lon = payload.get("lon")
        try:
            lat_f = float(lat) if lat is not None else None
            lon_f = float(lon) if lon is not None else None
        except (TypeError, ValueError):
            lat_f = lon_f = None
        try:
            results = await asyncio.to_thread(search_golf_courses, query, lat_f, lon_f)
            seen = set()
            merged = []
            for course in local_golf_courses(query) + results:
                key = str(course.get("id") or course.get("display_name") or course.get("name")).lower()
                if key in seen:
                    continue
                seen.add(key)
                merged.append(course)
            results = merged[:10]
            return {"ok": True, "results": results}
        except Exception as exc:
            local_results = local_golf_courses(query)
            return {"ok": bool(local_results), "results": local_results, "error": str(exc)}

    @app.get("/api/golf/courses")
    def api_golf_courses(q: str = "") -> dict[str, Any]:
        return {"ok": True, "results": local_golf_courses(q)}

    @app.post("/api/golf/course")
    async def api_golf_course(payload: dict[str, Any]) -> dict[str, Any]:
        course = payload.get("course")
        if not isinstance(course, dict):
            raise HTTPException(status_code=400, detail="course must be an object")
        safe_course = {
            "name": str(course.get("name") or "Golf course")[:120],
            "display_name": str(course.get("display_name") or "")[:240],
            "lat": course.get("lat"),
            "lon": course.get("lon"),
            "city": str(course.get("city") or "")[:80],
            "country": str(course.get("country") or "")[:80],
            "source": str(course.get("source") or "manual")[:40],
        }
        hole = course.get("hole")
        with state.lock:
            state.golf["course"] = safe_course
            if isinstance(hole, dict):
                current_hole = dict(state.golf.get("hole") or {})
                current_hole.update(
                    {
                        "number": int(hole.get("number") or current_hole.get("number") or 1),
                        "par": int(hole.get("par") or current_hole.get("par") or 4),
                        "yardage": hole.get("yardage", current_hole.get("yardage")),
                        "front": hole.get("front", current_hole.get("front")),
                        "center": hole.get("center", current_hole.get("center")),
                        "back": hole.get("back", current_hole.get("back")),
                        "green": hole.get("green", current_hole.get("green")),
                    }
                )
                state.golf["hole"] = current_hole
            state.golf["updated_at"] = time.time()
            snapshot = golf_snapshot(state.golf)
        return {"ok": True, "golf": snapshot}

    @app.post("/api/golf/hole")
    async def api_golf_hole(payload: dict[str, Any]) -> dict[str, Any]:
        def number(name: str, default: float | None = None) -> float | None:
            value = payload.get(name)
            if value in ("", None):
                return default
            try:
                return float(value)
            except (TypeError, ValueError):
                return default

        hole = {
            "number": int(clamp(number("number", 1) or 1, 1, 18)),
            "par": int(clamp(number("par", 4) or 4, 3, 6)),
            "yardage": int(clamp(number("yardage", 0) or 0, 0, 800)),
            "front": number("front"),
            "center": number("center", number("yardage", 0)),
            "back": number("back"),
            "green_depth": number("green_depth", 28),
            "green": payload.get("green") if isinstance(payload.get("green"), dict) else None,
        }
        with state.lock:
            state.golf["hole"] = hole
            state.golf["updated_at"] = time.time()
            snapshot = golf_snapshot(state.golf)
        return {"ok": True, "golf": snapshot}

    @app.post("/api/golf/position")
    async def api_golf_position(payload: dict[str, Any]) -> dict[str, Any]:
        try:
            position = {
                "lat": float(payload["lat"]),
                "lon": float(payload["lon"]),
                "accuracy": float(payload.get("accuracy", 0) or 0),
                "source": str(payload.get("source") or "browser")[:40],
                "timestamp": time.time(),
            }
        except (KeyError, TypeError, ValueError):
            raise HTTPException(status_code=400, detail="lat and lon are required")
        with state.lock:
            state.golf["position"] = position
            state.golf["updated_at"] = time.time()
            snapshot = golf_snapshot(state.golf)
        return {"ok": True, "golf": snapshot}

    @app.post("/api/golf/shot")
    async def api_golf_shot(payload: dict[str, Any]) -> dict[str, Any]:
        try:
            shot = {
                "club": str(payload.get("club") or "")[:40],
                "yards": float(payload.get("yards", 0) or 0),
                "result": str(payload.get("result") or "")[:120],
                "timestamp": time.time(),
            }
        except (TypeError, ValueError):
            raise HTTPException(status_code=400, detail="invalid shot")
        with state.lock:
            state.golf["last_shot"] = shot
            state.golf["updated_at"] = time.time()
            snapshot = golf_snapshot(state.golf)
        return {"ok": True, "golf": snapshot}

    @app.get("/api/frame/rgb.jpg")
    def rgb_frame() -> Response:
        now = now_s()
        with state.lock:
            state.last_camera_request = now
            data = state.rgb_jpeg
        if not data:
            raise HTTPException(status_code=503, detail="RGB frame not ready")
        return Response(content=data, media_type="image/jpeg")

    @app.get("/api/frame/depth.jpg")
    def depth_frame() -> Response:
        now = now_s()
        with state.lock:
            state.last_camera_request = now
            data = state.depth_jpeg
        if not data:
            raise HTTPException(status_code=503, detail="Depth frame not ready")
        return Response(content=data, media_type="image/jpeg")

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        with state.lock:
            state.active_clients += 1
        try:
            while True:
                await ws.send_json(state.snapshot())
                await asyncio.sleep(0.25)
        except WebSocketDisconnect:
            return
        finally:
            with state.lock:
                state.active_clients = max(0, state.active_clients - 1)

    return app


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8080, type=int)
    args = parser.parse_args()

    rclpy.init()
    state = SharedState()
    node = OperatorNode(state)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = create_app(node, state)
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="info")
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
