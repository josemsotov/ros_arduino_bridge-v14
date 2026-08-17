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
from collections import deque
from typing import Any

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String
import uvicorn


PRINTABLE = re.compile(r"^[ -~]{1,96}$")


def now_s() -> float:
    return time.monotonic()


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def stamp_age(stamp: float | None) -> float | None:
    if stamp is None:
        return None
    return round(now_s() - stamp, 3)


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


class SharedState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.motor_status: dict[str, Any] = {}
        self.encoder_counts = ""
        self.odom: dict[str, Any] = {}
        self.follower_debug: dict[str, Any] = {}
        self.follower_state: dict[str, Any] = {}
        self.field_state: dict[str, Any] = {
            "requested_mode": "PAUSE",
            "effective_mode": "PAUSE",
            "reason": "waiting_for_supervisor",
            "monitor_only": True,
        }
        self.gps_status: dict[str, Any] = {}
        self.gesture_status: dict[str, Any] = {}
        self.scan: dict[str, Any] = {}
        self.raw_rx = deque(maxlen=80)
        self.rgb_jpeg: bytes | None = None
        self.depth_jpeg: bytes | None = None
        self.rgb_stamp: float | None = None
        self.depth_stamp: float | None = None
        self.stamps: dict[str, float] = {}
        self.last_cmd: dict[str, float] = {"linear": 0.0, "angular": 0.0}
        self.cmd_vel: dict[str, float] = {"linear": 0.0, "angular": 0.0}
        self.follower_enabled = False
        self.robot_mode: str = "IDLE"
        self.stadia_state: dict[str, Any] = {
            "stadia": "disconnected",
            "mode": "off",
        }

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "motor_status": dict(self.motor_status),
                "encoder_counts": self.encoder_counts,
                "odom": dict(self.odom),
                "follower_debug": dict(self.follower_debug),
                "follower_state": dict(self.follower_state),
                "field_state": dict(self.field_state),
                "gps_status": dict(self.gps_status),
                "gesture_status": dict(self.gesture_status),
                "scan": dict(self.scan),
                "raw_rx": list(self.raw_rx),
                "last_cmd": dict(self.last_cmd),
                "cmd_vel": dict(self.cmd_vel),
                "follower_enabled": self.follower_enabled,
                "robot_mode": self.robot_mode,
                "stadia_state": dict(self.stadia_state),
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
        self.pub_stadia = self.create_publisher(String, "/stadia/control", 10)
        self.pub_stadia_speed = self.create_publisher(
            String, "/stadia/speed_limits", 10
        )
        self.pub_follower_request = self.create_publisher(
            Bool, "/operator/follower_request", 10
        )
        self.pub_field_mode = self.create_publisher(
            String, "/field/mode_request", 10
        )
        self.pub_hand_field_arm = self.create_publisher(
            Bool, "/hand_field/arm", 10
        )
        self.pub_raw = self.create_publisher(String, "/arduino/raw_command", 10)
        self.create_subscription(String, "/motor_status", self.motor_status_cb, 10)
        self.create_subscription(String, "/arduino/raw_rx", self.raw_rx_cb, 10)
        self.create_subscription(String, "/encoder_counts", self.encoder_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(Twist, "/follower/debug", self.follower_debug_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 20)
        self.create_subscription(String, "/follower/state", self.follower_state_cb, 10)
        self.create_subscription(String, "/stadia/state", self.stadia_state_cb, 10)
        self.create_subscription(String, "/field/state", self.field_state_cb, 10)
        self.create_subscription(String, "/gps/status", self.gps_status_cb, 10)
        self.create_subscription(String, "/gesture/status", self.gesture_status_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_subscription(Image, "/camera/rgb/image_raw", self.rgb_cb, 2)
        self.create_subscription(Image, "/camera/depth/image_raw", self.depth_cb, 2)
        self._last_rgb_encode = 0.0
        self._last_depth_encode = 0.0

    def publish_cmd(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = clamp(float(linear), -0.7, 0.7)
        msg.angular.z = clamp(float(angular), -1.5, 1.5)
        self.pub_cmd.publish(msg)
        with self.state.lock:
            self.state.last_cmd = {"linear": msg.linear.x, "angular": msg.angular.z}

    def publish_stop(self) -> None:
        self.publish_cmd(0.0, 0.0)

    def publish_enable(self, enabled: bool) -> None:
        self.pub_enable.publish(Bool(data=bool(enabled)))
        if not enabled:
            self.publish_stop()
        with self.state.lock:
            self.state.follower_enabled = bool(enabled)

    def publish_raw_command(self, command: str) -> None:
        cmd = command.strip()
        if "\n" in cmd or "\r" in cmd or not PRINTABLE.match(cmd):
            raise ValueError("Command must be one printable line, max 96 characters")
        self.pub_raw.publish(String(data=cmd))

    def publish_stadia_speed_limits(self, linear: float, angular: float) -> None:
        payload = {
            "linear": clamp(float(linear), 0.10, 0.40),
            "angular": clamp(float(angular), 0.25, 0.70),
        }
        self.pub_stadia_speed.publish(String(data=json.dumps(payload)))

    def set_robot_mode(self, mode: str) -> None:
        """Set exclusive operating mode. Deactivates conflicting modes."""
        mode = mode.upper()
        if mode not in ("IDLE", "FOLLOWER", "STADIA", "GESTURE"):
            raise ValueError(f"Unknown mode: {mode}")
        if mode == "FOLLOWER":
            self.pub_hand_field_arm.publish(Bool(data=False))
            with self.state.lock:
                stadia_connected = (
                    self.state.stadia_state.get("stadia") == "connected"
                )
            if not stadia_connected:
                self.pub_enable.publish(Bool(data=False))
                self.publish_stop()
                self.pub_stadia.publish(String(data="OFF"))
                raise ValueError(
                    "FOLLOWER requires a physically connected Stadia controller"
                )
            # StadiaNode is the only component allowed to authorize FOLLOWER.
            self.pub_follower_request.publish(Bool(data=True))
            self.publish_stop()
        elif mode == "STADIA":
            self.pub_hand_field_arm.publish(Bool(data=False))
            self.pub_follower_request.publish(Bool(data=False))
            self.pub_enable.publish(Bool(data=False))
            self.publish_stop()
            self.pub_stadia.publish(String(data="STADIA"))
        else:
            self.pub_hand_field_arm.publish(Bool(data=False))
            self.pub_follower_request.publish(Bool(data=False))
            self.pub_enable.publish(Bool(data=False))
            self.publish_stop()
            self.pub_stadia.publish(String(data="OFF"))
        with self.state.lock:
            self.state.follower_enabled = (mode == "FOLLOWER")
            self.state.robot_mode = mode
        field_mode = {
            "IDLE": "PAUSE",
            "STADIA": "STADIA",
            "FOLLOWER": "FOLLOW",
            "GESTURE": "PAUSE",
        }[mode]
        self.publish_field_mode(field_mode)

    def publish_field_mode(self, mode: str) -> None:
        mode = str(mode).strip().upper()
        aliases = {"IDLE": "PAUSE", "MANUAL": "STADIA", "FOLLOWER": "FOLLOW"}
        mode = aliases.get(mode, mode)
        valid = {
            "EMERGENCY_STOP", "PAUSE", "STADIA", "FOLLOW",
            "GO_TO", "RETURN_HOME",
        }
        if mode not in valid:
            raise ValueError(f"Unknown field mode: {mode}")
        self.pub_field_mode.publish(String(data=mode))

    def set_hand_field_armed(self, armed: bool) -> None:
        if not armed:
            self.pub_hand_field_arm.publish(Bool(data=False))
            self.publish_stop()
            self.pub_stadia.publish(String(data="STADIA"))
            with self.state.lock:
                self.state.robot_mode = "STADIA"
            return

        with self.state.lock:
            stadia = dict(self.state.stadia_state)
            rgb_age = stamp_age(self.state.rgb_stamp)
            depth_age = stamp_age(self.state.depth_stamp)
        if stadia.get("stadia") != "connected":
            raise ValueError("Hand-field test requires connected Stadia")
        if rgb_age is None or rgb_age > 1.0:
            raise ValueError("Fresh RGB camera stream is required")
        if depth_age is None or depth_age > 1.5:
            raise ValueError("Fresh depth stream is required")

        self.pub_follower_request.publish(Bool(data=False))
        self.pub_enable.publish(Bool(data=False))
        self.publish_stop()
        self.pub_stadia.publish(String(data="OFF"))

        # Do not arm against a stale "stadia" heartbeat. Wait until the
        # physical controller node confirms that manual output is paused.
        deadline = now_s() + 1.0
        while now_s() < deadline:
            with self.state.lock:
                confirmed_mode = self.state.stadia_state.get("mode")
                still_connected = (
                    self.state.stadia_state.get("stadia") == "connected"
                )
            if not still_connected:
                raise ValueError("Stadia disconnected while arming")
            if confirmed_mode == "off":
                break
            time.sleep(0.05)
        else:
            self.pub_stadia.publish(String(data="STADIA"))
            raise ValueError("Stadia did not confirm safe OFF state")

        self.pub_hand_field_arm.publish(Bool(data=True))
        with self.state.lock:
            self.state.follower_enabled = False
            self.state.robot_mode = "HAND_FIELD_ARMED"

    def stadia_state_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError, ValueError):
            return
        with self.state.lock:
            self.state.stadia_state = dict(payload)
            if payload.get("stadia") != "connected":
                self.state.follower_enabled = False
                self.state.robot_mode = "IDLE"
            elif payload.get("mode") in ("stadia", "off"):
                self.state.follower_enabled = False
                self.state.robot_mode = (
                    "STADIA" if payload.get("mode") == "stadia" else "IDLE"
                )

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

    def cmd_vel_cb(self, msg: Twist) -> None:
        with self.state.lock:
            self.state.cmd_vel = {
                "linear": round(float(msg.linear.x), 4),
                "angular": round(float(msg.angular.z), 4),
            }
            self.state.stamps["cmd_vel"] = now_s()

    def follower_state_cb(self, msg: String) -> None:
        with self.state.lock:
            self.state.follower_state = parse_json_or_raw(msg.data)
            self.state.stamps["follower_state"] = now_s()

    def field_state_cb(self, msg: String) -> None:
        with self.state.lock:
            self.state.field_state = parse_json_or_raw(msg.data)
            self.state.stamps["field_state"] = now_s()

    def gps_status_cb(self, msg: String) -> None:
        with self.state.lock:
            self.state.gps_status = parse_key_values(msg.data)
            self.state.stamps["gps_status"] = now_s()

    def gesture_status_cb(self, msg: String) -> None:
        with self.state.lock:
            self.state.gesture_status = parse_json_or_raw(msg.data)
            self.state.stamps["gesture_status"] = now_s()

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
        if t - self._last_rgb_encode < 0.25:
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
        if t - self._last_depth_encode < 0.75:
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
        if bgr.shape[1] > 480:
            scale = 480 / bgr.shape[1]
            bgr = cv2.resize(bgr, (480, int(bgr.shape[0] * scale)))
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
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
        if colored.shape[1] > 480:
            scale = 480 / colored.shape[1]
            colored = cv2.resize(colored, (480, int(colored.shape[0] * scale)))
        ok, buf = cv2.imencode(".jpg", colored, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        return buf.tobytes() if ok else None
    except Exception:
        return None


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

    async def reboot_host() -> None:
        # Give the HTTP response and final zero-velocity command time to leave
        # the process before systemd terminates the user services.
        await asyncio.sleep(1.0)
        subprocess.Popen(
            ["/usr/bin/systemctl", "reboot", "--no-wall"],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
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
        try:
            node.set_robot_mode("FOLLOWER" if enabled else "IDLE")
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc))
        return {"ok": True, "enabled": enabled}

    @app.post("/api/cmd_vel")
    async def api_cmd_vel(payload: dict[str, Any]) -> dict[str, Any]:
        linear = float(payload.get("linear", 0.0))
        angular = float(payload.get("angular", 0.0))
        node.publish_cmd(linear, angular)
        return {"ok": True, "linear": linear, "angular": angular}

    @app.post("/api/stadia/speed_limits")
    async def api_stadia_speed_limits(payload: dict[str, Any]) -> dict[str, Any]:
        linear = float(payload.get("linear", 0.20))
        angular = float(payload.get("angular", 0.40))
        node.publish_stadia_speed_limits(linear, angular)
        return {
            "ok": True,
            "linear": clamp(linear, 0.10, 0.40),
            "angular": clamp(angular, 0.25, 0.70),
        }

    @app.post("/api/stop")
    async def api_stop() -> dict[str, Any]:
        node.set_robot_mode("IDLE")
        node.publish_field_mode("EMERGENCY_STOP")
        return {"ok": True}

    @app.post("/api/field/mode")
    async def api_field_mode(payload: dict[str, Any]) -> dict[str, Any]:
        mode = str(payload.get("mode", "PAUSE")).upper()
        try:
            node.publish_field_mode(mode)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc))
        return {"ok": True, "requested_mode": mode}

    @app.post("/api/hand-field")
    async def api_hand_field(payload: dict[str, Any]) -> dict[str, Any]:
        armed = bool(payload.get("armed", False))
        try:
            node.set_hand_field_armed(armed)
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc))
        return {"ok": True, "armed": armed}

    @app.post("/api/system/reboot")
    async def api_system_reboot(payload: dict[str, Any]) -> dict[str, Any]:
        if str(payload.get("confirm", "")).strip().upper() != "REBOOT":
            raise HTTPException(
                status_code=400,
                detail="Explicit REBOOT confirmation is required",
            )

        # Fail safely before checking or scheduling the disruptive operation.
        node.set_robot_mode("IDLE")
        node.publish_stop()
        await asyncio.sleep(0.15)

        probe = subprocess.run(
            ["/usr/bin/systemctl", "reboot", "--dry-run", "--no-wall"],
            check=False,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            timeout=5.0,
        )
        if probe.returncode != 0:
            detail = (probe.stderr or "reboot authorization failed").strip()
            raise HTTPException(status_code=503, detail=detail)

        asyncio.create_task(reboot_host())
        return {"ok": True, "status": "rebooting"}

    @app.post("/api/mode")
    async def api_mode(payload: dict[str, Any]) -> dict[str, Any]:
        mode = str(payload.get("mode", "IDLE")).upper()
        try:
            node.set_robot_mode(mode)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc))
        return {"ok": True, "mode": mode}

    @app.post("/api/kiosk/exit")
    async def api_kiosk_exit(request: Request) -> dict[str, Any]:
        client_host = request.client.host if request.client else ""
        if client_host not in ("127.0.0.1", "::1"):
            raise HTTPException(status_code=403, detail="Kiosk exit is only allowed locally")
        node.set_robot_mode("IDLE")
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

    @app.get("/api/frame/rgb.jpg")
    def rgb_frame() -> Response:
        with state.lock:
            data = state.rgb_jpeg
        if not data:
            raise HTTPException(status_code=503, detail="RGB frame not ready")
        return Response(content=data, media_type="image/jpeg")

    async def rgb_mjpeg_frames():
        last_stamp = None
        while True:
            with state.lock:
                data = state.rgb_jpeg
                stamp = state.rgb_stamp
            if data and stamp != last_stamp:
                last_stamp = stamp
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(data)}\r\n\r\n".encode("ascii")
                    + data
                    + b"\r\n"
                )
            await asyncio.sleep(0.05)

    @app.get("/api/stream/rgb.mjpg")
    async def rgb_stream() -> StreamingResponse:
        return StreamingResponse(
            rgb_mjpeg_frames(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
        )

    @app.get("/api/frame/depth.jpg")
    def depth_frame() -> Response:
        with state.lock:
            data = state.depth_jpeg
        if not data:
            raise HTTPException(status_code=503, detail="Depth frame not ready")
        return Response(content=data, media_type="image/jpeg")

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        try:
            while True:
                await ws.send_json(state.snapshot())
                await asyncio.sleep(0.25)
        except WebSocketDisconnect:
            return

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
