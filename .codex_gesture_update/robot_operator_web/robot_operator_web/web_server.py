#!/usr/bin/env python3
import argparse
import asyncio
import json
import math
import os
import re
import threading
import time
from collections import deque
from typing import Any

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
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
        self.gesture_status: dict[str, Any] = {}
        self.scan: dict[str, Any] = {}
        self.raw_rx = deque(maxlen=80)
        self.rgb_jpeg: bytes | None = None
        self.depth_jpeg: bytes | None = None
        self.rgb_stamp: float | None = None
        self.depth_stamp: float | None = None
        self.stamps: dict[str, float] = {}
        self.last_cmd: dict[str, float] = {"linear": 0.0, "angular": 0.0}
        self.follower_enabled = False

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
                "raw_rx": list(self.raw_rx),
                "last_cmd": dict(self.last_cmd),
                "follower_enabled": self.follower_enabled,
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
        self.create_subscription(String, "/motor_status", self.motor_status_cb, 10)
        self.create_subscription(String, "/arduino/raw_rx", self.raw_rx_cb, 10)
        self.create_subscription(String, "/encoder_counts", self.encoder_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(Twist, "/follower/debug", self.follower_debug_cb, 10)
        self.create_subscription(String, "/follower/state", self.follower_state_cb, 10)
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


def create_app(node: OperatorNode, state: SharedState) -> FastAPI:
    share = get_package_share_directory("robot_operator_web")
    static_dir = os.path.join(share, "static")
    app = FastAPI(title="Smart Trolley Operator")
    app.mount("/static", StaticFiles(directory=static_dir), name="static")

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
