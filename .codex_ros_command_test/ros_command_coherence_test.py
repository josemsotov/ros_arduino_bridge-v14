#!/usr/bin/env python3
import json
import math
import statistics
import threading
import time
from dataclasses import dataclass, field

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


def parse_kv(line):
    out = {"raw": line}
    for token in line.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        try:
            out[key] = float(value) if "." in value or "e" in value.lower() else int(value)
        except ValueError:
            out[key] = value
    return out


def parse_enc(text):
    if not text:
        return None
    if text.startswith("e "):
        parts = text.split()
        if len(parts) >= 3:
            return int(parts[1]), int(parts[2])
    vals = {}
    for token in text.replace("=", " ").split():
        pass
    parts = text.replace("=", " ").split()
    try:
        return int(parts[1]), int(parts[3])
    except Exception:
        return None


def mean(values):
    values = [v for v in values if v is not None]
    return round(statistics.fmean(values), 4) if values else None


@dataclass
class Sample:
    t: float
    motor: dict | None = None
    enc_filtered: tuple[int, int] | None = None
    enc_raw: tuple[int, int] | None = None
    odom: tuple[float, float] | None = None
    scan_front: float | None = None


class CommandCoherenceNode(Node):
    def __init__(self):
        super().__init__("ros_command_coherence_test")
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_enable = self.create_publisher(Bool, "/follower/enable", 10)
        self.samples: list[Sample] = []
        self.last_motor = None
        self.last_enc_filtered = None
        self.last_enc_raw = None
        self.last_odom = None
        self.last_scan_front = None
        self.lock = threading.Lock()
        self.create_subscription(String, "/motor_status", self.motor_cb, 10)
        self.create_subscription(String, "/encoder_counts", self.enc_cb, 10)
        self.create_subscription(String, "/arduino/raw_rx", self.raw_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_timer(0.05, self.capture)

    def motor_cb(self, msg):
        if msg.data.startswith("T "):
            with self.lock:
                self.last_motor = parse_kv(msg.data)

    def enc_cb(self, msg):
        parsed = parse_enc(msg.data)
        if parsed:
            with self.lock:
                self.last_enc_filtered = parsed

    def raw_cb(self, msg):
        if msg.data.startswith("e "):
            parsed = parse_enc(msg.data)
            if parsed:
                with self.lock:
                    self.last_enc_raw = parsed
        elif msg.data.startswith("T "):
            with self.lock:
                self.last_motor = parse_kv(msg.data)

    def odom_cb(self, msg):
        with self.lock:
            self.last_odom = (float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z))

    def scan_cb(self, msg):
        front = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min or r >= msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= math.radians(60):
                front.append(r)
        with self.lock:
            self.last_scan_front = min(front) if front else None

    def capture(self):
        with self.lock:
            self.samples.append(
                Sample(
                    t=time.monotonic(),
                    motor=dict(self.last_motor) if self.last_motor else None,
                    enc_filtered=self.last_enc_filtered,
                    enc_raw=self.last_enc_raw,
                    odom=self.last_odom,
                    scan_front=self.last_scan_front,
                )
            )

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub_cmd.publish(msg)


def delta_pair(start, end):
    if not start or not end:
        return None
    return end[0] - start[0], end[1] - start[1]


def summarize_step(name, linear, angular, start_t, end_t, samples):
    step = [s for s in samples if start_t <= s.t <= end_t]
    motors = [s.motor for s in step if s.motor]
    first_filtered = next((s.enc_filtered for s in step if s.enc_filtered), None)
    last_filtered = next((s.enc_filtered for s in reversed(step) if s.enc_filtered), None)
    first_raw = next((s.enc_raw for s in step if s.enc_raw), None)
    last_raw = next((s.enc_raw for s in reversed(step) if s.enc_raw), None)
    odoms = [s.odom for s in step if s.odom]
    scans = [s.scan_front for s in step if s.scan_front is not None]

    arduino_lin = mean([m.get("lin") for m in motors])
    arduino_ang = mean([m.get("ang") for m in motors])
    lpwm = mean([m.get("Lpwm") for m in motors])
    rpwm = mean([m.get("Rpwm") for m in motors])
    lrpm = mean([m.get("Lrpm") for m in motors])
    rrpm = mean([m.get("Rrpm") for m in motors])
    odom_lin = mean([o[0] for o in odoms])
    odom_ang = mean([o[1] for o in odoms])

    lin_ok = arduino_lin is not None and abs(arduino_lin - linear) <= 0.025
    ang_ok = arduino_ang is not None and abs(arduino_ang - angular) <= 0.05
    pwm_expected = abs(linear) > 0.001 or abs(angular) > 0.001
    pwm_ok = (lpwm or 0) > 0 or (rpwm or 0) > 0 if pwm_expected else (lpwm in (0, None) and rpwm in (0, None))

    return {
        "step": name,
        "ros_cmd": {"linear": linear, "angular": angular},
        "arduino_echo_avg": {"linear": arduino_lin, "angular": arduino_ang, "ok": lin_ok and ang_ok},
        "motor_avg": {"Lpwm": lpwm, "Rpwm": rpwm, "Lrpm": lrpm, "Rrpm": rrpm, "pwm_ok": pwm_ok},
        "encoder_delta_filtered": delta_pair(first_filtered, last_filtered),
        "encoder_delta_raw": delta_pair(first_raw, last_raw),
        "odom_avg": {"linear": odom_lin, "angular": odom_ang},
        "scan_front_min_avg": mean(scans),
        "samples": len(step),
    }


def main():
    rclpy.init()
    node = CommandCoherenceNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    tests = [
        ("zero_baseline", 0.0, 0.0, 1.5),
        ("forward_slow", 0.06, 0.0, 1.2),
        ("stop_after_forward", 0.0, 0.0, 1.0),
        ("backward_slow", -0.06, 0.0, 1.2),
        ("stop_after_backward", 0.0, 0.0, 1.0),
        ("turn_left_slow", 0.0, 0.18, 1.2),
        ("stop_after_left", 0.0, 0.0, 1.0),
        ("turn_right_slow", 0.0, -0.18, 1.2),
        ("final_stop", 0.0, 0.0, 1.5),
    ]

    results = []
    try:
        node.pub_enable.publish(Bool(data=False))
        for _ in range(5):
            node.publish_cmd(0.0, 0.0)
            time.sleep(0.1)

        for name, linear, angular, duration in tests:
            start = time.monotonic()
            end = start + duration
            while time.monotonic() < end:
                node.publish_cmd(linear, angular)
                time.sleep(0.1)
            results.append(summarize_step(name, linear, angular, start, time.monotonic(), node.samples))

    finally:
        node.pub_enable.publish(Bool(data=False))
        for _ in range(10):
            node.publish_cmd(0.0, 0.0)
            time.sleep(0.05)
        time.sleep(0.5)
        node.destroy_node()
        rclpy.shutdown()

    print(json.dumps({"results": results}, indent=2))


if __name__ == "__main__":
    main()
