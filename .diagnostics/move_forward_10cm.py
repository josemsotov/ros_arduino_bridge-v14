#!/usr/bin/env python3
import math
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


TARGET_M = 0.20
SPEED_MPS = -0.14
TIMEOUT_S = 8.0
WHEEL_DIAMETER_M = 0.0332
PPR = 60.0
TARGET_PULSES = TARGET_M * PPR / (math.pi * WHEEL_DIAMETER_M)
STOP_LEAD_PULSES = 14.5
BRAKE_PULSES = max(1.0, TARGET_PULSES - STOP_LEAD_PULSES)


class PositionTest(Node):
    def __init__(self):
        super().__init__('position_test_backward_20cm')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel/gesture', 10)
        self.mode_pub = self.create_publisher(String, '/field/mode_request', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(String, '/encoder_counts', self.encoder_cb, 10)
        self.create_subscription(String, '/field/state', self.field_cb, 10)
        self.x0 = None
        self.y0 = None
        self.distance = 0.0
        self.encoder = 'unavailable'
        self.left = None
        self.right = None
        self.field_mode = 'UNKNOWN'

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.x0 is None:
            self.x0, self.y0 = x, y
        self.distance = math.hypot(x - self.x0, y - self.y0)

    def encoder_cb(self, msg):
        self.encoder = msg.data
        try:
            fields = dict(item.split('=') for item in msg.data.split())
            self.left = int(fields['L'])
            self.right = int(fields['R'])
        except (ValueError, KeyError):
            return

    def field_cb(self, msg):
        try:
            self.field_mode = str(
                json.loads(msg.data).get('effective_mode', 'PAUSE')
            ).upper()
        except (TypeError, ValueError, json.JSONDecodeError):
            self.field_mode = 'PAUSE'

    def mode(self, value):
        msg = String()
        msg.data = value
        self.mode_pub.publish(msg)

    def velocity(self, value):
        msg = Twist()
        msg.linear.x = value
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = PositionTest()
    deadline = time.monotonic() + 4.0
    while rclpy.ok() and (node.x0 is None or node.left is None) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.x0 is None or node.left is None:
        print('RESULT aborted=no_odom_or_encoders')
        node.mode('PAUSE')
        node.destroy_node()
        rclpy.shutdown()
        return

    # Allow DDS/Zenoh discovery and make the mode transition robust.
    mode_deadline = time.monotonic() + 3.0
    while rclpy.ok() and node.field_mode != 'GESTURE' and time.monotonic() < mode_deadline:
        node.mode('GESTURE')
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.08)
    if node.field_mode != 'GESTURE':
        print(f'RESULT aborted=mode_not_gesture mode={node.field_mode}')
        node.mode('PAUSE')
        node.destroy_node()
        rclpy.shutdown()
        return

    # Capture the signed-count baseline only after the direction-owning mode
    # is established, avoiding a discontinuity when a reverse command starts.
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.05)
    left0, right0 = node.left, node.right
    pulse_distance = 0.0
    start = time.monotonic()
    reached = False
    try:
        while rclpy.ok() and time.monotonic() - start < TIMEOUT_S:
            rclpy.spin_once(node, timeout_sec=0.02)
            pulse_distance = (
                abs(node.left - left0) + abs(node.right - right0)
            ) / 2.0
            if pulse_distance >= BRAKE_PULSES:
                reached = True
                break
            node.velocity(SPEED_MPS)
            time.sleep(0.03)
    finally:
        for _ in range(10):
            node.velocity(0.0)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.03)
        for _ in range(5):
            node.mode('PAUSE')
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.05)
        time.sleep(0.2)
        rclpy.spin_once(node, timeout_sec=0.1)
        print(
            f'RESULT reached={int(reached)} odom_m={node.distance:.4f} '
            f'pulses={pulse_distance:.1f} brake={BRAKE_PULSES:.1f} target={TARGET_PULSES:.1f} '
            f'elapsed_s={time.monotonic() - start:.3f} encoders="{node.encoder}"'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
