"""Passive observer for the current shared /cmd_vel control path."""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from .cmd_vel_guard_core import evaluate_velocity_guard


def _json_object(text):
    try:
        value = json.loads(text)
        return value if isinstance(value, dict) else {}
    except (TypeError, ValueError, json.JSONDecodeError):
        return {}


def _tokens(text):
    result = {}
    for token in str(text).split()[1:]:
        if '=' in token:
            key, value = token.split('=', 1)
            result[key] = value
    return result


class CmdVelGuard(Node):
    def __init__(self):
        super().__init__('cmd_vel_guard')
        self.declare_parameter('monitor_only', True)
        self.declare_parameter('cmd_timeout_s', 0.6)
        self.declare_parameter('max_linear', 0.45)
        self.declare_parameter('max_angular', 0.90)

        self.field = {}
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.last_cmd = 0.0
        self.motor = {}

        self.pub_state = self.create_publisher(String, '/cmd_vel_guard/state', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 20)
        self.create_subscription(String, '/field/state', self.field_cb, 10)
        self.create_subscription(String, '/motor_status', self.motor_cb, 10)
        self.create_timer(0.5, self.publish_state)
        self.get_logger().info(
            'cmd_vel guard active in monitor-only mode; control path unchanged')

    def cmd_cb(self, msg):
        self.cmd_linear = float(msg.linear.x)
        self.cmd_angular = float(msg.angular.z)
        self.last_cmd = time.monotonic()

    def field_cb(self, msg):
        self.field = _json_object(msg.data)

    def motor_cb(self, msg):
        self.motor = _tokens(msg.data)

    def _motor_active(self):
        try:
            values = (
                float(self.motor.get('Lpwm', 0)),
                float(self.motor.get('Rpwm', 0)),
                float(self.motor.get('Lrpm', 0)),
                float(self.motor.get('Rrpm', 0)),
            )
        except (TypeError, ValueError):
            return True
        return any(abs(value) > 1e-4 for value in values)

    def publish_state(self):
        now = time.monotonic()
        cmd_age = None if not self.last_cmd else now - self.last_cmd
        timeout = float(self.get_parameter('cmd_timeout_s').value)
        publishers = self.get_publishers_info_by_topic('/cmd_vel')
        result = evaluate_velocity_guard(
            effective_mode=self.field.get('effective_mode', 'PAUSE'),
            linear=self.cmd_linear,
            angular=self.cmd_angular,
            cmd_fresh=cmd_age is not None and cmd_age <= timeout,
            publisher_count=len(publishers),
            motor_active=self._motor_active(),
            max_linear=float(self.get_parameter('max_linear').value),
            max_angular=float(self.get_parameter('max_angular').value),
        )
        payload = {
            **result,
            'monitor_only': bool(self.get_parameter('monitor_only').value),
            'command_output_enabled': False,
            'effective_mode': self.field.get('effective_mode', 'PAUSE'),
            'cmd': {
                'linear': round(self.cmd_linear, 4),
                'angular': round(self.cmd_angular, 4),
                'age_s': None if cmd_age is None else round(cmd_age, 3),
            },
            'publisher_count': len(publishers),
            'publishers': sorted({info.node_name for info in publishers}),
            'motor_active': self._motor_active(),
        }
        self.pub_state.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
