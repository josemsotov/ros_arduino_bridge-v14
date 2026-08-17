"""Fail-closed single-output velocity arbiter."""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from .cmd_vel_mux_core import select_command


class CmdVelMux(Node):
    SOURCE_TOPICS = {
        'stadia': '/cmd_vel/stadia',
        'web': '/cmd_vel/web',
        'follower': '/cmd_vel/follower',
        'gesture': '/cmd_vel/gesture',
        'nav': '/cmd_vel/nav',
    }

    def __init__(self):
        super().__init__('cmd_vel_mux')
        self.declare_parameter('source_timeout_s', 0.35)
        self.declare_parameter('field_timeout_s', 1.5)
        self.declare_parameter('max_linear', 0.45)
        self.declare_parameter('max_angular', 0.90)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.sources = {
            name: {'linear': 0.0, 'angular': 0.0, 'stamp': 0.0}
            for name in self.SOURCE_TOPICS
        }
        self.field_mode = 'PAUSE'
        self.field_stamp = 0.0
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_state = self.create_publisher(String, '/cmd_vel_mux/state', 10)
        for name, topic in self.SOURCE_TOPICS.items():
            self.create_subscription(
                Twist, topic,
                lambda msg, source=name: self.source_cb(source, msg), 10)
        self.create_subscription(String, '/field/state', self.field_cb, 10)
        rate = max(5.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info('cmd_vel mux ready; fail-closed single output active')

    def source_cb(self, source, msg):
        self.sources[source] = {
            'linear': float(msg.linear.x),
            'angular': float(msg.angular.z),
            'stamp': time.monotonic(),
        }

    def field_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            self.field_mode = str(payload.get('effective_mode', 'PAUSE')).upper()
        except (TypeError, ValueError, json.JSONDecodeError):
            self.field_mode = 'PAUSE'
        self.field_stamp = time.monotonic()

    def tick(self):
        now = time.monotonic()
        source_timeout = float(self.get_parameter('source_timeout_s').value)
        field_timeout = float(self.get_parameter('field_timeout_s').value)
        field_fresh = now - self.field_stamp <= field_timeout
        mode = self.field_mode if field_fresh else 'PAUSE'
        sources = {
            name: {
                **value,
                'fresh': value['stamp'] > 0.0
                and now - value['stamp'] <= source_timeout,
            }
            for name, value in self.sources.items()
        }
        result = select_command(
            mode, sources,
            max_linear=float(self.get_parameter('max_linear').value),
            max_angular=float(self.get_parameter('max_angular').value),
        )
        msg = Twist()
        msg.linear.x = result['linear']
        msg.angular.z = result['angular']
        self.pub_cmd.publish(msg)
        state = {
            **result,
            'effective_mode': mode,
            'field_fresh': field_fresh,
            'fresh_sources': sorted(
                name for name, value in sources.items() if value['fresh']),
        }
        self.pub_state.publish(String(data=json.dumps(state)))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
