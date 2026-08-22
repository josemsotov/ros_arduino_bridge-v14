"""Monitor-only field operating-mode supervisor."""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .field_supervisor_core import normalize_request, select_effective_mode


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


class FieldSupervisor(Node):
    def __init__(self):
        super().__init__('field_supervisor')
        self.declare_parameter('monitor_only', True)
        self.declare_parameter('state_timeout_s', 2.0)
        self.declare_parameter('navigation_ready', False)
        self.declare_parameter('initial_mode', 'PAUSE')
        self.declare_parameter('gps_min_satellites', 6)
        self.declare_parameter('gps_max_hdop', 2.5)
        self.declare_parameter('gps_stable_time_s', 10.0)

        self.requested = normalize_request(
            self.get_parameter('initial_mode').value)
        self.emergency_latched = False
        self.stadia = {}
        self.follower = {}
        self.gps = {}
        self.last_seen = {'stadia': 0.0, 'follower': 0.0, 'gps': 0.0,
                          'motor': 0.0}
        self.motor_status = ''
        self.gps_quality_since = None

        self.pub_state = self.create_publisher(String, '/field/state', 10)
        self.create_subscription(
            String, '/field/mode_request', self.request_cb, 10)
        self.create_subscription(String, '/stadia/state', self.stadia_cb, 10)
        self.create_subscription(
            String, '/follower/state', self.follower_cb, 10)
        self.create_subscription(String, '/gps/status', self.gps_cb, 10)
        self.create_subscription(
            String, '/motor_status', self.motor_status_cb, 10)
        self.create_timer(0.5, self.publish_state)
        self.get_logger().info(
            'Field supervisor active in monitor-only mode; no motor output')

    def _mark(self, source):
        self.last_seen[source] = time.monotonic()

    def request_cb(self, msg):
        try:
            request = normalize_request(msg.data)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return
        if request == 'EMERGENCY_STOP':
            self.emergency_latched = True
        elif request == 'PAUSE':
            # Deliberate PAUSE is the only reset action in monitor-only mode.
            self.emergency_latched = False
        self.requested = request
        self.publish_state()

    def stadia_cb(self, msg):
        self.stadia = _json_object(msg.data)
        self._mark('stadia')

    def follower_cb(self, msg):
        self.follower = _json_object(msg.data)
        self._mark('follower')

    def gps_cb(self, msg):
        self.gps = _tokens(msg.data)
        self._mark('gps')
        fix = str(self.gps.get('fix', '')).lower() in ('1', 'true')
        try:
            satellites = int(float(self.gps.get('sats', 0)))
            hdop = float(self.gps.get('hdop', 99.9))
        except (TypeError, ValueError):
            satellites, hdop = 0, 99.9
        instant_ok = (
            fix
            and satellites >= int(
                self.get_parameter('gps_min_satellites').value)
            and 0.0 < hdop <= float(
                self.get_parameter('gps_max_hdop').value)
        )
        if instant_ok:
            if self.gps_quality_since is None:
                self.gps_quality_since = time.monotonic()
        else:
            self.gps_quality_since = None

    def motor_status_cb(self, msg):
        self.motor_status = msg.data
        self._mark('motor')

    def _fresh(self, source, now):
        timeout = float(self.get_parameter('state_timeout_s').value)
        return now - self.last_seen[source] <= timeout

    def publish_state(self):
        now = time.monotonic()
        fresh = {name: self._fresh(name, now) for name in self.last_seen}
        stadia_connected = self.stadia.get('stadia') == 'connected'
        gps_fix = str(self.gps.get('fix', '')).lower() in ('1', 'true')
        try:
            gps_satellites = int(float(self.gps.get('sats', 0)))
            gps_hdop = float(self.gps.get('hdop', 99.9))
        except (TypeError, ValueError):
            gps_satellites, gps_hdop = 0, 99.9
        stable_time = float(self.get_parameter('gps_stable_time_s').value)
        gps_quality_seconds = (
            0.0 if self.gps_quality_since is None
            else max(0.0, now - self.gps_quality_since)
        )
        gps_quality_ok = (
            fresh['gps'] and self.gps_quality_since is not None
            and gps_quality_seconds >= stable_time
        )
        effective, reason = select_effective_mode(
            self.requested,
            emergency_latched=self.emergency_latched,
            stadia_fresh=fresh['stadia'],
            stadia_connected=stadia_connected,
            stadia_mode=self.stadia.get('mode', ''),
            follower_fresh=fresh['follower'],
            follower_enabled=bool(self.follower.get('enabled', False)),
            gps_fresh=fresh['gps'],
            gps_fix=gps_fix,
            gps_quality_ok=gps_quality_ok,
            navigation_ready=bool(
                self.get_parameter('navigation_ready').value),
        )
        payload = {
            'requested_mode': self.requested,
            'effective_mode': effective,
            'reason': reason,
            'monitor_only': bool(self.get_parameter('monitor_only').value),
            'command_output_enabled': False,
            'emergency_latched': self.emergency_latched,
            'readiness': {
                'stadia_connected': stadia_connected,
                'follower_enabled': bool(self.follower.get('enabled', False)),
                'gps_fix': gps_fix,
                'gps_satellites': gps_satellites,
                'gps_hdop': gps_hdop,
                'gps_quality_ok': gps_quality_ok,
                'gps_quality_seconds': round(gps_quality_seconds, 1),
                'navigation_ready': bool(
                    self.get_parameter('navigation_ready').value),
            },
            'fresh': fresh,
        }
        self.pub_state.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = FieldSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
