#!/usr/bin/env python3
"""
ROS2 follower for the Smart Trolley.

Modes:
  FOLLOW   - keep the target inside the 2.0 m to 3.0 m band.
  APPROACH - after an open-palm command, approach to 0.5 m and stop.
  WAITING  - motors stopped, waiting for a new FOLLOW command.
"""

import json
import math
import struct

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String


class FollowerNode(Node):
    def __init__(self):
        super().__init__('robot_follower')

        self.declare_parameter('follow_distance', 2.50)
        self.declare_parameter('follow_min_distance', 2.00)
        self.declare_parameter('follow_max_distance', 3.00)
        self.declare_parameter('min_distance', 0.50)
        self.declare_parameter('max_detect_dist', 5.00)
        self.declare_parameter('min_angle_deg', 0.0)
        self.declare_parameter('search_angle_deg', 75.0)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 0.8)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('kp_angular', 1.2)
        self.declare_parameter('approach_distance', 0.50)
        self.declare_parameter('approach_tolerance', 0.08)
        self.declare_parameter('use_kinect', False)

        self.enabled = False
        self.gesture_enabled = False
        self.mode = 'WAITING'
        self.target_dist = None
        self.target_angle = None
        self.kinect_target_x = None

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 5)
        self.pub_debug = self.create_publisher(Twist, '/follower/debug', 5)
        self.pub_state = self.create_publisher(String, '/follower/state', 5)

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 5)
        self.create_subscription(Bool, '/follower/enable', self.enable_cb, 5)
        self.create_subscription(String, '/gesture/command', self.gesture_cb, 5)
        self.create_subscription(Bool, '/gesture/enable', self.gesture_enable_cb, 5)

        if self.get_parameter('use_kinect').value:
            self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 2)

        self.last_scan = self.get_clock().now()
        self.create_timer(0.1, self.safety_check)
        self.create_timer(0.5, self.publish_state)

        self.get_logger().info('Robot follower ready; waiting for /follower/enable true')

    def enable_cb(self, msg: Bool):
        self.enabled = bool(msg.data)
        if self.enabled:
            self.mode = 'FOLLOW'
            self.get_logger().info('Follower enabled')
        else:
            self.mode = 'WAITING'
            self.get_logger().info('Follower disabled; motors stopped')
            self._stop()
        self.publish_state()

    def gesture_cb(self, msg: String):
        if not self.gesture_enabled:
            self.get_logger().debug('Gesture ignored: GESTURE mode is not selected')
            return

        command = msg.data.strip()
        try:
            payload = json.loads(command)
            command = str(payload.get('command', command))
        except json.JSONDecodeError:
            pass

        command = command.upper()
        if command in ('APPROACH', 'COME_CLOSE', 'PALM_OPEN'):
            self.enabled = True
            self.mode = 'APPROACH'
            self.get_logger().info('Open-palm command: approaching to 0.5 m')
        elif command in ('FOLLOW', 'FOLLOW_ME', 'RESUME'):
            self.enabled = True
            self.mode = 'FOLLOW'
            self.get_logger().info('Follow command received')
        elif command in ('STOP', 'WAIT', 'PAUSE'):
            self.enabled = False
            self.mode = 'WAITING'
            self._stop()
            self.get_logger().info('Stop/wait command received')
        self.publish_state()

    def gesture_enable_cb(self, msg: Bool):
        self.gesture_enabled = bool(msg.data)
        if not self.gesture_enabled and self.mode == 'APPROACH':
            self.enabled = False
            self.mode = 'WAITING'
            self._stop()
        self.publish_state()

    def depth_cb(self, msg: Image):
        """Use Kinect depth as an optional angle hint."""
        try:
            w = msg.width
            mid_row_start = (msg.height // 2 - 20) * w
            mid_row_end = (msg.height // 2 + 20) * w
            col_sums = [0] * w
            col_cnt = [0] * w
            data = msg.data
            for i in range(mid_row_start, mid_row_end):
                val = struct.unpack_from('<H', bytes(data[2 * i:2 * i + 2]))[0]
                if 300 < val < 4000:
                    col = i % w
                    col_sums[col] += val
                    col_cnt[col] += 1
            avgs = [
                col_sums[c] / col_cnt[c] if col_cnt[c] > 0 else 99999
                for c in range(w)
            ]
            best_col = avgs.index(min(avgs))
            self.kinect_target_x = (best_col - w / 2) / (w / 2) * 0.497
        except Exception:
            self.kinect_target_x = None

    def scan_cb(self, msg: LaserScan):
        self.last_scan = self.get_clock().now()
        if not self.enabled:
            return

        p = self.get_all_params()
        search_rad = math.radians(p['search_angle_deg'])
        min_rad = math.radians(p['min_angle_deg'])
        points = []

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < 0.1 or r > p['max_detect_dist']:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) < min_rad or abs(angle) > search_rad:
                continue
            points.append((r, angle))

        if not points:
            self.get_logger().warn('No target in range', throttle_duration_sec=2)
            self._stop()
            return

        points.sort(key=lambda item: item[0])
        closest = points[:min(15, len(points))]
        target_dist = sum(item[0] for item in closest) / len(closest)
        target_angle = sum(item[1] for item in closest) / len(closest)

        if self.kinect_target_x is not None:
            target_angle = 0.6 * target_angle + 0.4 * self.kinect_target_x

        self.target_dist = target_dist
        self.target_angle = target_angle
        self._control(target_dist, target_angle, p)

    def _control(self, dist, angle, p):
        if self.mode == 'APPROACH':
            self._control_approach(dist, angle, p)
            return

        twist = Twist()
        if dist < p['min_distance']:
            twist.linear.x = -0.15
            twist.angular.z = 0.0
            self.get_logger().warn(
                f'Too close: {dist:.2f} m; backing up',
                throttle_duration_sec=1,
            )
        else:
            if dist < p['follow_min_distance']:
                dist_error = dist - p['follow_min_distance']
            elif dist > p['follow_max_distance']:
                dist_error = dist - p['follow_max_distance']
            else:
                dist_error = 0.0

            lin = p['kp_linear'] * dist_error
            ang = -p['kp_angular'] * angle
            twist.linear.x = max(-p['max_linear_vel'] * 0.5, min(p['max_linear_vel'], lin))
            twist.angular.z = max(-p['max_angular_vel'], min(p['max_angular_vel'], ang))

        self.pub_cmd.publish(twist)
        self._publish_debug(dist, angle)

    def _control_approach(self, dist, angle, p):
        target = p['approach_distance']
        tol = p['approach_tolerance']
        if dist <= target + tol:
            self._stop()
            self.enabled = False
            self.mode = 'WAITING'
            self.get_logger().info(
                f'Approach complete at {dist:.2f} m; waiting for a new follow command'
            )
            self.publish_state()
            return

        twist = Twist()
        lin = p['kp_linear'] * (dist - target)
        ang = -p['kp_angular'] * angle
        twist.linear.x = max(0.08, min(p['max_linear_vel'] * 0.65, lin))
        twist.angular.z = max(-p['max_angular_vel'], min(p['max_angular_vel'], ang))
        self.pub_cmd.publish(twist)
        self._publish_debug(dist, angle)

    def _publish_debug(self, dist, angle):
        dbg = Twist()
        dbg.linear.x = float(dist)
        dbg.angular.z = float(angle)
        self.pub_debug.publish(dbg)

    def _stop(self):
        self.pub_cmd.publish(Twist())

    def safety_check(self):
        if not self.enabled:
            return
        dt = (self.get_clock().now() - self.last_scan).nanoseconds / 1e9
        if dt > 0.5:
            self.get_logger().warn('No recent scan; safety stop', throttle_duration_sec=2)
            self._stop()

    def publish_state(self):
        payload = {
            'enabled': self.enabled,
            'gesture_enabled': self.gesture_enabled,
            'mode': self.mode,
            'target_dist': self.target_dist,
            'target_angle': self.target_angle,
        }
        self.pub_state.publish(String(data=json.dumps(payload)))

    def get_all_params(self):
        return {
            'follow_distance': self.get_parameter('follow_distance').value,
            'follow_min_distance': self.get_parameter('follow_min_distance').value,
            'follow_max_distance': self.get_parameter('follow_max_distance').value,
            'min_distance': self.get_parameter('min_distance').value,
            'max_detect_dist': self.get_parameter('max_detect_dist').value,
            'min_angle_deg': self.get_parameter('min_angle_deg').value,
            'search_angle_deg': self.get_parameter('search_angle_deg').value,
            'max_linear_vel': self.get_parameter('max_linear_vel').value,
            'max_angular_vel': self.get_parameter('max_angular_vel').value,
            'kp_linear': self.get_parameter('kp_linear').value,
            'kp_angular': self.get_parameter('kp_angular').value,
            'approach_distance': self.get_parameter('approach_distance').value,
            'approach_tolerance': self.get_parameter('approach_tolerance').value,
        }


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
