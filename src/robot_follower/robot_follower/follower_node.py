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

import numpy as np
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
        self.declare_parameter('visual_identity_enabled', True)
        self.declare_parameter('visual_identity_required', True)
        self.declare_parameter('visual_identity_threshold', 0.62)
        self.declare_parameter('visual_identity_timeout', 1.2)
        self.declare_parameter('visual_identity_samples', 4)
        self.declare_parameter('visual_target_fov_deg', 62.0)
        self.declare_parameter('min_linear_vel', 0.06)
        self.declare_parameter('linear_deadband', 0.06)
        self.declare_parameter('angular_deadband_deg', 2.0)
        self.declare_parameter('velocity_smoothing_alpha', 0.28)

        self.enabled = False
        self.mode = 'WAITING'
        self.target_dist = None
        self.target_angle = None
        self.kinect_target_x = None
        self.identity_profile = None
        self.identity_samples = []
        self.identity_verified = False
        self.identity_score = 0.0
        self.identity_status = 'idle'
        self.identity_description = ''
        self.identity_last_verified = self.get_clock().now()
        self.identity_enroll_active = False
        self.smoothed_linear = 0.0
        self.smoothed_angular = 0.0

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 5)
        self.pub_debug = self.create_publisher(Twist, '/follower/debug', 5)
        self.pub_state = self.create_publisher(String, '/follower/state', 5)

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 5)
        self.create_subscription(Bool, '/follower/enable', self.enable_cb, 5)
        self.create_subscription(String, '/gesture/command', self.gesture_cb, 5)

        if self.get_parameter('use_kinect').value:
            self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 2)
        if self.get_parameter('visual_identity_enabled').value:
            self.create_subscription(Image, '/camera/rgb/image_raw', self.rgb_cb, 2)

        self.last_scan = self.get_clock().now()
        self.create_timer(0.1, self.safety_check)
        self.create_timer(0.5, self.publish_state)

        self.get_logger().info('Robot follower ready; waiting for /follower/enable true')

    def enable_cb(self, msg: Bool):
        self.enabled = bool(msg.data)
        if self.enabled:
            self.mode = 'FOLLOW'
            self.identity_enroll_active = False
            if self.identity_profile is None:
                self._reset_identity()
            else:
                self.identity_status = 'armed'
                self.identity_verified = True
                self.identity_last_verified = self.get_clock().now()
            self.get_logger().info('Follower enabled')
        else:
            self.mode = 'WAITING'
            self.get_logger().info('Follower disabled; motors stopped')
            self._stop()
        self.publish_state()

    def gesture_cb(self, msg: String):
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
            self.identity_enroll_active = False
            if self.identity_profile is None:
                self._reset_identity()
            self.get_logger().info('Open-palm command: approaching to 0.5 m')
        elif command in ('FOLLOW', 'FOLLOW_ME', 'RESUME'):
            self.enabled = True
            self.mode = 'FOLLOW'
            self.identity_enroll_active = False
            if self.identity_profile is None:
                self._reset_identity()
            self.get_logger().info('Follow command received')
        elif command in ('FACE_ID_ENROLL', 'IDENTITY_ENROLL', 'ID_ENROLL'):
            self.enabled = False
            self.mode = 'WAITING'
            self._stop()
            self._reset_identity()
            self.identity_enroll_active = True
            self.identity_status = 'enrolling'
            self.get_logger().info('Face ID enrollment started')
        elif command in ('FACE_ID_CLEAR', 'IDENTITY_CLEAR', 'ID_CLEAR'):
            self.enabled = False
            self.mode = 'WAITING'
            self._stop()
            self._reset_identity()
            self.identity_enroll_active = False
            self.identity_status = 'cleared'
            self.get_logger().info('Face ID profile cleared')
        elif command in ('STOP', 'WAIT', 'PAUSE'):
            self.enabled = False
            self.mode = 'WAITING'
            self._stop()
            self.get_logger().info('Stop/wait command received')
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

    def rgb_cb(self, msg: Image):
        if not (self.enabled or self.identity_enroll_active):
            return
        if not self.get_parameter('visual_identity_enabled').value:
            return

        image = self._image_to_rgb(msg)
        if image is None:
            self.identity_status = 'bad_image'
            return

        signature, description = self._visual_signature(image, self._target_x_norm())
        if signature is None:
            self.identity_status = 'no_visual_sample'
            return

        if self.identity_profile is None:
            self.identity_samples.append(signature)
            needed = max(1, int(self.get_parameter('visual_identity_samples').value))
            self.identity_status = f'capturing {len(self.identity_samples)}/{needed}'
            self.identity_description = description
            if len(self.identity_samples) >= needed:
                self.identity_profile = np.mean(np.vstack(self.identity_samples[-needed:]), axis=0)
                self.identity_last_verified = self.get_clock().now()
                self.identity_verified = True
                self.identity_score = 1.0
                self.identity_status = 'locked'
                self.identity_enroll_active = False
                self.get_logger().info(f'Visual identity locked: {description}')
            return

        score = self._identity_similarity(signature, self.identity_profile)
        threshold = float(self.get_parameter('visual_identity_threshold').value)
        self.identity_score = score
        self.identity_description = description
        if score >= threshold:
            self.identity_verified = True
            self.identity_last_verified = self.get_clock().now()
            self.identity_status = 'verified'
        else:
            self.identity_verified = False
            self.identity_status = 'mismatch'

    def _image_to_rgb(self, msg: Image):
        try:
            enc = msg.encoding.lower()
            if enc in ('rgb8', 'bgr8'):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                return arr.copy() if enc == 'rgb8' else arr[:, :, ::-1].copy()
            if enc in ('rgba8', 'bgra8'):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
                return arr[:, :, :3].copy() if enc == 'rgba8' else arr[:, :, 2::-1].copy()
            if enc == 'mono8':
                mono = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
                return np.repeat(mono[:, :, None], 3, axis=2)
        except Exception:
            return None
        return None

    def _target_x_norm(self):
        angle = self.kinect_target_x if self.kinect_target_x is not None else self.target_angle
        if angle is None:
            return 0.5
        fov = math.radians(max(10.0, float(self.get_parameter('visual_target_fov_deg').value)))
        return max(0.05, min(0.95, 0.5 + float(angle) / fov))

    def _visual_signature(self, image, x_norm):
        h, w = image.shape[:2]
        cx = int(max(0, min(w - 1, x_norm * (w - 1))))
        half_w = max(16, int(w * 0.10))
        x1 = max(0, cx - half_w)
        x2 = min(w, cx + half_w)
        bands = (
            ('head', int(h * 0.18), int(h * 0.38)),
            ('shirt', int(h * 0.42), int(h * 0.74)),
        )

        features = []
        labels = []
        for name, y1, y2 in bands:
            crop = image[y1:y2, x1:x2].astype(np.float32)
            if crop.size == 0:
                return None, ''
            pixels = crop.reshape(-1, 3)
            brightness = pixels.mean(axis=1)
            usable = pixels[brightness > 18.0]
            if usable.size == 0:
                usable = pixels
            raw_mean = usable.mean(axis=0)
            total = float(raw_mean.sum())
            if total <= 1.0:
                return None, ''
            features.extend((raw_mean / total).tolist())
            labels.append(f'{name}:{self._color_label(raw_mean)}')

        return np.array(features, dtype=np.float32), ' '.join(labels)

    def _color_label(self, rgb):
        r, g, b = [float(v) for v in rgb]
        mx = max(r, g, b)
        mn = min(r, g, b)
        if mx < 55:
            return 'dark'
        if mx - mn < 22:
            return 'gray'
        if r > g * 1.15 and r > b * 1.15:
            return 'red'
        if g > r * 1.12 and g > b * 1.12:
            return 'green'
        if b > r * 1.12 and b > g * 1.12:
            return 'blue'
        if r > 130 and g > 100 and b < 95:
            return 'yellow'
        return 'mixed'

    def _identity_similarity(self, signature, profile):
        dist = float(np.linalg.norm(signature - profile))
        return max(0.0, min(1.0, 1.0 - dist / 0.65))

    def _reset_identity(self):
        if not self.get_parameter('visual_identity_enabled').value:
            return
        self.identity_profile = None
        self.identity_samples = []
        self.identity_verified = False
        self.identity_score = 0.0
        self.identity_status = 'capturing'
        self.identity_description = ''
        self.identity_last_verified = self.get_clock().now()
        self._reset_motion_filter()

    def _identity_allows_motion(self):
        if not self.get_parameter('visual_identity_enabled').value:
            return True
        if not self.get_parameter('visual_identity_required').value:
            return True
        if self.identity_profile is None:
            return False
        timeout = float(self.get_parameter('visual_identity_timeout').value)
        age = (self.get_clock().now() - self.identity_last_verified).nanoseconds / 1e9
        return age <= timeout

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
        if not self._identity_allows_motion():
            self.get_logger().warn(
                f'Visual identity not verified ({self.identity_status}); holding position',
                throttle_duration_sec=1,
            )
            self._stop()
            self.publish_state()
            return
        self._control(target_dist, target_angle, p)

    def _control(self, dist, angle, p):
        if self.mode == 'APPROACH':
            self._control_approach(dist, angle, p)
            return

        if dist < p['min_distance']:
            linear = -0.15
            angular = 0.0
            self.get_logger().warn(
                f'Too close: {dist:.2f} m; backing up',
                throttle_duration_sec=1,
            )
        else:
            target = (p['follow_min_distance'] + p['follow_max_distance']) * 0.5
            half_range = max(0.05, (p['follow_max_distance'] - p['follow_min_distance']) * 0.5)
            dist_error = dist - target
            if abs(dist_error) <= p['linear_deadband']:
                linear = 0.0
            else:
                ratio = min(1.0, abs(dist_error) / half_range)
                speed = p['min_linear_vel'] + (p['max_linear_vel'] - p['min_linear_vel']) * (ratio ** 1.25)
                linear = math.copysign(speed, dist_error)
                linear = max(-p['max_linear_vel'] * 0.65, min(p['max_linear_vel'], linear))

            if abs(angle) <= math.radians(p['angular_deadband_deg']):
                angular = 0.0
            else:
                angular = -p['kp_angular'] * angle
                angular = max(-p['max_angular_vel'], min(p['max_angular_vel'], angular))

        self._publish_motion(linear, angular, p)
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

        lin = p['kp_linear'] * (dist - target)
        ang = -p['kp_angular'] * angle
        lin = max(0.08, min(p['max_linear_vel'] * 0.65, lin))
        ang = max(-p['max_angular_vel'], min(p['max_angular_vel'], ang))
        self._publish_motion(lin, ang, p)
        self._publish_debug(dist, angle)

    def _publish_motion(self, linear, angular, p):
        alpha = max(0.05, min(1.0, float(p['velocity_smoothing_alpha'])))
        self.smoothed_linear += alpha * (float(linear) - self.smoothed_linear)
        self.smoothed_angular += alpha * (float(angular) - self.smoothed_angular)
        twist = Twist()
        twist.linear.x = self.smoothed_linear
        twist.angular.z = self.smoothed_angular
        self.pub_cmd.publish(twist)

    def _publish_debug(self, dist, angle):
        dbg = Twist()
        dbg.linear.x = float(dist)
        dbg.angular.z = float(angle)
        self.pub_debug.publish(dbg)

    def _stop(self):
        self._reset_motion_filter()
        self.pub_cmd.publish(Twist())

    def _reset_motion_filter(self):
        self.smoothed_linear = 0.0
        self.smoothed_angular = 0.0

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
            'mode': self.mode,
            'target_dist': self.target_dist,
            'target_angle': self.target_angle,
            'identity_status': self.identity_status,
            'identity_verified': self.identity_verified,
            'identity_score': round(float(self.identity_score), 3),
            'identity_description': self.identity_description,
            'identity_enroll_active': self.identity_enroll_active,
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
            'min_linear_vel': self.get_parameter('min_linear_vel').value,
            'linear_deadband': self.get_parameter('linear_deadband').value,
            'angular_deadband_deg': self.get_parameter('angular_deadband_deg').value,
            'velocity_smoothing_alpha': self.get_parameter('velocity_smoothing_alpha').value,
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
