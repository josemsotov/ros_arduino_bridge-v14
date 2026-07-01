#!/usr/bin/env python3
"""Open-palm gesture detector for the Smart Trolley."""

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class OpenPalmNode(Node):
    def __init__(self):
        super().__init__('open_palm_detector')

        self.declare_parameter('image_topic', '/camera/rgb/image_raw')
        self.declare_parameter('process_rate', 8.0)
        self.declare_parameter('gesture_hold_time', 0.7)
        self.declare_parameter('cooldown', 4.0)
        self.declare_parameter('require_raised_hand', True)
        self.declare_parameter('max_wrist_y', 0.78)
        self.declare_parameter('publish_image', True)

        self.pub_command = self.create_publisher(String, '/gesture/command', 5)
        self.pub_status = self.create_publisher(String, '/gesture/status', 5)
        self.pub_image = self.create_publisher(Image, '/gesture/image', 2)

        self.last_process = 0.0
        self.last_trigger = 0.0
        self.open_since = None
        self.mp_hands = None
        self.hands = None

        try:
            import mediapipe as mp
            self.mp_hands = mp.solutions.hands
            self.hands = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=1,
                min_detection_confidence=0.55,
                min_tracking_confidence=0.50,
            )
            self.create_subscription(
                Image,
                self.get_parameter('image_topic').value,
                self.image_cb,
                2,
            )
            self.get_logger().info('Open-palm detector ready')
        except Exception as exc:
            self.get_logger().error(f'MediaPipe unavailable: {exc}')
            self.create_timer(1.0, self.publish_missing_dependency)

    def publish_missing_dependency(self):
        self.publish_status('dependency_missing', False, 'mediapipe unavailable')

    def image_cb(self, msg: Image):
        now = time.monotonic()
        min_period = 1.0 / max(1.0, float(self.get_parameter('process_rate').value))
        if now - self.last_process < min_period:
            return
        self.last_process = now

        bgr = self.image_to_bgr(msg)
        if bgr is None:
            self.publish_status('bad_image', False, msg.encoding)
            return

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)
        detected = False
        raised = False
        open_palm = False

        if result.multi_hand_landmarks:
            detected = True
            landmarks = result.multi_hand_landmarks[0].landmark
            raised = self.is_raised(landmarks)
            open_palm = self.is_open_palm(landmarks)
            self.draw_landmarks(bgr, landmarks, open_palm, raised)

        valid = open_palm and (raised or not self.get_parameter('require_raised_hand').value)
        if valid:
            if self.open_since is None:
                self.open_since = now
            held = now - self.open_since
            if held >= self.get_parameter('gesture_hold_time').value:
                self.trigger_approach(now)
        else:
            self.open_since = None
            held = 0.0

        state = 'open_palm' if valid else 'tracking' if detected else 'no_hand'
        self.publish_status(state, valid, f'held={held:.2f}')
        if self.get_parameter('publish_image').value and self.pub_image.get_subscription_count() > 0:
            self.publish_image(msg, bgr)

    def trigger_approach(self, now):
        if now - self.last_trigger < self.get_parameter('cooldown').value:
            return
        payload = {
            'command': 'APPROACH',
            'source': 'open_palm',
            'stamp': now,
        }
        self.pub_command.publish(String(data=json.dumps(payload)))
        self.last_trigger = now
        self.open_since = None
        self.publish_status('approach_sent', True, 'command=APPROACH')
        self.get_logger().info('Open-palm gesture sent APPROACH command')

    def is_raised(self, landmarks):
        wrist_y = landmarks[0].y
        return wrist_y <= float(self.get_parameter('max_wrist_y').value)

    def is_open_palm(self, landmarks):
        extended = 0
        for tip, pip in ((8, 6), (12, 10), (16, 14), (20, 18)):
            if landmarks[tip].y < landmarks[pip].y:
                extended += 1

        palm_width = abs(landmarks[5].x - landmarks[17].x)
        thumb_spread = abs(landmarks[4].x - landmarks[5].x)
        thumb_open = palm_width > 0 and thumb_spread > palm_width * 0.45
        return extended >= 4 and thumb_open

    def draw_landmarks(self, bgr, landmarks, open_palm, raised):
        h, w = bgr.shape[:2]
        color = (40, 220, 80) if open_palm and raised else (50, 160, 255)
        for lm in landmarks:
            cv2.circle(bgr, (int(lm.x * w), int(lm.y * h)), 3, color, -1)
        label = 'APPROACH PALM' if open_palm and raised else 'HAND'
        cv2.putText(bgr, label, (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    def image_to_bgr(self, msg):
        try:
            enc = msg.encoding.lower()
            if enc == 'rgb8':
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            if enc == 'bgr8':
                return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            if enc == 'mono8':
                mono = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
                return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        except Exception:
            return None
        return None

    def publish_image(self, source_msg, bgr):
        out = Image()
        out.header = source_msg.header
        out.height = bgr.shape[0]
        out.width = bgr.shape[1]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = bgr.shape[1] * 3
        out.data = bgr.tobytes()
        self.pub_image.publish(out)

    def publish_status(self, state, active, detail):
        payload = {
            'state': state,
            'active': bool(active),
            'detail': detail,
            'stamp': time.monotonic(),
        }
        self.pub_status.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = OpenPalmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
