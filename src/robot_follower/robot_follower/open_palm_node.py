"""Gesture detector for Smart Trolley – detects 5 gestures via MediaPipe."""
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

GESTURE_CMDS = {
    'open_palm':  'APPROACH',
    'thumb_up':   'FOLLOW',
    'thumb_down': 'STOP',
    'peace_up':   'STADIA_ON',
    'peace_down': 'STADIA_OFF',
}

class OpenPalmNode(Node):
    def __init__(self):
        super().__init__('open_palm_detector')
        self.declare_parameter('image_topic', '/camera/rgb/image_raw')
        self.declare_parameter('cooldown', 3.0)
        self.declare_parameter('gesture_hold_time', 0.7)
        self.declare_parameter('require_raised_hand', False)
        self.declare_parameter('publish_image', True)
        self.pub_command = self.create_publisher(String, '/gesture/command', 5)
        self.pub_status  = self.create_publisher(String, '/gesture/status', 5)
        self.pub_image   = self.create_publisher(Image, '/gesture/image', 2)
        self.last_trigger = 0.0
        self.active_gesture = None
        self.gesture_since  = None
        try:
            import mediapipe as mp
            self._mp_hands = mp.solutions.hands.Hands(
                static_image_mode=False, max_num_hands=1,
                min_detection_confidence=0.6, min_tracking_confidence=0.5)
            self._mp_draw = mp.solutions.drawing_utils
            self.create_subscription(Image,
                self.get_parameter('image_topic').value,
                self.image_cb, 2)
        except ImportError:
            self.create_timer(1.0, self._warn_dep)

    def _warn_dep(self):
        self.pub_status.publish(String(data=json.dumps({'state':'dependency_missing'})))

    def image_cb(self, msg: Image):
        import cv2, numpy as np
        now = self.get_clock().now().nanoseconds * 1e-9
        bgr = self.image_to_bgr(msg)
        if bgr is None:
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        result = self._mp_hands.process(rgb)

        detected = None
        if result.multi_hand_landmarks:
            lm = result.multi_hand_landmarks[0].landmark
            self._mp_draw.draw_landmarks(bgr, result.multi_hand_landmarks[0],
                __import__('mediapipe').solutions.hands.HAND_CONNECTIONS)
            if   self.is_thumb_up(lm):   detected = 'thumb_up'
            elif self.is_thumb_down(lm): detected = 'thumb_down'
            elif self.is_peace_up(lm):   detected = 'peace_up'
            elif self.is_peace_down(lm): detected = 'peace_down'
            elif self.is_open_palm(lm):  detected = 'open_palm'

        hold_time = self.get_parameter('gesture_hold_time').value
        if detected:
            if self.active_gesture != detected:
                self.active_gesture = detected
                self.gesture_since  = now
            held = now - self.gesture_since
            if held >= hold_time:
                self._trigger(detected, now)
        else:
            self.active_gesture = None
            self.gesture_since  = None
            held = 0.0

        state = detected or 'no_hand'
        self.pub_status.publish(String(data=json.dumps(
            {'state': state, 'held': round(held, 2)})))
        if self.get_parameter('publish_image').value and \
                self.pub_image.get_subscription_count() > 0:
            self.publish_image(msg, bgr)

    def _trigger(self, gesture, now):
        if now - self.last_trigger < self.get_parameter('cooldown').value:
            return
        cmd = GESTURE_CMDS.get(gesture, gesture.upper())
        self.pub_command.publish(String(data=json.dumps(
            {'command': cmd, 'source': f'gesture_{gesture}', 'stamp': now})))
        self.last_trigger   = now
        self.active_gesture = None
        self.gesture_since  = None
        self.get_logger().info(f'Gesture {gesture} → {cmd}')

    # ── Gesture classifiers ────────────────────────────────────────────────────
    def _finger_extended_up(self, lm, tip, pip):
        return lm[tip].y < lm[pip].y

    def _finger_folded(self, lm, tip, pip):
        return lm[tip].y >= lm[pip].y

    def is_open_palm(self, lm):
        tips = [8, 12, 16, 20]; pips = [6, 10, 14, 18]
        extended = sum(1 for t,p in zip(tips,pips) if self._finger_extended_up(lm,t,p))
        palm_w = abs(lm[5].x - lm[17].x)
        thumb_open = palm_w > 0 and abs(lm[4].x - lm[5].x) > palm_w * 0.45
        return extended >= 4 and thumb_open

    def is_thumb_up(self, lm):
        # Thumb tip significantly above wrist; other 4 fingers folded
        thumb_up_enough = lm[4].y < lm[0].y - 0.15
        fingers_down = all(self._finger_folded(lm,t,p)
                           for t,p in [(8,6),(12,10),(16,14),(20,18)])
        return thumb_up_enough and fingers_down

    def is_thumb_down(self, lm):
        # Thumb tip significantly below wrist; other 4 fingers folded
        thumb_down_enough = lm[4].y > lm[0].y + 0.15
        fingers_down = all(self._finger_folded(lm,t,p)
                           for t,p in [(8,6),(12,10),(16,14),(20,18)])
        return thumb_down_enough and fingers_down

    def is_peace_up(self, lm):
        # Index + middle extended up; ring + pinky + thumb folded
        idx_up = self._finger_extended_up(lm, 8, 6)
        mid_up = self._finger_extended_up(lm, 12, 10)
        ring_fold = self._finger_folded(lm, 16, 14)
        pink_fold = self._finger_folded(lm, 20, 18)
        thumb_fold = lm[4].x > lm[3].x if lm[0].x < 0.5 else lm[4].x < lm[3].x
        return idx_up and mid_up and ring_fold and pink_fold and thumb_fold

    def is_peace_down(self, lm):
        # Index + middle pointing DOWN (tips below wrist); ring + pinky folded
        idx_down = lm[8].y > lm[0].y + 0.05
        mid_down = lm[12].y > lm[0].y + 0.05
        ring_fold = self._finger_folded(lm, 16, 14)
        pink_fold = self._finger_folded(lm, 20, 18)
        return idx_down and mid_down and ring_fold and pink_fold

    # ── Helpers ────────────────────────────────────────────────────────────────
    def image_to_bgr(self, msg):
        import cv2, numpy as np
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, -1))
            if msg.encoding in ('rgb8', 'rgb'):
                return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            if msg.encoding in ('bgr8', 'bgr'):
                return arr.copy()
            return None
        except Exception:
            return None

    def publish_image(self, source, bgr):
        import cv2
        out = Image()
        out.header = source.header
        out.height, out.width = bgr.shape[:2]
        out.encoding = 'bgr8'
        out.step = out.width * 3
        out.data = bgr.tobytes()
        self.pub_image.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = OpenPalmNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
