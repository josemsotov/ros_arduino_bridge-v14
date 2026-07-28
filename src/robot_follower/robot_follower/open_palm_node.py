"""Gesture detector for Smart Trolley – detects 5 gestures via MediaPipe."""
import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, String
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
        self.declare_parameter('max_inference_fps', 5.0)
        self.declare_parameter('hand_field_span_m', 0.30)
        self.declare_parameter('hand_field_min_m', 1.0)
        self.declare_parameter('hand_field_max_m', 4.0)
        self.declare_parameter('hand_field_target_m', 1.5)
        self.declare_parameter('hand_field_fov_deg', 62.0)
        self.declare_parameter('hand_field_angle_scale', 2.0)
        self.declare_parameter('hand_field_max_linear', 0.12)
        # Autonomous following is forward-only. When the subject reaches or
        # crosses the target distance, the robot must stop instead of reversing.
        self.declare_parameter('hand_field_max_reverse', 0.0)
        self.declare_parameter('hand_field_max_angular', 0.22)
        self.declare_parameter('hand_field_kp_linear', 0.10)
        self.declare_parameter('hand_field_kp_angular', 0.70)
        self.declare_parameter('hand_field_distance_deadband_m', 0.15)
        self.declare_parameter('hand_field_angle_deadband_deg', 3.0)
        self.declare_parameter('hand_field_smoothing_alpha', 0.25)
        self.declare_parameter('hand_field_lost_timeout', 0.45)
        self.pub_command = self.create_publisher(String, '/gesture/command', 5)
        self.pub_status  = self.create_publisher(String, '/gesture/status', 5)
        self.pub_image   = self.create_publisher(Image, '/gesture/image', 2)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_stadia = self.create_publisher(String, '/stadia/control', 5)
        self.pub_follower_request = self.create_publisher(
            Bool, '/operator/follower_request', 5
        )
        self.pub_tilt_set = self.create_publisher(Int32, '/kinect/tilt/set', 5)
        self.pub_tilt_save = self.create_publisher(Bool, '/kinect/tilt/save', 5)
        self.pub_tilt_calibration_arm = self.create_publisher(
            Bool, '/kinect/tilt_calibration/arm', 5
        )
        self.create_subscription(Bool, '/body_field/arm', self.body_arm_cb, 5)
        self.create_subscription(Bool, '/hand_field/arm', self.arm_cb, 5)
        self.create_subscription(
            Bool, '/kinect/tilt_calibration/arm',
            self.tilt_calibration_arm_cb, 5
        )
        self.create_subscription(Int32, '/kinect/tilt/state', self.tilt_state_cb, 5)
        self.create_subscription(String, '/stadia/state', self.stadia_cb, 5)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_cb, 2
        )
        self.last_trigger = 0.0
        self.active_gesture = None
        self.gesture_since  = None
        self.last_process = 0.0
        self.hand_field_armed = False
        self.body_field_armed = False
        self.tilt_calibration_armed = False
        self.kinect_tilt_degrees = 8
        self.last_tilt_command = 0.0
        self.hand_field_active = False
        self.hand_field_state = 'disarmed'
        self.stadia_connected = False
        self.stadia_mode = 'off'
        self.depth_image = None
        self.depth_stamp = 0.0
        self.hand_last_seen = 0.0
        self.baseline_depth_m = None
        self.baseline_hand_x = None
        self.simulated_distance_m = None
        self.simulated_angle_deg = None
        self.smooth_linear = 0.0
        self.smooth_angular = 0.0
        self.create_timer(0.1, self.safety_tick)
        try:
            import mediapipe as mp
            self._mp_hands = mp.solutions.hands.Hands(
                static_image_mode=False, max_num_hands=1,
                min_detection_confidence=0.6, min_tracking_confidence=0.5)
            # In the body-field test the hand occupies far fewer pixels than in
            # the hand-field bench test. Use a dedicated, more sensitive model
            # on an upper-body crop; keeping the models separate avoids making
            # the close-range hand test prone to false detections.
            self._mp_body_hands = mp.solutions.hands.Hands(
                static_image_mode=False, max_num_hands=1,
                model_complexity=1,
                min_detection_confidence=0.35,
                min_tracking_confidence=0.35)
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
        max_fps = max(1.0, float(self.get_parameter('max_inference_fps').value))
        if now - self.last_process < 1.0 / max_fps:
            return
        self.last_process = now
        bgr = self.image_to_bgr(msg)
        if bgr is None:
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        body_mode_frame = self.body_field_armed and not self.tilt_calibration_armed
        if body_mode_frame:
            height, width = rgb.shape[:2]
            # A centred full-body subject's raised hand should be within this
            # upper-body region. Cropping gives MediaPipe substantially more
            # useful hand pixels without changing the normal hand-field test.
            # The Kinect on the trolley is pitched upward, so a nearby
            # person's face, shoulders and raised hand occupy the lower part
            # of the RGB image. Preserve the full width and crop away only the
            # ceiling-heavy upper third.
            x0, x1 = 0, width
            y0, y1 = int(height * 0.32), height
            inference_rgb = rgb[y0:y1, x0:x1]
            result = self._mp_body_hands.process(inference_rgb)
        else:
            result = self._mp_hands.process(rgb)

        detected = None
        lm = None
        field_mode_frame = self.hand_field_armed
        if result.multi_hand_landmarks:
            lm = result.multi_hand_landmarks[0].landmark
            # Crop-relative landmarks are still valid for classifying finger
            # geometry. Only draw landmarks when coordinates match the frame.
            if not body_mode_frame:
                self._mp_draw.draw_landmarks(
                    bgr, result.multi_hand_landmarks[0],
                    __import__('mediapipe').solutions.hands.HAND_CONNECTIONS)
            if   self.is_index_up(lm):   detected = 'index_up'
            elif self.is_thumb_up(lm):   detected = 'thumb_up'
            elif self.is_thumb_down(lm): detected = 'thumb_down'
            elif self.is_peace_up(lm):   detected = 'peace_up'
            elif self.is_peace_down(lm): detected = 'peace_down'
            elif self.is_open_palm(lm):  detected = 'open_palm'
            self.hand_last_seen = now

        hold_time = self.get_parameter('gesture_hold_time').value
        if detected:
            if self.active_gesture != detected:
                self.active_gesture = detected
                self.gesture_since  = now
            held = now - self.gesture_since
        else:
            self.active_gesture = None
            self.gesture_since  = None
            held = 0.0

        if self.tilt_calibration_armed:
            self._process_tilt_calibration(lm, detected, held, now)
            if self.get_parameter('publish_image').value and \
                    self.pub_image.get_subscription_count() > 0:
                self.publish_image(msg, bgr)
            return

        if field_mode_frame:
            # Process field gestures only after active_gesture/gesture_since
            # belong to the gesture in this frame. Previously a one-frame
            # open-palm misclassification could reuse the index hold time and
            # pause the test immediately.
            if lm is not None:
                self._process_hand_field(lm, detected, now)
            state = self.hand_field_state
            self._publish_field_status(state, held)
            if self.get_parameter('publish_image').value and \
                    self.pub_image.get_subscription_count() > 0:
                self.publish_image(msg, bgr)
            return

        if detected and held >= hold_time:
            self._trigger(detected, now)

        state = detected or 'no_hand'
        self.pub_status.publish(String(data=json.dumps(
            {'state': state, 'held': round(held, 2)})))
        if self.get_parameter('publish_image').value and \
                self.pub_image.get_subscription_count() > 0:
            self.publish_image(msg, bgr)

    def _trigger(self, gesture, now):
        if self.hand_field_armed:
            return
        if now - self.last_trigger < self.get_parameter('cooldown').value:
            return
        if self.body_field_armed:
            if gesture == 'index_up':
                # Ask StadiaNode to enter FOLLOWER at the exact moment the
                # verified user confirms with the index. StadiaNode is the
                # sole publisher of authorized_enable, so takeover and safety
                # semantics remain centralized.
                self.pub_follower_request.publish(Bool(data=True))
                cmd = 'FOLLOW_REQUEST'
            elif gesture == 'open_palm':
                self.pub_follower_request.publish(Bool(data=False))
                cmd = 'STOP'
            else:
                return
        else:
            cmd = GESTURE_CMDS.get(gesture, gesture.upper())
        if cmd != 'FOLLOW_REQUEST':
            self.pub_command.publish(String(data=json.dumps(
                {'command': cmd, 'source': f'gesture_{gesture}', 'stamp': now})))
        self.last_trigger   = now
        self.active_gesture = None
        self.gesture_since  = None
        self.get_logger().info(f'Gesture {gesture} → {cmd}')

    def body_arm_cb(self, msg):
        self.body_field_armed = bool(msg.data)
        if self.body_field_armed and self.hand_field_armed:
            self._disarm_field('disarmed')

    def tilt_state_cb(self, msg):
        self.kinect_tilt_degrees = int(msg.data)

    def tilt_calibration_arm_cb(self, msg):
        self.tilt_calibration_armed = bool(msg.data)
        self.active_gesture = None
        self.gesture_since = None
        if self.tilt_calibration_armed:
            self.body_field_armed = False
            if self.hand_field_armed:
                self._disarm_field('disarmed')
            self._stop_field()
            self.get_logger().info(
                'Kinect tilt calibration armed; index follows, open palm saves'
            )

    def _process_tilt_calibration(self, lm, detected, held, now):
        state = 'tilt_waiting_index'
        hand_y = None
        if lm is not None:
            hand_y = float(lm[9].y)

        if detected == 'open_palm' and held >= 0.7:
            self.pub_tilt_save.publish(Bool(data=True))
            self.tilt_calibration_armed = False
            self.pub_tilt_calibration_arm.publish(Bool(data=False))
            self.last_trigger = now
            self.active_gesture = None
            self.gesture_since = None
            state = 'tilt_saved'
            self.get_logger().info(
                f'Kinect theoretical centre saved at {self.kinect_tilt_degrees} deg'
            )
        elif detected == 'index_up':
            state = 'tilt_following_index'
            if hand_y is not None and held >= 0.35 and now - self.last_tilt_command >= 0.25:
                error = 0.50 - hand_y
                if abs(error) >= 0.06:
                    step = 2 if abs(error) >= 0.18 else 1
                    requested = self.kinect_tilt_degrees + (
                        step if error > 0.0 else -step
                    )
                    requested = max(-15, min(25, requested))
                    self.pub_tilt_set.publish(Int32(data=requested))
                    self.last_tilt_command = now

        self.pub_status.publish(String(data=json.dumps({
            'state': state,
            'held': round(float(held), 2),
            'tilt_calibration_armed': self.tilt_calibration_armed,
            'kinect_tilt_degrees': self.kinect_tilt_degrees,
            'hand_y': None if hand_y is None else round(hand_y, 3),
        })))

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

    def is_index_up(self, lm):
        idx_up = self._finger_extended_up(lm, 8, 6)
        others_folded = all(
            self._finger_folded(lm, tip, pip)
            for tip, pip in [(12, 10), (16, 14), (20, 18)]
        )
        return idx_up and others_folded

    def arm_cb(self, msg):
        if not bool(msg.data):
            self._disarm_field('disarmed')
            return
        if not self.stadia_connected:
            self._disarm_field('blocked_stadia')
            return
        self.hand_field_armed = True
        self.hand_field_active = False
        self.hand_field_state = 'armed_waiting_index'
        self.baseline_depth_m = None
        self.baseline_hand_x = None
        self.simulated_distance_m = None
        self.simulated_angle_deg = None
        self.pub_stadia.publish(String(data='OFF'))
        self._stop_field()
        self.get_logger().warn(
            'Hand-field armed; hold index up to start wheel-bench test'
        )

    def stadia_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        self.stadia_connected = payload.get('stadia') == 'connected'
        self.stadia_mode = str(payload.get('mode', 'off'))
        if self.hand_field_armed and (
            not self.stadia_connected or self.stadia_mode == 'stadia'
        ):
            self._disarm_field('stadia_takeover')

    def depth_cb(self, msg):
        import numpy as np
        try:
            self.depth_image = np.frombuffer(
                msg.data, dtype=np.uint16
            ).reshape(msg.height, msg.width).copy()
            self.depth_stamp = self.get_clock().now().nanoseconds * 1e-9
        except Exception:
            self.depth_image = None

    def _depth_at_hand(self, lm):
        import numpy as np
        if self.depth_image is None:
            return None
        # Palm centre from wrist and MCP joints. Kinect v1 RGB/depth are not
        # perfectly registered, so use the nearest robust depth in a broad ROI.
        x = sum(lm[i].x for i in (0, 5, 9, 13, 17)) / 5.0
        y = sum(lm[i].y for i in (0, 5, 9, 13, 17)) / 5.0
        h, w = self.depth_image.shape
        cx = int(max(0, min(w - 1, x * w)))
        cy = int(max(0, min(h - 1, y * h)))
        # Kinect v1 RGB and depth images are not hardware-registered. Their
        # offset grows at close range, so a small same-coordinate ROI can land
        # on the background even while MediaPipe sees the hand correctly.
        # Search a wider neighbourhood and select the near robust surface; on
        # the bench this is the hand held in front of the operator.
        radius = max(16, int(min(h, w) * 0.10))
        roi = self.depth_image[
            max(0, cy - radius):min(h, cy + radius + 1),
            max(0, cx - radius):min(w, cx + radius + 1),
        ]
        valid = roi[(roi >= 350) & (roi <= 4000)]
        if valid.size < 40:
            return None
        return float(np.percentile(valid, 10)) / 1000.0

    def _process_hand_field(self, lm, detected, now):
        depth_m = self._depth_at_hand(lm)
        if detected == 'open_palm':
            if self.gesture_since is not None and (
                now - self.gesture_since
                >= float(self.get_parameter('gesture_hold_time').value)
            ):
                self.last_trigger = now
                self.hand_field_active = False
                self.hand_field_state = 'paused_open_palm'
                self._stop_field()
                self.get_logger().warn(
                    'Hand-field paused by open palm; hold index up to restart'
                )
            return

        if not self.hand_field_active:
            if detected != 'index_up':
                return
            if self.gesture_since is None or (
                now - self.gesture_since
                < float(self.get_parameter('gesture_hold_time').value)
            ):
                return
            if depth_m is None:
                self.hand_field_state = 'blocked_no_depth'
                self._stop_field()
                return
            self.baseline_depth_m = depth_m
            self.baseline_hand_x = sum(
                lm[i].x for i in (0, 5, 9, 13, 17)
            ) / 5.0
            self.hand_field_active = True
            self.hand_field_state = 'running'
            self.smooth_linear = 0.0
            self.smooth_angular = 0.0
            self._stop_field()
            self.get_logger().warn(
                f'Hand-field running; 1 m baseline={depth_m:.3f} m'
            )
            return

        if depth_m is None or self.baseline_depth_m is None:
            self.hand_field_state = 'stopped_no_depth'
            self.hand_field_active = False
            self._stop_field()
            return

        span = max(0.05, float(self.get_parameter('hand_field_span_m').value))
        min_m = float(self.get_parameter('hand_field_min_m').value)
        max_m = float(self.get_parameter('hand_field_max_m').value)
        fraction = max(0.0, min(1.0, (depth_m - self.baseline_depth_m) / span))
        simulated_distance = min_m + fraction * (max_m - min_m)

        center_x = sum(lm[i].x for i in (0, 5, 9, 13, 17)) / 5.0
        reference_x = (
            center_x if self.baseline_hand_x is None else self.baseline_hand_x
        )
        hand_angle_deg = (
            (center_x - reference_x)
            * float(self.get_parameter('hand_field_fov_deg').value)
        )
        hand_angle_deg = max(-15.0, min(15.0, hand_angle_deg))
        body_angle_deg = max(
            -30.0,
            min(
                30.0,
                hand_angle_deg
                * float(self.get_parameter('hand_field_angle_scale').value),
            ),
        )
        self.simulated_distance_m = simulated_distance
        self.simulated_angle_deg = body_angle_deg

        target = float(self.get_parameter('hand_field_target_m').value)
        dist_error = simulated_distance - target
        angle_error = math.radians(body_angle_deg)
        distance_deadband = float(
            self.get_parameter('hand_field_distance_deadband_m').value
        )
        if dist_error <= distance_deadband:
            target_linear = 0.0
        else:
            target_linear = (
                float(self.get_parameter('hand_field_kp_linear').value)
                * dist_error
            )
        target_linear = max(
            0.0,
            min(
                float(self.get_parameter('hand_field_max_linear').value),
                target_linear,
            ),
        )
        if target_linear == 0.0:
            # Do not let the smoothing ramp continue moving the robot after the
            # subject has reached or crossed the target distance.
            self.smooth_linear = 0.0
            self.smooth_angular = 0.0
            target_angular = 0.0
        elif abs(body_angle_deg) < float(
            self.get_parameter('hand_field_angle_deadband_deg').value
        ):
            target_angular = 0.0
            # Reaching the centre is a positioning stop, not a gradual coast.
            self.smooth_angular = 0.0
        else:
            target_angular = (
                float(self.get_parameter('hand_field_kp_angular').value)
                * angle_error
            )
        target_angular = max(
            -float(self.get_parameter('hand_field_max_angular').value),
            min(
                float(self.get_parameter('hand_field_max_angular').value),
                target_angular,
            ),
        )
        alpha = max(
            0.0,
            min(
                1.0,
                float(self.get_parameter('hand_field_smoothing_alpha').value),
            ),
        )
        # Remove steering memory when the target crosses from one side to the
        # other. Otherwise the low-pass filter briefly continues the old turn
        # and produces visible overshoot before reversing.
        if (
            target_angular != 0.0
            and self.smooth_angular != 0.0
            and target_angular * self.smooth_angular < 0.0
        ):
            self.smooth_angular = 0.0
        self.smooth_linear += alpha * (target_linear - self.smooth_linear)
        self.smooth_angular += alpha * (target_angular - self.smooth_angular)
        twist = Twist()
        twist.linear.x = self.smooth_linear
        twist.angular.z = self.smooth_angular
        self.pub_cmd.publish(twist)

    def safety_tick(self):
        if not self.hand_field_armed:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        timeout = float(self.get_parameter('hand_field_lost_timeout').value)
        if not self.stadia_connected:
            self._disarm_field('stopped_stadia_lost')
        elif self.hand_field_active and now - self.hand_last_seen > timeout:
            self.hand_field_active = False
            self.hand_field_state = 'stopped_hand_lost'
            self._stop_field()
        elif self.hand_field_active and now - self.depth_stamp > timeout:
            self.hand_field_active = False
            self.hand_field_state = 'stopped_depth_lost'
            self._stop_field()

    def _stop_field(self):
        self.smooth_linear = 0.0
        self.smooth_angular = 0.0
        self.pub_cmd.publish(Twist())

    def _disarm_field(self, state):
        self.hand_field_armed = False
        self.hand_field_active = False
        self.hand_field_state = state
        self.baseline_depth_m = None
        self.baseline_hand_x = None
        self._stop_field()

    def _publish_field_status(self, state, held=0.0):
        self.pub_status.publish(String(data=json.dumps({
            'state': state,
            'held': round(float(held), 2),
            'hand_field_armed': self.hand_field_armed,
            'hand_field_active': self.hand_field_active,
            'simulated_distance_m': (
                None if self.simulated_distance_m is None
                else round(self.simulated_distance_m, 3)
            ),
            'simulated_angle_deg': (
                None if self.simulated_angle_deg is None
                else round(self.simulated_angle_deg, 2)
            ),
            'linear_cmd': round(self.smooth_linear, 3),
            'angular_cmd': round(self.smooth_angular, 3),
        })))

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
