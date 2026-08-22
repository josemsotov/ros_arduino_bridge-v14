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
import os
import time

import cv2
import mediapipe as mp
import numpy as np
import rclpy
import supervision as sv
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
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
        self.declare_parameter('visual_identity_verify_once', True)
        self.declare_parameter('visual_target_fov_deg', 62.0)
        self.declare_parameter('person_detection_width', 384)
        self.declare_parameter('person_detection_stride', 3)
        self.declare_parameter('person_min_confidence', 0.35)
        self.declare_parameter('person_track_timeout', 0.80)
        self.declare_parameter('coral_person_enabled', True)
        self.declare_parameter('coral_detection_file', '/run/user/1000/coral_person.json')
        self.declare_parameter('coral_detection_timeout', 0.75)
        self.declare_parameter('lidar_visual_gate_deg', 14.0)
        self.declare_parameter('lidar_camera_yaw_deg', 90.0)
        self.declare_parameter('lidar_cluster_jump_m', 0.30)
        self.declare_parameter('lidar_min_cluster_points', 3)
        self.declare_parameter('lidar_distance_smoothing_alpha', 0.25)
        self.declare_parameter('lidar_angle_smoothing_alpha', 0.35)
        self.declare_parameter('lidar_max_target_jump_m', 0.60)
        self.declare_parameter('lidar_continuity_accept_m', 0.25)
        self.declare_parameter('lidar_switch_confirm_scans', 3)
        self.declare_parameter('lidar_candidate_match_m', 0.20)
        self.declare_parameter('face_distance_scale_m', 0.185)
        self.declare_parameter('lidar_face_distance_gate_m', 0.45)
        self.declare_parameter('face_distance_timeout', 1.50)
        self.declare_parameter('lidar_initial_distance_hint_m', 1.30)
        self.declare_parameter('lidar_initial_distance_gate_m', 0.35)
        self.declare_parameter('min_linear_vel', 0.06)
        self.declare_parameter('linear_deadband', 0.06)
        self.declare_parameter('angular_deadband_deg', 2.0)
        self.declare_parameter('velocity_smoothing_alpha', 0.28)
        self.declare_parameter('target_speed_alpha', 0.25)
        self.declare_parameter('target_speed_deadband', 0.04)
        self.declare_parameter('early_follow_min_scale', 0.25)
        self.declare_parameter('pivot_angle_deg', 18.0)
        self.declare_parameter('max_pivot_angular', 0.18)
        self.declare_parameter('moving_turn_ratio', 1.50)
        self.declare_parameter('gesture_test_linear_vel', 0.14)
        self.declare_parameter('gesture_test_angular_vel', 0.30)
        self.declare_parameter('gesture_test_duration', 1.0)
        # Face-only static calibration. It always starts in dry-run.
        self.declare_parameter('face_static_dry_run', True)
        self.declare_parameter('face_detection_confidence', 0.30)
        self.declare_parameter('face_identity_samples', 8)
        self.declare_parameter('face_identity_threshold', 0.78)
        self.declare_parameter('face_session_lost_timeout', 2.0)
        self.declare_parameter('face_process_stride', 1)
        self.declare_parameter('face_target_width_ratio', 0.22)
        self.declare_parameter('face_center_deadband', 0.055)
        self.declare_parameter('face_width_deadband', 0.025)
        self.declare_parameter('face_linear_kp', 0.90)
        self.declare_parameter('face_angular_kp', 1.10)
        self.declare_parameter('face_max_linear_vel', 0.15)
        self.declare_parameter('face_max_reverse_vel', 0.0)
        self.declare_parameter('face_max_angular_vel', 0.30)
        self.declare_parameter('require_face_session_to_start', True)

        self.enabled = False
        self.mode = 'WAITING'
        self.target_dist = None
        self.target_angle = None
        self.lidar_raw_target_dist = None
        self.lidar_raw_target_angle = None
        self.lidar_cluster_count = 0
        self.lidar_target_status = 'idle'
        self.lidar_pending_dist = None
        self.lidar_pending_angle = None
        self.lidar_pending_count = 0
        self.kinect_target_x = None
        self.identity_profile = None
        self.identity_samples = []
        self.identity_verified = False
        self.identity_score = 0.0
        self.identity_status = 'idle'
        self.identity_description = ''
        self.identity_last_verified = self.get_clock().now()
        self.identity_enroll_active = False
        self.identity_session_ok = False
        self.gesture_test_enabled = False
        self.gesture_test_label = ''
        self.gesture_test_until = self.get_clock().now()
        self.gesture_test_twist = Twist()
        self.smoothed_linear = 0.0
        self.smoothed_angular = 0.0
        self.estimated_target_speed = 0.0
        self.follow_activation_distance = None
        self.last_control_distance = None
        self.last_control_time = None
        self.person_detector = cv2.HOGDescriptor()
        self.person_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self.pose_detector = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=0,
            smooth_landmarks=True,
            min_detection_confidence=0.40,
            min_tracking_confidence=0.40,
        )
        self.person_tracker = sv.ByteTrack(frame_rate=10)
        self.person_frame_count = 0
        self.person_track_id = None
        self.person_box = None
        self.person_confidence = 0.0
        self.person_last_seen = self.get_clock().now()
        self.coral_status = 'waiting'
        self.coral_inference_ms = None
        self.coral_person_count = 0
        self.visual_target_angle = None
        face_confidence = float(
            self.get_parameter('face_detection_confidence').value
        )
        self.face_detector = mp.solutions.face_detection.FaceDetection(
            model_selection=0,
            min_detection_confidence=face_confidence,
        )
        self.face_detector_far = mp.solutions.face_detection.FaceDetection(
            model_selection=1,
            min_detection_confidence=face_confidence,
        )
        self.face_static_enabled = False
        self.face_identity_enroll_active = False
        self.face_identity_profile = None
        self.face_identity_samples = []
        self.face_identity_status = 'idle'
        self.face_identity_verified = False
        self.face_identity_session_ok = False
        self.face_identity_score = 0.0
        self.face_last_seen = self.get_clock().now()
        self.face_detected = False
        self.face_frame_count = 0
        self.face_x = None
        self.face_y = None
        self.face_width_ratio = None
        self.face_height_ratio = None
        self.face_distance_estimate = None
        self.face_lidar_distance_offset = None
        self.face_distance_last_seen = self.get_clock().now()
        self.face_predicted_linear = 0.0
        self.face_predicted_angular = 0.0
        # Fail closed: motion is rejected until Stadia explicitly reports
        # that FOLLOWER mode was intentionally selected.
        self.manual_override_active = True

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 5)
        self.pub_debug = self.create_publisher(Twist, '/follower/debug', 5)
        self.pub_state = self.create_publisher(String, '/follower/state', 5)
        self.pub_stadia = self.create_publisher(String, '/stadia/control', 5)

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 5)
        self.create_subscription(Bool, '/follower/enable', self.enable_cb, 5)
        self.create_subscription(
            Bool,
            '/follower/authorized_enable',
            self.authorized_enable_cb,
            5,
        )
        self.create_subscription(String, '/gesture/command', self.gesture_cb, 5)
        self.create_subscription(String, '/stadia/state', self.stadia_state_cb, 5)

        if self.get_parameter('use_kinect').value:
            self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 2)
        if self.get_parameter('visual_identity_enabled').value:
            self.create_subscription(Image, '/camera/rgb/image_raw', self.rgb_cb, 2)

        self.last_scan = self.get_clock().now()
        self.create_timer(0.1, self.safety_check)
        self.create_timer(0.05, self._gesture_test_tick)
        self.create_timer(0.5, self.publish_state)

        self.get_logger().info('Robot follower ready; waiting for /follower/enable true')

    def enable_cb(self, msg: Bool):
        if bool(msg.data) and self.manual_override_active:
            self.enabled = False
            self.mode = 'WAITING'
            self._stop()
            self.get_logger().warn(
                'Follower enable rejected; manual Stadia/off override is active'
            )
            self.publish_state()
            return
        if (
            bool(msg.data)
            and self.get_parameter('require_face_session_to_start').value
            and not self.face_identity_session_ok
        ):
            self.enabled = False
            self.mode = 'WAITING'
            self._stop()
            self.get_logger().warn(
                'Follower enable rejected; verified face session is required'
            )
            self.publish_state()
            return
        self.enabled = bool(msg.data)
        if self.enabled:
            self._reset_lidar_target('acquiring')
            self.face_static_enabled = False
            self.face_identity_enroll_active = False
            self.gesture_test_enabled = False
            self.gesture_test_label = ''
            self.mode = 'FOLLOW'
            self.identity_enroll_active = False
            if self.identity_profile is None:
                self._reset_identity()
            else:
                self.identity_status = 'armed'
                self.identity_verified = True
                self.identity_session_ok = True
                self.identity_last_verified = self.get_clock().now()
            self.get_logger().info('Follower enabled')
        else:
            self.manual_override_active = True
            if not self.face_static_enabled:
                self.mode = 'WAITING'
            self.identity_enroll_active = False
            self.get_logger().info('Follower disabled; motors stopped')
            self._stop()
            # Do not echo STADIA here. StadiaNode publishes /follower/enable
            # False while entering manual/off mode; replying on
            # /stadia/control would re-enter _set_stadia_mode(), which emits
            # another False and creates an unbounded control feedback loop.
            # Explicit STOP/FACE_STATIC_OFF paths restore Stadia themselves.
        self.publish_state()

    def authorized_enable_cb(self, msg: Bool):
        if not bool(msg.data):
            self.enable_cb(msg)
            return
        # This topic is published only by StadiaNode after an intentional
        # FOLLOWER selection, avoiding cross-topic ordering races.
        self.manual_override_active = False
        self.enable_cb(msg)

    def stadia_state_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
            mode = str(payload.get('mode', '')).strip().lower()
        except (json.JSONDecodeError, TypeError, ValueError):
            return
        if mode not in ('stadia', 'off', 'follower'):
            return
        # Status is informational and may arrive late. It may only tighten
        # safety. FOLLOWER authorization is granted exclusively by
        # /follower/authorized_enable.
        if mode in ('stadia', 'off'):
            self.manual_override_active = True
        if self.manual_override_active and self.enabled:
            self.enabled = False
            self.mode = 'WAITING'
            self._stop()
            self.get_logger().warn(
                f'{mode.upper()} override received; follower stopped'
            )
            self.publish_state()

    def gesture_cb(self, msg: String):
        command = msg.data.strip()
        try:
            payload = json.loads(command)
            command = str(payload.get('command', command))
        except json.JSONDecodeError:
            pass

        command = command.upper()
        if (
            self.manual_override_active
            and command in (
                'APPROACH', 'COME_CLOSE', 'PALM_OPEN',
                'FOLLOW', 'FOLLOW_ME', 'RESUME',
                'GESTURE_TEST_ON', 'GESTURE_TEST_ENABLE',
            )
        ):
            self.enabled = False
            self.mode = 'WAITING'
            self._stop()
            self.get_logger().warn(
                f'Gesture {command} rejected; manual override is active'
            )
            self.publish_state()
            return
        if (
            command in (
                'APPROACH', 'COME_CLOSE', 'PALM_OPEN',
                'FOLLOW', 'FOLLOW_ME', 'RESUME',
            )
            and self.get_parameter('require_face_session_to_start').value
            and not self.face_identity_session_ok
        ):
            self.enabled = False
            self.mode = 'WAITING'
            self._stop()
            self.get_logger().warn(
                f'Gesture {command} rejected; verified face session is required'
            )
            self.publish_state()
            return
        if command in ('FACE_STATIC_ENROLL', 'FACE_CALIBRATE_ENROLL'):
            self.enabled = False
            self.gesture_test_enabled = False
            self._reset_lidar_target('acquiring')
            self.face_static_enabled = True
            self.face_identity_enroll_active = True
            self.face_identity_profile = None
            self.face_identity_samples = []
            self.face_identity_status = 'enrolling'
            self.face_identity_verified = False
            self.face_identity_session_ok = False
            self.face_identity_score = 0.0
            self.face_distance_estimate = None
            self.face_lidar_distance_offset = None
            self.face_predicted_linear = 0.0
            self.face_predicted_angular = 0.0
            self.mode = 'FACE_STATIC_DRY_RUN'
            self.pub_stadia.publish(String(data='OFF'))
            self._stop()
            self.get_logger().info(
                'Face static enrollment started; motors forced to zero'
            )
        elif command in ('FACE_STATIC_ON', 'FACE_CALIBRATE_ON'):
            self.enabled = False
            self.gesture_test_enabled = False
            self._reset_lidar_target('acquiring')
            self.face_static_enabled = True
            self.face_identity_enroll_active = False
            self.mode = 'FACE_STATIC_DRY_RUN'
            self.pub_stadia.publish(String(data='OFF'))
            self._stop()
            self.face_identity_session_ok = False
            if self.face_identity_profile is None:
                self.face_identity_status = 'enrollment_required'
                self.face_identity_verified = False
            else:
                self.face_identity_status = 'verifying'
                self.face_identity_verified = False
            self.get_logger().info(
                'Face static dry-run enabled; motors forced to zero'
            )
        elif command in ('FACE_STATIC_OFF', 'FACE_CALIBRATE_OFF'):
            self.face_static_enabled = False
            self.face_identity_enroll_active = False
            self.face_identity_session_ok = False
            self.face_predicted_linear = 0.0
            self.face_predicted_angular = 0.0
            self.mode = 'WAITING'
            self._stop()
            self.pub_stadia.publish(String(data='STADIA'))
            self.get_logger().info('Face static mode disabled; Stadia restored')
        elif command in ('GESTURE_TEST_ON', 'GESTURE_TEST_ENABLE'):
            self.enabled = False
            self.mode = 'GESTURE_TEST'
            self.gesture_test_enabled = True
            self.gesture_test_label = 'ready'
            self._stop()
            self.get_logger().info('Gesture constant-speed test enabled')
        elif command in ('GESTURE_TEST_OFF', 'GESTURE_TEST_DISABLE'):
            self.enabled = False
            self.mode = 'WAITING'
            self.gesture_test_enabled = False
            self.gesture_test_label = ''
            self._stop()
            self.get_logger().info('Gesture constant-speed test disabled')
        elif self.gesture_test_enabled and command in (
            'APPROACH', 'COME_CLOSE', 'PALM_OPEN',
            'FOLLOW', 'FOLLOW_ME', 'RESUME',
            'STADIA_ON', 'STADIA_OFF',
            'STOP', 'WAIT', 'PAUSE',
        ):
            self._handle_gesture_test_command(command)
        elif command in ('APPROACH', 'COME_CLOSE', 'PALM_OPEN'):
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
            self.face_static_enabled = False
            self.face_identity_enroll_active = False
            self.face_identity_session_ok = False
            self.face_predicted_linear = 0.0
            self.face_predicted_angular = 0.0
            self.mode = 'WAITING'
            self.pub_stadia.publish(String(data='STADIA'))
            self._stop()
            self.get_logger().info('Stop/wait command received')
        elif command == 'STADIA_ON':
            self.pub_stadia.publish(__import__('std_msgs.msg', fromlist=['String']).String(data='ON'))
            self.get_logger().info('Stadia ON via gesture')
        elif command == 'STADIA_OFF':
            self.pub_stadia.publish(__import__('std_msgs.msg', fromlist=['String']).String(data='OFF'))
            self.get_logger().info('Stadia OFF via gesture')
        self.publish_state()

    def depth_cb(self, msg: Image):
        """Use Kinect depth as an optional angle hint."""
        if not self.enabled:
            return
        try:
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            center = msg.height // 2
            band = depth[max(0, center - 20):min(msg.height, center + 20)]
            valid = (band > 300) & (band < 4000)
            counts = valid.sum(axis=0)
            sums = np.where(valid, band, 0).sum(axis=0, dtype=np.float64)
            averages = np.full(msg.width, np.inf, dtype=np.float64)
            np.divide(sums, counts, out=averages, where=counts > 0)
            if not np.isfinite(averages).any():
                self.kinect_target_x = None
                return
            best_col = int(np.argmin(averages))
            self.kinect_target_x = (best_col - msg.width / 2) / (msg.width / 2) * 0.497
        except Exception:
            self.kinect_target_x = None

    def rgb_cb(self, msg: Image):
        if not (
            self.enabled
            or self.identity_enroll_active
            or self.face_static_enabled
            or self.face_identity_enroll_active
        ):
            return
        if not self.get_parameter('visual_identity_enabled').value:
            return

        image = self._image_to_rgb(msg)
        if image is None:
            self.identity_status = 'bad_image'
            return

        if self.face_static_enabled or self.face_identity_enroll_active:
            # Keep the body bearing current so LiDAR association can be
            # validated in telemetry-only face-static mode.
            self._update_person_track(image)
            self._process_face_static(image)
            return

        self._update_person_track(image)
        if not self._person_track_is_fresh():
            self.identity_status = 'no_person_track'
            self.identity_verified = False
            return

        signature, description = self._visual_signature(image, self.person_box)
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
                self.identity_session_ok = True
                self.identity_score = 1.0
                self.identity_status = 'locked'
                self.identity_enroll_active = False
                self.get_logger().info(f'Visual identity locked: {description}')
            return

        score = self._identity_similarity(signature, self.identity_profile)
        threshold = float(self.get_parameter('visual_identity_threshold').value)
        self.identity_score = score
        self.identity_description = description
        if self.get_parameter('visual_identity_verify_once').value and self.identity_session_ok:
            self.identity_verified = True
            self.identity_status = 'session_ok' if score < threshold else 'verified'
            if score >= threshold:
                self.identity_last_verified = self.get_clock().now()
            return
        if score >= threshold:
            self.identity_verified = True
            self.identity_session_ok = True
            self.identity_last_verified = self.get_clock().now()
            self.identity_status = 'verified'
        else:
            self.identity_verified = False
            self.identity_status = 'mismatch'

    def _process_face_static(self, image):
        self.face_frame_count += 1
        stride = max(1, int(self.get_parameter('face_process_stride').value))
        if self.face_frame_count % stride:
            return

        near_result = self.face_detector.process(image)
        far_result = self.face_detector_far.process(image)
        detections = []
        if near_result and near_result.detections:
            detections.extend(near_result.detections)
        if far_result and far_result.detections:
            detections.extend(far_result.detections)
        if not detections:
            self.face_detected = False
            missing_age = (
                self.get_clock().now() - self.face_last_seen
            ).nanoseconds / 1e9
            if missing_age > float(
                self.get_parameter('face_session_lost_timeout').value
            ):
                self.face_identity_session_ok = False
            self.face_x = None
            self.face_y = None
            self.face_width_ratio = None
            self.face_height_ratio = None
            self.face_identity_verified = False
            self.face_identity_status = 'no_face'
            self.face_predicted_linear = 0.0
            self.face_predicted_angular = 0.0
            self._stop()
            return

        h, w = image.shape[:2]
        candidates = []
        for detection in detections:
            box = detection.location_data.relative_bounding_box
            x1 = max(0, int(box.xmin * w))
            y1 = max(0, int(box.ymin * h))
            x2 = min(w, int((box.xmin + box.width) * w))
            y2 = min(h, int((box.ymin + box.height) * h))
            if x2 - x1 >= 24 and y2 - y1 >= 24:
                candidates.append(((x2 - x1) * (y2 - y1), (x1, y1, x2, y2)))
        if not candidates:
            self.face_detected = False
            self.face_x = None
            self.face_y = None
            self.face_width_ratio = None
            self.face_height_ratio = None
            self.face_identity_verified = False
            self.face_identity_status = 'invalid_face_box'
            self.face_predicted_linear = 0.0
            self.face_predicted_angular = 0.0
            self._stop()
            return

        _, (x1, y1, x2, y2) = max(candidates, key=lambda item: item[0])
        self.face_detected = True
        self.face_last_seen = self.get_clock().now()
        self.face_x = ((x1 + x2) * 0.5) / w
        self.face_y = ((y1 + y2) * 0.5) / h
        self.face_width_ratio = (x2 - x1) / w
        self.face_height_ratio = (y2 - y1) / h
        distance_scale = float(
            self.get_parameter('face_distance_scale_m').value
        )
        measured_face_distance = distance_scale / max(
            0.03, float(self.face_width_ratio)
        )
        self.face_distance_estimate = (
            measured_face_distance
            if self.face_distance_estimate is None
            else 0.75 * self.face_distance_estimate
            + 0.25 * measured_face_distance
        )
        self.face_distance_last_seen = self.get_clock().now()

        signature = self._face_signature(image[y1:y2, x1:x2])
        if signature is None:
            self.face_identity_verified = False
            self.face_identity_status = 'bad_face_sample'
            self.face_predicted_linear = 0.0
            self.face_predicted_angular = 0.0
            self._stop()
            return

        if self.face_identity_enroll_active:
            self.face_identity_samples.append(signature)
            needed = max(
                2, int(self.get_parameter('face_identity_samples').value)
            )
            count = len(self.face_identity_samples)
            self.face_identity_status = f'capturing {count}/{needed}'
            if count >= needed:
                profile = np.mean(
                    np.vstack(self.face_identity_samples[-needed:]), axis=0
                )
                norm = float(np.linalg.norm(profile))
                self.face_identity_profile = profile / max(norm, 1e-6)
                self.face_identity_enroll_active = False
                self.face_identity_verified = True
                self.face_identity_session_ok = True
                self.face_identity_score = 1.0
                self.face_identity_status = 'locked'
                self.get_logger().info('Face identity locked for static test')
            self.face_predicted_linear = 0.0
            self.face_predicted_angular = 0.0
            self._stop()
            return

        if self.face_identity_profile is None:
            self.face_identity_verified = False
            self.face_identity_status = 'enrollment_required'
            self.face_predicted_linear = 0.0
            self.face_predicted_angular = 0.0
            self._stop()
            return

        score = float(np.dot(signature, self.face_identity_profile))
        self.face_identity_score = max(-1.0, min(1.0, score))
        threshold = float(self.get_parameter('face_identity_threshold').value)
        if self.face_identity_session_ok:
            self.face_identity_verified = True
            self.face_identity_status = (
                'verified' if score >= threshold else 'session_ok'
            )
        elif score >= threshold:
            self.face_identity_verified = True
            self.face_identity_session_ok = True
            self.face_identity_status = 'verified'
        else:
            self.face_identity_verified = False
            self.face_identity_status = 'mismatch'
        if not self.face_identity_verified:
            self.face_predicted_linear = 0.0
            self.face_predicted_angular = 0.0
            self._stop()
            return

        x_error = float(self.face_x) - 0.5
        target_width = float(
            self.get_parameter('face_target_width_ratio').value
        )
        width_error = target_width - float(self.face_width_ratio)
        if abs(x_error) < float(
            self.get_parameter('face_center_deadband').value
        ):
            x_error = 0.0
        if abs(width_error) < float(
            self.get_parameter('face_width_deadband').value
        ):
            width_error = 0.0

        linear = float(self.get_parameter('face_linear_kp').value) * width_error
        angular = -float(self.get_parameter('face_angular_kp').value) * x_error
        linear = max(
            0.0,
            min(float(self.get_parameter('face_max_linear_vel').value), linear),
        )
        angular_limit = float(
            self.get_parameter('face_max_angular_vel').value
        )
        angular = max(-angular_limit, min(angular_limit, angular))
        self.face_predicted_linear = linear
        self.face_predicted_angular = angular

        # Static phase is deliberately telemetry-only. Even if the parameter is
        # accidentally changed, this first implementation never drives motors.
        self._stop()

    @staticmethod
    def _face_signature(face_crop):
        if face_crop is None or face_crop.size == 0:
            return None
        crop = cv2.resize(face_crop, (64, 64), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(crop, cv2.COLOR_RGB2GRAY)
        hsv = cv2.cvtColor(crop, cv2.COLOR_RGB2HSV)

        gray_hist = cv2.calcHist([gray], [0], None, [32], [0, 256]).reshape(-1)
        hue_hist = cv2.calcHist([hsv], [0], None, [24], [0, 180]).reshape(-1)
        sat_hist = cv2.calcHist([hsv], [1], None, [16], [0, 256]).reshape(-1)

        low = cv2.resize(gray, (16, 16), interpolation=cv2.INTER_AREA)
        dct = cv2.dct(low.astype(np.float32) / 255.0)[:6, :6].reshape(-1)
        dct = np.abs(dct)

        blocks = []
        for block in (gray_hist, hue_hist, sat_hist, dct):
            block = block.astype(np.float32)
            norm = float(np.linalg.norm(block))
            if norm < 1e-6:
                return None
            blocks.append(block / norm)
        vector = np.concatenate(blocks)
        norm = float(np.linalg.norm(vector))
        return vector / max(norm, 1e-6)

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

    def _visual_signature(self, image, person_box=None):
        h, w = image.shape[:2]
        if person_box is None:
            cx = int(max(0, min(w - 1, self._target_x_norm() * (w - 1))))
            half_w = max(16, int(w * 0.10))
            x1, x2 = max(0, cx - half_w), min(w, cx + half_w)
            top, bottom = 0, h
        else:
            x1, top, x2, bottom = [int(v) for v in person_box]
            x1, x2 = max(0, x1), min(w, x2)
            top, bottom = max(0, top), min(h, bottom)
            if x2 - x1 < 16 or bottom - top < 32:
                return None, ''
        person_h = bottom - top
        bands = (
            ('head', top + int(person_h * 0.05), top + int(person_h * 0.28)),
            ('shirt', top + int(person_h * 0.30), top + int(person_h * 0.68)),
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

    def _coral_person_detections(self, width, height):
        if not self.get_parameter('coral_person_enabled').value:
            self.coral_status = 'disabled'
            return None
        path = str(self.get_parameter('coral_detection_file').value)
        try:
            with open(path, 'r', encoding='utf-8') as stream:
                payload = json.load(stream)
            age = time.time() - float(payload.get('stamp', 0.0))
            timeout = float(self.get_parameter('coral_detection_timeout').value)
            if age < -1.0 or age > timeout:
                self.coral_status = 'stale'
                self.coral_person_count = 0
                return None
            if not payload.get('ok', False):
                self.coral_status = 'error'
                self.coral_person_count = 0
                return None
            boxes = []
            scores = []
            min_conf = float(self.get_parameter('person_min_confidence').value)
            for person in payload.get('people', []):
                score = float(person.get('score', 0.0))
                box = person.get('box', [])
                if score < min_conf or len(box) != 4:
                    continue
                x1, y1, x2, y2 = [float(value) for value in box]
                if x2 <= x1 or y2 <= y1:
                    continue
                boxes.append([
                    max(0.0, min(1.0, x1)) * width,
                    max(0.0, min(1.0, y1)) * height,
                    max(0.0, min(1.0, x2)) * width,
                    max(0.0, min(1.0, y2)) * height,
                ])
                scores.append(score)
            self.coral_status = 'active'
            self.coral_inference_ms = payload.get('inference_ms')
            self.coral_person_count = len(boxes)
            return boxes, scores
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            self.coral_status = 'unavailable'
            self.coral_person_count = 0
            return None

    def _update_person_track(self, image):
        self.person_frame_count += 1
        stride = max(1, int(self.get_parameter('person_detection_stride').value))
        if self.person_frame_count % stride:
            return

        h, w = image.shape[:2]
        detect_width = max(160, int(self.get_parameter('person_detection_width').value))
        scale = min(1.0, detect_width / float(w))
        frame = cv2.resize(image, (int(w * scale), int(h * scale))) if scale < 1.0 else image
        boxes, scores = [], []
        coral_detections = self._coral_person_detections(w, h)
        if coral_detections is not None:
            boxes, scores = coral_detections
        min_conf = float(self.get_parameter('person_min_confidence').value)
        inv_scale = 1.0 / scale
        pose_result = self.pose_detector.process(frame) if not boxes else None
        if pose_result is not None and pose_result.pose_landmarks:
            visible = [
                landmark for landmark in pose_result.pose_landmarks.landmark
                if landmark.visibility >= 0.35
            ]
            if len(visible) >= 6:
                xs = [landmark.x * frame.shape[1] for landmark in visible]
                ys = [landmark.y * frame.shape[0] for landmark in visible]
                x1, x2 = min(xs), max(xs)
                y1, y2 = min(ys), max(ys)
                box_w, box_h = max(20.0, x2 - x1), max(40.0, y2 - y1)
                x1, x2 = x1 - box_w * 0.20, x2 + box_w * 0.20
                y1, y2 = y1 - box_h * 0.18, y2 + box_h * 0.30
                score = float(np.mean([landmark.visibility for landmark in visible]))
                boxes.append([
                    max(0.0, x1) * inv_scale,
                    max(0.0, y1) * inv_scale,
                    min(float(frame.shape[1] - 1), x2) * inv_scale,
                    min(float(frame.shape[0] - 1), y2) * inv_scale,
                ])
                scores.append(score)

        # HOG remains as a fallback when a full body is visible but Pose misses it.
        if not boxes:
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            rects, weights = self.person_detector.detectMultiScale(
                bgr, winStride=(8, 8), padding=(8, 8), scale=1.05
            )
            for (x, y, bw, bh), confidence in zip(rects, weights):
                score = float(confidence)
                if score < min_conf:
                    continue
                boxes.append([x * inv_scale, y * inv_scale,
                              (x + bw) * inv_scale, (y + bh) * inv_scale])
                scores.append(score)

        detections = sv.Detections(
            xyxy=np.asarray(boxes, dtype=np.float32).reshape(-1, 4),
            confidence=np.asarray(scores, dtype=np.float32),
            class_id=np.zeros(len(boxes), dtype=int),
        )
        tracked = self.person_tracker.update_with_detections(detections)
        if len(tracked) == 0:
            return

        ids = tracked.tracker_id
        selected = None
        if self.person_track_id is not None and ids is not None:
            matches = np.flatnonzero(ids == self.person_track_id)
            if len(matches):
                selected = int(matches[0])
        if selected is None:
            expected_x = self._target_x_norm()
            centers = (tracked.xyxy[:, 0] + tracked.xyxy[:, 2]) * 0.5 / max(1, w)
            selected = int(np.argmin(np.abs(centers - expected_x)))

        self.person_box = tracked.xyxy[selected].copy()
        self.person_track_id = int(ids[selected]) if ids is not None else None
        self.person_confidence = (
            float(tracked.confidence[selected]) if tracked.confidence is not None else 0.0
        )
        center_x = float(self.person_box[0] + self.person_box[2]) * 0.5 / max(1, w)
        fov = math.radians(float(self.get_parameter('visual_target_fov_deg').value))
        self.visual_target_angle = (center_x - 0.5) * fov
        self.person_last_seen = self.get_clock().now()

    def _person_track_is_fresh(self):
        if self.person_box is None:
            return False
        timeout = float(self.get_parameter('person_track_timeout').value)
        age = (self.get_clock().now() - self.person_last_seen).nanoseconds / 1e9
        return age <= timeout

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
        self.identity_session_ok = False
        self.identity_score = 0.0
        self.identity_status = 'capturing'
        self.identity_description = ''
        self.identity_last_verified = self.get_clock().now()
        self.person_track_id = None
        self.person_box = None
        self.visual_target_angle = None
        self.person_tracker.reset()
        self._reset_motion_filter()

    def _identity_allows_motion(self):
        if not self.get_parameter('visual_identity_enabled').value:
            return True
        if not self.get_parameter('visual_identity_required').value:
            return True
        if not self._person_track_is_fresh() or self.identity_profile is None:
            return False
        if self.get_parameter('visual_identity_verify_once').value:
            return self.identity_session_ok
        timeout = float(self.get_parameter('visual_identity_timeout').value)
        age = (self.get_clock().now() - self.identity_last_verified).nanoseconds / 1e9
        return age <= timeout

    def scan_cb(self, msg: LaserScan):
        self.last_scan = self.get_clock().now()
        if self.gesture_test_enabled:
            return
        if not self.enabled and not self.face_static_enabled:
            return

        p = self.get_all_params()
        if (
            self.get_parameter('visual_identity_enabled').value
            and not self._person_track_is_fresh()
        ):
            self.lidar_pending_dist = None
            self.lidar_pending_angle = None
            self.lidar_pending_count = 0
            self.lidar_target_status = 'no_person_track'
            self._stop()
            self.publish_state()
            return
        if (
            self.target_dist is None
            and self.get_parameter('visual_identity_enabled').value
            and not self.face_identity_session_ok
        ):
            self.lidar_target_status = 'awaiting_verified_face'
            self._stop()
            self.publish_state()
            return

        search_rad = math.radians(p['search_angle_deg'])
        min_rad = math.radians(p['min_angle_deg'])
        lidar_camera_yaw = math.radians(
            float(self.get_parameter('lidar_camera_yaw_deg').value)
        )
        points = []

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < 0.1 or r > p['max_detect_dist']:
                continue
            raw_angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(
                math.sin(raw_angle - lidar_camera_yaw),
                math.cos(raw_angle - lidar_camera_yaw),
            )
            if abs(angle) < min_rad or abs(angle) > search_rad:
                continue
            if self.visual_target_angle is not None:
                gate = math.radians(float(self.get_parameter('lidar_visual_gate_deg').value))
                if abs(angle - self.visual_target_angle) > gate:
                    continue
            points.append((i, float(r), angle))

        if not points:
            self.lidar_cluster_count = 0
            self.lidar_target_status = 'no_points'
            self.get_logger().warn('No target in range', throttle_duration_sec=2)
            self._stop()
            return

        jump_limit = float(self.get_parameter('lidar_cluster_jump_m').value)
        clusters = []
        current = [points[0]]
        for point in points[1:]:
            previous = current[-1]
            if point[0] - previous[0] <= 2 and abs(point[1] - previous[1]) <= jump_limit:
                current.append(point)
            else:
                clusters.append(current)
                current = [point]
        clusters.append(current)

        min_points = max(
            2, int(self.get_parameter('lidar_min_cluster_points').value)
        )
        clusters = [cluster for cluster in clusters if len(cluster) >= min_points]
        self.lidar_cluster_count = len(clusters)
        if not clusters:
            self.lidar_target_status = 'no_cluster'
            self.get_logger().warn(
                'No coherent LiDAR cluster in visual gate',
                throttle_duration_sec=2,
            )
            self._stop()
            return

        reference_angle = (
            self.visual_target_angle
            if self.visual_target_angle is not None
            else self.kinect_target_x
        )
        if reference_angle is None:
            reference_angle = self.target_angle if self.target_angle is not None else 0.0

        candidates = []
        max_jump = float(self.get_parameter('lidar_max_target_jump_m').value)
        for cluster in clusters:
            distance = float(np.median([point[1] for point in cluster]))
            angle = float(np.median([point[2] for point in cluster]))
            score = abs(angle - reference_angle)
            if self.target_dist is not None:
                score += 0.35 * abs(distance - self.target_dist) / max(0.1, max_jump)
            # Prefer a wider physical return when angle and continuity agree.
            score -= min(0.08, len(cluster) * 0.004)
            candidates.append((score, distance, angle, len(cluster)))

        face_distance_fresh = False
        if self.face_distance_estimate is not None:
            face_distance_age = (
                self.get_clock().now() - self.face_distance_last_seen
            ).nanoseconds / 1e9
            face_distance_fresh = face_distance_age <= float(
                self.get_parameter('face_distance_timeout').value
            )
        if face_distance_fresh and self.target_dist is not None:
            face_gate = float(
                self.get_parameter('lidar_face_distance_gate_m').value
            )
            expected_lidar_distance = self.face_distance_estimate
            if self.face_lidar_distance_offset is not None:
                expected_lidar_distance += self.face_lidar_distance_offset
            compatible = [
                candidate
                for candidate in candidates
                if abs(candidate[1] - expected_lidar_distance) <= face_gate
            ]
            if not compatible:
                _, raw_dist, raw_angle, _ = min(
                    candidates, key=lambda item: item[0]
                )
                self.lidar_raw_target_dist = raw_dist
                self.lidar_raw_target_angle = raw_angle
                self.lidar_target_status = 'face_distance_mismatch'
                self._stop()
                self.publish_state()
                return
            candidates = compatible

        initial_hint = float(
            self.get_parameter('lidar_initial_distance_hint_m').value
        )
        if self.target_dist is None and initial_hint > 0.0:
            initial_gate = float(
                self.get_parameter('lidar_initial_distance_gate_m').value
            )
            initial_candidates = [
                candidate
                for candidate in candidates
                if abs(candidate[1] - initial_hint) <= initial_gate
            ]
            if not initial_candidates:
                _, raw_dist, raw_angle, _ = min(
                    candidates, key=lambda item: item[0]
                )
                self.lidar_raw_target_dist = raw_dist
                self.lidar_raw_target_angle = raw_angle
                self.lidar_target_status = 'initial_distance_mismatch'
                self._stop()
                self.publish_state()
                return
            candidates = initial_candidates

        _, raw_dist, raw_angle, cluster_size = min(
            candidates, key=lambda item: item[0]
        )
        self.lidar_raw_target_dist = raw_dist
        self.lidar_raw_target_angle = raw_angle
        if (
            self.target_dist is None
            and face_distance_fresh
            and self.face_distance_estimate is not None
        ):
            self.face_lidar_distance_offset = (
                raw_dist - self.face_distance_estimate
            )

        if (
            self.target_dist is not None
            and abs(raw_dist - self.target_dist) > max_jump
        ):
            self.lidar_target_status = 'jump_rejected'
            self.get_logger().warn(
                f'LiDAR target jump rejected: {self.target_dist:.2f} -> {raw_dist:.2f} m',
                throttle_duration_sec=1,
            )
            self._stop()
            self.publish_state()
            return

        continuity_limit = float(
            self.get_parameter('lidar_continuity_accept_m').value
        )
        if (
            self.target_dist is not None
            and abs(raw_dist - self.target_dist) > continuity_limit
        ):
            self.lidar_pending_dist = raw_dist
            self.lidar_pending_angle = raw_angle
            self.lidar_pending_count = 1
            self.lidar_target_status = 'distance_discontinuity'
            self._stop()
            self.publish_state()
            return

        self.lidar_pending_dist = None
        self.lidar_pending_angle = None
        self.lidar_pending_count = 0

        dist_alpha = max(
            0.05,
            min(
                1.0,
                float(
                    self.get_parameter('lidar_distance_smoothing_alpha').value
                ),
            ),
        )
        angle_alpha = max(
            0.05,
            min(
                1.0,
                float(self.get_parameter('lidar_angle_smoothing_alpha').value),
            ),
        )
        target_dist = (
            raw_dist
            if self.target_dist is None
            else self.target_dist + dist_alpha * (raw_dist - self.target_dist)
        )
        target_angle = (
            raw_angle
            if self.target_angle is None
            else self.target_angle + angle_alpha * (raw_angle - self.target_angle)
        )

        if self.visual_target_angle is not None:
            target_angle = 0.65 * target_angle + 0.35 * self.visual_target_angle
        elif self.kinect_target_x is not None:
            target_angle = 0.6 * target_angle + 0.4 * self.kinect_target_x

        self.target_dist = target_dist
        self.target_angle = target_angle
        self.lidar_target_status = f'tracking:{cluster_size}'
        if not self.enabled:
            # Face-static calibration observes the exact target association
            # used by FOLLOW without ever allowing motor output.
            self._stop()
            self._publish_debug(target_dist, target_angle)
            self.publish_state()
            return
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

        target = (p['follow_min_distance'] + p['follow_max_distance']) * 0.5
        dist_error = dist - target
        now = self.get_clock().now()
        if self.follow_activation_distance is None:
            self.follow_activation_distance = float(dist)
        if self.last_control_distance is not None and self.last_control_time is not None:
            dt = (now - self.last_control_time).nanoseconds / 1e9
            if 0.05 <= dt <= 1.0:
                relative_speed = (float(dist) - self.last_control_distance) / dt
                observed_speed = relative_speed + max(0.0, self.smoothed_linear)
                observed_speed = max(0.0, min(p['max_linear_vel'], observed_speed))
                speed_alpha = max(
                    0.05, min(1.0, float(p['target_speed_alpha']))
                )
                self.estimated_target_speed += speed_alpha * (
                    observed_speed - self.estimated_target_speed
                )
        self.last_control_distance = float(dist)
        self.last_control_time = now

        if abs(angle) <= math.radians(p['angular_deadband_deg']):
            angular = 0.0
        else:
            angular = -p['kp_angular'] * angle
            angular = max(
                -p['max_angular_vel'], min(p['max_angular_vel'], angular)
            )

        speed_deadband = float(p['target_speed_deadband'])
        target_speed = (
            self.estimated_target_speed
            if self.estimated_target_speed >= speed_deadband
            else 0.0
        )
        if dist_error <= p['linear_deadband']:
            # React as soon as the subject starts walking away, but deliberately
            # lag at first so the gap can grow toward the desired 2.5 m. At the
            # target distance the feed-forward reaches the subject's speed.
            baseline = min(float(self.follow_activation_distance), target - 0.1)
            span = max(0.1, target - baseline)
            progress = max(0.0, min(1.0, (float(dist) - baseline) / span))
            min_scale = max(
                0.0, min(1.0, float(p['early_follow_min_scale']))
            )
            linear = target_speed * (
                min_scale + (1.0 - min_scale) * progress
            )
            if target_speed == 0.0:
                linear = 0.0
        else:
            half_range = max(0.05, (p['follow_max_distance'] - p['follow_min_distance']) * 0.5)
            ratio = min(1.0, dist_error / half_range)
            speed = p['min_linear_vel'] + (p['max_linear_vel'] - p['min_linear_vel']) * (ratio ** 1.25)
            linear = max(
                0.0, min(p['max_linear_vel'], max(speed, target_speed))
            )

        if linear <= 0.0 and float(dist) <= target:
            self.smoothed_linear = 0.0
        elif linear > 0.0:
            # A real early-follow command must clear the traction dead zone.
            linear = max(float(p['min_linear_vel']), linear)

        pivot_angle = math.radians(float(p['pivot_angle_deg']))
        if linear < float(p['min_linear_vel']):
            if abs(angle) < pivot_angle:
                angular = 0.0
            else:
                pivot_limit = abs(float(p['max_pivot_angular']))
                angular = max(-pivot_limit, min(pivot_limit, angular))
        else:
            # Keep a forward arc: do not let angular correction dominate a
            # small linear command and drive one wheel backwards.
            turn_limit = max(
                0.05, abs(float(linear)) * float(p['moving_turn_ratio'])
            )
            angular = max(-turn_limit, min(turn_limit, angular))
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
        self.estimated_target_speed = 0.0
        self.follow_activation_distance = None
        self.last_control_distance = None
        self.last_control_time = None

    def _reset_lidar_target(self, status='idle'):
        self.target_dist = None
        self.target_angle = None
        self.lidar_raw_target_dist = None
        self.lidar_raw_target_angle = None
        self.lidar_cluster_count = 0
        self.lidar_pending_dist = None
        self.lidar_pending_angle = None
        self.lidar_pending_count = 0
        self.lidar_target_status = status
        self.face_lidar_distance_offset = None

    def _handle_gesture_test_command(self, command):
        if command in ('STOP', 'WAIT', 'PAUSE'):
            self.gesture_test_label = 'stop'
            self._stop()
            return

        linear = float(self.get_parameter('gesture_test_linear_vel').value)
        angular = float(self.get_parameter('gesture_test_angular_vel').value)
        if command in ('APPROACH', 'COME_CLOSE', 'PALM_OPEN'):
            self._start_gesture_test_motion(linear, 0.0, 'approach_const')
        elif command in ('FOLLOW', 'FOLLOW_ME', 'RESUME'):
            self._start_gesture_test_motion(linear, 0.0, 'follow_const')
        elif command == 'STADIA_ON':
            self._start_gesture_test_motion(0.0, angular, 'turn_left_const')
        elif command == 'STADIA_OFF':
            self._start_gesture_test_motion(0.0, -angular, 'turn_right_const')

    def _start_gesture_test_motion(self, linear, angular, label):
        duration = max(0.1, float(self.get_parameter('gesture_test_duration').value))
        self.enabled = False
        self.mode = 'GESTURE_TEST'
        self.gesture_test_label = label
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.gesture_test_twist = twist
        self.gesture_test_until = self.get_clock().now() + Duration(seconds=duration)
        self.pub_cmd.publish(twist)

    def _gesture_test_tick(self):
        if not self.gesture_test_enabled:
            return
        if self.gesture_test_label in ('', 'ready', 'stop'):
            return
        if self.get_clock().now() >= self.gesture_test_until:
            self.gesture_test_label = 'ready'
            self._stop()
            self.publish_state()
            return
        self.pub_cmd.publish(self.gesture_test_twist)

    def safety_check(self):
        if not self.enabled:
            return
        dt = (self.get_clock().now() - self.last_scan).nanoseconds / 1e9
        if dt > 0.5:
            self.get_logger().warn('No recent scan; safety stop', throttle_duration_sec=2)
            self.enabled = False
            self.mode = 'WAITING'
            self._stop()
            self.pub_stadia.publish(String(data='STADIA'))
            self.publish_state()

    def publish_state(self):
        self._coral_person_detections(1, 1)
        payload = {
            'enabled': self.enabled,
            'mode': self.mode,
            'target_dist': self.target_dist,
            'target_angle': self.target_angle,
            'lidar_raw_target_dist': self.lidar_raw_target_dist,
            'lidar_raw_target_angle': self.lidar_raw_target_angle,
            'lidar_cluster_count': self.lidar_cluster_count,
            'lidar_target_status': self.lidar_target_status,
            'identity_status': self.identity_status,
            'identity_verified': self.identity_verified,
            'identity_score': round(float(self.identity_score), 3),
            'identity_description': self.identity_description,
            'identity_enroll_active': self.identity_enroll_active,
            'identity_session_ok': self.identity_session_ok,
            'visual_identity_verify_once': self.get_parameter('visual_identity_verify_once').value,
            'person_track_id': self.person_track_id,
            'person_track_confidence': round(float(self.person_confidence), 3),
            'person_track_fresh': self._person_track_is_fresh(),
            'coral_status': self.coral_status,
            'coral_person_count': self.coral_person_count,
            'coral_inference_ms': (
                None if self.coral_inference_ms is None
                else round(float(self.coral_inference_ms), 3)
            ),
            'coral_person_enabled': self.get_parameter('coral_person_enabled').value,
            'gesture_test_enabled': self.gesture_test_enabled,
            'gesture_test_label': self.gesture_test_label,
            'face_static_enabled': self.face_static_enabled,
            'face_static_dry_run': True,
            'face_detected': self.face_detected,
            'face_x': None if self.face_x is None else round(float(self.face_x), 3),
            'face_y': None if self.face_y is None else round(float(self.face_y), 3),
            'face_width_ratio': (
                None if self.face_width_ratio is None
                else round(float(self.face_width_ratio), 3)
            ),
            'face_height_ratio': (
                None if self.face_height_ratio is None
                else round(float(self.face_height_ratio), 3)
            ),
            'face_distance_estimate': (
                None if self.face_distance_estimate is None
                else round(float(self.face_distance_estimate), 3)
            ),
            'face_lidar_distance_offset': (
                None if self.face_lidar_distance_offset is None
                else round(float(self.face_lidar_distance_offset), 3)
            ),
            'face_identity_status': self.face_identity_status,
            'face_identity_verified': self.face_identity_verified,
            'face_identity_session_ok': self.face_identity_session_ok,
            'face_identity_score': round(float(self.face_identity_score), 3),
            'face_identity_enroll_active': self.face_identity_enroll_active,
            'face_predicted_linear': round(float(self.face_predicted_linear), 3),
            'face_predicted_angular': round(float(self.face_predicted_angular), 3),
            'manual_override_active': self.manual_override_active,
            'estimated_target_speed': round(
                float(self.estimated_target_speed), 3
            ),
            'follow_activation_distance': self.follow_activation_distance,
            'require_face_session_to_start': self.get_parameter(
                'require_face_session_to_start'
            ).value,
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
            'target_speed_alpha': self.get_parameter('target_speed_alpha').value,
            'target_speed_deadband': self.get_parameter('target_speed_deadband').value,
            'early_follow_min_scale': self.get_parameter('early_follow_min_scale').value,
            'pivot_angle_deg': self.get_parameter('pivot_angle_deg').value,
            'max_pivot_angular': self.get_parameter('max_pivot_angular').value,
            'moving_turn_ratio': self.get_parameter('moving_turn_ratio').value,
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
