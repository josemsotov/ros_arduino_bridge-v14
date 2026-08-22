#!/usr/bin/env python3
"""Add a face-verified static calibration mode to robot_follower."""

from pathlib import Path
import shutil
import time


path = Path(
    "/home/josemsotov/robot_ws/src/"
    "robot_follower/robot_follower/follower_node.py"
)
text = path.read_text()


def replace_once(old: str, new: str, label: str) -> None:
    global text
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one match, found {count}")
    text = text.replace(old, new, 1)


replace_once(
    """        self.declare_parameter('gesture_test_duration', 1.0)
""",
    """        self.declare_parameter('gesture_test_duration', 1.0)
        # Face-only static calibration. It always starts in dry-run.
        self.declare_parameter('face_static_dry_run', True)
        self.declare_parameter('face_detection_confidence', 0.55)
        self.declare_parameter('face_identity_samples', 6)
        self.declare_parameter('face_identity_threshold', 0.72)
        self.declare_parameter('face_process_stride', 2)
        self.declare_parameter('face_target_width_ratio', 0.22)
        self.declare_parameter('face_center_deadband', 0.055)
        self.declare_parameter('face_width_deadband', 0.025)
        self.declare_parameter('face_linear_kp', 0.90)
        self.declare_parameter('face_angular_kp', 1.10)
        self.declare_parameter('face_max_linear_vel', 0.15)
        self.declare_parameter('face_max_reverse_vel', 0.08)
        self.declare_parameter('face_max_angular_vel', 0.30)
""",
    "face parameters",
)

replace_once(
    """        self.visual_target_angle = None

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 5)
""",
    """        self.visual_target_angle = None
        self.face_detector = mp.solutions.face_detection.FaceDetection(
            model_selection=0,
            min_detection_confidence=float(
                self.get_parameter('face_detection_confidence').value
            ),
        )
        self.face_static_enabled = False
        self.face_identity_enroll_active = False
        self.face_identity_profile = None
        self.face_identity_samples = []
        self.face_identity_status = 'idle'
        self.face_identity_verified = False
        self.face_identity_score = 0.0
        self.face_detected = False
        self.face_frame_count = 0
        self.face_x = None
        self.face_y = None
        self.face_width_ratio = None
        self.face_height_ratio = None
        self.face_predicted_linear = 0.0
        self.face_predicted_angular = 0.0

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 5)
""",
    "face state",
)

replace_once(
    """        if self.enabled:
            self.gesture_test_enabled = False
""",
    """        if self.enabled:
            self.face_static_enabled = False
            self.face_identity_enroll_active = False
            self.gesture_test_enabled = False
""",
    "normal follower disables face mode",
)

replace_once(
    """        command = command.upper()
        if command in ('GESTURE_TEST_ON', 'GESTURE_TEST_ENABLE'):
""",
    """        command = command.upper()
        if command in ('FACE_STATIC_ENROLL', 'FACE_CALIBRATE_ENROLL'):
            self.enabled = False
            self.gesture_test_enabled = False
            self.face_static_enabled = True
            self.face_identity_enroll_active = True
            self.face_identity_profile = None
            self.face_identity_samples = []
            self.face_identity_status = 'enrolling'
            self.face_identity_verified = False
            self.face_identity_score = 0.0
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
            self.face_static_enabled = True
            self.face_identity_enroll_active = False
            self.mode = 'FACE_STATIC_DRY_RUN'
            self.pub_stadia.publish(String(data='OFF'))
            self._stop()
            if self.face_identity_profile is None:
                self.face_identity_status = 'enrollment_required'
                self.face_identity_verified = False
            self.get_logger().info(
                'Face static dry-run enabled; motors forced to zero'
            )
        elif command in ('FACE_STATIC_OFF', 'FACE_CALIBRATE_OFF'):
            self.face_static_enabled = False
            self.face_identity_enroll_active = False
            self.face_predicted_linear = 0.0
            self.face_predicted_angular = 0.0
            self.mode = 'WAITING'
            self._stop()
            self.pub_stadia.publish(String(data='STADIA'))
            self.get_logger().info('Face static mode disabled; Stadia restored')
        elif command in ('GESTURE_TEST_ON', 'GESTURE_TEST_ENABLE'):
""",
    "face commands",
)

replace_once(
    """        elif command in ('STOP', 'WAIT', 'PAUSE'):
            self.enabled = False
            self.mode = 'WAITING'
            self._stop()
""",
    """        elif command in ('STOP', 'WAIT', 'PAUSE'):
            self.enabled = False
            self.face_static_enabled = False
            self.face_identity_enroll_active = False
            self.face_predicted_linear = 0.0
            self.face_predicted_angular = 0.0
            self.mode = 'WAITING'
            self.pub_stadia.publish(String(data='OFF'))
            self._stop()
""",
    "stop face mode",
)

replace_once(
    """    def rgb_cb(self, msg: Image):
        if not (self.enabled or self.identity_enroll_active):
            return
""",
    """    def rgb_cb(self, msg: Image):
        if not (
            self.enabled
            or self.identity_enroll_active
            or self.face_static_enabled
            or self.face_identity_enroll_active
        ):
            return
""",
    "rgb face gate",
)

replace_once(
    """        self._update_person_track(image)
""",
    """        if self.face_static_enabled or self.face_identity_enroll_active:
            self._process_face_static(image)
            return

        self._update_person_track(image)
""",
    "rgb face routing",
)

replace_once(
    """    def _image_to_rgb(self, msg: Image):
""",
    """    def _process_face_static(self, image):
        self.face_frame_count += 1
        stride = max(1, int(self.get_parameter('face_process_stride').value))
        if self.face_frame_count % stride:
            return

        result = self.face_detector.process(image)
        detections = result.detections if result and result.detections else []
        if not detections:
            self.face_detected = False
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
            self.face_identity_verified = False
            self.face_identity_status = 'invalid_face_box'
            self.face_predicted_linear = 0.0
            self.face_predicted_angular = 0.0
            self._stop()
            return

        _, (x1, y1, x2, y2) = max(candidates, key=lambda item: item[0])
        self.face_detected = True
        self.face_x = ((x1 + x2) * 0.5) / w
        self.face_y = ((y1 + y2) * 0.5) / h
        self.face_width_ratio = (x2 - x1) / w
        self.face_height_ratio = (y2 - y1) / h

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
        self.face_identity_verified = score >= threshold
        self.face_identity_status = (
            'verified' if self.face_identity_verified else 'mismatch'
        )
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
            -float(self.get_parameter('face_max_reverse_vel').value),
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
        gray = cv2.cvtColor(face_crop, cv2.COLOR_RGB2GRAY)
        gray = cv2.resize(gray, (24, 24), interpolation=cv2.INTER_AREA)
        gray = cv2.equalizeHist(gray).astype(np.float32)
        vector = gray.reshape(-1)
        vector -= float(vector.mean())
        norm = float(np.linalg.norm(vector))
        if norm < 1e-6:
            return None
        return vector / norm

    def _image_to_rgb(self, msg: Image):
""",
    "face processing methods",
)

replace_once(
    """            'gesture_test_enabled': self.gesture_test_enabled,
            'gesture_test_label': self.gesture_test_label,
""",
    """            'gesture_test_enabled': self.gesture_test_enabled,
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
            'face_identity_status': self.face_identity_status,
            'face_identity_verified': self.face_identity_verified,
            'face_identity_score': round(float(self.face_identity_score), 3),
            'face_identity_enroll_active': self.face_identity_enroll_active,
            'face_predicted_linear': round(float(self.face_predicted_linear), 3),
            'face_predicted_angular': round(float(self.face_predicted_angular), 3),
""",
    "face state telemetry",
)

stamp = time.strftime("%Y%m%d_%H%M%S")
backup = path.with_name(f"{path.name}.before_face_static_{stamp}")
shutil.copy2(path, backup)
path.write_text(text)
print(f"patched {path}")
print(f"backup  {backup}")
