#!/usr/bin/env python3
"""Verify face once, then retain identity while the tracked face stays visible."""

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
    """        self.declare_parameter('face_identity_threshold', 0.78)
        self.declare_parameter('face_process_stride', 1)
""",
    """        self.declare_parameter('face_identity_threshold', 0.78)
        self.declare_parameter('face_session_lost_timeout', 2.0)
        self.declare_parameter('face_process_stride', 1)
""",
    "face session timeout parameter",
)

replace_once(
    """        self.face_identity_verified = False
        self.face_identity_score = 0.0
        self.face_detected = False
""",
    """        self.face_identity_verified = False
        self.face_identity_session_ok = False
        self.face_identity_score = 0.0
        self.face_last_seen = self.get_clock().now()
        self.face_detected = False
""",
    "face session state",
)

replace_once(
    """            self.face_identity_status = 'enrolling'
            self.face_identity_verified = False
            self.face_identity_score = 0.0
""",
    """            self.face_identity_status = 'enrolling'
            self.face_identity_verified = False
            self.face_identity_session_ok = False
            self.face_identity_score = 0.0
""",
    "reset session during enrollment",
)

replace_once(
    """            if self.face_identity_profile is None:
                self.face_identity_status = 'enrollment_required'
                self.face_identity_verified = False
""",
    """            self.face_identity_session_ok = False
            if self.face_identity_profile is None:
                self.face_identity_status = 'enrollment_required'
                self.face_identity_verified = False
            else:
                self.face_identity_status = 'verifying'
                self.face_identity_verified = False
""",
    "verify on each face static start",
)

replace_once(
    """        elif command in ('FACE_STATIC_OFF', 'FACE_CALIBRATE_OFF'):
            self.face_static_enabled = False
            self.face_identity_enroll_active = False
            self.face_predicted_linear = 0.0
""",
    """        elif command in ('FACE_STATIC_OFF', 'FACE_CALIBRATE_OFF'):
            self.face_static_enabled = False
            self.face_identity_enroll_active = False
            self.face_identity_session_ok = False
            self.face_predicted_linear = 0.0
""",
    "clear session on face off",
)

replace_once(
    """            self.face_identity_enroll_active = False
            self.face_predicted_linear = 0.0
            self.face_predicted_angular = 0.0
            self.mode = 'WAITING'
            self.pub_stadia.publish(String(data='OFF'))
""",
    """            self.face_identity_enroll_active = False
            self.face_identity_session_ok = False
            self.face_predicted_linear = 0.0
            self.face_predicted_angular = 0.0
            self.mode = 'WAITING'
            self.pub_stadia.publish(String(data='OFF'))
""",
    "clear session on global stop",
)

replace_once(
    """        if not detections:
            self.face_detected = False
            self.face_x = None
""",
    """        if not detections:
            self.face_detected = False
            missing_age = (
                self.get_clock().now() - self.face_last_seen
            ).nanoseconds / 1e9
            if missing_age > float(
                self.get_parameter('face_session_lost_timeout').value
            ):
                self.face_identity_session_ok = False
            self.face_x = None
""",
    "expire missing face session",
)

replace_once(
    """        self.face_detected = True
        self.face_x = ((x1 + x2) * 0.5) / w
""",
    """        self.face_detected = True
        self.face_last_seen = self.get_clock().now()
        self.face_x = ((x1 + x2) * 0.5) / w
""",
    "refresh face last seen",
)

replace_once(
    """                self.face_identity_verified = True
                self.face_identity_score = 1.0
                self.face_identity_status = 'locked'
""",
    """                self.face_identity_verified = True
                self.face_identity_session_ok = True
                self.face_identity_score = 1.0
                self.face_identity_status = 'locked'
""",
    "lock enrolled session",
)

replace_once(
    """        self.face_identity_score = max(-1.0, min(1.0, score))
        threshold = float(self.get_parameter('face_identity_threshold').value)
        self.face_identity_verified = score >= threshold
        self.face_identity_status = (
            'verified' if self.face_identity_verified else 'mismatch'
        )
        if not self.face_identity_verified:
""",
    """        self.face_identity_score = max(-1.0, min(1.0, score))
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
""",
    "session identity logic",
)

replace_once(
    """            'face_identity_verified': self.face_identity_verified,
            'face_identity_score': round(float(self.face_identity_score), 3),
""",
    """            'face_identity_verified': self.face_identity_verified,
            'face_identity_session_ok': self.face_identity_session_ok,
            'face_identity_score': round(float(self.face_identity_score), 3),
""",
    "session telemetry",
)

stamp = time.strftime("%Y%m%d_%H%M%S")
backup = path.with_name(f"{path.name}.before_face_session_{stamp}")
shutil.copy2(path, backup)
path.write_text(text)
print(f"patched {path}")
print(f"backup  {backup}")
