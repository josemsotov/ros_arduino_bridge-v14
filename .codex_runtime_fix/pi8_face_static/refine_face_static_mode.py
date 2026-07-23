#!/usr/bin/env python3
"""Refine face detection and identity stability for static dry-run tests."""

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
    """        self.declare_parameter('face_detection_confidence', 0.55)
        self.declare_parameter('face_identity_samples', 6)
        self.declare_parameter('face_identity_threshold', 0.72)
        self.declare_parameter('face_process_stride', 2)
""",
    """        self.declare_parameter('face_detection_confidence', 0.35)
        self.declare_parameter('face_identity_samples', 8)
        self.declare_parameter('face_identity_threshold', 0.78)
        self.declare_parameter('face_process_stride', 1)
""",
    "face detection defaults",
)

replace_once(
    """        self.face_detector = mp.solutions.face_detection.FaceDetection(
            model_selection=0,
""",
    """        self.face_detector = mp.solutions.face_detection.FaceDetection(
            model_selection=1,
""",
    "long-range face detector",
)

replace_once(
    """        else:
            self.mode = 'WAITING'
            self.identity_enroll_active = False
""",
    """        else:
            if not self.face_static_enabled:
                self.mode = 'WAITING'
            self.identity_enroll_active = False
""",
    "preserve face static mode",
)

replace_once(
    """        if not detections:
            self.face_detected = False
            self.face_identity_verified = False
""",
    """        if not detections:
            self.face_detected = False
            self.face_x = None
            self.face_y = None
            self.face_width_ratio = None
            self.face_height_ratio = None
            self.face_identity_verified = False
""",
    "clear missing face coordinates",
)

replace_once(
    """        if not candidates:
            self.face_detected = False
            self.face_identity_verified = False
""",
    """        if not candidates:
            self.face_detected = False
            self.face_x = None
            self.face_y = None
            self.face_width_ratio = None
            self.face_height_ratio = None
            self.face_identity_verified = False
""",
    "clear invalid face coordinates",
)

replace_once(
    """    @staticmethod
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
""",
    """    @staticmethod
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
""",
    "stable face signature",
)

stamp = time.strftime("%Y%m%d_%H%M%S")
backup = path.with_name(f"{path.name}.before_face_refine_{stamp}")
shutil.copy2(path, backup)
path.write_text(text)
print(f"patched {path}")
print(f"backup  {backup}")
