#!/usr/bin/env python3
"""Use near and far MediaPipe face detectors for Kinect RGB."""

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
    """        self.declare_parameter('face_detection_confidence', 0.35)
""",
    """        self.declare_parameter('face_detection_confidence', 0.30)
""",
    "face confidence",
)

replace_once(
    """        self.face_detector = mp.solutions.face_detection.FaceDetection(
            model_selection=1,
            min_detection_confidence=float(
                self.get_parameter('face_detection_confidence').value
            ),
        )
""",
    """        face_confidence = float(
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
""",
    "dual detector initialization",
)

replace_once(
    """        result = self.face_detector.process(image)
        detections = result.detections if result and result.detections else []
""",
    """        near_result = self.face_detector.process(image)
        far_result = self.face_detector_far.process(image)
        detections = []
        if near_result and near_result.detections:
            detections.extend(near_result.detections)
        if far_result and far_result.detections:
            detections.extend(far_result.detections)
""",
    "dual detector processing",
)

stamp = time.strftime("%Y%m%d_%H%M%S")
backup = path.with_name(f"{path.name}.before_dual_face_{stamp}")
shutil.copy2(path, backup)
path.write_text(text)
print(f"patched {path}")
print(f"backup  {backup}")
