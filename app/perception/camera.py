"""USB webcam capture on the MPU.

Isolates all V4L2/OpenCV details so the detector never touches the camera API
directly. Install OpenCV via apt (python3-opencv), not pip, on ARM/Debian.
"""

import cv2

from .. import config


class Camera:
    def __init__(self, index=config.CAMERA_INDEX,
                 width=config.FRAME_WIDTH, height=config.FRAME_HEIGHT):
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera at index {index}")

    def read(self):
        """Return the latest frame (BGR ndarray), or None on failure."""
        ok, frame = self.cap.read()
        return frame if ok else None

    def release(self):
        self.cap.release()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.release()
