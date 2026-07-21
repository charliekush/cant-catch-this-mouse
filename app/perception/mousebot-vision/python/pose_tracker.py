"""
pose_tracker.py
---------------
MouseBot perception v3: full-body POSE tracking. Instead of a bounding box,
we detect 17 body keypoints (COCO layout: nose, eyes, ears, shoulders,
elbows, wrists, hips, knees, ankles) and track the skeleton - arms and legs
included, drawn as a stick figure.

The camera still reports DIRECTION only (LiDAR owns depth). Bearing now
comes from the TORSO CENTER (mean of shoulders + hips), which is more
stable than a bounding-box center: waving arms or a lifted leg shifts a
box's center, but barely moves the torso.

Backends (same interface):

  1. MediaPipePoseDetector - Google MediaPipe Pose. Best quality, easy pip
     install, ideal for laptop development. 33 landmarks, mapped down to
     the 17 COCO keypoints.

  2. MoveNetDetector - MoveNet SinglePose Lightning (.tflite), ~5 ms/frame
     class of model, built for edge devices -> the UNO Q path.
     Put the model at data/movenet_lightning.tflite

PoseTracker adds per-keypoint EMA smoothing (stick figure doesn't jitter),
occlusion handling (low-confidence joints freeze at their last position
instead of jumping), N-frame confirmation, and a lost-target timeout.
"""

from dataclasses import dataclass
from typing import Optional
import math
import os
import time

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

# ---------------- C920 optics ----------------
HFOV_DEG = 70.42

# ---------------- COCO-17 keypoint layout ----------------
KEYPOINT_NAMES = [
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
]
NOSE = 0
L_SHOULDER, R_SHOULDER = 5, 6
L_ELBOW, R_ELBOW = 7, 8
L_WRIST, R_WRIST = 9, 10
L_HIP, R_HIP = 11, 12
L_KNEE, R_KNEE = 13, 14
L_ANKLE, R_ANKLE = 15, 16
TORSO = (L_SHOULDER, R_SHOULDER, L_HIP, R_HIP)

# Skeleton edges grouped by body part (used for drawing)
SKELETON = {
    "head":  [(NOSE, L_SHOULDER), (NOSE, R_SHOULDER)],
    "torso": [(L_SHOULDER, R_SHOULDER), (L_SHOULDER, L_HIP),
              (R_SHOULDER, R_HIP), (L_HIP, R_HIP)],
    "arms":  [(L_SHOULDER, L_ELBOW), (L_ELBOW, L_WRIST),
              (R_SHOULDER, R_ELBOW), (R_ELBOW, R_WRIST)],
    "legs":  [(L_HIP, L_KNEE), (L_KNEE, L_ANKLE),
              (R_HIP, R_KNEE), (R_KNEE, R_ANKLE)],
}

# MediaPipe Pose has 33 landmarks; these indices map them onto COCO-17.
MEDIAPIPE_TO_COCO = [0, 2, 5, 7, 8, 11, 12, 13, 14, 15, 16,
                     23, 24, 25, 26, 27, 28]


@dataclass
class PoseDetection:
    """Raw keypoints for one frame: (17, 3) array of (x, y, conf),
    x/y normalized 0..1 in frame coordinates."""
    kps: np.ndarray


@dataclass
class PoseTrack:
    """Smoothed, confirmed skeleton - what the demo draws and the evasion
    policy consumes."""
    bearing_deg: float       # from torso center; negative = pursuer LEFT
    kps: np.ndarray          # smoothed (17, 3): x, y, conf
    torso_cx: float          # normalized torso center (defines the bearing)
    torso_cy: float
    confidence: float        # mean confidence of visible keypoints
    age_frames: int
    fresh: bool


# ---------------------------------------------------------------------------
# Geometry (pure math - unit tested)
# ---------------------------------------------------------------------------
def bearing_from_cx(cx: float, hfov_deg: float = HFOV_DEG) -> float:
    """True pinhole mapping: bearing = atan((cx - 0.5) / f),
    f = 0.5 / tan(HFOV/2). Exact out to the frame edges."""
    half = math.radians(hfov_deg / 2.0)
    f = 0.5 / math.tan(half)
    return math.degrees(math.atan2(cx - 0.5, f))


def torso_center(kps: np.ndarray, min_conf: float = 0.3):
    """Confidence-weighted center of shoulders+hips; falls back to the mean
    of all confident keypoints if the torso is occluded. Returns
    (cx, cy) or None if nothing is confident."""
    idx = list(TORSO)
    pts, w = kps[idx, :2], kps[idx, 2]
    mask = w >= min_conf
    if mask.sum() >= 2:
        w = w[mask]
        return tuple((pts[mask] * w[:, None]).sum(0) / w.sum())
    mask_all = kps[:, 2] >= min_conf
    if mask_all.sum() >= 3:
        w = kps[mask_all, 2]
        return tuple((kps[mask_all, :2] * w[:, None]).sum(0) / w.sum())
    return None


# ---------------------------------------------------------------------------
# Backend 1: MediaPipe Pose (laptop development - best quality)
# ---------------------------------------------------------------------------
class MediaPipePoseDetector:
    def __init__(self, min_confidence: float = 0.5):
        import mediapipe as mp
        self.pose = mp.solutions.pose.Pose(
            model_complexity=1,
            min_detection_confidence=min_confidence,
            min_tracking_confidence=min_confidence,
        )

    def detect(self, frame_bgr) -> Optional[PoseDetection]:
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        res = self.pose.process(rgb)
        if res.pose_landmarks is None:
            return None
        lm = res.pose_landmarks.landmark
        kps = np.zeros((17, 3), np.float32)
        for coco_i, mp_i in enumerate(MEDIAPIPE_TO_COCO):
            p = lm[mp_i]
            kps[coco_i] = (p.x, p.y, p.visibility)
        return PoseDetection(kps=kps)

    def close(self):
        self.pose.close()


# ---------------------------------------------------------------------------
# Backend 2: MoveNet SinglePose Lightning (.tflite) - the UNO Q path
# ---------------------------------------------------------------------------
class MoveNetDetector:
    """MoveNet outputs [1, 1, 17, 3] as (y, x, score), already in COCO-17
    order. Input is a square resize (192x192 for Lightning); normalized
    outputs map straight back onto the original frame, so the aspect
    distortion cancels."""

    def __init__(self, model_path: str, min_confidence: float = 0.3):
        try:
            from tflite_runtime.interpreter import Interpreter
        except ImportError:                      # laptop with full TF instead
            from tensorflow.lite.python.interpreter import Interpreter
        self.interpreter = Interpreter(model_path=model_path, num_threads=4)
        self.interpreter.allocate_tensors()
        self.inp = self.interpreter.get_input_details()[0]
        self.out = self.interpreter.get_output_details()[0]
        _, self.in_h, self.in_w, _ = self.inp["shape"]
        self.min_confidence = min_confidence

    def detect(self, frame_bgr) -> Optional[PoseDetection]:
        img = cv2.resize(frame_bgr, (self.in_w, self.in_h))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        data = np.expand_dims(img, axis=0)
        if self.inp["dtype"] == np.float32:
            data = np.float32(data)
        else:                                    # quantized: uint8/int8
            data = data.astype(self.inp["dtype"])
        self.interpreter.set_tensor(self.inp["index"], data)
        self.interpreter.invoke()
        out = self.interpreter.get_tensor(self.out["index"])[0, 0]  # (17,3)

        kps = np.zeros((17, 3), np.float32)
        kps[:, 0] = out[:, 1]                    # x
        kps[:, 1] = out[:, 0]                    # y
        kps[:, 2] = out[:, 2]                    # score
        if (kps[:, 2] >= self.min_confidence).sum() < 3:
            return None                          # nobody in frame
        return PoseDetection(kps=kps)


def build_pose_detector(movenet_path: str, min_confidence: float = 0.5):
    """MediaPipe if installed (laptop), else MoveNet if the model file
    exists (UNO Q), else a clear error telling you what to install."""
    try:
        d = MediaPipePoseDetector(min_confidence)
        print("[pose] MediaPipe Pose active (33->17 keypoints)")
        return d
    except ImportError:
        pass
    if os.path.exists(movenet_path):
        d = MoveNetDetector(movenet_path)
        print(f"[pose] MoveNet TFLite active: {movenet_path}")
        return d
    raise SystemExit(
        "No pose backend available.\n"
        "  Laptop:  pip install mediapipe\n"
        "           (if pip can't find it on Python 3.13, make an env:\n"
        "            conda create -n mousebot python=3.11 -y\n"
        "            conda activate mousebot\n"
        "            pip install mediapipe 'opencv-python==4.10.0.84')\n"
        f"  UNO Q:   download MoveNet SinglePose Lightning .tflite to\n"
        f"           {movenet_path}"
    )


# ---------------------------------------------------------------------------
# Temporal tracker: per-keypoint smoothing + occlusion freeze
# ---------------------------------------------------------------------------
class PoseTracker:
    """
    * CONFIRM_FRAMES consecutive detections before a track is reported.
    * Per-keypoint EMA on (x, y) - the stick figure moves smoothly.
    * Occlusion freeze: a keypoint below KP_CONF keeps its last smoothed
      position (a briefly hidden ankle doesn't teleport); its stored
      confidence decays so the renderer can fade it out.
    * Lost-target timeout -> track dropped, robot reverts to scan/wander.
    """

    KP_CONF = 0.3

    def __init__(self, detector, confirm_frames: int = 3,
                 alpha: float = 0.5, lost_timeout_s: float = 1.0):
        self.detector = detector
        self.confirm_frames = confirm_frames
        self.alpha = alpha
        self.lost_timeout_s = lost_timeout_s
        self._reset()

    def _reset(self):
        self.kps = None
        self.hits = 0
        self.age = 0
        self.last_seen_t = 0.0

    def update(self, frame_bgr) -> Optional[PoseTrack]:
        det = self.detector.detect(frame_bgr)
        now = time.time()

        if det is not None:
            if self.kps is None:
                self.kps = det.kps.copy()
            else:
                a = self.alpha
                good = det.kps[:, 2] >= self.KP_CONF
                # smooth confident joints toward the new detection
                self.kps[good, :2] = (a * det.kps[good, :2]
                                      + (1 - a) * self.kps[good, :2])
                self.kps[good, 2] = det.kps[good, 2]
                # occluded joints: freeze position, decay confidence
                self.kps[~good, 2] *= 0.8
            self.hits += 1
            self.last_seen_t = now
            fresh = True
        else:
            if self.kps is None:
                return None
            if now - self.last_seen_t > self.lost_timeout_s:
                self._reset()
                return None
            fresh = False

        if self.hits < self.confirm_frames:
            return None

        center = torso_center(self.kps, self.KP_CONF)
        if center is None:
            return None
        cx, cy = center

        vis = self.kps[:, 2] >= self.KP_CONF
        conf = float(self.kps[vis, 2].mean()) if vis.any() else 0.0

        self.age += 1
        return PoseTrack(
            bearing_deg=bearing_from_cx(cx),
            kps=self.kps.copy(),
            torso_cx=float(cx), torso_cy=float(cy),
            confidence=conf,
            age_frames=self.age,
            fresh=fresh,
        )
