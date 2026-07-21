"""
identity.py
-----------
Person re-identification for MouseBot: only track ENROLLED people (you and
your partner), ignore strangers.

How it works (pose-guided appearance signature):
  * The pose keypoints give us the exact torso region (shoulders -> hips),
    so we crop precisely the person's shirt - no background pollution.
  * Signature = normalized 2D Hue-Saturation histogram of that patch
    (HSV is much more lighting-tolerant than RGB; we drop the V channel).
  * Enrollment stores ~40 signatures per person captured while they move
    around (covers pose/lighting variation).
  * At runtime, each fresh track's signature is compared (histogram
    correlation) against every enrolled sample; per-person score = best
    sample match; a temporal voting window smooths the decision so one
    noisy frame can't flip the identity.

Honest limitations (put these in the report):
  * It identifies CLOTHING, not faces. Enroll on demo day, in demo clothes.
  * You and your partner should wear clearly different shirt colors -
    two people in near-identical shirts are indistinguishable to this.
  * Extreme lighting changes (sunlight -> dark hallway) lower scores;
    enrolling in the demo environment fixes this.
"""

import os
import time
from collections import deque
from typing import Dict, Optional, Tuple

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

from pose_tracker import L_SHOULDER, R_SHOULDER, L_HIP, R_HIP

H_BINS, S_BINS = 30, 32          # hue-saturation histogram resolution
KP_CONF = 0.3
MIN_PATCH_PX = 24                # torso crop must be at least this tall/wide


# ---------------------------------------------------------------------------
# Signature extraction
# ---------------------------------------------------------------------------
def torso_signature(frame_bgr, kps: np.ndarray) -> Optional[np.ndarray]:
    """
    HS histogram of the torso patch defined by the 4 torso keypoints.
    Returns a flattened, L1-normalized float32 histogram, or None if the
    torso isn't confidently visible / the crop is too small.
    """
    idx = [L_SHOULDER, R_SHOULDER, L_HIP, R_HIP]
    if (kps[idx, 2] < KP_CONF).any():
        return None

    H, W = frame_bgr.shape[:2]
    xs = kps[idx, 0] * W
    ys = kps[idx, 1] * H
    # shrink 10% toward the center so we sample shirt, not arms/background
    cx, cy = xs.mean(), ys.mean()
    xs = cx + (xs - cx) * 0.9
    ys = cy + (ys - cy) * 0.9
    x1, x2 = int(max(0, xs.min())), int(min(W, xs.max()))
    y1, y2 = int(max(0, ys.min())), int(min(H, ys.max()))
    if (x2 - x1) < MIN_PATCH_PX or (y2 - y1) < MIN_PATCH_PX:
        return None

    patch = frame_bgr[y1:y2, x1:x2]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    # ignore very dark pixels (shadows) - they carry no reliable hue
    mask = cv2.inRange(hsv, (0, 0, 40), (180, 255, 255))
    hist = cv2.calcHist([hsv], [0, 1], mask, [H_BINS, S_BINS],
                        [0, 180, 0, 256])
    # Smooth before normalizing: without this, a solid-color shirt is a
    # single-bin spike, and a marginally different shade lands in the
    # neighboring bin -> correlation collapses to ~0 for the SAME shirt.
    # Blurring spreads each spike over adjacent hue/sat bins so close
    # shades correlate highly while distant colors still don't.
    hist = cv2.GaussianBlur(hist, (7, 7), 1.5)
    total = hist.sum()
    if total < 1e-6:
        return None
    return (hist / total).astype(np.float32).ravel()


def signature_score(sig: np.ndarray, enrolled: np.ndarray) -> float:
    """Best histogram correlation between sig and any enrolled sample.
    Correlation is in [-1, 1]; ~1 = same shirt, ~0 = unrelated."""
    best = -1.0
    a = sig.reshape(H_BINS, S_BINS)
    for row in enrolled:
        b = row.reshape(H_BINS, S_BINS)
        best = max(best, float(cv2.compareHist(a, b, cv2.HISTCMP_CORREL)))
    return best


# ---------------------------------------------------------------------------
# Enrollment database (one .npz per person in data/identities/)
# ---------------------------------------------------------------------------
class IdentityDB:
    def __init__(self, dir_path: str):
        self.dir = dir_path
        os.makedirs(self.dir, exist_ok=True)
        self.people: Dict[str, np.ndarray] = {}
        self.load()

    def load(self):
        self.people.clear()
        for fn in sorted(os.listdir(self.dir)):
            if fn.endswith(".npz"):
                name = fn[:-4]
                data = np.load(os.path.join(self.dir, fn))
                self.people[name] = data["signatures"]
        return self

    def save(self, name: str, signatures: np.ndarray):
        path = os.path.join(self.dir, f"{name}.npz")
        np.savez_compressed(path, signatures=np.asarray(signatures,
                                                        np.float32))
        self.people[name] = np.asarray(signatures, np.float32)
        return path

    def names(self):
        return list(self.people.keys())

    def __len__(self):
        return len(self.people)


# ---------------------------------------------------------------------------
# Runtime matcher with temporal voting
# ---------------------------------------------------------------------------
class IdentityMatcher:
    """
    Feed it a signature per fresh frame; it votes over a sliding window so
    a single noisy frame can't flip the identity. decide() returns
    (name, mean_score) or (None, best_score) when nobody enrolled matches.
    """

    def __init__(self, db: IdentityDB, threshold: float = 0.55,
                 window: int = 8, min_votes: int = 3):
        self.db = db
        self.threshold = threshold
        self.window = window
        self.min_votes = min_votes
        self.scores: Dict[str, deque] = {
            n: deque(maxlen=window) for n in db.names()}

    def reset(self):
        for d in self.scores.values():
            d.clear()

    def update(self, sig: Optional[np.ndarray]):
        if sig is None:
            return
        for name, enrolled in self.db.people.items():
            self.scores[name].append(signature_score(sig, enrolled))

    def decide(self) -> Tuple[Optional[str], float]:
        best_name, best_mean = None, -1.0
        for name, d in self.scores.items():
            if len(d) >= self.min_votes:
                m = float(np.mean(d))
                if m > best_mean:
                    best_name, best_mean = name, m
        if best_name is not None and best_mean >= self.threshold:
            return best_name, best_mean
        return None, best_mean


# ---------------------------------------------------------------------------
# Enrollment capture helper
# ---------------------------------------------------------------------------
class Enroller:
    """Collects up to n_samples torso signatures, spaced sample_gap_s apart,
    while the person moves around in frame."""

    def __init__(self, n_samples: int = 40, sample_gap_s: float = 0.25):
        self.n_samples = n_samples
        self.sample_gap_s = sample_gap_s
        self.samples = []
        self._last_t = 0.0

    def offer(self, frame_bgr, kps) -> bool:
        """Returns True when enrollment is complete."""
        now = time.time()
        if now - self._last_t >= self.sample_gap_s:
            sig = torso_signature(frame_bgr, kps)
            if sig is not None:
                self.samples.append(sig)
                self._last_t = now
        return len(self.samples) >= self.n_samples

    @property
    def progress(self) -> float:
        return len(self.samples) / float(self.n_samples)
