"""
human_style.py
--------------
Recreates the three techniques that make vladmandic/human's body tracking
look so smooth - in Python, on our stack (Human itself is a TensorFlow.js
library for browsers/Node, so it can't run in the UNO Q's Python pipeline;
notably its default body model is MoveNet Lightning, the same model our
pose_tracker already supports, so what we recreate here is the smoothness
layer, not the model):

  1. DECOUPLED LOOPS - detection runs in a background thread on the latest
     frame; the draw loop never blocks on the model. (Human: "run detection
     in a separate web worker thread".)

  2. TEMPORAL INTERPOLATION - the skeleton actually drawn each display
     frame is interpolated between the last two detection results, so even
     a 10 fps model renders as a gliding 30 fps figure. (Human:
     "intelligent temporal interpolation to provide smooth results
     regardless of processing performance" / human.next().)

  3. POLISHED RENDERING - dark-outlined thick limbs, per-part colors,
     joints faded by confidence.
"""

import threading
import time
from typing import Optional

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

from pose_tracker import PoseTrack, SKELETON, bearing_from_cx


# ---------------------------------------------------------------------------
# 2) Temporal interpolation between two tracks (pure math - unit tested)
# ---------------------------------------------------------------------------
def interpolate_tracks(a: PoseTrack, t_a: float,
                       b: PoseTrack, t_b: float,
                       t_now: float) -> PoseTrack:
    """
    Linear interpolation of every keypoint (and the torso center) between
    detection result `a` (at time t_a) and `b` (at t_b), evaluated at
    t_now. Clamped to [a, b] - we interpolate, never extrapolate, so the
    figure can't overshoot past the newest real detection.
    """
    if t_b <= t_a:
        return b
    u = (t_now - t_a) / (t_b - t_a)
    u = max(0.0, min(1.0, u))

    kps = (1 - u) * a.kps + u * b.kps
    cx = (1 - u) * a.torso_cx + u * b.torso_cx
    cy = (1 - u) * a.torso_cy + u * b.torso_cy
    return PoseTrack(
        bearing_deg=bearing_from_cx(cx),
        kps=kps,
        torso_cx=cx, torso_cy=cy,
        confidence=(1 - u) * a.confidence + u * b.confidence,
        age_frames=b.age_frames,
        fresh=b.fresh,
    )


# ---------------------------------------------------------------------------
# 1) Threaded tracker: detect in the background, never block the draw loop
# ---------------------------------------------------------------------------
class ThreadedPoseTracker:
    """
    Wraps a PoseTracker. submit(frame) is non-blocking and always replaces
    the pending frame with the newest one (latest-frame slot - stale frames
    are dropped, matching Human's frame-change strategy). current() returns
    the interpolated PoseTrack for *right now*, or None if no target.
    """

    def __init__(self, tracker, max_result_age_s: float = 1.5):
        self.tracker = tracker
        self.max_result_age_s = max_result_age_s
        self._frame = None
        self._frame_lock = threading.Condition()
        self._prev = None            # (track, t)
        self._last = None            # (track, t)
        self._res_lock = threading.Lock()
        self._stop = False
        self.detect_fps = 0.0
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    # ---- producer side (camera loop) ----
    def submit(self, frame) -> None:
        with self._frame_lock:
            self._frame = frame          # overwrite: only newest frame matters
            self._frame_lock.notify()

    # ---- consumer side (draw loop) ----
    def current(self) -> Optional[PoseTrack]:
        now = time.time()
        with self._res_lock:
            prev, last = self._prev, self._last
        if last is None:
            return None
        track_b, t_b = last
        if now - t_b > self.max_result_age_s:
            return None                  # detections went stale -> no target
        if prev is None:
            return track_b
        track_a, t_a = prev
        # Render ONE detection period behind real time: as the time since
        # the newest result (t_b) elapses over one period (t_b - t_a), glide
        # from the previous pose to the newest one. Without this shift the
        # interpolation factor would always clamp at 1.0 (we always render
        # AFTER t_b) and you'd just see raw model steps. Costs one model
        # period of latency (~50-100 ms) - imperceptible, and worth it.
        period = t_b - t_a
        return interpolate_tracks(track_a, t_b, track_b, t_b + period, now)

    def stop(self):
        self._stop = True
        with self._frame_lock:
            self._frame_lock.notify()
        self._thread.join(timeout=2.0)

    # ---- background worker ----
    def _worker(self):
        n, t0 = 0, time.time()
        while not self._stop:
            with self._frame_lock:
                while self._frame is None and not self._stop:
                    self._frame_lock.wait(timeout=0.1)
                frame, self._frame = self._frame, None
            if self._stop or frame is None:
                continue
            track = self.tracker.update(frame)
            t = time.time()
            with self._res_lock:
                if track is not None:
                    self._prev = self._last
                    self._last = (track, t)
                else:
                    # keep last result; current() ages it out naturally
                    pass
            n += 1
            if t - t0 >= 1.0:
                self.detect_fps = n / (t - t0)
                n, t0 = 0, t


# ---------------------------------------------------------------------------
# 3) Human-style rendering
# ---------------------------------------------------------------------------
COLORS = {
    "head":  (255, 0, 255),   # magenta
    "torso": (0, 220, 255),   # amber-yellow
    "arms":  (255, 200, 0),   # cyan-blue
    "legs":  (80, 255, 80),   # green
}
KP_DRAW_CONF = 0.3


def draw_human_style(frame, track: PoseTrack, detect_fps: float = 0.0,
                     label: str = None, known: bool = True):
    H, W = frame.shape[:2]
    kps = track.kps

    def pt(i):
        return int(kps[i, 0] * W), int(kps[i, 1] * H)

    # strangers render in red; enrolled people in normal part colors
    palette = COLORS if known else {p: (0, 0, 220) for p in COLORS}

    # limbs: dark outline first, colored core on top -> crisp "3D" look
    for part, edges in SKELETON.items():
        for i, j in edges:
            if kps[i, 2] >= KP_DRAW_CONF and kps[j, 2] >= KP_DRAW_CONF:
                cv2.line(frame, pt(i), pt(j), (30, 30, 30), 7, cv2.LINE_AA)
        for i, j in edges:
            if kps[i, 2] >= KP_DRAW_CONF and kps[j, 2] >= KP_DRAW_CONF:
                cv2.line(frame, pt(i), pt(j), palette[part], 3, cv2.LINE_AA)

    # joints: brightness scales with confidence (fades during occlusion)
    for i in range(17):
        c = float(kps[i, 2])
        if c >= KP_DRAW_CONF:
            v = int(120 + 135 * min(1.0, c))
            cv2.circle(frame, pt(i), 6, (30, 30, 30), -1, cv2.LINE_AA)
            cv2.circle(frame, pt(i), 4, (v, v, v), -1, cv2.LINE_AA)

    # torso center = bearing point
    tc = (int(track.torso_cx * W), int(track.torso_cy * H))
    color = (0, 255, 0) if track.fresh else (0, 200, 255)
    cv2.drawMarker(frame, tc, color, cv2.MARKER_TILTED_CROSS, 24, 3)

    side = "LEFT" if track.bearing_deg < -3 else \
           "RIGHT" if track.bearing_deg > 3 else "AHEAD"
    who = label if label else ("person" if known else "UNKNOWN - ignored")
    hud = (f"{who}  bearing {track.bearing_deg:+.1f} deg ({side})  "
           f"conf {track.confidence:.2f}")
    if detect_fps:
        hud += f"  model {detect_fps:.1f} fps"
    cv2.rectangle(frame, (6, 8), (16 + 9 * len(hud), 40), (30, 30, 30), -1)
    cv2.putText(frame, hud, (12, 32), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (255, 255, 255), 2, cv2.LINE_AA)
