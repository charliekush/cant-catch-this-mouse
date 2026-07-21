"""Pose + identity backend, wrapped to the same surface as PersonDetector.

PersonDetector.best(frame) returns Detection(bbox, score, cls) | None, and
geometry.py / evasion.py / main.py only ever consume that shape. This class
implements the same .best(frame) surface over PoseTracker + IdentityMatcher,
so it drops into build_detector() as a swap-in alternative backend (see
config.DETECTOR_BACKEND): bbox is synthesized from the visible keypoints'
pixel extent, and a gated-out stranger (or nobody enrolled) reports the same
way PersonDetector does when nobody is in frame -- None, not an error.
"""

from .. import config
from .detector import Detection
from .pose_tracker import PoseTracker, build_pose_detector
from .identity import IdentityDB, IdentityMatcher, torso_signature


class PoseIdentityDetector:
    def __init__(self, model_path=config.POSE_MODEL_PATH,
                 identity_dir=config.IDENTITY_DIR,
                 id_threshold=config.ID_THRESHOLD,
                 any_person=False,
                 min_confidence=config.POSE_MIN_CONFIDENCE,
                 person_class=config.PERSON_CLASS_ID):
        detector = build_pose_detector(model_path, min_confidence)
        self.tracker = PoseTracker(detector)
        self.db = IdentityDB(identity_dir)
        self.gate = len(self.db) > 0 and not any_person
        self.matcher = (IdentityMatcher(self.db, threshold=id_threshold)
                        if self.gate else None)
        self.person_class = person_class
        self._was_tracking = False

    def best(self, frame):
        """Run pose tracking (+ identity gating) on a BGR frame.

        Returns a Detection compatible with PersonDetector's output, or None
        if nobody is tracked or the tracked person isn't an enrolled match.
        """
        track = self.tracker.update(frame)
        if track is None:
            self._was_tracking = False
            return None

        if not self._was_tracking and self.matcher:
            self.matcher.reset()          # new target -> fresh vote window
        self._was_tracking = True

        score = track.confidence
        if self.gate:
            self.matcher.update(torso_signature(frame, track.kps))
            name, score = self.matcher.decide()
            if name is None:
                return None                # stranger -> no detection, robot ignores them

        bbox = self._keypoints_bbox(track.kps, frame.shape)
        if bbox is None:
            return None
        return Detection(bbox=bbox, score=float(score), cls=self.person_class)

    @staticmethod
    def _keypoints_bbox(kps, frame_shape, min_conf=config.POSE_KP_MIN_CONF):
        """Pixel-space bbox spanning the confidently-visible keypoints."""
        h, w = frame_shape[:2]
        vis = kps[:, 2] >= min_conf
        if vis.sum() < 2:
            return None
        xs, ys = kps[vis, 0] * w, kps[vis, 1] * h
        return (float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max()))
