"""Pure geometry: turn a person bounding box into bearing and proximity.

No I/O, no state -- trivially unit-testable with hand-written boxes.
A bbox is (x_min, y_min, x_max, y_max) in pixel coordinates.
"""

from .. import config


def bbox_to_bearing(bbox, frame_width=config.FRAME_WIDTH):
    """Return bearing in [-1, 1]: -1 = far left, 0 = centered, +1 = far right.

    Uses the horizontal center of the box relative to the frame center.
    """
    x_min, _, x_max, _ = bbox
    box_center_x = 0.5 * (x_min + x_max)
    normalized = box_center_x / frame_width          # 0..1 across the frame
    return 2.0 * normalized - 1.0                     # -1..1


def bbox_to_proximity(bbox, frame_height=config.FRAME_HEIGHT):
    """Return proximity in [0, 1]: 0 = far, 1 = very close.

    Uses box height as a distance proxy (a standing person's real height is
    roughly constant indoors, so a taller box means they are closer).
    """
    _, y_min, _, y_max = bbox
    box_frac = (y_max - y_min) / frame_height
    lo, hi = config.PROXIMITY_MIN_BOX_FRAC, config.PROXIMITY_MAX_BOX_FRAC
    proximity = (box_frac - lo) / (hi - lo)
    return max(0.0, min(1.0, proximity))              # clamp to [0, 1]
