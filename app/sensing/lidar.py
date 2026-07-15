"""Dual-LD06 processing: mask -> transform to robot frame -> merge -> sectorize.

Because the UNO Q sits centrally amid wiring, one LiDAR cannot see 360. We run a
FRONT and a REAR LD06; each covers the arc the central clutter blocks for the
other. Each unit also sees its own body, the clutter, and the other LiDAR as
phantom returns -- those fixed arcs are masked out per unit before merging.

Downstream (evasion policy) never sees raw points: everything is reduced to a
handful of per-sector minimum distances.
"""

import math

from .. import config


def _angle_in_arc(angle, start, end):
    """True if angle (deg) lies within [start, end], handling wraparound."""
    a = angle % 360
    s, e = start % 360, end % 360
    if s <= e:
        return s <= a <= e
    return a >= s or a <= e   # arc wraps through 0


def apply_masks(points, masks):
    """Drop points whose angle falls inside any masked arc (LiDAR frame)."""
    if not masks:
        return points
    kept = []
    for p in points:
        if any(_angle_in_arc(p.angle_deg, s, e) for (s, e) in masks):
            continue
        kept.append(p)
    return kept


def to_robot_frame(points, offset):
    """Transform points from a LiDAR's local frame into the robot frame.

    offset = (x, y, yaw) of the LiDAR relative to robot center (m, m, rad).
    Returns list of (angle_deg_robot, distance_m) tuples.
    """
    ox, oy, oyaw = offset
    out = []
    for p in points:
        # point position in LiDAR frame
        a = math.radians(p.angle_deg)
        px = p.distance_m * math.cos(a)
        py = p.distance_m * math.sin(a)
        # rotate by yaw, then translate by the mounting offset
        rx = px * math.cos(oyaw) - py * math.sin(oyaw) + ox
        ry = px * math.sin(oyaw) + py * math.cos(oyaw) + oy
        dist = math.hypot(rx, ry)
        ang = math.degrees(math.atan2(ry, rx))
        out.append((ang, dist))
    return out


def merge(front_points, rear_points):
    """Mask, transform, and combine both units into one robot-frame point list."""
    f = apply_masks(front_points, config.FRONT_LIDAR_MASKS)
    r = apply_masks(rear_points, config.REAR_LIDAR_MASKS)
    fr = to_robot_frame(f, config.FRONT_LIDAR_OFFSET)
    rr = to_robot_frame(r, config.REAR_LIDAR_OFFSET)
    return fr + rr


def sectorize(robot_points, sectors=config.SECTORS):
    """Reduce a merged point list to nearest-obstacle distance per sector.

    Returns a dict {sector_name: min_distance_m}; a sector with no points
    returns math.inf (nothing seen there).
    """
    result = {name: math.inf for name in sectors}
    for ang, dist in robot_points:
        for name, (start, end) in sectors.items():
            if _angle_in_arc(ang, start, end):
                if dist < result[name]:
                    result[name] = dist
    return result


def process(front_points, rear_points):
    """Full pipeline: two raw scans -> six sector minimums."""
    return sectorize(merge(front_points, rear_points))


def is_cornered(sectors, threshold=config.CORNER_DISTANCE):
    """Corner condition: walls converging ahead on both front-side sectors."""
    return (sectors["front_left"] < threshold and
            sectors["front_right"] < threshold)
