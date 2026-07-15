"""Escape policy. Pure function so it can be swapped without touching the loop.

Input:  pursuer bearing, pursuer proximity, LiDAR sector distances.
Output: (heading, speed) where heading is in [-1, 1] (-1 hard left, +1 hard
right) and speed is a PWM magnitude.

Start reactive: steer to keep the pursuer behind you, flee faster as they close,
and override toward open space if LiDAR says you are being cornered. A learned
policy can later replace the body of escape() with the same signature.
"""

import math

from .. import config
from ..sensing import lidar


def escape(bearing, proximity, sectors):
    """Return (heading, speed).

    heading: [-1, 1], steering command (differential drive maps it to L/R PWM)
    speed:   PWM magnitude in [0, MAX_SPEED]
    """
    # 1) Base flee: turn AWAY from the pursuer. If they are on our right
    #    (bearing > 0), steer left (negative heading), and vice versa.
    heading = -bearing

    # 2) Speed scales with how close they are -- closer pursuer, faster flee.
    speed = config.BASE_SPEED + config.PROXIMITY_SPEED_GAIN * proximity
    speed = min(speed, config.MAX_SPEED)

    # 3) Corner override: if boxed in ahead, steer toward the most open side
    #    regardless of the pursuer -- escaping the trap takes priority.
    if lidar.is_cornered(sectors):
        heading = _toward_open_side(sectors)

    # 4) Soft-stop scaling: if something is close directly ahead, ease speed so
    #    the turn can take effect before we hit it. (Hard stop lives on the STM32.)
    if sectors["front"] < config.SAFETY_DISTANCE:
        speed *= 0.3

    return _clamp_heading(heading), speed


def _toward_open_side(sectors):
    """Pick the side with more clearance; return a strong heading toward it."""
    left_clear = min(sectors["front_left"], sectors["left"])
    right_clear = min(sectors["front_right"], sectors["right"])
    # more clearance on the left -> steer left (negative heading)
    return -0.9 if left_clear >= right_clear else 0.9


def _clamp_heading(h):
    return max(-1.0, min(1.0, h))


def heading_to_pwm(heading, speed):
    """Map (heading, speed) to (left_pwm, right_pwm) for differential drive."""
    turn = config.TURN_GAIN * heading
    left = speed + turn
    right = speed - turn
    # clamp both wheels into valid PWM range
    left = int(max(-config.MAX_SPEED, min(config.MAX_SPEED, left)))
    right = int(max(-config.MAX_SPEED, min(config.MAX_SPEED, right)))
    return left, right
