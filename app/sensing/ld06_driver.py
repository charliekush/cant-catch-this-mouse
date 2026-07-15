"""Thin wrapper over an LD06 UART stream.

The LD06 streams packets over serial (~230400 baud). Rather than decode the
protocol from scratch, prefer the LDRobot/ldlidar driver or a community Python
parser and adapt it here. This wrapper's only job is to hand back completed
scans as lists of (angle_deg, distance_m, intensity) in the LiDAR's own frame.

Angle convention here: degrees, 0 = the LiDAR's forward axis, CCW positive.
Whatever parser you use, normalize its output to that convention in _parse().
"""

from collections import namedtuple

import serial

Point = namedtuple("Point", ["angle_deg", "distance_m", "intensity"])


class LD06:
    def __init__(self, port, baud=230400):
        self.ser = serial.Serial(port, baud, timeout=1.0)
        self._buffer = bytearray()

    def read_scan(self):
        """Block until one full 360 revolution is assembled; return list[Point].

        Implementation note: accumulate packets until the angle wraps past 360,
        then emit the collected points as one scan. Replace the body with calls
        into your chosen LD06 parser -- keep the return type as list[Point].
        """
        raise NotImplementedError(
            "Wire this to the LD06 parser (LDRobot driver or community lib). "
            "Return a list of Point(angle_deg, distance_m, intensity)."
        )

    def close(self):
        self.ser.close()
