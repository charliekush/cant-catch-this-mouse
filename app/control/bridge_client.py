"""RPC client to the STM32 over the Arduino UNO Q Bridge (arduino.app_utils).

Wraps Bridge so the rest of the app calls set_motion / stop / get_ultrasonic
and never sees transport details. This also makes the whole MPU loop testable
with a StubBridge (below) that needs no hardware.

Bridge is a process-wide singleton that connects lazily on first use, so
there is no separate handle to construct or store. The arduino.app_utils
import is deferred into each method, rather than done at module level, so
this module (and StubBridge below) can still be imported on a machine
without that package installed -- e.g. for --stub runs off-board. Motor
commands use Bridge.notify (fire-and-forget) since they are sent every
control loop iteration and a dropped ack should never stall motion; the
ultrasonic read uses Bridge.call since it needs a return value.
"""


class BridgeClient:
    def set_motion(self, left_pwm, right_pwm):
        """Command wheel PWMs (-255..255). Sign sets direction."""
        from arduino.app_utils import Bridge

        Bridge.notify("set_motion", left_pwm, right_pwm)

    def stop(self):
        from arduino.app_utils import Bridge

        Bridge.notify("stop")

    def get_ultrasonic(self):
        """Single atomic read of the front ultrasonic backstop (meters)."""
        from arduino.app_utils import Bridge

        return Bridge.call("get_range")


class StubBridge:
    """Hardware-free stand-in: prints commands so the MPU loop can run on a laptop."""

    def set_motion(self, left_pwm, right_pwm):
        print(f"[stub] set_motion(L={left_pwm}, R={right_pwm})")

    def stop(self):
        print("[stub] stop()")

    def get_ultrasonic(self):
        return float("inf")   # pretend nothing is ahead
