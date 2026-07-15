"""RPC client to the STM32 over Arduino_RouterBridge.

Wraps the Bridge so the rest of the app calls set_motion / stop / get_ultrasonic
and never sees transport details. This also makes the whole MPU loop testable
with a StubBridge (below) that needs no hardware.

NOTE: The RPC method names here are PLACEHOLDERS. Verify them against the actual
Arduino_RouterBridge API before relying on them.
"""


class BridgeClient:
    def __init__(self):
        # from arduino.bridge import Bridge  # (verify actual import path)
        # self._bridge = Bridge()
        raise NotImplementedError(
            "Instantiate the real Arduino_RouterBridge here and map the calls "
            "below to its verified RPC API."
        )

    def set_motion(self, left_pwm, right_pwm):
        """Command wheel PWMs (-255..255). Sign sets direction."""
        # return self._bridge.call("set_motion", left_pwm, right_pwm)
        ...

    def stop(self):
        # return self._bridge.call("stop")
        ...

    def get_ultrasonic(self):
        """Single atomic read of the front ultrasonic backstop (meters)."""
        # return self._bridge.call("get_range")
        ...


class StubBridge:
    """Hardware-free stand-in: prints commands so the MPU loop can run on a laptop."""

    def set_motion(self, left_pwm, right_pwm):
        print(f"[stub] set_motion(L={left_pwm}, R={right_pwm})")

    def stop(self):
        print("[stub] stop()")

    def get_ultrasonic(self):
        return float("inf")   # pretend nothing is ahead
