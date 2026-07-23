"""Top-level evasion loop (runs on the UNO Q MPU).

    capture -> detect -> geometry -> read+merge+sectorize LiDAR
            -> evasion policy -> PWM -> bridge -> log -> repeat

Run with --stub to exercise the whole loop with no hardware (fake bridge, and
you can point the camera at a video file / webcam on a laptop).
"""

import argparse
import math
import time

from . import config
from .perception.camera import Camera
from .perception.detector import PersonDetector
from .perception import geometry
from .perception import identity
from .sensing import lidar
from .sensing.ld19_driver import LD19
from .control import evasion
from .control.bridge_client import BridgeClient, StubBridge
from .utils.logging import RunLogger


def build_bridge(stub):
    return StubBridge() if stub else BridgeClient()


def run(model_path, front_port, rear_port, log_path, stub=False):
    camera = Camera()
    detector = PersonDetector(model_path)
    bridge = build_bridge(stub)
    logger = RunLogger(log_path)

    # Identity gate: on when anyone is enrolled (see scripts/enroll.py).
    # When on, EVERY detected person is scored each frame and the best-matching
    # enrolled one is chosen, so a stranger standing closer (or arriving first)
    # cannot hold the tracker hostage.
    id_db = identity.IdentityDB()
    gate = config.IDENTITY_GATE and len(id_db) > 0
    selector = identity.PursuerSelector(id_db) if gate else None
    lost_frames = 0
    if gate:
        print(f"[identity] gate ON -- fleeing only: {id_db.names()}")
    else:
        print("[identity] gate off -- fleeing any person")

    front_lidar = rear_lidar = None
    if not stub:
        front_lidar = LD19(front_port)
        rear_lidar = LD19(rear_port)

    try:
        while True:
            t0 = time.time()

            # --- perception ---
            frame = camera.read()
            if frame is None:
                continue
            pursuer = None
            if gate:
                # Score every person in frame; track whoever is enrolled.
                detections = detector.detect(frame)
                person, pursuer = selector.select(frame, detections)
                if person is None:
                    # Either nobody is in frame, or only strangers are. Reset
                    # only after a sustained absence of ANY person, so a
                    # flickered detection does not wipe accumulated votes.
                    lost_frames = 0 if detections else lost_frames + 1
                    if lost_frames >= config.ID_LOST_FRAMES_RESET:
                        selector.reset()
                else:
                    lost_frames = 0
            else:
                person = detector.best(frame)
                pursuer = "person" if person is not None else None

            if person is not None:
                bearing = geometry.bbox_to_bearing(person.bbox)
                proximity = geometry.bbox_to_proximity(person.bbox)
            else:
                bearing, proximity = 0.0, 0.0   # no pursuer seen -> coast/idle

            # --- LiDAR (skipped in stub) ---
            if stub:
                sectors = {name: math.inf for name in config.SECTORS}
            else:
                sectors = lidar.process(front_lidar.read_scan(),
                                        rear_lidar.read_scan())

            # --- decide & act ---
            heading, speed = evasion.escape(bearing, proximity, sectors)
            left_pwm, right_pwm = evasion.heading_to_pwm(heading, speed)
            bridge.set_motion(left_pwm, right_pwm)

            # --- eval / logging ---
            caught = proximity >= config.CAUGHT_PROXIMITY
            fps = 1.0 / max(time.time() - t0, 1e-6)
            logger.log(fps, bearing, proximity, caught,
                       sectors, left_pwm, right_pwm, identity_name=pursuer)
            if caught:
                print(f"[caught] proximity={proximity:.2f} -- run ends")
                break

    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()
        camera.release()
        logger.close()
        if front_lidar:
            front_lidar.close()
        if rear_lidar:
            rear_lidar.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="models/person_detect.tflite")
    ap.add_argument("--front-port", default="/dev/ttyUSB0")
    ap.add_argument("--rear-port", default="/dev/ttyUSB1")
    ap.add_argument("--log", default="run.csv")
    ap.add_argument("--stub", action="store_true",
                    help="run with no hardware (fake bridge, no LiDAR)")
    args = ap.parse_args()
    run(args.model, args.front_port, args.rear_port, args.log, stub=args.stub)


if __name__ == "__main__":
    main()
