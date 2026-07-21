"""
human_demo.py
-------------
Laptop-only dev tool for the pose+identity backend (app.perception.pose_identity):
smooth stick-figure tracking of ENROLLED people only, with a live display.

Owns its own local webcam capture (cv2.VideoCapture) and enrollment UX, which
the robot loop has no use for -- app.main's --backend pose path talks to
PoseTracker/IdentityMatcher directly, over the robot's ESP32 camera stream,
with no display and no local webcam. Use this script to enroll people and to
sanity-check tracking/identity behavior before trusting it on-robot.

Identity gating: if anyone is enrolled (data/identities/*.npz), the tracker
verifies the tracked person's torso appearance against the enrolled
signatures. Enrolled -> green skeleton with their name. Stranger -> red
skeleton labeled UNKNOWN, console shows [ignored]. Disable gating with
--any-person.

ENROLL each team member once (in demo clothes, in the demo room):
    python -m scripts.human_demo --display --enroll jaafar
    python -m scripts.human_demo --display --enroll ryan
Stand ~2-3 m back, slowly turn and move around; it auto-collects 40 torso
samples (~15 s) and saves data/identities/<name>.npz.

RUN:
    python -m scripts.human_demo --display                 # laptop webcam
    python -m scripts.human_demo --display --any-person    # gating off
    python -m scripts.human_demo --display --quality accurate
"""

import argparse
import sys
import time

import cv2

from app import config
from app.perception.pose_tracker import (PoseTracker, MediaPipePoseDetector,
                                         MoveNetDetector)
from app.perception.human_style import ThreadedPoseTracker, draw_human_style
from app.perception.identity import (IdentityDB, IdentityMatcher, Enroller,
                                     torso_signature)

MODELS_DIR = "models"


def build_detector(quality: str, min_conf: float):
    try:
        import mediapipe as mp  # noqa: F401
        det = MediaPipePoseDetector(min_conf)
        if quality == "accurate":
            det.pose.close()
            det.pose = mp.solutions.pose.Pose(
                model_complexity=2,
                min_detection_confidence=min_conf,
                min_tracking_confidence=min_conf)
        print(f"[pose] MediaPipe active (quality={quality})")
        return det
    except ImportError:
        pass
    name = "movenet_thunder.tflite" if quality == "accurate" else "movenet_lightning.tflite"
    path = f"{MODELS_DIR}/{name}"
    import os
    if os.path.exists(path):
        print(f"[pose] MoveNet active: {name}")
        return MoveNetDetector(path)
    raise SystemExit(
        f"No pose backend. Laptop: pip install mediapipe\n"
        f"UNO Q: download MoveNet to {path}")


def run_enrollment(cap, tracker, name, display):
    """Collect torso signatures for one person and save them."""
    enroller = Enroller()
    db = IdentityDB(config.IDENTITY_DIR)
    print(f"[enroll] Enrolling '{name}'. Stand 2-3 m back, full body in "
          f"frame, slowly turn/move. Collecting {enroller.n_samples} samples...")
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.2)
            continue
        track = tracker.update(frame)
        done = False
        if track and track.fresh:
            done = enroller.offer(frame, track.kps)
        if display:
            if track:
                draw_human_style(frame, track, label=f"enrolling {name} "
                                 f"{enroller.progress:.0%}")
            cv2.imshow("MouseBot - enrollment", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("[enroll] aborted")
                return
        if done:
            path = db.save(name, enroller.samples)
            print(f"[enroll] saved {len(enroller.samples)} signatures -> "
                  f"{path}")
            print(f"[enroll] enrolled people: {db.names()}")
            return


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--quality", choices=["fast", "accurate"], default="fast")
    ap.add_argument("--confidence", type=float, default=config.POSE_MIN_CONFIDENCE)
    ap.add_argument("--display", action="store_true")
    ap.add_argument("--enroll", metavar="NAME",
                    help="enrollment mode: capture this person's signature")
    ap.add_argument("--any-person", action="store_true",
                    help="disable identity gating (track anyone)")
    ap.add_argument("--id-threshold", type=float, default=config.ID_THRESHOLD,
                    help="identity match threshold (raise if strangers pass)")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        sys.exit(f"Could not open camera {args.camera}.")

    tracker = PoseTracker(build_detector(args.quality, args.confidence),
                          confirm_frames=2)

    if args.enroll:
        run_enrollment(cap, tracker, args.enroll.strip().lower(),
                       args.display)
        cap.release()
        if args.display:
            cv2.destroyAllWindows()
        return

    db = IdentityDB(config.IDENTITY_DIR)
    gate = (len(db) > 0) and not args.any_person
    matcher = IdentityMatcher(db, threshold=args.id_threshold) if gate else None
    if gate:
        print(f"[identity] gating ON - tracking only: {db.names()} "
              f"(threshold {args.id_threshold})")
    else:
        print("[identity] gating OFF - tracking any person"
              + ("" if args.any_person else
                 "  (no one enrolled; use --enroll NAME)"))

    threaded = ThreadedPoseTracker(tracker)
    frames, t0 = 0, time.time()
    last_print = 0.0
    was_tracking = False
    print("[demo] running. Ctrl-C or 'q' to stop.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.5)
                continue
            frames += 1

            threaded.submit(frame)
            track = threaded.current()
            now = time.time()

            name, known = None, True
            if track:
                if not was_tracking and matcher:
                    matcher.reset()          # new target -> fresh vote window
                was_tracking = True
                if gate:
                    matcher.update(torso_signature(frame, track.kps))
                    name, score = matcher.decide()
                    known = name is not None
            else:
                was_tracking = False

            authorized = track is not None and (not gate or known)

            if track and now - last_print > 0.1:
                last_print = now
                who = name if name else ("person" if not gate else "UNKNOWN")
                print(f"TRACK  {who:<10s} bearing {track.bearing_deg:+7.1f} "
                      f"deg  conf {track.confidence:.2f}  "
                      f"(display {frames/(now-t0):.1f} fps, "
                      f"model {threaded.detect_fps:.1f} fps)"
                      + ("" if authorized else "  [ignored]"))

            if args.display:
                if track:
                    draw_human_style(frame, track, threaded.detect_fps,
                                     label=name, known=(not gate) or known)
                else:
                    cv2.putText(frame, "searching for person...", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.imshow("MouseBot - identity-gated tracking", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        threaded.stop()
        cap.release()
        if args.display:
            cv2.destroyAllWindows()
        print(f"\n[demo] display {frames/(time.time()-t0):.1f} fps avg")


if __name__ == "__main__":
    main()
