"""Save camera frames to disk for debugging and offline detector testing.

    python -m scripts.collect_frames --out data/frames --count 100 --interval 0.2
"""

import argparse
import os
import time

import cv2

from app.perception.camera import Camera


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data/frames")
    ap.add_argument("--count", type=int, default=100)
    ap.add_argument("--interval", type=float, default=0.2)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    camera = Camera()

    saved = 0
    while saved < args.count:
        frame = camera.read()
        if frame is None:
            continue
        path = os.path.join(args.out, f"frame_{saved:04d}.jpg")
        cv2.imwrite(path, frame)
        saved += 1
        time.sleep(args.interval)

    camera.release()
    print(f"saved {saved} frames to {args.out}")


if __name__ == "__main__":
    main()
