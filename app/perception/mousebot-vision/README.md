# MouseBot — Pose Tracking with Identity Gating (C920 + UNO Q)

Smooth stick-figure tracking of the pursuer, gated to **enrolled people
only**: the robot tracks you and your partner, and ignores strangers.
The camera reports **bearing only** (0 = ahead, − = left, + = right);
depth/range is the LiDAR's job on the MCU side.

## Files (this is the whole vision subsystem)

```
mousebot/
├── python/
│   ├── pose_tracker.py   <- 17-keypoint pose backends (MediaPipe / MoveNet)
│   │                        + temporal tracker (confirm, EMA, occlusion freeze)
│   ├── human_style.py    <- threaded detection + temporal interpolation
│   │                        + skeleton rendering (vladmandic/human techniques)
│   ├── identity.py       <- person re-ID: torso appearance signatures,
│   │                        enrollment DB, temporal-voting matcher
│   └── human_demo.py     <- THE entry point (enroll + run)
└── data/
    ├── identities/           <- created by enrollment (one .npz per person)
    └── movenet_lightning.tflite   <- add for the UNO Q (see below)
```

## Setup (laptop)

```
pip install mediapipe "opencv-python==4.10.0.84" numpy
```
(If mediapipe won't install on Python 3.13: `conda create -n mousebot
python=3.11 -y && conda activate mousebot`, then the pip line above.)

## 1. Enroll each team member (once, ~15 s each)

Do this **in the demo room, wearing demo-day clothes**, and wear clearly
different shirt colors from each other:

```
cd mousebot/python
python3 human_demo.py --display --enroll jaafar
python3 human_demo.py --display --enroll ryan
```

Stand 2–3 m back with your full body in frame and slowly turn/move; it
auto-collects 40 torso appearance samples and saves
`data/identities/<name>.npz`. Re-enroll any time you change clothes.

## 2. Run

```
python3 human_demo.py --display            # laptop
python3 human_demo.py --bridge             # UNO Q, headless
```

Once anyone is enrolled, identity gating turns on automatically:
enrolled person → **green skeleton with their name**, bearing streamed
(and RPC'd to the MCU with --bridge); anyone else → **red skeleton,
"UNKNOWN — ignored"**, no bearing sent, console shows `[ignored]`.
`--any-person` disables gating; `--id-threshold` (default 0.55) raises or
lowers strictness; `--quality accurate` uses the heavier model.

## How the re-ID works (for the report)

The single-pose model (MediaPipe / MoveNet — same default model family as
the vladmandic/human library) locks onto the most prominent person; the
identity gate then verifies who that is. The pose keypoints give the exact
torso quad (shoulders→hips), which is cropped and summarized as a smoothed
2D Hue–Saturation histogram (V dropped for lighting tolerance; histogram
Gaussian-blurred so near-identical shades correlate across bin edges —
without this, a solid-color shirt is a single-bin spike and the same shirt
under slightly different light scores ~0). Runtime score = best correlation
against the person's 40 enrolled samples, decided over an 8-frame voting
window (min 3 votes) so one noisy frame can't flip the identity.

Measured on synthetic tests: same shirt under a lighting shift scores
0.87–0.98; a different-colored shirt scores ≈ −0.05; a stranger is
rejected. The voting window is also unit-tested: a single frame can never
flip the decision.

**Honest limitations** (state these in the report): it identifies
*clothing*, not faces — enroll in demo clothes; two people in
near-identical shirts are indistinguishable; extreme lighting changes
lower scores (enroll in the demo environment). These are acceptable
trade-offs vs. face recognition, which fails at distance/from behind —
exactly the geometries an evasion robot sees — and needs much heavier
dependencies.

## UNO Q

```
scp -r mousebot arduino@<board>.local:~/
pip install opencv-python-headless numpy --break-system-packages
```
MediaPipe is heavy on the board — download **MoveNet SinglePose Lightning**
(.tflite, ~3 MB) to `data/movenet_lightning.tflite`; the code auto-selects
it. Enroll on the laptop and just copy `data/identities/` to the board
(signatures are camera-agnostic histograms). `--bridge` calls
`Bridge.call("set_target", bearing_deci_deg)` at up to 20 Hz — only for
enrolled people; the motor sketch should `Bridge.provide("set_target",...)`
and fuse with the LiDAR range.

## Tuning

`--id-threshold`: raise toward 0.7 if strangers slip through; lower toward
0.45 if you get flagged UNKNOWN in bad light. `IdentityMatcher(window=8,
min_votes=3)`: bigger window = more stable, slower to react.
`PoseTracker(alpha=0.5)`: skeleton smoothing. Zone/steering thresholds
live on the MCU side with the LiDAR.
