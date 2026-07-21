# Pose Tracking with Identity Gating

Smooth stick-figure tracking of the pursuer, gated to **enrolled people
only**: the robot tracks you and your partner, and ignores strangers.
Like the bbox backend, the camera reports **bearing only**; depth/range is
the LiDAR's job (see `docs/architecture.md`).

This is an alternative `app.perception` backend, not a separate app. Set
`config.DETECTOR_BACKEND = "pose"` (or `python -m app.main --backend pose`)
to run it in place of the default bbox `PersonDetector` — both implement the
same `.best(frame) -> Detection | None` surface, so `geometry.py`,
`evasion.py`, and the bridge integration are unchanged either way (see
`app/config.py`'s "Detector backend" section for the full list of tunables).

## Files

```
app/perception/
├── pose_tracker.py    <- 17-keypoint pose backends (MediaPipe / MoveNet)
│                          + temporal tracker (confirm, EMA, occlusion freeze)
├── human_style.py      <- threaded detection + temporal interpolation
│                          + skeleton rendering (vladmandic/human techniques)
├── identity.py          <- person re-ID: torso appearance signatures,
│                          enrollment DB, temporal-voting matcher
└── pose_identity.py     <- PoseIdentityDetector: wraps the above behind the
                            same .best(frame) surface as PersonDetector

scripts/human_demo.py    <- laptop-only enrollment + live-display dev tool

data/identities/          <- created by enrollment (one .npz per person)
models/movenet_lightning.tflite  <- add for the UNO Q (see below)
```

## Setup (laptop)

```
pip install mediapipe "opencv-python==4.10.0.84" numpy
```
(If mediapipe won't install on Python 3.13: `conda create -n mousebot
python=3.11 -y && conda activate mousebot`, then the pip line above.)

## 1. Enroll each team member (once, ~15 s each)

Do this **in the demo room, wearing demo-day clothes**, and wear clearly
different shirt colors from each other, from the repo root:

```
python -m scripts.human_demo --display --enroll jaafar
python -m scripts.human_demo --display --enroll ryan
```

Stand 2-3 m back with your full body in frame and slowly turn/move; it
auto-collects 40 torso appearance samples and saves
`data/identities/<name>.npz`. Re-enroll any time you change clothes.

## 2. Try it on the laptop webcam

```
python -m scripts.human_demo --display
```

Once anyone is enrolled, identity gating turns on automatically: enrolled
person -> **green skeleton with their name**; anyone else -> **red skeleton,
"UNKNOWN"**, console shows `[ignored]`. `--any-person` disables gating;
`--id-threshold` (default `config.ID_THRESHOLD`) raises or lowers
strictness; `--quality accurate` uses the heavier model. This script is a
laptop dev tool only (owns its own webcam capture + display) — it is not
what runs on the robot.

## 3. Run it on the robot

No separate entry point: `app/main.py`'s loop (camera -> detector -> geometry
-> evasion -> bridge) is backend-agnostic. Point it at the pose backend:

```
python -m app.main --backend pose
```

Frames come from the same ESP32-WROVER `Camera` the bbox backend uses;
bearing/proximity flow through the same `geometry.py` + `evasion.escape()` +
`bridge_client.set_motion(left_pwm, right_pwm)` path. A gated-out stranger
(or nobody tracked) reports as no detection, exactly like `PersonDetector`
seeing nobody — the loop coasts/idles the same way either backend runs.

## How the re-ID works (for the report)

The single-pose model (MediaPipe / MoveNet — same default model family as
the vladmandic/human library) locks onto the most prominent person; the
identity gate then verifies who that is. The pose keypoints give the exact
torso quad (shoulders->hips), which is cropped and summarized as a smoothed
2D Hue-Saturation histogram (V dropped for lighting tolerance; histogram
Gaussian-blurred so near-identical shades correlate across bin edges —
without this, a solid-color shirt is a single-bin spike and the same shirt
under slightly different light scores ~0). Runtime score = best correlation
against the person's 40 enrolled samples, decided over an 8-frame voting
window (min 3 votes) so one noisy frame can't flip the identity.

Measured on synthetic tests: same shirt under a lighting shift scores
0.87-0.98; a different-colored shirt scores ~ -0.05; a stranger is
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

MediaPipe is heavy on the board — download **MoveNet SinglePose Lightning**
(.tflite, ~3 MB) to `models/movenet_lightning.tflite` (`config.POSE_MODEL_PATH`);
`PoseIdentityDetector` auto-selects MediaPipe if installed, else MoveNet if
the model file exists there. Enroll on the laptop and copy `data/identities/`
to the board (signatures are camera-agnostic histograms).

## Tuning

`config.ID_THRESHOLD`: raise toward 0.7 if strangers slip through; lower
toward 0.45 if you get flagged UNKNOWN in bad light. `config.ID_VOTE_WINDOW` /
`config.ID_MIN_VOTES`: bigger window = more stable, slower to react.
`config.POSE_EMA_ALPHA`: skeleton smoothing. Zone/steering thresholds live in
`config.py` alongside the evasion policy.
