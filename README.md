# Evasion Bot

An autonomous robot car that uses on-device vision to detect a pursuing person and
drive away to avoid being caught, while using dual 2D LiDAR for corner detection and
obstacle avoidance. All perception and decision-making run on-device on an Arduino UNO Q.

ECE 180 — Team #4 — UC San Diego / JSOE

---

## What it does

The robot watches for a person with its camera, works out roughly **where** that person is
(left/right bearing) and **how close** they are (proximity), and steers to keep them behind
it — fleeing faster as they get closer. Two LiDARs give it a 360° picture of surrounding
walls so it can avoid driving itself into a corner while escaping.

"Caught" is defined as the pursuer's proximity crossing a set threshold (see `app/config.py`).
The primary evaluation metric is **time-to-capture**.

---

## Hardware

- **Arduino UNO Q** — dual processor:
  - **Qualcomm MPU (Debian Linux)** — camera capture, person detection, LiDAR processing, evasion policy
  - **STM32U585 (Zephyr RTOS)** — motor PWM, ultrasonic safety reflex, executes motion commands
- **ELEGOO Smart Robot Car V4.0** chassis (TB6612 motor driver, DC motors)
- **Camera** — currently the kit's stock **ESP32-WROVER camera module**: its own
  microcontroller, hosting a WiFi AP and streaming MJPEG over HTTP from its own
  web server. It is *not* a USB device — it links to the main shield only via a
  4-pin UART header for command relay, never for video (see `docs/architecture.md`).
  A second, USB-connected webcam plugged directly into the MPU may be added later.
- **2× LDRobot LD19** 2D LiDAR — one front, one rear (see LiDAR note below)
- **Ultrasonic sensor** (from kit) — retained purely as an STM32-side emergency-stop backstop

### Why two LiDARs

The UNO Q sits in the **center** of the top plate, surrounded by wiring and mounts. That central
clutter obstructs a single LiDAR's 360° sweep no matter where it's placed, and a riser can't clear
the wire height. So we mount **one LD19 at the front and one at the rear**: each covers the arc the
central obstruction blocks for the other, and the two scans are merged into one 360° picture.

Each unit's body, the central clutter, and the *other* LiDAR appear as fixed phantom returns and are
**masked out per-unit** before the scans are merged (see `app/sensing/lidar.py`).

---

## Architecture

Two processors, two very different jobs:

```
                 ┌─────────────────────── UNO Q MPU (Debian, Python) ───────────────────────┐
   ESP32 cam  ──▶│ camera ─▶ detector ─▶ geometry (bearing, proximity) ─┐                    │
   LD19 front ──▶│ lidar (mask ─▶ merge ─▶ sectorize) ──────────────────┼─▶ evasion policy ─┼─┐
   LD19 rear  ──▶│                                                       ┘                    │ │
                 └────────────────────────────────────────────────────────────────────────┘ │
                                                                                              │ motion cmd (RPC)
                 ┌─────────────────────── STM32 (Zephyr) ──────────────────────────────────┐ │
                 │ set_motion(left_pwm, right_pwm) ◀───────────────────────────────────────┼─┘
                 │ ultrasonic safety reflex: if range < STOP → halt motors locally         │
                 └──────────────────────────────────────────────────────────────────────────┘
```

Key principle: the STM32 holds a **local safety reflex** (halt if the ultrasonic reads too close)
that does not wait on the MPU. A slow vision frame can never cause a head-on collision.
The LiDAR does all the *smart* spatial reasoning on the MPU; the ultrasonic is a dumb, fast backstop.

---

## The main loop (MPU)

```
capture frame
  ─▶ detect person            → bbox
  ─▶ geometry                 → bearing, proximity
read + merge + sectorize LiDAR → sector distances (front, FL, FR, left, right, rear)
  ─▶ evasion policy           → heading, speed
  ─▶ convert to L/R PWM
  ─▶ bridge.set_motion(...)   → STM32
  ─▶ log everything (for time-to-capture eval)
repeat
```

Person detection is swappable behind `config.DETECTOR_BACKEND` (`--backend` on
`app.main`): the default `"bbox"` backend (`app/perception/detector.py`) is a
TFLite bounding-box detector; an alternative `"pose"` backend
(`app/perception/pose_identity.py`) does 17-keypoint pose tracking gated to
enrolled team members only (strangers are ignored). Both expose the same
`.best(frame) -> Detection | None` surface, so nothing downstream changes
with the backend — see `app/perception/README.md`.

---

## Repo layout

```
mouse-bot/
├── README.md
├── sketch.yaml                 # arduino-app-cli build manifest
├── .gitignore
│
├── sketch/
│   └── sketch.ino              # STM32: motors, ultrasonic reflex, Bridge RPC
│
├── app/                        # MPU (Python)
│   ├── main.py                 # top-level loop
│   ├── config.py               # all tunables in one place, incl. DETECTOR_BACKEND
│   ├── perception/
│   │   ├── README.md           # pose+identity backend: setup, enrollment, re-ID design notes
│   │   ├── camera.py           # ESP32-WROVER MJPEG capture (kit's stock camera) + StubCamera
│   │   ├── detector.py         # TFLite bbox person detection ("bbox" backend)
│   │   ├── geometry.py         # bbox → bearing + proximity (pure)
│   │   ├── pose_tracker.py     # 17-keypoint pose backends (MediaPipe / MoveNet) + tracker
│   │   ├── identity.py         # person re-ID: torso signatures, enrollment DB, voting matcher
│   │   ├── human_style.py      # threaded detection + temporal interpolation + rendering
│   │   └── pose_identity.py    # PoseIdentityDetector ("pose" backend, gated to enrolled people)
│   ├── sensing/
│   │   ├── lidar.py            # LD19 read, mask, merge, sectorize
│   │   └── ld19_driver.py      # thin wrapper over lds2d's LD19 driver
│   ├── control/
│   │   ├── evasion.py          # escape policy (pure)
│   │   └── bridge_client.py    # RPC to STM32 (motion, ultrasonic backstop)
│   └── utils/
│       └── logging.py          # structured run logs
│
├── models/                     # TFLite model(s) live here (gitignored if large)
├── data/
│   └── identities/              # enrolled person re-ID signatures (one .npz per person)
├── scripts/
│   ├── benchmark_fps.py        # week-1 gate: detection FPS on the MPU
│   ├── lidar_viz.py            # visualize/verify merged scan + masks
│   ├── collect_frames.py       # save frames for debug/eval
│   └── human_demo.py           # laptop dev tool: enroll people + preview pose+identity tracking
└── docs/
    └── architecture.md         # message schema, pin map, LiDAR offsets/masks
```

---

## Build order (de-risks the unknowns first)

1. **Bridge with a stub** — run the MPU loop end-to-end printing fake motion commands, no hardware.
2. **Detector on recorded video** — benchmark FPS off-robot. **Week-1 gate: must clear ~10 FPS.**
   Re-run with the LiDAR driver(s) active, since both share the MPU.
3. **STM32 sketch alone** — motors + ultrasonic reflex, driven by manual RPC calls.
4. **LiDAR bring-up** — one LD19, then two: verify masks and scan merge with `lidar_viz.py`.
5. **Join everything** — real bridge, camera, motors, LiDAR.

---

## Setup notes

- Install OpenCV via **apt** (`sudo apt install python3-opencv`), *not* pip, on the board's ARM/Debian.
- Camera is the kit's stock ESP32-WROVER module (WiFi AP + MJPEG-over-HTTP), not a USB webcam.
  `app/perception/camera.py` pulls `config.CAMERA_STREAM_URL` over plain `urllib` — the MPU must
  already be joined to the ESP32's WiFi AP (`192.168.4.1`) for this to work; see
  `docs/architecture.md`'s Camera section. Not yet validated against the real ESP32 (on-device
  bring-up is next — see `DEVELOPMENT_LOG.md`); use `--stub` (`StubCamera`) to run the rest of the
  loop without it.
- `Arduino_RouterBridge` (STM32 side) and `arduino.app_utils.Bridge` (MPU side) are both used as
  documented in `docs/architecture.md`, verified against their actual source.
- LiDAR parsing uses `lds2d` (`pip install lds2d`) rather than a hand-rolled protocol decoder; see
  `app/sensing/ld19_driver.py` for the caveat on its LD19 support being unverified on real hardware
  by lds2d's own maintainers.
- Keep any STM32 sensor read a single atomic RPC — no multi-round-trip reads.
- Measure each LD19's mounting offset (x, y, yaw) from the robot center — the scan merge depends on it.

---

## Must haves vs nice to haves

**Must haves:** on-device person detection · bearing + proximity · reactive escape steering ·
LiDAR-based corner detection + avoidance · time-to-capture metric.

**Nice to haves:** learned evasion policy (RL in sim → transfer) · multi-pursuer / target
re-ID · live FPV / detection overlay stream.
