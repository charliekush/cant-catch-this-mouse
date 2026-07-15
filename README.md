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
- **USB webcam** — plugged directly into the MPU
- **2× LDRobot LD06** 2D LiDAR — one front, one rear (see LiDAR note below)
- **Ultrasonic sensor** (from kit) — retained purely as an STM32-side emergency-stop backstop

### Why two LiDARs

The UNO Q sits in the **center** of the top plate, surrounded by wiring and mounts. That central
clutter obstructs a single LiDAR's 360° sweep no matter where it's placed, and a riser can't clear
the wire height. So we mount **one LD06 at the front and one at the rear**: each covers the arc the
central obstruction blocks for the other, and the two scans are merged into one 360° picture.

Each unit's body, the central clutter, and the *other* LiDAR appear as fixed phantom returns and are
**masked out per-unit** before the scans are merged (see `app/sensing/lidar.py`).

---

## Architecture

Two processors, two very different jobs:

```
                 ┌─────────────────────── UNO Q MPU (Debian, Python) ───────────────────────┐
   USB webcam ──▶│ camera ─▶ detector ─▶ geometry (bearing, proximity) ─┐                    │
   LD06 front ──▶│ lidar (mask ─▶ merge ─▶ sectorize) ──────────────────┼─▶ evasion policy ─┼─┐
   LD06 rear  ──▶│                                                       ┘                    │ │
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

---

## Repo layout

```
evasion-bot/
├── README.md
├── sketch.yaml                 # arduino-app-cli build manifest
├── .gitignore
│
├── sketch/
│   └── sketch.ino              # STM32: motors, ultrasonic reflex, Bridge RPC
│
├── app/                        # MPU (Python)
│   ├── main.py                 # top-level loop
│   ├── config.py               # all tunables in one place
│   ├── perception/
│   │   ├── camera.py           # USB webcam capture
│   │   ├── detector.py         # TFLite person detection
│   │   └── geometry.py         # bbox → bearing + proximity (pure)
│   ├── sensing/
│   │   ├── lidar.py            # LD06 read, mask, merge, sectorize
│   │   └── ld06_driver.py      # thin wrapper over the LD06 UART driver
│   ├── control/
│   │   ├── evasion.py          # escape policy (pure)
│   │   └── bridge_client.py    # RPC to STM32 (motion, ultrasonic backstop)
│   └── utils/
│       └── logging.py          # structured run logs
│
├── models/                     # TFLite model(s) live here (gitignored if large)
├── scripts/
│   ├── benchmark_fps.py        # week-1 gate: detection FPS on the MPU
│   ├── lidar_viz.py            # visualize/verify merged scan + masks
│   └── collect_frames.py       # save frames for debug/eval
└── docs/
    └── architecture.md         # message schema, pin map, LiDAR offsets/masks
```

---

## Build order (de-risks the unknowns first)

1. **Bridge with a stub** — run the MPU loop end-to-end printing fake motion commands, no hardware.
2. **Detector on recorded video** — benchmark FPS off-robot. **Week-1 gate: must clear ~10 FPS.**
   Re-run with the LiDAR driver(s) active, since both share the MPU.
3. **STM32 sketch alone** — motors + ultrasonic reflex, driven by manual RPC calls.
4. **LiDAR bring-up** — one LD06, then two: verify masks and scan merge with `lidar_viz.py`.
5. **Join everything** — real bridge, camera, motors, LiDAR.

---

## Setup notes

- Install OpenCV via **apt** (`sudo apt install python3-opencv`), *not* pip, on the board's ARM/Debian.
- `Arduino_RouterBridge` must be listed in `sketch.yaml`; **verify the RPC API names against the
  actual library** before writing against them.
- Keep any STM32 sensor read a single atomic RPC — no multi-round-trip reads.
- Measure each LD06's mounting offset (x, y, yaw) from the robot center — the scan merge depends on it.

---

## Must haves vs nice to haves

**Must haves:** on-device person detection · bearing + proximity · reactive escape steering ·
LiDAR-based corner detection + avoidance · time-to-capture metric.

**Nice to haves:** learned evasion policy (RL in sim → transfer) · multi-pursuer / target
re-ID · live FPV / detection overlay stream.
