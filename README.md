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
- **Camera** — either a USB webcam plugged directly into the MPU (V4L2), or
  the kit's stock **ESP32-WROVER camera module**: its own microcontroller,
  hosting a WiFi AP and streaming MJPEG over HTTP from its own web server. It
  is *not* a USB device — it links to the main shield only via a 4-pin UART
  header for command relay, never for video (see `docs/architecture.md`).
  Selected via `config.CAMERA_SOURCE` / `--camera`.
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
   Camera     ──▶│ camera ─▶ detector ─▶ geometry (bearing, proximity) ─┐                    │
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
│   ├── config.py               # all tunables in one place
│   ├── perception/
│   │   ├── camera.py           # USB webcam / ESP32-CAM stream capture
│   │   ├── detector.py         # TFLite person detection
│   │   ├── geometry.py         # bbox → bearing + proximity (pure)
│   │   ├── identity.py         # person re-ID: torso signatures, enrollment DB, voting matcher
│   │   ├── pose_tracker.py     # pose-backend tracker (currently unwired, see below)
│   │   ├── pose_identity.py    # pose+identity backend (currently unwired, see below)
│   │   ├── human_style.py      # threaded detection + temporal interpolation + rendering
│   │   └── README.md           # pose+identity backend: setup, enrollment, re-ID design notes
│   ├── sensing/
│   │   ├── lidar.py            # LD19 read, mask, merge, sectorize
│   │   └── ld19_driver.py      # thin wrapper over lds2d's LD19 driver
│   ├── control/
│   │   ├── evasion.py          # escape policy (pure)
│   │   ├── bridge_client.py    # RPC to STM32 (motion, ultrasonic backstop)
│   │   └── bt_console.py       # Bluetooth remote-control server, see below
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
│   ├── bt_client.py            # Linux Bluetooth client for bt_console.py
│   └── human_demo.py           # laptop dev tool for the pose backend (currently unwired)
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
- `Arduino_RouterBridge` (STM32 side) and `arduino.app_utils.Bridge` (MPU side) are both used as
  documented in `docs/architecture.md`, verified against their actual source.
- LiDAR parsing uses `lds2d` (`pip install lds2d`) rather than a hand-rolled protocol decoder; see
  `app/sensing/ld19_driver.py` for the caveat on its LD19 support being unverified on real hardware
  by lds2d's own maintainers.
- Keep any STM32 sensor read a single atomic RPC — no multi-round-trip reads.
- Measure each LD19's mounting offset (x, y, yaw) from the robot center — the scan merge depends on it.
- `app/perception/pose_tracker.py`, `pose_identity.py`, and `human_style.py` are a pose-based
  detection backend that is currently unwired from `app.main` (no `--backend` switch); see
  `DEVELOPMENT_LOG.md` if re-integrating it.

---

## Must haves vs nice to haves

**Must haves:** on-device person detection · bearing + proximity · reactive escape steering ·
LiDAR-based corner detection + avoidance · time-to-capture metric.

**Nice to haves:** learned evasion policy (RL in sim → transfer) · multi-pursuer / target
re-ID · live FPV / detection overlay stream.

## Bluetooth command console

`app/control/bt_console.py` runs a small RFCOMM (Bluetooth serial) server on
the MPU as a wireless remote for the robot, separate from the `app.main`
evasion loop itself:

```
python -m app.control.bt_console --front-port /dev/ttyUSB0 --rear-port /dev/ttyUSB1
```

Pair the board once via `bluetoothctl` (`power on`, `discoverable on`,
`pairable on`, `agent on`, then pair from your phone/laptop). From a phone,
any Bluetooth serial-terminal app connecting on RFCOMM channel 1 works. From
a Linux PC, use `scripts/bt_client.py` rather than the `rfcomm` CLI tool —
`rfcomm`/`hcitool`/`sdptool` are legacy `bluez-utils` tools deprecated
upstream and missing on plenty of distros:

```
python -m scripts.bt_client 14:B5:CD:EA:BB:09
```

Either way, send one command per line:

- `start` — launch `app.main.run` in a background thread
- `stop` — signal it to stop and join (the loop's own `finally` block halts
  the motors via `bridge.stop()`)
- `usb` — `lsusb` output plus any `/dev/ttyUSB*`/`/dev/ttyACM*` ports found,
  handy for confirming which port is the front vs. rear LD19

It uses the standard library's `socket.AF_BLUETOOTH`/`BTPROTO_RFCOMM`
directly (Linux-only, needs BlueZ installed — `sudo apt install bluetooth
bluez`), so no extra Python package is required. `--stub` makes `start` run
the evasion loop hardware-free, same as `app.main --stub`.

## Getting the detection model

The model binaries are gitignored; fetch them once per machine:

```
bash scripts/fetch_models.sh
```

This pulls a uint8-quantized SSD MobileNet V2 (COCO, 300x300) to
`models/person_detect.tflite`. It has been verified against `detector.py`
unmodified: uint8 input, output order `[boxes, classes, scores]`, person =
class 0. See `models/README.md` for the full tensor layout and an alternative
model.

## Watching the camera live in a browser

No display needed on the board -- stream to any browser on the same network:

```
python3 -m scripts.watch --camera 0                 # raw feed
python3 -m scripts.watch --camera 0 --annotate       # boxes + identity gate live
```

Then open `http://<board-hostname>.local:8080/` from your laptop. Frame rate
and JPEG quality are capped (`--fps`, `--quality`) so the stream does not
compete with the detector for CPU/bandwidth. Ctrl-C on the board to stop.

## Testing the vision stack

```
python -m scripts.vision_preview --inspect              # check a new model file
python -m scripts.vision_preview                        # live camera
python -m scripts.vision_preview --source data/frames   # offline, saved frames
python -m scripts.vision_preview --source shot.jpg --save out.jpg --no-window
```

Draws every person detection, marks which one the identity gate selected
(green = selected, red = rejected), and overlays the exact bearing and
proximity the evasion policy will receive. Use it to sanity-check the bearing
sign and to tune `PROXIMITY_MIN/MAX_BOX_FRAC` -- stand at the distance that
should read "caught" and adjust until proximity crosses `CAUGHT_PROXIMITY`.

`--inspect` prints the model's input shape/dtype and output tensor order; if
detections look wrong, that is the first thing to check, since
`detector._read_outputs()` assumes the order `[boxes, classes, scores]` and it
varies between models.

## Evaluating a run

```
python -m scripts.analyze_runs data/runs/*.csv            # time-to-capture
python -m scripts.analyze_runs data/runs/*.csv --plot survival.png
```

Reports survival time per run (runs where the pursuer never got close are
marked ESCAPED and credited the full duration), plus mean loop FPS, how much of
the run the pursuer was visible, and how often the robot was cornered. Compare
configurations on **mean survival across several runs** -- single runs are noisy.

## Tests

```
pip install pytest
python -m pytest tests/ -q
```

54 unit tests over the pure logic (geometry, evasion policy, LiDAR sectorizing,
identity gate, run analysis). No hardware, camera, or model file required, so
they run on any laptop. See `tests/README.md`.
