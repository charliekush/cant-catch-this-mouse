# Architecture

## Processor split

| Concern | Processor | Notes |
|---|---|---|
| Camera capture | MPU (Debian) | USB webcam via V4L2/OpenCV |
| Person detection (TFLite) | MPU | `app/perception/detector.py` |
| Bearing / proximity | MPU | `app/perception/geometry.py` (pure) |
| LiDAR read / mask / merge / sectorize | MPU | both LD06s on MPU UART |
| Evasion policy | MPU | `app/control/evasion.py` (pure) |
| Motor PWM (TB6612) | STM32 (Zephyr) | executes commands from MPU |
| Ultrasonic safety reflex | STM32 | **local** hard-stop, no MPU round-trip |

The MPU decides; the STM32 acts and keeps one fast local reflex so a slow vision
frame can never cause a head-on collision.

## MPU -> STM32 messages (RPC over Arduino_RouterBridge)

> Method names are PLACEHOLDERS -- verify against the real library.

- `set_motion(left_pwm, right_pwm)` — signed PWM (-255..255), sign = direction
- `stop()` — halt both motors
- `get_range()` — single atomic ultrasonic read (meters); the STM32 already
  enforces its own stop, this is just for telemetry/backup

## LiDAR geometry (fill in from measurement)

Two LD06s, front and rear, because the centrally-mounted UNO Q + wiring
obstructs any single unit's 360 sweep. Each unit's fixed phantom returns
(central clutter, own body, the other LiDAR) are masked per-unit before merge.

Measure and record:

| Unit | Offset (x, y, yaw) | Masked arcs (deg) |
|---|---|---|
| Front LD06 | (____, ____, ____) | (____, ____) |
| Rear LD06 | (____, ____, ____) | (____, ____) |

These live in `app/config.py` (`*_LIDAR_OFFSET`, `*_LIDAR_MASKS`). Verify with
`scripts/lidar_viz.py`: the masked arcs should show no phantom close returns, and
a known object (e.g. a box at a measured spot) should land correctly in the merged
robot frame.

## Sector reduction

Merged points reduce to six sector minimums (front, front_left, front_right,
left, right, rear). The evasion policy only ever sees these six numbers, never a
raw point cloud. Corner condition: `front_left` and `front_right` both closer
than `CORNER_DISTANCE`.

## Pin map (VERIFY chassis -> UNO Q GPIO)

The ELEGOO chassis wires TB6612 + ultrasonic to specific R3 pins. The UNO Q is
UNO-footprint compatible, but confirm each control pin lands on GPIO the STM32
actually drives before trusting `sketch/sketch.ino`.

| Signal | Sketch pin | Verified? |
|---|---|---|
| PWMA / AIN1 / AIN2 (left motor) | 5 / 7 / 8 | [ ] |
| PWMB / BIN1 / BIN2 (right motor) | 6 / 9 / 11 | [ ] |
| STBY | 3 | [ ] |
| Ultrasonic TRIG / ECHO | 12 / 13 | [ ] |
