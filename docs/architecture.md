# Architecture

## Processor split

| Concern | Processor | Notes |
|---|---|---|
| Camera capture | MPU (Debian) | USB webcam via V4L2/OpenCV |
| Person detection (TFLite) | MPU | `app/perception/detector.py` |
| Bearing / proximity | MPU | `app/perception/geometry.py` (pure) |
| LiDAR read / mask / merge / sectorize | MPU | both LD19s on MPU UART |
| Evasion policy | MPU | `app/control/evasion.py` (pure) |
| Motor PWM (TB6612) | STM32 (Zephyr) | executes commands from MPU |
| Ultrasonic safety reflex | STM32 | **local** hard-stop, no MPU round-trip |

The MPU decides; the STM32 acts and keeps one fast local reflex so a slow vision
frame can never cause a head-on collision.

## MPU -> STM32 messages (RPC over the Bridge)

MPU side (`app/control/bridge_client.py`) uses `arduino.app_utils.Bridge`;
STM32 side (`sketch/sketch.ino`) uses `Arduino_RouterBridge`'s `Bridge`
object. Both APIs were verified against their actual source before use.

- `set_motion(left_pwm, right_pwm)` — signed PWM (-255..255), sign = direction.
  Sent via `Bridge.notify` (fire-and-forget): it's sent every control loop
  iteration, so a dropped ack should never stall motion. Registered on the
  STM32 with `Bridge.provide_safe`, since the handler calls
  `digitalWrite`/`analogWrite`.
- `stop()` — halt both motors. Also `Bridge.notify` / `provide_safe`.
- `get_range()` — single atomic ultrasonic read (meters); the STM32 already
  enforces its own stop, this is just for telemetry/backup. Sent via
  `Bridge.call` since it needs a return value. Registered with
  `Bridge.provide_safe`, since the handler calls `pulseIn`.

## LiDAR geometry (fill in from measurement)

Two LD19s, front and rear, because the centrally-mounted UNO Q + wiring
obstructs any single unit's 360 sweep. Each unit's fixed phantom returns
(central clutter, own body, the other LiDAR) are masked per-unit before merge.

`app/sensing/ld19_driver.py` wraps `lds2d` (PyPI, Apache-2.0), a maintained
plain-pyserial parser that supports LD19 directly, rather than a hand-rolled
protocol decoder. Its LD19 support is ported from the kaiaai/LDS C++ library
and unit-tested against synthetic packets, but not yet hardware-confirmed by
lds2d's own maintainers -- validate its output with `scripts/lidar_viz.py`
before trusting it.

Measure and record:

| Unit | Offset (x, y, yaw) | Masked arcs (deg) |
|---|---|---|
| Front LD19 | (____, ____, ____) | (____, ____) |
| Rear LD19 | (____, ____, ____) | (____, ____) |

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

This shield exposes only one direction pin per TB6612 channel to the MCU
(confirmed against ELEGOO's own `DeviceDriverSet_xxx0.h`/`.cpp` reference
code); the other direction input is hardwired on the shield PCB.

| Signal | Sketch pin | Verified? |
|---|---|---|
| PWMA / AIN1 (right motor) | 5 / 7 | [x] |
| PWMB / BIN1 (left motor) | 6 / 8 | [x] |
| STBY | 3 | [x] |
| Ultrasonic TRIG / ECHO | 12 / 13 | [ ] |

## Target re-identification (identity gate)

`app/perception/identity.py` implements the proposal's "target re-identification"
nice-to-have with no extra model: the person bbox's torso region (shirt) becomes
a smoothed HS histogram, matched against enrolled samples with temporal voting.
The gate turns on automatically once anyone is enrolled (`scripts/enroll.py`),
and the main loop treats an unrecognized person as "no pursuer" so the robot
ignores strangers.

**Multi-candidate selection.** With the gate on, the loop calls
`detector.detect()` (all people) rather than `detector.best()` (most prominent
one) and hands every detection to `PursuerSelector`, which returns the
best-matching enrolled person. This matters because the detector's notion of
"best" is score/prominence: a stranger standing closer, or simply present
first, would otherwise occupy the tracker and an enrolled pursuer entering the
frame would never be evaluated. Each candidate also keeps its OWN voting
window, associated frame-to-frame by bbox IoU (`ID_IOU_MATCH`), so a
stranger's low scores never dilute an enrolled person's average -- an enrolled
pursuer is recognised in exactly `ID_MIN_VOTES` frames regardless of how many
strangers share the frame. Candidates that go unmatched for
`ID_CANDIDATE_MAX_MISSED` frames are dropped, so the selector releases a
pursuer who leaves instead of clinging to a stale box. Cost is sub-millisecond per frame, so it does not threaten
the 10 FPS week-1 gate. Identity is written to the run log (`identity` column)
for the evaluation.

Enroll each pursuer in demo clothes, in the demo room:

    python -m scripts.enroll --name charlie
    python -m scripts.enroll --name jaafar

Tunables live in `app/config.py` under "Identity gate". Limitation: identifies
clothing, not faces -- two pursuers in near-identical shirts are
indistinguishable; extreme lighting changes lower scores. This is an acceptable
trade for a chase robot, since faces are unusable at chase distance / from
behind (exactly what the camera sees), and it keeps the whole pipeline at one
model to protect the FPS budget.


## Evaluation: time-to-capture

`RunLogger` writes one CSV row per loop iteration; `scripts/analyze_runs.py`
turns those into the project's headline metric:

    python -m scripts.analyze_runs data/runs/*.csv
    python -m scripts.analyze_runs data/runs/*.csv --plot survival.png

Survival time is the timestamp of the first `caught == 1` row. Runs where the
pursuer never crossed `CAUGHT_PROXIMITY` are reported as ESCAPED with survival
equal to the full run length, so they are not averaged in as instant captures.
The report also surfaces mean loop FPS (flagged when it falls under the 10 FPS
gate), how much of the run the pursuer was visible, and how often the robot was
boxed in -- the last two explain *why* a survival number came out the way it
did.

Compare configurations by mean survival across several runs, never a single
run: run-to-run spread is large (the synthetic sanity check showed a ~5 s
standard deviation across three runs), so one good run proves nothing.


## Vision bring-up

`scripts/vision_preview.py` is the visual counterpart to `benchmark_fps.py`:
where the benchmark answers "is it fast enough", the preview answers "is it
seeing the right thing, and are the numbers we hand the controller correct".
It renders detections, the identity gate's choice, and the live bearing /
proximity values, and it runs on saved frames as well as a live camera, so
detector regressions can be checked offline without the robot.

Two model-specific assumptions live in `detector.py` and are the usual cause of
a model that "loads but detects nothing": `_preprocess()` feeds a uint8
quantized input, and `_read_outputs()` reads output tensors in the order
`[boxes, classes, scores]`. Run `vision_preview.py --inspect` against a new
model file to see its actual input dtype and output layout before debugging
anything else.
