# Architecture

## Processor split

| Concern | Processor | Notes |
|---|---|---|
| Camera capture | MPU (Debian) | ELEGOO kit's ESP32-WROVER camera, MJPEG over HTTP via `urllib` (not V4L2/USB) — see note below |
| Person detection (TFLite) | MPU | `app/perception/detector.py` |
| Bearing / proximity | MPU | `app/perception/geometry.py` (pure) |
| LiDAR read / mask / merge / sectorize | MPU | both LD19s on MPU UART |
| Evasion policy | MPU | `app/control/evasion.py` (pure) |
| Motor PWM (TB6612) | STM32 (Zephyr) | executes commands from MPU |
| Ultrasonic safety reflex | STM32 | **local** hard-stop, no MPU round-trip |

The MPU decides; the STM32 acts and keeps one fast local reflex so a slow vision
frame can never cause a head-on collision.

## Camera

The robot currently uses the stock ELEGOO ESP32-WROVER camera module that
ships with the kit (a second, USB-connected webcam plugged directly into the
UNO Q's own USB host may be added later, but is not present yet). This module
is its own microcontroller, not a USB device:

- It connects to the main shield only through a 4-pin serial header
  (`XH2.54-4P`: VCC/GND/TX/RX, UART) used for command relay, never for video.
- ELEGOO's reference firmware (`docs/ELEGOO Smart Robot Car Kit V4.0
  2021.04.14/02 Manual & Main Code & APP/04 Code of Carmer (ESP32)/`) boots
  it in `WIFI_AP` mode with its own SSID and default IP `192.168.4.1`, and
  registers two HTTP servers via the stock Espressif `app_httpd.cpp`: one on
  `server_port` (80, `/`, `/control`, `/status`, `/capture` for a single JPEG
  snapshot) and one on `server_port + 1` (81, `/stream` for the MJPEG feed).
  A separate TCP socket on port 100 relays JSON motion commands between a
  connected client and the main board's `Serial2` (UART) — video and control
  are separate channels, and video never reaches the shield or a USB line.

`app/perception/camera.py` pulls frames from `http://192.168.4.1:81/stream`
(`config.CAMERA_STREAM_URL`) using the standard library's `urllib` — not
`requests` (not installed in `.venv`, and not worth adding for this) and not
`cv2.VideoCapture` on the URL (that backend needs an FFMPEG-enabled OpenCV
build, which the board's apt `python3-opencv` package isn't guaranteed to
have; decoding a JPEG buffer via `cv2.imdecode` doesn't need one). It scans
the byte stream for JPEG SOI/EOI markers rather than parsing the multipart
boundary and `Content-Length` headers, since a literal `0xFFD8`/`0xFFD9`
can't appear inside valid JPEG scan data.

**Network prerequisite**: the MPU must already be a WiFi client of the
ESP32's AP (or of whatever network the ESP32 is reflashed to join in STA
mode) for any of this to work — `camera.py` does not join the network for
you. This has not been validated against the real ESP32 yet (see
`DEVELOPMENT_LOG.md` Next Steps); `StubCamera` in `camera.py` lets
`python -m app.main --stub` exercise the rest of the loop without it.

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
