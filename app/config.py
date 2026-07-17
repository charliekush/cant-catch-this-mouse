"""All tunable constants in one place.

During tuning you will be twiddling these constantly; keep them here rather than
scattered through the code so tuning stays sane.
"""

# ---- Camera / perception ----
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_INDEX = 0                # /dev/video0
DETECT_SCORE_THRESHOLD = 0.5    # min confidence to count a person detection
PERSON_CLASS_ID = 0             # class index for "person" in the TFLite model

# ---- Proximity model ----
# Person box height (as a fraction of frame height) used as a distance proxy.
# Larger box = closer person. Tune these two anchors on the real setup.
PROXIMITY_MIN_BOX_FRAC = 0.15   # far  -> proximity ~0
PROXIMITY_MAX_BOX_FRAC = 0.90   # near -> proximity ~1

# ---- "Caught" definition ----
# Loss condition: pursuer proximity crosses this threshold.
CAUGHT_PROXIMITY = 0.85

# ---- LiDAR ----
LIDAR_SCAN_HZ = 10              # LD19 nominal scan rate
# Per-unit angular masks: arcs (deg, robot frame) where each LiDAR sees the
# central clutter / the other LiDAR / its own body. Points inside are dropped.
# Measure these once on the real mount and fill them in. (start, end) in degrees.
FRONT_LIDAR_MASKS = [(150, 210)]   # rear arc blocked by central board (example)
REAR_LIDAR_MASKS = [(-30, 30)]     # front arc blocked by central board (example)
# Mounting offsets from robot center (meters, radians). Fill from measurement.
FRONT_LIDAR_OFFSET = (0.10, 0.0, 0.0)      # (x, y, yaw)
REAR_LIDAR_OFFSET = (-0.10, 0.0, 3.14159)  # rear unit faces backward

# Sector boundaries (deg, robot frame, 0 = straight ahead, +CCW).
# Each sector reports the nearest obstacle distance within its arc.
SECTORS = {
    "front":       (-20, 20),
    "front_left":  (20, 60),
    "front_right": (-60, -20),
    "left":        (60, 120),
    "right":       (-120, -60),
    "rear":        (120, 240),   # wraps through 180
}

# ---- Corner detection ----
CORNER_DISTANCE = 0.40          # m; walls closer than this in both front sides = corner
SAFETY_DISTANCE = 0.25          # m; LiDAR soft-stop distance on the MPU

# ---- Evasion policy ----
BASE_SPEED = 120                # PWM baseline when fleeing
MAX_SPEED = 255                 # PWM cap
PROXIMITY_SPEED_GAIN = 135      # extra PWM scaled by proximity (closer -> faster)
TURN_GAIN = 100                 # how hard bearing maps into differential steering

# ---- Control loop ----
LOOP_HZ_TARGET = 10             # aspirational loop rate; gated by detector FPS
