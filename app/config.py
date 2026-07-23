"""All tunable constants in one place.

During tuning you will be twiddling these constantly; keep them here rather than
scattered through the code so tuning stays sane.
"""

# ---- Camera / perception ----
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_INDEX = 0                # /dev/video0 (find with: v4l2-ctl --list-devices)
# Capture source: an int V4L2 index (USB webcam) OR a stream URL string
# (ESP32-CAM, e.g. "http://192.168.4.1:81/stream"). Scripts accept --camera to
# override this without editing the file.
CAMERA_SOURCE = CAMERA_INDEX
DETECT_SCORE_THRESHOLD = 0.5    # min confidence to count a person detection
PERSON_CLASS_ID = 0             # class index for "person" in the TFLite model
DETECT_THREADS = 4              # interpreter threads; the UNO Q's Cortex-A53
                                 # is quad-core, and 1 thread measured 6 FPS vs
                                 # the 10 FPS gate. Try 2 or 3 if the LiDAR
                                 # driver(s) also compete for CPU once running.

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


# ---- Identity gate (target re-ID nice-to-have) ----
# The robot only flees ENROLLED pursuers; strangers are ignored. Gating turns
# on automatically once anyone is enrolled (scripts/enroll.py); set
# IDENTITY_GATE = False to force it off (e.g. for the FPS benchmark).
IDENTITY_GATE = True
IDENTITY_DIR = "data/identities"
ID_MATCH_THRESHOLD = 0.62       # min correlation to accept a match at all
ID_MATCH_MARGIN = 0.08          # winner must beat the runner-up by this much,
                                # else the frame is "too close to call" and no
                                # one is named. THE main fix for confusing
                                # similar/distant people -- raise to be stricter.
ID_VOTE_WINDOW = 8              # frames of score history per person
ID_MIN_VOTES = 3                # frames needed before a decision is allowed
ID_LOST_FRAMES_RESET = 10       # consecutive no-person frames before vote reset
# Torso crop as fractions of the person bbox (sample shirt, not arms/background)
ID_TORSO_X_FRAC = 0.20          # trim this fraction off each side
ID_TORSO_Y_TOP_FRAC = 0.18      # top of torso band (below the head)
ID_TORSO_Y_BOT_FRAC = 0.52      # bottom of torso band (above the hips)
ID_MIN_PATCH_PX = 20            # torso crop smaller than this is unusable
ID_MIN_PERSON_H_FRAC = 0.35     # person bbox must fill >=35% of frame height;
                                # smaller = too far to identify reliably, so
                                # the gate stays silent instead of guessing.
                                # Lower toward 0.25 if you need longer range and
                                # accept more risk; raise for stricter, closer-only.
ID_MIN_SHIRT_PX = 400           # torso must have at least this many non-dark
                                # pixels for a trustworthy colour histogram
ID_IOU_MATCH = 0.30             # bbox IoU to treat a detection as the same person
ID_CANDIDATE_MAX_MISSED = 15    # frames a candidate survives without a match
# Track lock: once a candidate holds >= ID_LOCK_SCORE for ID_LOCK_SECONDS
# straight, commit to that identity permanently (for the life of the track) and
# stop re-questioning it -- gives the servo cam a stable target.
ID_LOCK_SCORE = 0.90            # sustained score required to commit
ID_LOCK_SECONDS = 2.0           # how long it must hold before locking
ID_H_BINS = 30                  # hue histogram bins
ID_S_BINS = 32                  # saturation histogram bins
