"""All tunable constants in one place.

During tuning you will be twiddling these constantly; keep them here rather than
scattered through the code so tuning stays sane.
"""

# ---- Camera / perception ----
FRAME_WIDTH = 640                # nominal ESP32 stream resolution (its framesize setting)
FRAME_HEIGHT = 480
# ESP32-WROVER camera module's MJPEG stream (stock Espressif CameraWebServer
# layout: stream server on server_port + 1). Default AP IP/port when the
# module boots in WIFI_AP mode, per ELEGOO's reference firmware. Network
# prerequisite: the MPU must already be a WiFi client of this AP (or of
# whatever network the ESP32 is reflashed to join in STA mode) -- camera.py
# does not join the network for you.
CAMERA_STREAM_URL = "http://192.168.4.1:81/stream"
CAMERA_TIMEOUT_S = 5.0           # seconds before a stalled connect/read gives up
DETECT_SCORE_THRESHOLD = 0.5    # min confidence to count a person detection
PERSON_CLASS_ID = 0             # class index for "person" in the TFLite model

# ---- Detector backend ----
# "bbox"  -> PersonDetector (detector.py): TFLite SSD, box-height proximity proxy.
# "pose"  -> PoseIdentityDetector (pose_identity.py): 17-keypoint pose tracking
#            gated to enrolled people; unknown people report as no detection.
# Both implement the same .best(frame) -> Detection | None surface, so
# geometry.py and the rest of the loop don't change with the backend.
DETECTOR_BACKEND = "bbox"

# ---- Pose + identity backend ----
CAMERA_HFOV_DEG = 70.42                    # C920 horizontal FOV; recalibrate per camera
POSE_MODEL_PATH = "models/movenet_lightning.tflite"  # MoveNet SinglePose Lightning (UNO Q path)
POSE_MIN_CONFIDENCE = 0.5
POSE_CONFIRM_FRAMES = 3         # consecutive detections before a track is reported
POSE_EMA_ALPHA = 0.5            # per-keypoint smoothing factor
POSE_LOST_TIMEOUT_S = 1.0       # track dropped if unseen this long
POSE_KP_MIN_CONF = 0.3          # keypoint visibility threshold (tracking + bbox synthesis)

IDENTITY_DIR = "data/identities"           # one .npz per enrolled person
ID_THRESHOLD = 0.55             # raise toward 0.7 if strangers slip through, lower toward 0.45 in bad light
ID_VOTE_WINDOW = 8              # frames considered for the identity vote
ID_MIN_VOTES = 3                # min votes in the window before a decision is trusted
ID_HIST_H_BINS = 30             # hue bins for the torso appearance histogram
ID_HIST_S_BINS = 32             # saturation bins
ID_MIN_TORSO_PATCH_PX = 24      # torso crop must be at least this tall/wide to sign
ENROLL_N_SAMPLES = 40           # torso signatures captured per enrollment
ENROLL_SAMPLE_GAP_S = 0.25      # min seconds between captured enrollment samples

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
