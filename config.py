import numpy as np

# ===========================
# PD Controller
# ===========================

Kp = 0.5
Kd = 0.7

# ===========================
# Curve Handling
# ===========================

CURVE_STEERING_BOOST = 2
LENGTH_RATIO_THRESHOLD = 0.6
CURVE_CONFIDENCE_THRESHOLD = 2

# ===========================
# ROI for Lane Detection
# ===========================

ROI_TOP = 280
ROI_BOTTOM = 500
ROI_LEFT = 0
ROI_RIGHT = 720

# ===========================
# Steering & Speed Limits
# ===========================

MIN_STEERING = -1.0
MAX_STEERING = 1.0
CURVE_MAX_STEERING = 1.2

BASE_SPEED = 0.2
MIN_SPEED = 0.1
CURVE_SPEED = 0.15
MAX_ERROR = 2.0

# ===========================
# Edge / Line Detection
# ===========================

CANNY_LOW = 50
CANNY_HIGH = 150

HOUGH_THRESHOLD = 50
MIN_LINE_LENGTH = 40
MAX_LINE_GAP = 40

MIN_SLOPE = 0.25
MAX_HORIZONTAL_SLOPE = 0.2

MAX_FRAMES_WITHOUT_LANES = 10

# ===========================
# HSV Color Ranges
# ===========================

# Traffic light detection
RED_LOW1 = np.array([0, 100, 100])
RED_HIGH1 = np.array([10, 255, 255])
RED_LOW2 = np.array([160, 100, 100])
RED_HIGH2 = np.array([180, 255, 255])

YELLOW_LOW = np.array([20, 100, 100])
YELLOW_HIGH = np.array([30, 255, 255])

GREEN_LOW = np.array([40, 50, 50])
GREEN_HIGH = np.array([90, 255, 255])

# Red object detection (crosswalk/person)
LOWER_RED = np.array([0, 100, 100])
UPPER_RED = np.array([180, 255, 255])
