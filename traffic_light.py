import cv2
from config import (
    RED_LOW1,
    RED_HIGH1,
    RED_LOW2,
    RED_HIGH2,
    YELLOW_LOW,
    YELLOW_HIGH,
    GREEN_LOW,
    GREEN_HIGH,
)

TRAFFIC_NONE, TRAFFIC_RED, TRAFFIC_YELLOW, TRAFFIC_GREEN = 0, 1, 2, 3


def detect_traffic_light(frame):
    """Detect traffic light color in the upper third of the frame."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    height, _, _ = frame.shape
    roi = hsv[0 : int(height / 3), :]

    mask_red = cv2.bitwise_or(
        cv2.inRange(roi, RED_LOW1, RED_HIGH1),
        cv2.inRange(roi, RED_LOW2, RED_HIGH2),
    )
    mask_yellow = cv2.inRange(roi, YELLOW_LOW, YELLOW_HIGH)
    mask_green = cv2.inRange(roi, GREEN_LOW, GREEN_HIGH)

    red_pixels = cv2.countNonZero(mask_red)
    yellow_pixels = cv2.countNonZero(mask_yellow)
    green_pixels = cv2.countNonZero(mask_green)

    if red_pixels > 300:
        return TRAFFIC_RED
    elif yellow_pixels > 300:
        return TRAFFIC_YELLOW
    elif green_pixels > 300:
        return TRAFFIC_GREEN
    return TRAFFIC_NONE
