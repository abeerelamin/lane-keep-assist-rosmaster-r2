import cv2
import numpy as np
from config import (
    CANNY_LOW,
    CANNY_HIGH,
    HOUGH_THRESHOLD,
    MIN_LINE_LENGTH,
    MAX_LINE_GAP,
    MIN_SLOPE,
    MAX_HORIZONTAL_SLOPE,
    LENGTH_RATIO_THRESHOLD,
    CURVE_STEERING_BOOST,
    CURVE_CONFIDENCE_THRESHOLD,
)


def fit_and_draw_curve(x_points, y_points, color, roi):
    """Fit a 2nd-degree curve to lane points and draw it on the ROI."""
    if len(x_points) < 3:
        if len(x_points) == 2:
            cv2.line(roi, (x_points[0], y_points[0]), (x_points[1], y_points[1]), color, 2)
        return None, 0, 0

    try:
        poly = np.polyfit(y_points, x_points, 2)
        curve_func = np.poly1d(poly)
        y_vals = np.linspace(min(y_points), max(y_points), 100)
        x_vals = curve_func(y_vals).astype(int)

        for i in range(len(y_vals) - 1):
            cv2.line(
                roi,
                (x_vals[i], int(y_vals[i])),
                (x_vals[i + 1], int(y_vals[i + 1])),
                color,
                2,
            )

        return curve_func, len(y_vals), abs(poly[0])
    except np.linalg.LinAlgError:
        print("Warning: Failed to fit curve")
        return None, 0, 0


def filter_horizontal_lines(x_points, y_points):
    """Filter out horizontal / near-horizontal lines that confuse lane detection."""
    if len(x_points) < 2:
        return x_points, y_points

    filtered_x, filtered_y = [], []
    for i in range(0, len(x_points), 2):
        if i + 1 < len(x_points):
            x1, y1 = x_points[i], y_points[i]
            x2, y2 = x_points[i + 1], y_points[i + 1]
            if x2 == x1:
                continue
            slope = abs((y2 - y1) / (x2 - x1))
            if slope > MAX_HORIZONTAL_SLOPE:
                filtered_x.extend([x1, x2])
                filtered_y.extend([y1, y2])

    return filtered_x, filtered_y


def process_lane_frame(roi, prev_lane_width, frames_without_lanes, curve_confidence):
    """
    Full lane detection pipeline on a ROI.

    Returns:
      lane_center, lanes_detected, horizontal_detected,
      frames_without_lanes, curve_confidence, in_curve,
      curve_boost, prev_lane_width, display_roi, gray,
      blurred, edges
    """
    display_roi = roi.copy()
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, CANNY_LOW, CANNY_HIGH)

    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180,
        threshold=HOUGH_THRESHOLD,
        minLineLength=MIN_LINE_LENGTH,
        maxLineGap=MAX_LINE_GAP,
    )

    left_x, left_y, right_x, right_y, horizontal_x, horizontal_y = [], [], [], [], [], []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) < MAX_HORIZONTAL_SLOPE:
                horizontal_x.extend([x1, x2])
                horizontal_y.extend([y1, y2])
            elif slope < -MIN_SLOPE:
                left_x.extend([x1, x2])
                left_y.extend([y1, y2])
            elif slope > MIN_SLOPE:
                right_x.extend([x1, x2])
                right_y.extend([y1, y2])

    left_x, left_y = filter_horizontal_lines(left_x, left_y)
    right_x, right_y = filter_horizontal_lines(right_x, right_y)

    left_curve, left_length, _ = fit_and_draw_curve(left_x, left_y, (0, 255, 0), display_roi)
    right_curve, right_length, _ = fit_and_draw_curve(right_x, right_y, (0, 255, 0), display_roi)

    if horizontal_x:
        fit_and_draw_curve(horizontal_x, horizontal_y, (0, 165, 255), display_roi)

    frame_width = roi.shape[1]
    frame_center = frame_width // 2
    lane_center = frame_center
    lanes_detected = False
    curve_detected = False
    curve_boost = 1.0
    horizontal_detected = len(horizontal_x) > 5

    left_detected = len(left_x) > 0
    right_detected = len(right_x) > 0

    if left_detected and right_detected:
        left_mean = np.mean(left_x)
        right_mean = np.mean(right_x)
        lane_center = int((left_mean + right_mean) / 2)
        lane_width = abs(right_mean - left_mean)
        prev_lane_width = 0.9 * prev_lane_width + 0.1 * lane_width
        lanes_detected = True
        frames_without_lanes = 0

        lane_ratio = (
            min(left_length, right_length) / max(left_length, right_length)
            if max(left_length, right_length) > 0
            else 1
        )
        if lane_ratio < LENGTH_RATIO_THRESHOLD and not horizontal_detected:
            curve_detected = True
            curve_boost = CURVE_STEERING_BOOST

    elif left_detected and not horizontal_detected:
        lane_center = int(np.mean(left_x) + prev_lane_width / 2)
        lanes_detected = True
        frames_without_lanes = 0
        curve_detected = True
        curve_boost = CURVE_STEERING_BOOST

    elif right_detected and not horizontal_detected:
        lane_center = int(np.mean(right_x) - prev_lane_width / 2)
        lanes_detected = True
        frames_without_lanes = 0
        curve_detected = True
        curve_boost = CURVE_STEERING_BOOST

    else:
        frames_without_lanes += 1

    # Update curve confidence and in_curve status
    if curve_detected:
        curve_confidence = min(curve_confidence + 1, CURVE_CONFIDENCE_THRESHOLD + 5)
    else:
        curve_confidence = max(curve_confidence - 1, 0)

    in_curve = curve_confidence >= CURVE_CONFIDENCE_THRESHOLD

    return (
        lane_center,
        lanes_detected,
        horizontal_detected,
        frames_without_lanes,
        curve_confidence,
        in_curve,
        curve_boost,
        prev_lane_width,
        display_roi,
        gray,
        blurred,
        edges,
    )
