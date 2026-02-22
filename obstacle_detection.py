import cv2
import numpy as np
import time
import ydlidar
from config import LOWER_RED, UPPER_RED

# Local LaserScan object
scan = ydlidar.LaserScan()


def check_obstacle_and_scan(laser, angle_min, angle_max, range_threshold):
    """Check for obstacles within a specified angle range and distance using LIDAR."""
    if not laser.doProcessSimple(scan):
        print("Failed to process scan.")
        return False

    for point in scan.points:
        angle_degrees = np.degrees(point.angle) % 360
        range_val = point.range
        if angle_min <= angle_degrees <= angle_max and 0 < range_val <= range_threshold:
            return True
    return False


def detect_obstacles_and_red_objects(frame, laser, bot, speed_drive_motor):
    """
    Combined function to detect close obstacles and large red objects.

    Returns:
      obstacle_or_red_detected (bool),
      frame (possibly annotated),
      updated speed_drive_motor (int)
    """
    # 1) Check for close objects (0.5m, 160°–200°)
    if check_obstacle_and_scan(laser, 160, 200, 0.5):
        print("🚗 Object very close! Stopping.")
        bot.set_car_motion(0, 0, 0)
        bot.set_motor(0, 0, 0, 0)

        # Check for large red objects
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_RED, UPPER_RED)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            area = w * h
            print(f"📸 Red contour detected, area: {area}")
            if area > 1000:  # Minimum area to ignore noise
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                return True, frame, speed_drive_motor

        print("🚨 No significant red object detected. Beeping.")
        try:
            for _ in range(5):
                bot.set_beep(1)
                time.sleep(0.2)
                bot.set_beep(0)
                time.sleep(0.2)
        except AttributeError:
            print("⚠️ Buzzer function not available.")
        return True, frame, speed_drive_motor

    # 2) If no close object, check for large red objects
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_RED, UPPER_RED)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        area = w * h
        print(f"📸 Red contour detected, area: {area}")
        if area > 1000:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            if area > 10000:
                print("🚶 Person or crossing line detected.")
                if check_obstacle_and_scan(laser, 160, 200, 0.4):
                    print("🛑 Obstacle within 0.4m. Stopping.")
                    speed_drive_motor = 0
                else:
                    speed_drive_motor = 30
            else:
                speed_drive_motor = 30

            bot.set_motor(speed_drive_motor, speed_drive_motor, speed_drive_motor, speed_drive_motor)
            return True, frame, speed_drive_motor

    print("🚨 No red object detected. Stopping.")
    bot.set_motor(0, 0, 0, 0)
    speed_drive_motor = 0
    return False, frame, speed_drive_motor
