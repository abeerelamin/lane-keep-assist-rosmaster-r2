import cv2
import numpy as np
import time
import signal
import sys
import ydlidar

from Rosmaster_Lib import Rosmaster

from config import (
    ROI_TOP,
    ROI_BOTTOM,
    ROI_LEFT,
    ROI_RIGHT,
    Kp,
    Kd,
    MIN_STEERING,
    MAX_STEERING,
    CURVE_MAX_STEERING,
    BASE_SPEED,
    MIN_SPEED,
    CURVE_SPEED,
    MAX_ERROR,
    MAX_FRAMES_WITHOUT_LANES,
)

from lane_detection import process_lane_frame
from traffic_light import (
    detect_traffic_light,
    TRAFFIC_RED,
    TRAFFIC_YELLOW,
)
from obstacle_detection import detect_obstacles_and_red_objects

bot = None
cap = None
laser = None


def cleanup(sig=None, frame=None):
    """Stop robot, turn off LIDAR, and release resources."""
    print("Stopping robot and releasing resources...")
    global bot, cap, laser
    try:
        if bot is not None:
            bot.set_car_motion(0, 0, 0)
            bot.set_motor(0, 0, 0, 0)
    except Exception:
        pass

    try:
        if laser is not None:
            laser.turnOff()
            laser.disconnecting()
    except Exception:
        pass

    try:
        if cap is not None:
            cap.release()
    except Exception:
        pass

    cv2.destroyAllWindows()
    sys.exit(0)


def init_hardware():
    global bot, cap, laser

    # Robot
    bot = Rosmaster()
    bot.create_receive_threading()

    # Camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        sys.exit(1)

    # LIDAR
    ports = ydlidar.lidarPortList()
    port = "/dev/ydlidar"
    for _, value in ports.items():
        port = value

    laser = ydlidar.CYdLidar()
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 512000)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 20)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 32.0)
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.01)

    if not laser.initialize():
        print("Failed to initialize LIDAR.")
        sys.exit(1)
    if not laser.turnOn():
        print("Failed to turn on LIDAR.")
        sys.exit(1)


def main_loop():
    global bot, cap, laser

    prev_error = 0.0
    prev_time = time.time()
    prev_lane_width = 200
    frames_without_lanes = 0
    curve_confidence = 0
    last_steering = 0.0
    last_steering_time = time.time()
    speed_drive_motor = 30  # For red object detection

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to grab frame")
                break

            placeholder = np.zeros((ROI_BOTTOM - ROI_TOP, ROI_RIGHT - ROI_LEFT), dtype=np.uint8)
            display_roi = frame[ROI_TOP:ROI_BOTTOM, ROI_LEFT:ROI_RIGHT].copy()

            # 1) Obstacle & red object detection
            obstacle_or_red, frame, speed_drive_motor = detect_obstacles_and_red_objects(
                frame, laser, bot, speed_drive_motor
            )

            if obstacle_or_red and speed_drive_motor == 0:
                cv2.imshow("Lane Following (PD)", display_roi)
                cv2.imshow("Red Detection", frame)
                cv2.imshow("gray", placeholder)
                cv2.imshow("blurred", placeholder)
                cv2.imshow("edges", placeholder)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            # 2) Traffic light detection
            traffic_state = detect_traffic_light(frame)
            if traffic_state == TRAFFIC_RED:
                print("Red light detected. Stopping.")
                bot.set_car_motion(0, 0, 0)
                cv2.imshow("Lane Following (PD)", display_roi)
                cv2.imshow("Red Detection", frame)
                cv2.imshow("gray", placeholder)
                cv2.imshow("blurred", placeholder)
                cv2.imshow("edges", placeholder)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            speed = MIN_SPEED if traffic_state == TRAFFIC_YELLOW else BASE_SPEED

            # 3) Lane detection
            roi = frame[ROI_TOP:ROI_BOTTOM, ROI_LEFT:ROI_RIGHT]
            (
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
            ) = process_lane_frame(
                roi,
                prev_lane_width,
                frames_without_lanes,
                curve_confidence,
            )

            frame_width = display_roi.shape[1]
            frame_center = frame_width // 2

            if frames_without_lanes >= MAX_FRAMES_WITHOUT_LANES:
                print("Warning: Lost lane detection for too many frames. Stopping.")
                bot.set_car_motion(0, 0, 0)
                cv2.imshow("Lane Following (PD)", display_roi)
                cv2.imshow("Red Detection", frame)
                cv2.imshow("gray", gray)
                cv2.imshow("blurred", blurred)
                cv2.imshow("edges", edges)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            # 4) PD control for steering
            error = (lane_center - frame_center) / frame_width
            current_time = time.time()
            dt = current_time - prev_time if current_time != prev_time else 0.01
            derivative = (error - prev_error) / dt

            if in_curve:
                raw_steering = Kp * error * curve_boost + Kd * derivative
                raw_steering = max(-CURVE_MAX_STEERING, min(CURVE_MAX_STEERING, raw_steering))
            else:
                raw_steering = Kp * error + Kd * derivative
                raw_steering = max(MIN_STEERING, min(MAX_STEERING, raw_steering))

            steering_dt = current_time - last_steering_time
            smoothing_factor = min(1.0, steering_dt * 5.0)
            steering = smoothing_factor * raw_steering + (1 - smoothing_factor) * last_steering

            last_steering = steering
            last_steering_time = current_time

            error_magnitude = abs(error)
            if in_curve:
                speed = CURVE_SPEED
            elif error_magnitude > MAX_ERROR / 2:
                speed = max(MIN_SPEED, BASE_SPEED * (1 - error_magnitude / MAX_ERROR))

            bot.set_car_motion(speed, -steering, 0)

            prev_error = error
            prev_time = current_time

            height = ROI_BOTTOM - ROI_TOP
            cv2.line(display_roi, (lane_center, 0), (lane_center, height), (255, 0, 0), 2)
            cv2.line(display_roi, (frame_center, 0), (frame_center, height), (0, 0, 255), 2)

            cv2.putText(
                display_roi,
                f"Error: {error:.2f}",
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )
            cv2.putText(
                display_roi,
                f"Steering: {steering:.2f}",
                (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )
            cv2.putText(
                display_roi,
                f"Speed: {speed:.2f}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

            curve_status = f"In Curve ({curve_confidence})" if in_curve else f"Straight ({curve_confidence})"
            curve_color = (0, 165, 255) if in_curve else (255, 255, 255)
            cv2.putText(
                display_roi,
                f"Path: {curve_status}",
                (10, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                curve_color,
                1,
            )

            status_color = (0, 255, 0) if lanes_detected else (0, 0, 255)
            status_text = "Lanes: Detected" if lanes_detected else f"Lanes: Lost ({frames_without_lanes})"
            cv2.putText(
                display_roi,
                status_text,
                (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                status_color,
                1,
            )

            cv2.putText(
                display_roi,
                f"Horizontal: {'Detected' if horizontal_detected else 'None'}",
                (10, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

            cv2.imshow("Lane Following (PD)", display_roi)
            cv2.imshow("Red Detection", frame)
            cv2.imshow("gray", gray)
            cv2.imshow("blurred", blurred)
            cv2.imshow("edges", edges)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        cleanup()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, cleanup)
    init_hardware()
    main_loop()
