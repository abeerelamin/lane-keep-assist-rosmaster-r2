# Lane Keep Assist System with Obstacle Avoidance and Traffic Light Detection

This project implements a lane keep assist (LKA) system for a small-scale autonomous robot (Rosmaster R2).  
The robot is capable of following lanes, detecting traffic light states, stopping at red crossings, and avoiding obstacles using camera and LiDAR sensors.

## Project Context
- Course: Autonomous Systems (CENG340)
- Platform: Rosmaster R2 (Ackermann steering)
- Type: Team project (5 members)

## My Contribution
I was responsible for:
- Lane detection using OpenCV (ROI selection, edge detection, Hough transform)
- PD controller design for steering and speed adjustment
- Integration of lane following with obstacle detection and traffic light logic
- Testing, tuning, and debugging the full system

## System Features
- Real-time lane detection using camera input
- PD-based steering control with curve handling
- Traffic light detection (red, yellow, green) using HSV color segmentation
- Red object detection for crosswalk/pedestrian simulation
- Obstacle detection using YDLIDAR
- Safety mechanisms (timeouts, emergency stop, loss-of-lane handling)

## Technologies Used
- Python
- OpenCV
- ROS
- NumPy
- YDLIDAR SDK
- Rosmaster SDK

## How It Works
1. Camera frames are processed to detect lane boundaries and compute lane center deviation.
2. A PD controller adjusts steering and speed to keep the robot centered.
3. LiDAR data is continuously scanned for nearby obstacles.
4. Traffic light and red object detection override lane following when stopping is required.
5. All modules are integrated into a single control loop with safety prioritization.

## Limitations
- Designed for indoor or controlled environments
- Sensitive to lighting variations due to HSV-based detection
- Machine learning-based traffic sign recognition was explored but not deployed due to hardware constraints

## Future Improvements
- Adaptive color thresholding for varying lighting conditions
- Optimized ROS node synchronization
- Deployment of lightweight ML models on embedded hardware

