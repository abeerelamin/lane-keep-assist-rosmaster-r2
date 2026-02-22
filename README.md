# 🚗 Lane Keep Assist System  
## Obstacle Avoidance & Traffic Light Detection  

An integrated **Lane Keep Assist (LKA)** system implemented on the **Rosmaster R2** autonomous robot.  
The system combines computer vision, LiDAR sensing, and PD control to enable real-time lane following, traffic light detection, and obstacle avoidance.

---

## 📌 Project Overview

This project implements a real-time autonomous navigation system capable of:

- 🚘 Lane following using camera input  
- 🚦 Traffic light detection (Red / Yellow / Green)  
- 🛑 Red crosswalk / pedestrian simulation detection  
- 📡 Obstacle detection using YDLIDAR  
- ⚙️ PD-based steering and adaptive speed control  
- 🧠 Integrated safety prioritization logic  

The system was designed and tested in a controlled indoor environment.

---

## 👩‍💻 My Contribution

I was responsible for:

- Implementing lane detection using OpenCV:
  - ROI selection  
  - Canny edge detection  
  - Hough Transform  
- Designing and tuning the PD controller  
- Integrating lane following with:
  - Obstacle detection  
  - Traffic light logic  
- Full system testing, tuning, and debugging  

---

## 🏗 System Architecture

The system is built using **ROS** with modular nodes:

- `Lane Detection Node`
- `PD Controller Node`
- `Traffic Light Detection Node`
- `Red Object Detection Node`
- `Obstacle Avoidance Node`

### Safety Priority Order

1. 🚨 Obstacle Detection (Highest Priority)  
2. 🚦 Traffic Light Detection  
3. 🚘 Lane Following  

---

## 🔍 Lane Detection Pipeline

1. Frame capture from camera  
2. Region of Interest (ROI) cropping  
3. Grayscale conversion  
4. Gaussian blur  
5. Canny edge detection  
6. Hough Transform line detection  
7. Lane center estimation  
8. Error calculation  

### Error Formula

```
error = (lane_center - desired_center) / frame_width
```

---

## 🎯 PD Controller

Steering control:

```
steering = Kp * error + Kd * (error - prev_error) / dt
```

### Features

- Curve detection boost  
- Speed reduction in curves  
- Steering clipping  
- Smooth oscillation control  

---

## 🚦 Traffic Light Detection

- HSV color segmentation  
- Dual-range red detection  
- Brightness filtering  
- Contour area thresholding  
- 60-second red light timeout logic  

### Behavior

- 🔴 Red → Full stop  
- 🟡 Yellow → Reduced speed  
- 🟢 Green → Normal speed  

---

## 📡 Obstacle Detection (YDLIDAR)

- 2D LiDAR scanning  
- 160°–200° front sector monitoring  
- < 0.5m emergency stop threshold  
- Crosswalk detection via red object segmentation  

---

## 🛡 Safety Features

- Emergency stop for close obstacles  
- Lane loss timeout handling  
- Red light timeout recovery  
- Exception handling and clean shutdown  
- Real-time debugging visualization  

---

## 🧪 Performance Highlights

- Average lane error (straight path): **-0.05**  
- Red light stop reaction: **< 0.5s**  
- Obstacle stop reaction: **< 0.3s**  
- CNN traffic sign prototype accuracy (PC testing): **90%**  

---

## ⚠️ Limitations

- Designed for indoor / controlled environments  
- Sensitive to lighting changes (HSV-based detection)  
- CNN model not deployed due to Jetson Nano constraints  
- Sharp curves remain challenging  

---

## 🔮 Future Improvements

- Adaptive HSV thresholding  
- Lightweight ML model deployment  
- ROS node synchronization optimization  
- Model quantization for embedded systems  

---

## 🛠 Technologies Used

- Python  
- OpenCV  
- ROS  
- NumPy  
- YDLIDAR SDK  
- Rosmaster SDK  
- Jetson Nano  

---

## 📷 Demo

🎥 Demo Video: [Add Link Here]

---

## 📂 Project Structure

```
.
├── lane_detection/
├── traffic_light/
├── obstacle_detection/
├── main.py
├── requirements.txt
```

---

## 🚀 Installation (Example)

```bash
# Clone repository
git clone https://github.com/yourusername/Lane-Keep-Assist-Rosmaster.git

# Install dependencies
pip install -r requirements.txt

# Launch ROS
roslaunch your_package main.launch
```

