<p align="center">
  <img src="https://raw.githubusercontent.com/ros-infrastructure/artwork/master/ros_logo.svg" alt="ROS 2 Logo" width="450">
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Jazzy-blue" alt="ROS2 Jazzy">
  <img src="https://img.shields.io/badge/Simulation-Gazebo%20Sim-green" alt="Gazebo Sim">
  <img src="https://img.shields.io/badge/Perception-Camera--Based-orange" alt="Camera Based Perception">
  <img src="https://img.shields.io/badge/Status-Active-success" alt="Project Status">
</p>

# ROS 2 Camera-Based Autonomous Rover

**ROS2 Camera-Based Autonomous Rover** is a **simulation-first mobile robotics project** built using **ROS 2 (Jazzy)** and **Gazebo Sim (gz-sim)**.  
The project focuses on **vision-based perception**, **obstacle reasoning**, and **clean ROS 2 architecture**, with an emphasis on **explainable autonomy** rather than black-box navigation.

This repository is structured to scale from **pure visualization → perception → decision-making → control**.

---

## 🌟 Why This Project?

This project was built to:

- Understand **camera-based perception pipelines**
- Practice **clean ROS 2 package separation**
- Develop **obstacle avoidance without LiDAR**
- Visualize **what the robot “sees” and “decides”**
- Build autonomy incrementally instead of end-to-end black boxes

The rover operates in a simulated environment and detects obstacles **purely from RGB camera input**, making it suitable for low-cost real-world robots.

---

## 🚗 Rover Capabilities (Current)

- 🎥 **RGB Camera Integration**
- 👁️ **Camera-Based Obstacle Detection**
- 📐 **Region-of-Interest (ROI) Processing**
- 🧱 **Edge-Based Obstacle Reasoning**
- 🧠 **Left / Center / Right Obstacle Classification**
- 🖼️ **Real-Time Perception Visualization**
- 🧪 **Fully Simulated in Gazebo Sim**

> 🚧 Motion control and FSM-based navigation are intentionally **not enabled yet** to maintain perception clarity.

---

## 🧠 Perception Pipeline

The obstacle avoidance logic follows a transparent vision pipeline:

1. **RGB Image Acquisition**
2. **ROI Cropping (Front View)**
3. **Grayscale Conversion**
4. **Gaussian Blur**
5. **Canny Edge Detection**
6. **Spatial Analysis**
   - Left / Center / Right regions
7. **Obstacle Decision Output**
   - `LEFT`, `CENTER`, `RIGHT`, or `NONE`

This approach prioritizes **interpretability and debugging visibility**.

---

## 🧩 System Architecture

The project uses a **modular ROS 2 design**:

### 🧱 Simulation
- Gazebo Sim (gz-sim)
- Custom rover URDF/Xacro
- RGB camera sensor

### 🧠 ROS 2 Packages
- **four_wheel_description**
  - Robot model
  - Sensors
  - Gazebo integration
- **four_control**
  - Camera visualization
  - Obstacle perception logic
  - Decision reasoning (no control yet)

### 👀 Visualization
- OpenCV windows (camera + ROI edges)
- Gazebo Sim GUI
- RViz2 (optional)

This separation ensures:
- Easy debugging
- Clear responsibility boundaries
- Smooth transition to real hardware later

---

## 🧪 Obstacle Avoidance Logic (Camera-Based)

Obstacle detection is performed using **image structure**, not distance sensors.

Decision logic:

- **CENTER obstacle** → highest priority
- **LEFT obstacle** → free space likely on right
- **RIGHT obstacle** → free space likely on left
- **NONE** → clear path

The result is visualized directly on the processed image to ensure **decision correctness before control is enabled**.

---

## 🛠️ Tech Stack

- **ROS 2 Jazzy**
- **Gazebo Sim (gz-sim)**
- **Python (rclpy)**
- **OpenCV**
- **cv_bridge**
- **URDF / Xacro**
- **RViz2**

---

## ▶️ How to Run

### Build the workspace
```bash
colcon build
source install/setup.bash

```

### Launch the rover in Gazebo
```bash
ros2 launch four_wheel_description rover.launch.py
```

### Run camera-based obstacle visualization
```bash
ros2 launch four_control visualization.launch.py
```

