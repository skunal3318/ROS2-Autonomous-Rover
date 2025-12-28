<p align="center">
  <img src="https://raw.githubusercontent.com/ros-infrastructure/artwork/master/ros_logo.svg" alt="ROS 2 Logo" width="450">
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Jazzy-blue" alt="ROS2 Jazzy">
  <img src="https://img.shields.io/badge/Simulation-Gazebo-green" alt="Gazebo">
  <img src="https://img.shields.io/badge/Status-Active-success" alt="Project Status">
</p>

*ROS2 Autonomous Rover* is a **simulation-first autonomous mobile robot** built using **ROS 2 (Jazzy)** and **Gazebo (gz-sim)**.  
The project focuses on **core autonomy concepts** such as perception, decision-making, and control, implemented through a clean ROS2 architecture and a finite state machine (FSM).

---

## 🌟 Why This Project?

This project was built to:
- Understand **end-to-end autonomous robot architecture**
- Practice **ROS2 node-based design**
- Implement **FSM-driven obstacle avoidance**
- Bridge the gap between **simulation and real-world robotics**

The rover is designed to move autonomously in a simulated environment while reacting to obstacles using LiDAR-based perception.

---

## 🚗 Rover Capabilities

- 🧭 **Autonomous Forward Navigation**
- 🚧 **LiDAR-Based Obstacle Detection**
- 🔄 **FSM-Controlled Obstacle Avoidance**
  - Forward → Stop → Reverse → Scan → Turn → Forward
- 🎥 **Camera Integration** (for perception & visualization)
- 🕹️ **Manual Teleoperation Support**
- 🔌 **ROS2 Topic-Based Control (`cmd_vel`)**
- 🧪 **Fully Simulated in Gazebo (gz-sim)**

---

## 🧠 System Architecture

The rover follows a modular ROS2 design:

- **Gazebo Simulation**
  - Robot model (URDF/Xacro)
  - Sensors (LiDAR, Camera)
- **ROS2 Nodes**
  - Sensor processing
  - FSM-based control logic
  - Velocity command publisher
- **Visualization**
  - RViz2
  - `rqt_graph`

This separation ensures clarity, scalability, and easy transition to real hardware.

---

## 🔄 Finite State Machine (FSM)

The obstacle avoidance behavior is driven by a simple but effective FSM:

1. **FORWARD** – Move straight
2. **STOP** – Brief halt on obstacle detection
3. **REVERSE** – Move backward to create space
4. **SCAN** – Compare left vs right clearance
5. **TURN_LEFT / TURN_RIGHT** – Rotate toward safer direction
6. **FORWARD** – Resume motion

This approach avoids black-box planners and emphasizes **decision-making transparency**.

---

## 🛠️ Tech Stack

- **ROS 2 Jazzy**
- **Gazebo (gz-sim)**
- **Python (rclpy)**
- **URDF / Xacro**
- **RViz2**
- **rqt_graph**

---

## ▶️ How to Run

```bash
# Build the workspace
colcon build
source install/setup.bash

# Launch the simulation
ros2 launch rover_bringup rover.launch.xml

# Run obstacle avoidance node
ros2 run rover_control obstacle_avoidance.py
