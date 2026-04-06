# 🎨🦾 Color-Based Pick & Place Robot Arm (OpenCV + PyMoveIt2 + ROS 2)

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-34a853?style=flat\&logo=ros)](https://docs.ros.org/en/jazzy/)
[![MoveIt 2](https://img.shields.io/badge/Motion_Planning-MoveIt2-blue?style=flat)](https://moveit.ros.org/)
[![OpenCV](https://img.shields.io/badge/Computer_Vision-OpenCV-white?style=flat\&logo=opencv)](#)
[![PyMoveIt2](https://img.shields.io/badge/API-PyMoveIt2-orange?style=flat)](#)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](#)

---

## 🚀 Overview

This project is an **intelligent robotic manipulation system** where a robot arm autonomously:

* 🎨 Detects **colored cubes (Red, Green, Blue)** using OpenCV
* 🧠 Computes object positions in real-time
* 🦾 Plans motion using **PyMoveIt2 (Python API for MoveIt 2)**
* 📦 Picks and places objects into a **designated drop zone (trash bin)**

It demonstrates a complete **vision → planning → manipulation pipeline** using ROS 2.

---

## 🤖 What This Robot Does

👉 The robot observes a table in front of it and:

1. Detects colored cubes using camera input
2. Identifies their position based on color segmentation
3. Moves towards the cube
4. Grasps it using the gripper
5. Moves to a predefined drop location
6. Releases the cube into a bin

✔ Fully autonomous loop
✔ Works for multiple colored objects
✔ Real-time perception + execution

---

## 🧠 System Architecture

```
Camera Feed
     ↓
OpenCV Color Detection
     ↓
Object Position Estimation
     ↓
PyMoveIt2 Motion Planning
     ↓
Robot Arm Execution
     ↓
Pick → Move → Drop
```

---

## ✨ Key Features

### 🎨 Color-Based Object Detection

* Uses OpenCV for:

  * HSV color segmentation
  * Contour detection
* Detects:

  * 🔴 Red cubes
  * 🟢 Green cubes
  * 🔵 Blue cubes

---

### 🦾 Motion Planning with PyMoveIt2

* Python interface for MoveIt 2
* Enables:

  * Easy scripting of robot motion
  * Fast prototyping without heavy C++
* Used for:

  * Pick and place execution
  * Gripper control
  * Pose-based movement

---

### 📦 Autonomous Pick & Place

* Fully automated pipeline:

  * Detect → Reach → Pick → Place
* Handles multiple objects sequentially

---

### 🧩 Modular ROS 2 Architecture

* Separate packages for:

  * Robot description
  * Controllers
  * Vision system
  * Motion planning

---

## 📁 Project Structure

```
panda_ws/
└── src/
    ├── hv_arm/
    │   ├── config/
    │   ├── launch/
    │   │   ├── launch_sim.launch.py
    │   │   └── rsp.launch.py
    │   ├── meshes/
    │   ├── models/
    │   ├── urdf/
    │   ├── worlds/
    │   └── src/
    │
    ├── hv_controller/
    │   ├── config/
    │   ├── launch/
    │   └── hv_controller/
    │       └── slider_controller.py
    │
    ├── moveit_config/
    │   ├── config/
    │   │   ├── joint_limits.yaml
    │   │   ├── kinematics.yaml
    │   │   ├── moveit_controllers.yaml
    │   │   └── panda.srdf
    │   ├── launch/
    │   │   └── moveit.launch.py
    │   └── rviz/
    │
    ├── panda_vision/
    │   └── panda_vision/
    │       └── color_detector.py
    │
    └── pymoveit2/
        ├── examples/
        │   └── pick_and_place.py   ⭐ (Main script used)
        └── pymoveit2/
```

---

## 🧠 How It Works

### 1️⃣ Vision (OpenCV)

* Captures camera feed
* Applies:

  * Color thresholding (HSV)
  * Contour extraction
* Outputs object position

---

### 2️⃣ Planning (PyMoveIt2)

* Converts detected position → robot pose
* Plans trajectory using MoveIt 2

---

### 3️⃣ Execution

* Robot:

  * Moves to object
  * Closes gripper
  * Moves to drop location
  * Opens gripper

---

## ⚙️ Tech Stack

### Robotics

* ROS 2 Jazzy
* MoveIt 2
* Gazebo Harmonic
* RViz2

### Vision

* OpenCV (Python)

### Motion API

* PyMoveIt2 (Python wrapper for MoveIt 2)

---

## 🚀 Getting Started

### 1️⃣ Install Dependencies

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-joint-state-publisher \
ros-jazzy-xacro python3-opencv
```

---

### 2️⃣ Build Workspace

```bash
cd ~/panda_ws
colcon build
source install/setup.bash
```

---

### 3️⃣ Launch Robot Simulation

```bash
ros2 launch hv_arm launch_sim.launch.py
```

---

### 4️⃣ Start MoveIt

```bash
ros2 launch moveit_config moveit.launch.py
```

---

### 5️⃣ Run Pick & Place Script

```bash
ros2 run pymoveit2 pick_and_place.py
```
<img width="1857" height="1170" alt="Screenshot from 2026-04-06 17-24-30" src="https://github.com/user-attachments/assets/e5abb130-ac05-4a3a-88cd-fc5a28dd6d03" />


<img width="1857" height="1170" alt="Screenshot from 2026-04-06 17-24-45" src="https://github.com/user-attachments/assets/2e46cb7e-08a0-468f-8552-6e92da956542" />


<img width="1857" height="1170" alt="Screenshot from 2026-04-06 17-25-30" src="https://github.com/user-attachments/assets/6acfbfe3-7cdd-41f8-837e-99974db86bb1" />


[Screencast from 2026-04-06 17-27-47.webm](https://github.com/user-attachments/assets/1943e85a-5090-454c-ab39-e48829055edd)

---

## 🎯 Highlights

* 🎨 Color-based intelligent manipulation
* 🦾 Simple yet powerful PyMoveIt2 integration
* ⚡ Fully autonomous pick-and-drop system
* 🧠 Clean perception → action pipeline
* 🔌 Beginner-friendly MoveIt 2 Python workflow

---

## 🔮 Future Improvements

* 📏 Depth estimation for better grasping
* 🦾 Real hardware deployment

---

## 👨‍💻 Author

**Harivathsha**
Robotics Engineering | Manipulation | AI Robotics

---

**"See color. Plan motion. Move with precision." 🎨🦾**
