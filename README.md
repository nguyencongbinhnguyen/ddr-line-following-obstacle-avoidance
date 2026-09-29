# Differential-Drive Robot for Line Following and Obstacle Avoidance

This repository contains the source code developed for a differential-drive mobile robot capable of line following and obstacle avoidance.

The system is implemented using a hierarchical architecture with two main processing levels:

- **High-level processing** on NVIDIA Jetson Nano using ROS, camera, and LiDAR.
- **Low-level control** on Arduino Mega 2560 for real-time motion control, sensor feedback processing, and motor actuation.

---

## System Overview

The robot uses:

- Kinect camera for line detection and extraction of geometric tracking information.
- YDLIDAR for obstacle detection and environment sensing.
- NVIDIA Jetson Nano for high-level perception and behavior coordination.
- Arduino Mega 2560 for low-level control.
- Wheel encoders for linear velocity estimation.
- MPU6050 IMU for angular velocity measurement.
- Two DC motors in a differential-drive configuration.

The robot operates in two primary tasks:

1. Line following.
2. Obstacle avoidance and rejoining the original line.

During obstacle avoidance, LiDAR data is used to generate a virtual reference line around the obstacle so that the control structure can continue to operate using a unified line-following formulation.

---

## Repository Structure

```text
ddr-line-following-obstacle-avoidance/
├── arduino/
│   ├── RobotTypes.h
│   └── robot_BN.ino
│
├── jetson/
│   ├── node_lidar_driver/
│   ├── node_line_detection/
│   ├── node_main_controller/
│   └── CMakeLists.txt
│
└── README.md
