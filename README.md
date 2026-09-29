# Vision-Guided-Human-Following-Mobile-Robot
This project implements a human-following mobile robot using a decoupled perception–control architecture. Computationally intensive human detection is performed on an external laptop using the YOLOv8n deep learning model, while the Arduino Mega 2560 performs real-time motor control and safety operations.
# Vision-Guided Human-Following Mobile Robot

## Overview

This project presents a vision-guided mobile robot capable of detecting and following a human using **YOLOv8n** for object detection and an **Arduino Mega 2560** for real-time embedded control.

The system uses a decoupled architecture in which computationally intensive vision processing is performed on an external laptop, while the Arduino handles motor control, obstacle avoidance, safety logic, and manual override.

## Key Features

- Real-time human detection using **YOLOv8n**
- Human tracking using bounding-box centroid calculation
- Arduino Mega 2560 based motion control
- Differential-drive robot locomotion
- Obstacle detection using **HC-SR04 ultrasonic sensor**
- Bluetooth-based manual override using **HC-05**
- USB serial communication between laptop and Arduino
- Finite-state-machine based safety and motion control

## System Architecture

```text
Mobile Phone Camera
        ↓
   Laptop / PC
        ↓
      YOLOv8n
        ↓
Human Detection
        ↓
Centroid Calculation
        ↓
Motion Command
   L / F / R / S
        ↓
 USB Serial Communication
        ↓
 Arduino Mega 2560
        ↓
Safety & Motion Control
        ↓
 Relay Modules
        ↓
 DC Motors
        ↓
   Robot Movement
