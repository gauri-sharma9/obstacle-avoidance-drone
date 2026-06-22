# obstacle-avoidance-drone
# Autonomous Drone Obstacle Avoidance System

## Overview

This project aims to develop a modular obstacle avoidance framework for autonomous drones using simulated sensors and ArduPilot.

The system will support:

- Camera
- LiDAR
- Radar

and will be designed to remain sensor-agnostic so that future sensors can be integrated without changing the planning algorithm.

---

## Software Stack

- Ubuntu 22.04
- ROS2 Humble
- Gazebo
- ArduPilot SITL
- QGroundControl
- Python
- OpenCV
- YOLOv8

---

## System Pipeline

Sensors
(Camera, LiDAR, Radar)

↓

Sensor Processing

↓

Sensor Fusion

↓

Obstacle Map

↓

Risk Assessment

↓

Path Planning

↓

ArduPilot

↓

Drone Control

---

## Project Goal

Detect obstacles, assess collision risk, generate safe trajectories and avoid collisions while reaching the destination safely.
