# Autonomous Drone Obstacle Avoidance System (3-Sensor Prototype)

A modular, multi-modal obstacle detection and avoidance framework for autonomous drones using **Camera**, **LiDAR**, and **Radar** fused inputs integrated with **ROS 2 Humble**, **Ignition Gazebo**, and **ArduPilot SITL (BendyRuler OA)**.

---

## 🏗️ Architecture

```
[ Gazebo Simulation ]
   ├── GPU LiDAR (/lidar)      ──► lidar_sector_node.py ──┐ (72 sectors, cm)
   ├── Monocular Cam (/camera)  ──► camera_detector_node.py─┼─► fusion_node.py ──► mavlink_obstacle_node.py ──► ArduPilot SITL
   └── World Obstacle Map      ──► radar_node.py (MAVLink) ┘   (72 sectors, cm)        (OBSTACLE_DISTANCE)         (BendyRuler OA)
                                                                       │
                                                                       ▼
                                                              avoidance_node.py ──► Log Output / User Feedback
                                                                       ▲
                                                              mission_commander_node.py ──► Takeoff & Autonomous Navigation
```

---

## ⚡ Quick Start

### 1. Build the ROS 2 Workspace
Navigating to `ros2_ws`, building, and returning to the project root:
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
cd ..
```

### 2. Launch Full Simulation & Flight Pipeline
Now run the launch script from the project root:
```bash
./run_sim.sh
```

This automatically:
1. Starts **ArduPilot SITL** (Iris quadcopter with parameters `PRX1_TYPE=2`, `OA_TYPE=1`).
2. Launches **Ignition Gazebo** (`demo_world.sdf` with obstacle corridor).
3. Connects `ros_gz_bridge` and starts all 7 ROS 2 nodes (`lidar_sector_node`, `camera_detector_node`, `radar_node`, `fusion_node`, `avoidance_node`, `mavlink_obstacle_node`, `mission_commander_node`).
4. Arms the drone, takes off to 3m, and flies `GOTO (15.0, 0.0, 3.0)` through the obstacle field.

---

## 📊 Live Monitoring Commands

While the simulation is running, open a new terminal (with ROS 2 sourced) to monitor topics:

- **Fused Obstacle Distances (72 sectors)**:
  ```bash
  ros2 topic echo /fused/obstacle_sectors_cm
  ```
- **Avoidance Logger Commands**:
  ```bash
  ros2 topic echo /avoidance/command
  ```
- **Camera Detection Status**:
  ```bash
  ros2 topic echo /camera/detection_status
  ```
- **Fusion Summary**:
  ```bash
  ros2 topic echo /fusion/obstacle_summary
  ```

---

## 🛠️ Modifying & Customizing Nodes

All ROS 2 nodes are located in `ros2_ws/src/obstacle_avoidance/obstacle_avoidance/`:
- `lidar_sector_node.py`: Converts 3D PointCloud2 data into 72 horizontal distance sectors ($5^\circ$ resolution).
- `radar_node.py`: Simulates RF radar detection with noise, beam spreading, and live MAVLink position tracking.
- `camera_detector_node.py`: Monocular HSV color detection + central ROI edge density threat analysis.
- `fusion_node.py`: Multi-modal min-distance fusion with camera bearing confidence scaling ($0.85$).
- `mavlink_obstacle_node.py`: Streams `OBSTACLE_DISTANCE` (msg 330) to ArduPilot via MAVLink UDP.
- `mission_commander_node.py`: Autonomous MAVLink takeoff and waypoint flight controller.
