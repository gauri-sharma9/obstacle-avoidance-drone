# System Architecture

## Sensor Layer

Inputs:

- Camera
- LiDAR
- Radar

Responsibilities:

- Environment sensing
- Object detection
- Distance estimation

---

## Sensor Fusion Layer

Convert all sensor outputs into a common obstacle representation.

Obstacle:

- Position
- Velocity
- Confidence

This allows future sensors to be added without changing the planner.

---

## Risk Assessment Layer

Calculate:

- Distance
- Relative velocity
- Time To Collision (TTC)

Generate risk score.

---

## Path Planning Layer

Generate collision-free trajectory.

Approaches:

- Simple Avoidance
- BendyRuler
- Dijkstra

---

## Control Layer

Send navigation commands to ArduPilot.
