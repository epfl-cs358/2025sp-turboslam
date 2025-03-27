# CS-358 - Making Intelligent Things 2025 Spring
## Project: TurboSLAM - Fast Autonomous Car using Simultaneous Localization and Mapping

## Contributors
- Vincent Palma
- Loris Baldisserotto
- Ivan Dylevskiy
- Ilian Changkakoti
- Sarah Lim

[Google Doc](https://docs.google.com/document/d/1Hz-CrI_mGBuuY4Cr6uao3CeAetyIk8hIU1p6bzpezqY/edit?usp=sharing)

[Project Proposal](https://www.overleaf.com/read/mtyzjbnkwfxt#540f41)

## Project Overview
This project aims to develop an autonomous system for Simultaneous Localization and Mapping (SLAM) using a 2D LiDAR sensor mounted on a servo motor. The system is designed to map an environment and navigate to given coordinates while avoiding obstacles. The project is based on modifying a **Tamiya Blitzer Beetle** RC car, replacing its original control system with an ESP32 microcontroller.

## Table of Contents
- [Description](#description)
- [User Stories](#user-stories)
- [Software](#software)
- [Mechanical Build](#mechanical-build)
- [Challenges & Solutions](#challenges--solutions)
- [Component List](#component-list)
- [Milestones & Organization](#milestones--organization)
- [Evaluation Metrics](#evaluation-metrics)

## Description
The system consists of two main phases:
1. **Mapping Phase:** The robot explores the environment while the LiDAR sensor scans at multiple angles using a servo motor, creating a 3D map from 2D slices.
2. **Navigation Phase:** The system autonomously moves towards a user-specified target, avoiding dynamic obstacles in real-time.

The **ESP32** microcontroller handles remote communication and data processing, making the project a cost-effective solution for indoor and outdoor navigation applications.

## User Stories
### 1. Assisting Visually Impaired Users in Indoor Navigation
**As a** visually impaired person,  
**I want** a robot that maps my surroundings and detects obstacles,  
**So that** I can navigate safely without colliding or falling.

### 2. Helping People with Mobility Issues
**As a** wheelchair user,  
**I want** a system that detects floor hazards like stairs,  
**So that** I can navigate safely.

### 3. Obstacle Avoidance for Robotic Vacuums
**As a** homeowner,  
**I want** my robotic vacuum to avoid furniture and stairs,  
**So that** it can clean efficiently without getting stuck.

### 4. Indoor Navigation for Large Buildings
**As a** visitor in a shopping mall,  
**I want** a 3D-mapping system to guide me,  
**So that** I can reach my destination without external help.

### 5. Search and Rescue in Disaster Areas
**As a** rescue worker,  
**I want** a robotic system that maps collapsed buildings,  
**So that** I can locate survivors safely.

## Software
- SLAM Algorithm Implementation
- Real-time WiFi Communication via ESP32
- Path Planning & Obstacle Avoidance
- Data Processing & Visualization

## Mechanical Build
The original car chassis was unsuitable for mounting sensors, so we built a three-layer structure:
1. **Base Layer:** Chassis
2. **Middle Layer:** Microcontroller & Electronics Mount
3. **Top Layer:** LiDAR Mount (Servo-tilted 2D LiDAR for 3D mapping)

![Chassis](IMG_3694.jpeg)  
![Microcontroller Support](IMG_3695.jpeg)  
![LiDAR Mount](IMG_3696.jpeg)

## Challenges & Solutions
### 1. **Building a Custom Chassis**
**Issue:** The original car body was impractical for mounting components.
**Solution:** We modified a **Tamiya Blitzer Beetle**, which offers an accessible and stable platform.

### 2. **Simulating a 3D LiDAR with a 2D Sensor**
**Issue:** 3D LiDARs are expensive and out of budget.
**Solution:** We mounted a **2D LiDAR on a servo motor**, allowing it to scan at different angles to simulate a 3D scan.

### 3. **Detecting Unexpected Obstacles**
**Issue:** Mapping alone does not detect obstacles appearing after the initial scan.
**Solution:** We implemented a **front-mounted laser sensor** for real-time obstacle detection.

### 4. **Remote Communication & Control**
**Issue:** The system lacked a remote control and radar detector.
**Solution:** We used an **ESP32 microcontroller** for **WiFi-based remote communication**.

## Component List
| Category | Quantity | Cost (CHF) |
|----------|------|----------|------------|
| Tamiya Blitzer Beetle | 1 | 129 |
| LiDAR 360 | 1 | 79.90 |
| 7.2V Battery | 1 | 23.90 (+ 5.0) |
| Accelerometer/Magnetometer 9DoF IMU | 1 | 19.90 |
| Servo | 1 | 21.0 |
| HC-SR04 | 1 | 2.0 |
| ESP32 | 1 | 4.0 |
| DMS15 Servo | 1 | 5.0 |
| Encoder | 1 | 3.90 |

## Milestones & Organization
Project board: [Monday.com Board](https://limsarah112s-team.monday.com/boards/1877147787)

## Evaluation Metrics
### 1. **Navigation Precision & Path Efficiency**
- **Metric:** Deviation from planned path (cm/m)
- **Measurement:** Compare actual vs. planned path using positional tracking

### 2. **WiFi Communication Latency**
- **Metric:** Transmission delay (ms)
- **Measurement:** Timestamp logs for command transmission and execution

### 3. **Battery Life & Power Efficiency**
- **Metric:** Continuous runtime before recharge
- **Measurement:** Track time in mapping and navigation modes

## Related Projects
- [Laser Scan to Point Cloud](https://github.com/carlosmccosta/laserscan_to_pointcloud?tab=readme-ov-file)
- [ESP32 with ESC Controller](https://github.com/Tales-sv/Esp32-ESC_Controler)
- [LiDAR-Based SLAM](https://github.com/w111liang222/lidar-slam-detection)
- [SLAM Using 2D LiDAR](https://github.com/meyiao/LaserSLAM)





---
**MIT - Spring 2025**

