```markdown
![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue)
![Nav2](https://img.shields.io/badge/Nav2-Navigation-green)
![micro-ROS](https://img.shields.io/badge/micro--ROS-ESP32-orange)
![License](https://img.shields.io/badge/License-Apache--2.0-lightgrey)

# GPS Autonomous Navigation (ROS 2)

A ROS 2–based autonomous navigation system that allows a mobile robot to navigate
to **GPS-defined goals** using **LiDAR-based SLAM**, **Nav2**, and **micro-ROS motor control**.

---

## Overview

This project implements a complete GPS-based autonomous navigation pipeline in **ROS 2 Humble**.
GPS latitude and longitude inputs are converted into local map-frame coordinates and sent
to the **Nav2 navigation stack**, enabling autonomous path planning and obstacle avoidance.

High-level navigation runs on a Linux system (PC / Jetson), while low-level motor control
is handled by an **ESP32 microcontroller using micro-ROS**, subscribed to `/cmd_vel`.

---

## System Architecture

```

┌─────────────────────────────────────────────────────────────────────┐
│                GPS-Based Autonomous Navigation System                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────┐        ┌──────────────┐                          │
│  │     GPS      │        │    LiDAR     │                          │
│  │   (/fix)     │        │   (/scan)    │                          │
│  └──────┬───────┘        └──────┬───────┘                          │
│         │                         │                                │
│         ▼                         ▼                                │
│  ┌──────────────────┐     ┌──────────────────┐                   │
│  │ GPS → Local Node │     │  SLAM Toolbox     │                   │
│  │ lat/lon → x,y    │     │  Builds map       │                   │
│  └──────┬───────────┘     └──────┬───────────┘                   │
│         │                         │                                │
│         └──────────────┬──────────┘                                │
│                        ▼                                           │
│              ┌──────────────────────┐                              │
│              │   Localization (TF)  │                              │
│              │   map → odom → base  │                              │
│              └─────────┬────────────┘                              │
│                        ▼                                           │
│              ┌──────────────────────┐                              │
│              │      Nav2 Stack      │                              │
│              │  Planner + Controller│                              │
│              │  Costmaps            │                              │
│              └─────────┬────────────┘                              │
│                        ▼                                           │
│                    /cmd_vel                                      │
│                        ▼                                           │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │                micro-ROS (ESP32)                             │  │
│  │  - Subscribes to /cmd_vel                                    │  │
│  │  - Converts velocity to PWM                                  │  │
│  │  - Drives motors                                              │  │
│  └───────────────┬─────────────────────────────────────────────┘  │
│                  ▼                                                  │
│              Motor Driver → Motors                                  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘

````

---

## Features

- GPS to local coordinate conversion (latitude/longitude → map frame)
- Interactive GPS goal sender via terminal
- Real-time SLAM using LiDAR and SLAM Toolbox
- Autonomous navigation using Nav2
- Obstacle avoidance with local costmaps
- ESP32 motor control using micro-ROS
- RViz visualization support

---

## Hardware Requirements

- RPLidar A1 M8 (or compatible LiDAR)
- Differential drive mobile robot
- ESP32 microcontroller
- Motor driver (PWM / H-bridge)

---

## Software Requirements

- Ubuntu 20.04 / 22.04
- ROS 2 Humble
- Nav2
- SLAM Toolbox
- micro-ROS

---

## Installation

### 1. Install ROS 2 and Nav2
```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
````

### 2. Build the Workspace

```bash
cd ~/ROS_WS
colcon build
source install/setup.bash
```

---

## Run Order (IMPORTANT)

Follow this order strictly:

1. Start LiDAR driver
2. Start SLAM Toolbox
3. Launch Nav2
4. Start GPS goal sender
5. Start micro-ROS agent + ESP32 firmware

Incorrect order may cause Nav2 or goals to fail.

---

## Quick Start (Recommended)

```bash
ros2 launch gps_autonomous_navigation complete_navigation_launch.py
```

With custom GPS origin:

```bash
ros2 launch gps_autonomous_navigation complete_navigation_launch.py \
  origin_lat:=12.9716 \
  origin_lon:=77.5946
```

---

## Individual Component Launch

```bash
# Robot description
ros2 launch gps_autonomous_navigation robot_description_launch.py

# LiDAR
ros2 launch gps_autonomous_navigation lidar_launch.py

# SLAM
ros2 launch gps_autonomous_navigation slam_toolbox_launch.py

# Nav2
ros2 launch gps_autonomous_navigation nav2_launch.py

# GPS Goal Sender
ros2 launch gps_autonomous_navigation gps_goal_sender_launch.py
```

---

## GPS Goal Commands

```
<latitude> <longitude>
<latitude>,<longitude>
origin <latitude> <longitude>
cancel
status
help
exit
```

---

## How GPS Coordinates Work

1. First GPS coordinate sets the local origin (0,0)
2. All future coordinates are converted relative to this origin
3. Converted goals are sent to Nav2 as map-frame goals

---

## TF Tree

```
map
 └── odom
      └── base_footprint
           └── base_link
                └── laser_frame
```

---

## Key Topics

| Topic      | Type                      | Description        |
| ---------- | ------------------------- | ------------------ |
| /scan      | sensor_msgs/LaserScan     | LiDAR data         |
| /map       | nav_msgs/OccupancyGrid    | SLAM-generated map |
| /cmd_vel   | geometry_msgs/Twist       | Velocity commands  |
| /goal_pose | geometry_msgs/PoseStamped | Navigation goal    |
| /odom      | nav_msgs/Odometry         | Robot odometry     |

---

## Navigation Output

Example of GPS-based navigation visualized in RViz:

![Navigation Output](images/output.jpeg)

---

## Motor Control (micro-ROS)

Low-level motor control is handled by an ESP32 using micro-ROS.
The ESP32 subscribes to `/cmd_vel` and converts velocity commands into PWM signals.

Firmware repository:
[https://github.com/KavyaSivakumar2006/micro-ros-motor-control](https://github.com/KavyaSivakumar2006/micro-ros-motor-control)

---

## License

Apache-2.0

```

