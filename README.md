GPS Autonomous Navigation (ROS 2)








A ROS 2 package for GPS-based autonomous navigation using LiDAR, SLAM Toolbox, and the Nav2 stack, with low-level motor control via micro-ROS on ESP32.

Overview

This project implements an autonomous navigation pipeline where GPS coordinates
(latitude, longitude) are converted into local map-frame goals and executed
using the ROS 2 Nav2 stack.

The system supports:

Real-time SLAM mapping

Obstacle avoidance

GPS-based goal navigation

Hardware motor control using micro-ROS

The architecture cleanly separates high-level navigation (ROS 2) and
low-level motor control (ESP32).

Key Features

GPS → local map coordinate conversion

Interactive GPS goal sender (terminal-based)

Real-time mapping using SLAM Toolbox

Autonomous navigation using Nav2

LiDAR integration (RPLidar A1 M8)

RViz visualization support

micro-ROS based motor control

System Architecture

The system follows a layered ROS 2 + micro-ROS design, where navigation runs
on a Linux system and motor control runs on an ESP32.

1. Sensor Layer

GPS

Publishes global position

Topic: /fix

LiDAR (RPLidar A1 M8)

Publishes laser scans

Topic: /scan

2. Perception & Mapping

SLAM Toolbox

Builds a 2D occupancy grid map

Performs scan matching

Publishes the global map

Published Topics

/map

/tf

3. GPS Processing

GPS → Local Converter Node

Converts latitude & longitude into local (x, y) map coordinates

Uses the first GPS point as the origin (0, 0)

Publishes navigation goals for Nav2

Published Topic

/goal_pose

4. Localization & TF

Standard ROS 2 TF chain:

map → odom → base_footprint → base_link


This ensures Nav2 can correctly plan paths in the map frame.

5. Navigation (Nav2)

Global path planning

Local obstacle avoidance

Costmap generation

Velocity command generation

Subscribed Topics

/map

/odom

/goal_pose

Published Topic

/cmd_vel

6. Motor Control (micro-ROS)

micro-ROS Agent

Runs on Linux / SBC

Bridges ROS 2 and embedded hardware

ESP32 (micro-ROS client)

Subscribes to /cmd_vel

Converts velocity commands into PWM

Drives motor driver hardware

Subscribed Topic

/cmd_vel

Hardware Requirements

RPLidar A1 M8 (or compatible LiDAR)

Differential-drive mobile robot

ESP32 microcontroller (for motor control)

Software Requirements

ROS 2 Humble

Nav2

SLAM Toolbox

RViz2

micro-ROS

Required ROS 2 Packages
sudo apt update
sudo apt install ros-$ROS_DISTRO-slam-toolbox
sudo apt install ros-$ROS_DISTRO-navigation2
sudo apt install ros-$ROS_DISTRO-nav2-bringup
sudo apt install ros-$ROS_DISTRO-robot-state-publisher
sudo apt install ros-$ROS_DISTRO-joint-state-publisher
sudo apt install ros-$ROS_DISTRO-xacro
sudo apt install ros-$ROS_DISTRO-tf2-ros
sudo apt install ros-$ROS_DISTRO-tf2-geometry-msgs

Build Instructions
cd ~/ROS_WS
colcon build
source install/setup.bash

Run Order (Important)

Follow this order strictly:

Start LiDAR driver

Start SLAM Toolbox (or localization)

Launch Nav2 bringup

Start GPS goal sender

Start micro-ROS agent and ESP32 firmware (hardware only)

Incorrect order may cause Nav2 or GPS goals to fail.

Usage
Complete Navigation (Recommended)
ros2 launch gps_autonomous_navigation complete_navigation_launch.py


With custom parameters:

ros2 launch gps_autonomous_navigation complete_navigation_launch.py \
  serial_port:=/dev/ttyUSB0 \
  origin_lat:=12.9716 \
  origin_lon:=77.5946

Without RViz
ros2 launch gps_autonomous_navigation complete_navigation_launch.py use_rviz:=false

Individual Components (Debugging)
# Robot description
ros2 launch gps_autonomous_navigation robot_description_launch.py

# LiDAR only
ros2 launch gps_autonomous_navigation lidar_launch.py

# SLAM only
ros2 launch gps_autonomous_navigation slam_toolbox_launch.py

# Nav2 only
ros2 launch gps_autonomous_navigation nav2_launch.py

# GPS Goal Sender only
ros2 launch gps_autonomous_navigation gps_goal_sender_launch.py

GPS Goal Sender Usage
============================================================
GPS NAVIGATION GOAL SENDER
============================================================

<lat> <lon>        - Send GPS goal
<lat>,<lon>        - Send GPS goal with comma
origin <lat> <lon> - Set GPS origin
cancel             - Cancel navigation
status             - Navigation status
help               - Show help
quit / exit        - Exit

Navigation Output

The image below shows GPS-based navigation using Nav2 with a SLAM-generated map:

Motor Control (micro-ROS)

Low-level motor control is implemented using ESP32 + micro-ROS.

ESP32 subscribes to /cmd_vel

Converts velocity commands to PWM

Drives motors in real time

👉 Firmware repository
https://github.com/KavyaSivakumar2006/micro-ros-motor-control

License

Apache-2.0
