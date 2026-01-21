#!/usr/bin/env python3
"""
Launch file for RPLidar A1 M8 using sllidar_ros2 package
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    frame_id = LaunchConfiguration("frame_id")
    inverted = LaunchConfiguration("inverted")
    angle_compensate = LaunchConfiguration("angle_compensate")
    scan_mode = LaunchConfiguration("scan_mode")

    declare_serial_port = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for the lidar",
    )

    declare_serial_baudrate = DeclareLaunchArgument(
        "serial_baudrate",
        default_value="115200",
        description="Baud rate for serial communication",
    )

    declare_frame_id = DeclareLaunchArgument(
        "frame_id",
        default_value="laser_frame",
        description="Frame ID for the laser scan",
    )

    declare_inverted = DeclareLaunchArgument(
        "inverted", default_value="false", description="Invert the scan direction"
    )

    declare_angle_compensate = DeclareLaunchArgument(
        "angle_compensate",
        default_value="true",
        description="Enable angle compensation",
    )

    declare_scan_mode = DeclareLaunchArgument(
        "scan_mode", default_value="Standard", description="Scan mode for RPLidar A1 M8"
    )

    # RPLidar node from sllidar_ros2 package
    rplidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        parameters=[
            {
                "serial_port": serial_port,
                "serial_baudrate": serial_baudrate,
                "frame_id": frame_id,
                "inverted": inverted,
                "angle_compensate": angle_compensate,
                "scan_mode": scan_mode,
            }
        ],
        remappings=[
            ("/scan", "/scan"),
        ],
    )

    return LaunchDescription(
        [
            declare_serial_port,
            declare_serial_baudrate,
            declare_frame_id,
            declare_inverted,
            declare_angle_compensate,
            declare_scan_mode,
            rplidar_node,
        ]
    )
