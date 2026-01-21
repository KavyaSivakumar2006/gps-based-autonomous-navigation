#!/usr/bin/env python3
"""
Launch file for GPS goal sender node
Allows sending GPS coordinates as navigation goals
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare("gps_nav").find("gps_nav")

    # Launch arguments
    origin_lat = LaunchConfiguration("origin_lat")
    origin_lon = LaunchConfiguration("origin_lon")

    declare_origin_lat = DeclareLaunchArgument(
        "origin_lat",
        default_value="0.0",
        description="GPS origin latitude (reference point)",
    )

    declare_origin_lon = DeclareLaunchArgument(
        "origin_lon",
        default_value="0.0",
        description="GPS origin longitude (reference point)",
    )

    # GPS Goal Sender node
    gps_goal_sender_node = Node(
        package="gps_nav",
        executable="gps_goal_sender.py",
        name="gps_goal_sender",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "origin_lat": origin_lat,
                "origin_lon": origin_lon,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_origin_lat,
            declare_origin_lon,
            gps_goal_sender_node,
        ]
    )
