#!/usr/bin/env python3
"""
Launch file for bringing up the robot description (URDF)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare("gps_nav").find("gps_nav")

    # URDF file path
    urdf_file = os.path.join(pkg_share, "urdf", "qbotix_rover.urdf.xacro")

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    # Robot description from xacro
    robot_description = Command(["xacro ", urdf_file])

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
                "publish_frequency": 30.0,
            }
        ],
    )

    # Joint state publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            robot_state_publisher_node,
            joint_state_publisher_node,
        ]
    )
