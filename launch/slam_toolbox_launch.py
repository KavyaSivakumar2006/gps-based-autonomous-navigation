#!/usr/bin/env python3
"""
Launch file for SLAM Toolbox (Online Async mode)
Provides SLAM functionality using RPLidar data
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

    # SLAM Toolbox params file
    slam_params_file = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params = LaunchConfiguration("slam_params_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=slam_params_file,
        description="Full path to the SLAM Toolbox parameters file",
    )

    # SLAM Toolbox node (Online Async)
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_slam_params,
            slam_toolbox_node,
        ]
    )
