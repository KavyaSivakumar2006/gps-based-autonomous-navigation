#!/usr/bin/env python3
"""
Simple SLAM Launch - Just SLAM and Lidar for testing
Use this to verify SLAM works before adding Nav2
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare("gps_nav").find("gps_nav")

    # Paths
    urdf_file = os.path.join(pkg_share, "urdf", "qbotix_rover.urdf.xacro")
    slam_params = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "gps_nav.rviz")

    # Launch arguments
    serial_port = LaunchConfiguration("serial_port")

    declare_serial_port = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for RPLidar",
    )

    # Robot description
    robot_description = Command(["xacro ", urdf_file])

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "publish_frequency": 50.0,
            }
        ],
    )

    # 2. Laser Odometry: odom -> base_footprint (uses cmd_vel for dead reckoning)
    laser_odom = Node(
        package="gps_nav",
        executable="laser_odom",
        name="laser_odometry",
        output="screen",
        parameters=[
            {
                "base_frame": "base_footprint",
                "odom_frame": "odom",
                "laser_frame": "laser_frame",
            }
        ],
    )

    # 2b. Initial map -> odom TF (SLAM Toolbox will override this once it starts)
    static_map_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    # 3. RPLidar Node
    rplidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        parameters=[
            {
                "serial_port": serial_port,
                "serial_baudrate": 115200,
                "frame_id": "laser_frame",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Standard",
            }
        ],
    )

    # 4. SLAM Toolbox (with delay to let TFs settle)
    slam_toolbox = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params],
            )
        ],
    )

    # 5. RViz
    rviz = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            )
        ],
    )

    return LaunchDescription(
        [
            declare_serial_port,
            robot_state_publisher,
            laser_odom,
            static_map_odom_tf,
            rplidar_node,
            slam_toolbox,
            rviz,
        ]
    )
