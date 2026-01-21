#!/usr/bin/env python3
"""
Complete GPS Navigation Launch File with SLAM Odometry
This launch file starts all components needed for GPS navigation:
- Robot description (URDF)
- RPLidar A1 M8
- Fake odometry (SLAM will provide real odometry via scan matching)
- SLAM Toolbox
- Nav2 Navigation Stack
- GPS Goal Sender (interactive terminal)
- RViz2 (optional)

Usage:
  ros2 launch gps_nav complete_navigation_launch.py
  ros2 launch gps_nav complete_navigation_launch.py serial_port:=/dev/ttyUSB1
  ros2 launch gps_nav complete_navigation_launch.py origin_lat:=12.9716 origin_lon:=77.5946
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare("gps_nav").find("gps_nav")

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    origin_lat = LaunchConfiguration("origin_lat")
    origin_lon = LaunchConfiguration("origin_lon")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_fake_odom = LaunchConfiguration("use_fake_odom")

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_serial_port = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for RPLidar A1 M8",
    )

    declare_serial_baudrate = DeclareLaunchArgument(
        "serial_baudrate",
        default_value="115200",
        description="Baud rate for RPLidar serial communication",
    )

    declare_origin_lat = DeclareLaunchArgument(
        "origin_lat",
        default_value="0.0",
        description="GPS origin latitude (first coordinate will be used if 0.0)",
    )

    declare_origin_lon = DeclareLaunchArgument(
        "origin_lon",
        default_value="0.0",
        description="GPS origin longitude (first coordinate will be used if 0.0)",
    )

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Launch RViz2 for visualization"
    )

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(pkg_share, "rviz", "gps_nav.rviz"),
        description="Path to RViz config file",
    )

    declare_use_fake_odom = DeclareLaunchArgument(
        "use_fake_odom",
        default_value="true",
        description="Use fake odometry (set false if you have wheel encoders)",
    )

    # ==================== ROBOT DESCRIPTION ====================
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "robot_description_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # ==================== LIDAR ====================
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "lidar_launch.py")
        ),
        launch_arguments={
            "serial_port": serial_port,
            "serial_baudrate": serial_baudrate,
            "frame_id": "laser_frame",
        }.items(),
    )

    # ==================== FAKE ODOMETRY ====================
    fake_odom_node = Node(
        package="gps_nav",
        executable="fake_odom_publisher.py",
        name="fake_odom_publisher",
        output="screen",
        parameters=[
            {
                "publish_tf": True,
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "publish_rate": 30.0,
            }
        ],
        condition=IfCondition(use_fake_odom),
    )

    # ==================== SLAM TOOLBOX ====================
    # Delay SLAM to ensure odom is available
    slam_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "slam_toolbox_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        ],
    )

    # ==================== NAV2 ====================
    # Delay Nav2 to ensure SLAM and map are available
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "nav2_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        ],
    )

    # ==================== GPS GOAL SENDER ====================
    # Delay GPS goal sender to ensure Nav2 is ready
    gps_goal_sender_node = TimerAction(
        period=8.0,
        actions=[
            Node(
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
        ],
    )

    # ==================== RVIZ2 ====================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            # Declare arguments
            declare_use_sim_time,
            declare_serial_port,
            declare_serial_baudrate,
            declare_origin_lat,
            declare_origin_lon,
            declare_use_rviz,
            declare_rviz_config,
            declare_use_fake_odom,
            # Launch nodes
            robot_description_launch,
            lidar_launch,
            fake_odom_node,
            slam_launch,
            nav2_launch,
            gps_goal_sender_node,
            rviz_node,
        ]
    )
