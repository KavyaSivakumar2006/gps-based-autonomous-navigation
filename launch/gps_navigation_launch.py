#!/usr/bin/env python3
"""
Main launch file for GPS Navigation with SLAM
Launches all components: Robot description, Lidar, SLAM, Nav2, and GPS goal sender
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directories
    pkg_share = FindPackageShare("gps_nav").find("gps_nav")

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")
    origin_lat = LaunchConfiguration("origin_lat")
    origin_lon = LaunchConfiguration("origin_lon")
    rviz_config = LaunchConfiguration("rviz_config")

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_serial_port = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for RPLidar",
    )

    declare_origin_lat = DeclareLaunchArgument(
        "origin_lat", default_value="0.0", description="GPS origin latitude"
    )

    declare_origin_lon = DeclareLaunchArgument(
        "origin_lon", default_value="0.0", description="GPS origin longitude"
    )

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(pkg_share, "rviz", "gps_nav.rviz"),
        description="Path to RViz config file",
    )

    # Include robot description launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "robot_description_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Include lidar launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "lidar_launch.py")
        ),
        launch_arguments={
            "serial_port": serial_port,
            "frame_id": "laser_frame",
        }.items(),
    )

    # Include SLAM Toolbox launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "slam_toolbox_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Include Nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "nav2_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
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

    # RViz2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_serial_port,
            declare_origin_lat,
            declare_origin_lon,
            declare_rviz_config,
            robot_description_launch,
            lidar_launch,
            slam_launch,
            nav2_launch,
            gps_goal_sender_node,
            rviz_node,
        ]
    )
