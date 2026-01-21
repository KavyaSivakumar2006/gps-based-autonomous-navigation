#!/usr/bin/env python3
"""
Bringup Launch File - Start navigation components step by step
Use this for debugging TF issues
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare("gps_nav").find("gps_nav")

    # Paths
    urdf_file = os.path.join(pkg_share, "urdf", "qbotix_rover.urdf.xacro")
    slam_params = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")
    nav2_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
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

    # ========== 1. ROBOT STATE PUBLISHER ==========
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "publish_frequency": 50.0,
                "use_sim_time": False,
            }
        ],
    )

    # ========== 2. ODOM TF PUBLISHER (must start early) ==========
    odom_tf_publisher = Node(
        package="gps_nav",
        executable="odom_tf_publisher.py",
        name="odom_tf_publisher",
        output="screen",
        parameters=[
            {
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "publish_rate": 50.0,
            }
        ],
    )

    # ========== 3. RPLIDAR (after TF is ready) ==========
    rplidar_node = TimerAction(
        period=1.0,
        actions=[
            Node(
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
        ],
    )

    # ========== 4. SLAM TOOLBOX (after lidar is publishing) ==========
    slam_toolbox = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params, {"use_sim_time": False}],
            )
        ],
    )

    # ========== 5. NAV2 (after map is available) ==========
    nav2_controller = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_params],
            )
        ],
    )

    nav2_planner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[nav2_params],
            )
        ],
    )

    nav2_behaviors = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[nav2_params],
            )
        ],
    )

    nav2_bt_navigator = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[nav2_params],
            )
        ],
    )

    nav2_lifecycle_manager = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": [
                            "controller_server",
                            "planner_server",
                            "behavior_server",
                            "bt_navigator",
                        ],
                    }
                ],
            )
        ],
    )

    # ========== 6. RVIZ ==========
    rviz = TimerAction(
        period=2.0,
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

    # ========== 7. GPS GOAL SENDER (after everything is ready) ==========
    gps_goal_sender = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="gps_nav",
                executable="gps_goal_sender.py",
                name="gps_goal_sender",
                output="screen",
                emulate_tty=True,
            )
        ],
    )

    return LaunchDescription(
        [
            declare_serial_port,
            robot_state_publisher,
            odom_tf_publisher,
            rplidar_node,
            slam_toolbox,
            nav2_controller,
            nav2_planner,
            nav2_behaviors,
            nav2_bt_navigator,
            nav2_lifecycle_manager,
            rviz,
            gps_goal_sender,
        ]
    )
