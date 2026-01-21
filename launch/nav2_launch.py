#!/usr/bin/env python3
"""
Launch file for Nav2 navigation stack
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get package share directories
    pkg_share = FindPackageShare("gps_nav").find("gps_nav")
    nav2_bringup_share = FindPackageShare("nav2_bringup").find("nav2_bringup")

    # Nav2 params file
    nav2_params_file = os.path.join(pkg_share, "config", "nav2_params.yaml")

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=nav2_params_file,
        description="Full path to the Nav2 parameters file",
    )

    # Rewrite the YAML to set use_sim_time
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={"use_sim_time": use_sim_time},
        convert_types=True,
    )

    # Lifecycle manager for navigation nodes
    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    # Nav2 Controller Server
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        remappings=[
            ("cmd_vel", "/cmd_vel"),
            ("odom", "/odom"),
        ],
    )

    # Nav2 Smoother Server
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
    )

    # Nav2 Planner Server
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
    )

    # Nav2 Behavior Server
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        remappings=[
            ("cmd_vel", "/cmd_vel"),
            ("odom", "/odom"),
        ],
    )

    # Nav2 BT Navigator
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        remappings=[
            ("cmd_vel", "/cmd_vel"),
            ("odom", "/odom"),
        ],
    )

    # Nav2 Waypoint Follower
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
    )

    # Nav2 Velocity Smoother
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        remappings=[
            ("cmd_vel", "/cmd_vel"),
            ("cmd_vel_smoothed", "/cmd_vel_smoothed"),
            ("odom", "/odom"),
        ],
    )

    # Lifecycle Manager
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": lifecycle_nodes,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_autostart,
            declare_params_file,
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            lifecycle_manager,
        ]
    )
