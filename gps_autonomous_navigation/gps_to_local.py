#!/usr/bin/env python3
"""
GPS to Local Coordinate Converter Node
Converts GPS coordinates (lat, lon) to local map coordinates (x, y)
using the first received GPS coordinate as the origin reference.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
import math
import json


class GPSToLocalConverter(Node):
    """
    Node that converts GPS coordinates to local map frame coordinates.
    Uses the Haversine formula for distance calculation and stores
    an origin point for reference.
    """

    def __init__(self):
        super().__init__("gps_to_local_converter")

        # Parameters for GPS origin (reference point)
        self.declare_parameter("origin_lat", 0.0)
        self.declare_parameter("origin_lon", 0.0)
        self.declare_parameter("auto_set_origin", True)

        self.origin_lat = (
            self.get_parameter("origin_lat").get_parameter_value().double_value
        )
        self.origin_lon = (
            self.get_parameter("origin_lon").get_parameter_value().double_value
        )
        self.auto_set_origin = (
            self.get_parameter("auto_set_origin").get_parameter_value().bool_value
        )

        self.origin_set = False
        if self.origin_lat != 0.0 and self.origin_lon != 0.0:
            self.origin_set = True
            self.get_logger().info(
                f"GPS Origin set from parameters: lat={self.origin_lat}, lon={self.origin_lon}"
            )

        # Earth's radius in meters
        self.EARTH_RADIUS = 6371000.0

        # Subscriber for GPS goal input (as JSON string: {"lat": x, "lon": y})
        self.gps_goal_sub = self.create_subscription(
            String, "/gps_goal", self.gps_goal_callback, 10
        )

        # Subscriber for setting origin
        self.set_origin_sub = self.create_subscription(
            String, "/set_gps_origin", self.set_origin_callback, 10
        )

        # Publisher for local goal pose
        self.local_goal_pub = self.create_publisher(PoseStamped, "/goal_pose_local", 10)

        # Publisher for converted coordinates info
        self.info_pub = self.create_publisher(String, "/gps_conversion_info", 10)

        self.get_logger().info("GPS to Local Converter Node started")
        self.get_logger().info("Waiting for GPS goal on topic: /gps_goal")
        self.get_logger().info(
            'Send GPS coordinates as JSON: {"lat": <latitude>, "lon": <longitude>}'
        )

    def set_origin_callback(self, msg):
        """Set the GPS origin from incoming message."""
        try:
            data = json.loads(msg.data)
            self.origin_lat = float(data["lat"])
            self.origin_lon = float(data["lon"])
            self.origin_set = True
            self.get_logger().info(
                f"GPS Origin set: lat={self.origin_lat}, lon={self.origin_lon}"
            )
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(
                f'Invalid origin format: {e}. Use: {{"lat": x, "lon": y}}'
            )

    def gps_goal_callback(self, msg):
        """Process incoming GPS goal and convert to local coordinates."""
        try:
            data = json.loads(msg.data)
            lat = float(data["lat"])
            lon = float(data["lon"])

            # Auto-set origin if enabled and not set
            if not self.origin_set and self.auto_set_origin:
                self.origin_lat = lat
                self.origin_lon = lon
                self.origin_set = True
                self.get_logger().info(
                    f"Auto-set GPS Origin: lat={self.origin_lat}, lon={self.origin_lon}"
                )
                self.get_logger().info("First goal will be at (0, 0). Send next goal.")
                return

            if not self.origin_set:
                self.get_logger().warn(
                    "GPS Origin not set. Set origin first or enable auto_set_origin."
                )
                return

            # Convert GPS to local coordinates
            x, y = self.gps_to_local(lat, lon)

            # Create and publish goal pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 1.0

            self.local_goal_pub.publish(goal_pose)

            # Publish info
            info_msg = String()
            info_msg.data = json.dumps(
                {
                    "input_lat": lat,
                    "input_lon": lon,
                    "origin_lat": self.origin_lat,
                    "origin_lon": self.origin_lon,
                    "local_x": x,
                    "local_y": y,
                }
            )
            self.info_pub.publish(info_msg)

            self.get_logger().info(
                f"Converted GPS ({lat}, {lon}) -> Local ({x:.3f}, {y:.3f}) meters"
            )

        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f"Invalid GPS goal format: {e}")
            self.get_logger().error(
                'Use format: {"lat": <latitude>, "lon": <longitude>}'
            )

    def gps_to_local(self, lat, lon):
        """
        Convert GPS coordinates to local XY coordinates.
        Uses equirectangular approximation (good for small distances).
        X = East direction (positive = east)
        Y = North direction (positive = north)
        """
        # Convert to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)

        # Calculate differences
        d_lat = lat_rad - origin_lat_rad
        d_lon = lon_rad - origin_lon_rad

        # Equirectangular approximation
        # X is east-west (longitude difference)
        x = self.EARTH_RADIUS * d_lon * math.cos((origin_lat_rad + lat_rad) / 2.0)

        # Y is north-south (latitude difference)
        y = self.EARTH_RADIUS * d_lat

        return x, y

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """
        Calculate the great circle distance between two points
        on the earth (specified in decimal degrees).
        Returns distance in meters.
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        d_lat = math.radians(lat2 - lat1)
        d_lon = math.radians(lon2 - lon1)

        a = (
            math.sin(d_lat / 2) ** 2
            + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(d_lon / 2) ** 2
        )
        c = 2 * math.asin(math.sqrt(a))

        return self.EARTH_RADIUS * c


def main(args=None):
    rclpy.init(args=args)
    node = GPSToLocalConverter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
