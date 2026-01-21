#!/usr/bin/env python3
"""
GPS Goal Sender Node - WITH WAYPOINT NAVIGATION
- Subscribes to /fix topic to get current GPS position
- Gets robot's current position in map frame via TF
- For long distances: breaks goal into smaller waypoints (5m steps)
- Sends waypoints one at a time to Nav2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)
import math
import json
import threading
import time


class GPSGoalSender(Node):
    """
    GPS Navigation with waypoint support for long distances.
    """

    def __init__(self):
        super().__init__("gps_goal_sender")

        # Waypoint parameters
        self.WAYPOINT_DISTANCE = 5.0  # Max distance per waypoint (meters)

        # Current GPS position from /fix topic
        self.current_lat = None
        self.current_lon = None
        self.gps_received = False

        # Final goal GPS (for waypoint navigation)
        self.final_goal_lat = None
        self.final_goal_lon = None
        self.navigating_to_gps = False

        # Earth's radius in meters
        self.EARTH_RADIUS = 6371000.0

        # TF2 for getting robot position in map frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to GPS /fix topic
        self.fix_subscription = self.create_subscription(
            NavSatFix, "/fix", self.fix_callback, 10
        )
        self.get_logger().info("Subscribing to /fix topic for GPS position...")

        # Nav2 action client
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Publisher for GPS goal info
        self.gps_goal_pub = self.create_publisher(String, "/gps_goal", 10)

        # Current goal handle
        self._goal_handle = None
        self._goal_in_progress = False

        self.get_logger().info("GPS Goal Sender Node started")
        self.get_logger().info(f"Waypoint distance: {self.WAYPOINT_DISTANCE}m")
        self.get_logger().info("Waiting for Nav2 action server...")

        # Wait for action server
        if self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info("Nav2 action server is available!")
        else:
            self.get_logger().warn(
                "Nav2 action server not available. Goals will be queued."
            )

        # Start input thread
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def fix_callback(self, msg):
        """Callback for GPS /fix topic."""
        if msg.status.status >= 0:  # Valid GPS fix
            self.current_lat = msg.latitude
            self.current_lon = msg.longitude

            if not self.gps_received:
                self.gps_received = True
                self.get_logger().info(
                    f"[GPS FIX] Receiving GPS: lat={self.current_lat:.8f}, lon={self.current_lon:.8f}"
                )

    def get_robot_map_position(self):
        """Get robot's current position in map frame via TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            q = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

            return x, y, yaw

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not get robot position: {e}")
            return None, None, None

    def input_loop(self):
        """Loop to get GPS coordinates from terminal input."""
        time.sleep(2.0)
        self.print_instructions()

        while rclpy.ok():
            try:
                user_input = input("\n> ").strip()

                if not user_input:
                    continue

                if user_input.lower() == "help":
                    self.print_instructions()
                elif user_input.lower() == "cancel":
                    self.cancel_goal()
                elif user_input.lower() == "status":
                    self.print_status()
                elif user_input.lower() == "gps":
                    self.print_gps_status()
                elif user_input.lower() == "pos":
                    self.print_map_position()
                elif user_input.lower() in ["quit", "exit"]:
                    self.get_logger().info("Shutting down...")
                    rclpy.shutdown()
                    break
                else:
                    self.parse_and_send_goal(user_input)

            except EOFError:
                break
            except Exception as e:
                self.get_logger().error(f"Input error: {e}")

    def print_instructions(self):
        """Print usage instructions."""
        print("\n" + "=" * 60)
        print("GPS NAVIGATION GOAL SENDER (with waypoints)")
        print("=" * 60)
        print(f"\nLong distances are broken into {self.WAYPOINT_DISTANCE}m waypoints")
        print("\nCommands:")
        print("  <lat> <lon>     - Navigate to GPS coordinate")
        print("  gps             - Show current GPS from /fix")
        print("  pos             - Show robot position in map frame")
        print("  cancel          - Cancel current navigation")
        print("  status          - Show navigation status")
        print("  help            - Show this help")
        print("  quit/exit       - Exit")
        print("=" * 60)

        if self.gps_received:
            print(
                f"\n[CURRENT GPS] lat={self.current_lat:.8f}, lon={self.current_lon:.8f}"
            )
        else:
            print("\n[!] Waiting for GPS fix from /fix topic...")

    def print_status(self):
        """Print current navigation status."""
        if self._goal_in_progress:
            print("\n[STATUS] Navigation in progress...")
            if self.navigating_to_gps and self.final_goal_lat:
                print(
                    f"[FINAL GPS GOAL] lat={self.final_goal_lat:.8f}, lon={self.final_goal_lon:.8f}"
                )
        else:
            print("\n[STATUS] Ready for new goal")

    def print_gps_status(self):
        """Print current GPS position."""
        if self.gps_received and self.current_lat is not None:
            print(f"\n[CURRENT GPS from /fix]")
            print(f"  Latitude:  {self.current_lat:.8f}")
            print(f"  Longitude: {self.current_lon:.8f}")
        else:
            print("\n[!] No GPS data received. Check /fix topic.")

    def print_map_position(self):
        """Print robot's position in map frame."""
        x, y, yaw = self.get_robot_map_position()
        if x is not None:
            print(f"\n[ROBOT POSITION in map frame]")
            print(f"  X: {x:.3f} m")
            print(f"  Y: {y:.3f} m")
            print(f"  Yaw: {math.degrees(yaw):.1f} degrees")
        else:
            print("\n[!] Could not get robot position from TF")

    def gps_to_offset(self, goal_lat, goal_lon):
        """Calculate offset in meters from current GPS to goal GPS."""
        if self.current_lat is None or self.current_lon is None:
            return None, None

        lat_rad = math.radians(goal_lat)
        lon_rad = math.radians(goal_lon)
        curr_lat_rad = math.radians(self.current_lat)
        curr_lon_rad = math.radians(self.current_lon)

        d_lat = lat_rad - curr_lat_rad
        d_lon = lon_rad - curr_lon_rad

        dx = self.EARTH_RADIUS * d_lon * math.cos((curr_lat_rad + lat_rad) / 2.0)
        dy = self.EARTH_RADIUS * d_lat

        return dx, dy

    def parse_and_send_goal(self, user_input):
        """Parse GPS coordinates and send navigation goal with waypoints."""
        if not self.gps_received:
            print("\n[ERROR] No GPS data from /fix topic!")
            return

        robot_x, robot_y, robot_yaw = self.get_robot_map_position()
        if robot_x is None:
            print("\n[ERROR] Cannot get robot position!")
            return

        try:
            if "," in user_input:
                parts = [p.strip() for p in user_input.split(",")]
            else:
                parts = user_input.split()

            if len(parts) < 2:
                print("\n[ERROR] Please provide both latitude and longitude")
                return

            goal_lat = float(parts[0])
            goal_lon = float(parts[1])

            if not (-90 <= goal_lat <= 90) or not (-180 <= goal_lon <= 180):
                print(f"\n[ERROR] Invalid coordinates")
                return

            # Store final goal for waypoint navigation
            self.final_goal_lat = goal_lat
            self.final_goal_lon = goal_lon
            self.navigating_to_gps = True

            # Calculate total offset
            dx, dy = self.gps_to_offset(goal_lat, goal_lon)
            if dx is None:
                print("\n[ERROR] Could not calculate GPS offset")
                return

            distance = math.sqrt(dx**2 + dy**2)

            print(f"\n" + "=" * 50)
            print(
                f"[CURRENT GPS]  lat={self.current_lat:.8f}, lon={self.current_lon:.8f}"
            )
            print(f"[GOAL GPS]     lat={goal_lat:.8f}, lon={goal_lon:.8f}")
            print(f"[TOTAL DISTANCE] {distance:.2f} meters")
            print("=" * 50)

            # Calculate waypoint
            if distance > self.WAYPOINT_DISTANCE:
                # Break into smaller waypoint
                ratio = self.WAYPOINT_DISTANCE / distance
                waypoint_dx = dx * ratio
                waypoint_dy = dy * ratio
                goal_x = robot_x + waypoint_dx
                goal_y = robot_y + waypoint_dy
                print(
                    f"[WAYPOINT] Sending {self.WAYPOINT_DISTANCE}m waypoint toward goal"
                )
                print(f"[WAYPOINT] x={goal_x:.3f}m, y={goal_y:.3f}m")
            else:
                # Close enough - send final goal
                goal_x = robot_x + dx
                goal_y = robot_y + dy
                print(f"[FINAL GOAL] x={goal_x:.3f}m, y={goal_y:.3f}m")
                self.navigating_to_gps = False

            print("=" * 50)

            # Publish info
            gps_msg = String()
            gps_msg.data = json.dumps(
                {
                    "current_gps": {"lat": self.current_lat, "lon": self.current_lon},
                    "goal_gps": {"lat": goal_lat, "lon": goal_lon},
                    "distance": distance,
                    "using_waypoint": distance > self.WAYPOINT_DISTANCE,
                }
            )
            self.gps_goal_pub.publish(gps_msg)

            # Send goal
            self.send_goal(goal_x, goal_y)

        except ValueError as e:
            print(f"\n[ERROR] Could not parse coordinates: {e}")

    def send_goal(self, x, y, yaw=0.0):
        """Send navigation goal to Nav2."""
        if not self._action_client.server_is_ready():
            print("\n[WARN] Nav2 not ready, waiting...")
            if not self._action_client.wait_for_server(timeout_sec=5.0):
                print("[ERROR] Nav2 action server not available!")
                return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        print(f"\n[SENDING TO NAV2] Goal: x={x:.3f}m, y={y:.3f}m")

        self._goal_in_progress = True

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response."""
        self._goal_handle = future.result()

        if not self._goal_handle.accepted:
            print("\n[REJECTED] Goal was rejected by Nav2")
            self._goal_in_progress = False
            self.navigating_to_gps = False
            return

        print("[ACCEPTED] Goal accepted, navigating...")

        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        print(f"\r[NAV] Distance remaining: {distance:.2f}m    ", end="", flush=True)

    def goal_result_callback(self, future):
        """Handle goal result - send next waypoint if needed."""
        result = future.result()
        status = result.status

        self._goal_in_progress = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            print("\n[SUCCESS] Waypoint reached!")

            # Check if we need to continue to final GPS goal
            if self.navigating_to_gps and self.final_goal_lat is not None:
                # Calculate remaining distance to final goal
                dx, dy = self.gps_to_offset(self.final_goal_lat, self.final_goal_lon)
                if dx is not None:
                    remaining_dist = math.sqrt(dx**2 + dy**2)
                    print(
                        f"[INFO] Remaining distance to final GPS goal: {remaining_dist:.2f}m"
                    )

                    if remaining_dist > 0.5:  # More than 0.5m to go
                        print("[INFO] Sending next waypoint...")
                        time.sleep(0.5)  # Brief pause
                        # Re-send the goal (will calculate new waypoint)
                        self.parse_and_send_goal(
                            f"{self.final_goal_lat} {self.final_goal_lon}"
                        )
                    else:
                        print("[SUCCESS] Final GPS destination reached!")
                        self.navigating_to_gps = False
                        self.final_goal_lat = None
                        self.final_goal_lon = None

        elif status == GoalStatus.STATUS_ABORTED:
            print("\n[ABORTED] Navigation aborted")
            self.navigating_to_gps = False
        elif status == GoalStatus.STATUS_CANCELED:
            print("\n[CANCELED] Navigation canceled")
            self.navigating_to_gps = False
        else:
            print(f"\n[FINISHED] Status: {status}")

    def cancel_goal(self):
        """Cancel current goal."""
        self.navigating_to_gps = False
        self.final_goal_lat = None
        self.final_goal_lon = None

        if self._goal_handle is not None and self._goal_in_progress:
            print("\n[CANCELING]...")
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            print("\n[INFO] No active goal to cancel")

    def cancel_done_callback(self, future):
        """Handle cancel response."""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            print("[CANCELED] Goal canceled")
        else:
            print("[INFO] Cancel failed or already completed")
        self._goal_in_progress = False


def main(args=None):
    rclpy.init(args=args)
    node = GPSGoalSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
