#!/usr/bin/env python3
"""
Goal Pose Publisher Node
Subscribes to local goal poses and publishes to Nav2's goal_pose topic.
Acts as a bridge between GPS converter and Nav2.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus


class GoalPosePublisher(Node):
    """
    Node that receives local goal poses and sends them to Nav2.
    """

    def __init__(self):
        super().__init__("goal_pose_publisher")

        # Nav2 action client
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Subscribe to local goal pose
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose_local", self.goal_callback, 10
        )

        # Also publish to /goal_pose for RViz compatibility
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self._goal_handle = None

        self.get_logger().info("Goal Pose Publisher Node started")
        self.get_logger().info("Waiting for Nav2 action server...")

        # Wait for action server in background
        self.create_timer(1.0, self.check_action_server)
        self._server_ready = False

    def check_action_server(self):
        """Check if action server is ready."""
        if not self._server_ready:
            if self._action_client.server_is_ready():
                self._server_ready = True
                self.get_logger().info("Nav2 action server is ready!")

    def goal_callback(self, msg):
        """Handle incoming goal pose."""
        self.get_logger().info(
            f"Received goal: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}"
        )

        # Publish to /goal_pose for RViz
        self.goal_pub.publish(msg)

        # Send to Nav2
        self.send_goal(msg)

    def send_goal(self, pose_stamped):
        """Send navigation goal to Nav2."""
        if not self._action_client.server_is_ready():
            self.get_logger().warn("Nav2 action server not ready, waiting...")
            if not self._action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Nav2 action server not available!")
                return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info("Sending navigation goal to Nav2...")

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response."""
        self._goal_handle = future.result()

        if not self._goal_handle.accepted:
            self.get_logger().warn("Goal was rejected by Nav2")
            return

        self.get_logger().info("Goal accepted, navigating...")

        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation goal reached successfully!")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Navigation goal was canceled")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Navigation goal aborted")
        else:
            self.get_logger().info(f"Goal finished with status: {status}")

        self._goal_handle = None

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        self.get_logger().info(
            f"Distance remaining: {distance:.2f}m", throttle_duration_sec=2.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
