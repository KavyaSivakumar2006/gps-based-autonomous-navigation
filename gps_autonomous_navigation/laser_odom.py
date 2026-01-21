#!/usr/bin/env python3
"""
Dead Reckoning Odometry Node
Integrates /cmd_vel to estimate robot pose.
Publishes odom -> base_footprint transform.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import math


class DeadReckoningOdom(Node):
    def __init__(self):
        super().__init__("laser_odometry")

        # Parameters
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("odom_frame", "odom")

        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value

        # Current pose estimate
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Current velocity from cmd_vel
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom", 50)

        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        # Timer for integrating odometry at 50 Hz
        self.dt = 0.02  # 50 Hz
        self.timer = self.create_timer(self.dt, self.update_odom)

        self.get_logger().info("Dead Reckoning Odometry Node started")
        self.get_logger().info(f"Publishing TF: {self.odom_frame} -> {self.base_frame}")
        self.get_logger().info("Subscribing to /cmd_vel for velocity commands")

    def cmd_vel_callback(self, msg):
        """Store latest cmd_vel."""
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vtheta = msg.angular.z

        # Log when we receive commands
        if abs(self.vx) > 0.01 or abs(self.vtheta) > 0.01:
            self.get_logger().info(
                f"CMD_VEL: vx={self.vx:.3f}, vtheta={self.vtheta:.3f}",
                throttle_duration_sec=1.0,
            )

    def update_odom(self):
        """Integrate velocity to update pose and publish."""
        # Dead reckoning integration
        # For differential drive: x' = x + vx*cos(theta)*dt, y' = y + vx*sin(theta)*dt

        delta_x = (
            self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)
        ) * self.dt
        delta_y = (
            self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)
        ) * self.dt
        delta_theta = self.vtheta * self.dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Get current time
        current_time = self.get_clock().now()

        # Publish TF: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Velocity
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.vtheta

        # Covariance (rough estimates)
        odom_msg.pose.covariance[0] = 0.01  # x
        odom_msg.pose.covariance[7] = 0.01  # y
        odom_msg.pose.covariance[35] = 0.01  # theta
        odom_msg.twist.covariance[0] = 0.01  # vx
        odom_msg.twist.covariance[7] = 0.01  # vy
        odom_msg.twist.covariance[35] = 0.01  # vtheta

        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
