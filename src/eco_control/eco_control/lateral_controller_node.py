#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import math


class LateralControllerNode(Node):
    def __init__(self):
        super().__init__("lateral_controller_node")

        # Parameters (all in degrees for better readability)
        self.declare_parameter("kp", 1.0)  # proportional gain in degrees
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("wheelbase", 1.42)  # meters
        self.declare_parameter(
            "max_angular_velocity_deg", 180
        )  # degrees/sec (equivalent to 0.5 rad/s)
        self.declare_parameter("center_angle_deg", 180.0)  # degrees

        self.kp = self.get_parameter("kp").value
        self.ki = self.get_parameter("ki").value
        self.kd = self.get_parameter("kd").value

        # Store max angular velocity in degrees/sec
        self.max_angular_velocity_deg = self.get_parameter(
            "max_angular_velocity_deg"
        ).value

        self.wheelbase = self.get_parameter("wheelbase").value

        # Center angle for steering (in degrees)
        self.center_angle_deg = self.get_parameter("center_angle_deg").value

        # PID controller state
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now()

        # Target and current angular velocity (in degrees/sec)
        self.target_angular_velocity_deg = self.center_angle_deg
        self.current_angular_velocity_deg = 0.0

        # Current steering angle (in degrees) and vehicle speed
        self.current_steering_angle_deg = 0.0
        self.current_speed = 0.8  # m/s

        # Publishers
        self.steering_pub = self.create_publisher(
            Float32, "/control/steering_commands", 10
        )

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/planning/cmd_vel", self.cmd_vel_callback, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odometry_callback, 10
        )


        # Timer for control updates
        self.timer = self.create_timer(0.02, self.control_update)

        self.get_logger().info(
            f"Lateral controller initialized! with kp: {self.kp}, ki: {self.ki}, kd: {self.kd}, max_angular_velocity_deg: {self.max_angular_velocity_deg}, center_angle_deg: {self.center_angle_deg}"
        )

    def cmd_vel_callback(self, msg):
        # Convert from [-1, 1] to [-max_angular_velocity_deg + center_angle, max_angular_velocity_deg + center_angle]
        self.target_angular_velocity_deg = (
            msg.angular.z * -1 * self.max_angular_velocity_deg
        ) + self.center_angle_deg 
        

    def odometry_callback(self, msg):
        # Convert from rad/s to degrees/sec
        self.current_angular_velocity_deg = math.degrees(msg.twist.twist.angular.z)

        # Extract linear speed from odometry
        self.current_speed = np.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )

    def control_update(self):
        steering_angle_msg = Float32()
        steering_angle_msg.data = self.target_angular_velocity_deg
        self.steering_pub.publish(steering_angle_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LateralControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
