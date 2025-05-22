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
            "max_angular_velocity_deg", 70
        )  # degrees/sec (equivalent to 0.5 rad/s)
        self.declare_parameter("center_angle_deg", 90.0)  # degrees

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

        self.steering_angle_sub = self.create_subscription(
            Float32, "/sensor/steering_angle", self.steering_angle_callback, 10
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
        self.get_logger().info(
            f"Target angular velocity: {self.target_angular_velocity_deg}°/s"
        )

    def odometry_callback(self, msg):
        # Convert from rad/s to degrees/sec
        self.current_angular_velocity_deg = math.degrees(msg.twist.twist.angular.z)

        # Extract linear speed from odometry
        self.current_speed = np.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )

    def steering_angle_callback(self, msg):
        # Steering angle feedback is already in degrees
        angle_degrees = msg.data

        # Check for obviously invalid values
        if abs(angle_degrees) > 360:
            self.get_logger().warn(
                f"Received invalid steering angle: {angle_degrees}°, ignoring"
            )
            return

        self.get_logger().info(f"Current steering angle: {angle_degrees}°")

        self.current_steering_angle_deg = angle_degrees

    def control_update(self):
        # Calculate time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        # Convert target angular velocity to target steering angle using bicycle model
        if self.current_speed < 0.05:
            # When stationary or very slow, use center position
            target_steering_angle_deg = self.center_angle_deg
        else:
            # Bicycle model for normal operation
            effective_speed = max(0.1, abs(self.current_speed))

            # Convert target angular velocity from degrees/sec to rad/sec
            target_angular_velocity_rad = math.radians(
                self.target_angular_velocity_deg - self.center_angle_deg
            )

            # Calculate steering angle in radians
            target_steering_angle_rad = math.atan2(
                target_angular_velocity_rad * self.wheelbase, effective_speed
            )

            # Convert back to degrees and add center offset
            target_steering_angle_deg = (
                math.degrees(target_steering_angle_rad) + self.center_angle_deg
            )

        # Calculate error (difference between target and current steering angle)
        error = target_steering_angle_deg - self.current_steering_angle_deg

        # Update error sum (with anti-windup)
        self.error_sum += error * dt
        max_windup = 30.0  # degrees
        self.error_sum = max(-max_windup, min(max_windup, self.error_sum))

        # Calculate error derivative
        error_derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error

        # PID control
        control_output = (
            self.kp * error + self.ki * self.error_sum + self.kd * error_derivative
        )

        # Apply control output to get final steering angle
        final_steering_angle_deg = self.current_steering_angle_deg + control_output

        # Limit to valid range
        max_angle = self.center_angle_deg + self.max_angular_velocity_deg
        min_angle = self.center_angle_deg - self.max_angular_velocity_deg
        final_steering_angle_deg = min(
            max_angle, max(final_steering_angle_deg, min_angle)
        )

        # Rate limiting
        if hasattr(self, "last_steering_command"):
            max_change_per_update = 5.0  # degrees per update
            if (
                abs(final_steering_angle_deg - self.last_steering_command)
                > max_change_per_update
            ):
                direction = (
                    1 if final_steering_angle_deg > self.last_steering_command else -1
                )
                final_steering_angle_deg = (
                    self.last_steering_command + direction * max_change_per_update
                )

        self.last_steering_command = final_steering_angle_deg

        # Apply low-pass filter
        if not hasattr(self, "filtered_steering_command"):
            self.filtered_steering_command = self.center_angle_deg

        self.filtered_steering_command = (
            0.3 * final_steering_angle_deg + 0.7 * self.filtered_steering_command
        )

        # Publish steering command
        steering_msg = Float32()
        steering_msg.data = float(self.filtered_steering_command)
        self.steering_pub.publish(steering_msg)

        # Log information periodically
        if self.get_clock().now().nanoseconds % 1e9 < 0.02 * 1e9:
            self.get_logger().info(
                f"Error: {error:.1f}°, "
                f"Control output: {control_output:.1f}°, "
                f"Target steering: {target_steering_angle_deg:.1f}°, "
                f"Final steering: {final_steering_angle_deg:.1f}°, "
                f"Filtered steering: {self.filtered_steering_command:.1f}°, "
                f"Current steering: {self.current_steering_angle_deg:.1f}°, "
                f"Speed: {self.current_speed:.2f} m/s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LateralControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
