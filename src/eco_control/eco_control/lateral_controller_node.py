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
        super().__init__('lateral_controller_node')
        
        # Parameters (all in degrees for better readability)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('max_steering_angle_deg', 30.0)  # degrees
        self.declare_parameter('wheelbase', 1.42)  # meters
        self.declare_parameter('max_angular_velocity_deg', 28.6)  # degrees/sec (equivalent to 0.5 rad/s)
        
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        
        # Store max steering angle in degrees
        self.max_steering_angle_deg = self.get_parameter('max_steering_angle_deg').value
        
        # Store max angular velocity in degrees/sec
        self.max_angular_velocity_deg = self.get_parameter('max_angular_velocity_deg').value
        
        self.wheelbase = self.get_parameter('wheelbase').value
        
        # PID controller state
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now()
        
        # Target and current angular velocity (in degrees/sec)
        self.target_angular_velocity_deg = 0.0
        self.current_angular_velocity_deg = 0.0
        
        # Current steering angle (in degrees) and vehicle speed
        self.current_steering_angle_deg = 0.0
        self.current_speed = 0.0
        
        # Publishers
        self.steering_pub = self.create_publisher(
            Float32,
            '/control/steering_commands',
            10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/planning/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odometry_callback,
            10)
        
        self.steering_angle_sub = self.create_subscription(
            Float32,
            '/sensor/steering_angle',
            self.steering_angle_callback,
            10)
        
        # Timer for control updates
        self.timer = self.create_timer(0.02, self.control_update)
        
        self.get_logger().info(f'Lateral controller initialized with max steering angle: {self.max_steering_angle_deg} degrees')
    
    def cmd_vel_callback(self, msg):
        # Convert from rad/s to degrees/sec
        self.target_angular_velocity_deg = math.degrees(msg.angular.z)
    
    def odometry_callback(self, msg):
        # Convert from rad/s to degrees/sec
        self.current_angular_velocity_deg = math.degrees(msg.twist.twist.angular.z)
        
        # Extract linear speed from odometry
        self.current_speed = np.sqrt(
            msg.twist.twist.linear.x**2 +
            msg.twist.twist.linear.y**2
        )
    
    def steering_angle_callback(self, msg):
        # Steering angle feedback is already in degrees
        angle_degrees = msg.data
        
        # Check for obviously invalid values
        if abs(angle_degrees) > 360:
            self.get_logger().warn(f"Received invalid steering angle: {angle_degrees}°, ignoring")
            return
            
        self.current_steering_angle_deg = angle_degrees
    
    def control_update(self):
        # Calculate time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0:
            return
        
        # Calculate error (difference between target and current angular velocity in degrees/sec)
        error = self.target_angular_velocity_deg - self.current_angular_velocity_deg
        
        # Update error sum (with anti-windup)
        self.error_sum += error * dt
        max_windup = 30.0  # degrees/sec
        self.error_sum = max(-max_windup, min(max_windup, self.error_sum))
        
        # Calculate error derivative
        error_derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error
        
        # PID control
        control_output = (
            self.kp * error +
            self.ki * self.error_sum +
            self.kd * error_derivative
        )
        
        # Calculate target steering angle using bicycle model
        # Ensure we have a minimum speed to avoid division by zero
        if self.current_speed < 0.05:  # Very low speed threshold
            # Simple proportional mapping when stationary
            target_steering_angle_deg = self.target_angular_velocity_deg * (self.max_steering_angle_deg / self.max_angular_velocity_deg)
        else:
            # Bicycle model for normal operation
            effective_speed = max(0.1, abs(self.current_speed))
            
            # Convert angular velocity from degrees/sec to rad/sec for the bicycle model calculation
            target_angular_velocity_rad = math.radians(self.target_angular_velocity_deg)
            
            # Calculate steering angle in radians
            target_steering_angle_rad = math.atan2(target_angular_velocity_rad * self.wheelbase, effective_speed)
            
            # Convert back to degrees
            target_steering_angle_deg = math.degrees(target_steering_angle_rad)
        
        # Limit steering angle
        target_steering_angle_deg = max(-self.max_steering_angle_deg, min(self.max_steering_angle_deg, target_steering_angle_deg))
        
        # Apply PID correction to account for tracking errors (in degrees)
        # This helps compensate for model inaccuracies and disturbances
        corrected_steering_angle_deg = target_steering_angle_deg + control_output
        
        # Final limit check
        corrected_steering_angle_deg = max(-self.max_steering_angle_deg, min(self.max_steering_angle_deg, corrected_steering_angle_deg))
        
        # Publish steering command (in degrees, as expected by the hardware interface)
        steering_msg = Float32()
        steering_msg.data = float(corrected_steering_angle_deg)
        self.steering_pub.publish(steering_msg)
        
        # Log information periodically
        if self.get_clock().now().nanoseconds % int(1e9) < 2e7:  # Log roughly every second
            self.get_logger().info(
                f'Error: {error:.1f}°/s, '
                f'Control output: {control_output:.1f}°, '
                f'PID sum: {self.error_sum:.1f}°·s, '
                f'PID derivative: {error_derivative:.1f}°/s², '
                f'Target angular: {self.target_angular_velocity_deg:.1f}°/s, '
                f'Current angular: {self.current_angular_velocity_deg:.1f}°/s, '
                f'Speed: {self.current_speed:.2f} m/s, '
                f'Base steering: {target_steering_angle_deg:.1f}°, '
                f'Corrected steering: {corrected_steering_angle_deg:.1f}°, '
                f'Current steering: {self.current_steering_angle_deg:.1f}°'
            )

def main(args=None):
    rclpy.init(args=args)
    node = LateralControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()