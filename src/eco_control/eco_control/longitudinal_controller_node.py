#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
import numpy as np

class LongitudinalControllerNode(Node):
    def __init__(self):
        super().__init__('longitudinal_controller_node')
        
        # Parameters
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.05)
        self.declare_parameter('max_throttle', 1.0)
        self.declare_parameter('min_throttle', -0.5)  # For braking
        
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_throttle = self.get_parameter('max_throttle').value
        self.min_throttle = self.get_parameter('min_throttle').value
        
        # PID controller state
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now()
        
        # Target and current speed
        self.target_speed = 0.0
        self.current_speed = 0.0
        
        # Publishers
        self.motor_pub = self.create_publisher(
            Float32,
            '/control/motor_speed',
            10)
        
        self.event_pub = self.create_publisher(
            Bool,
            '/control/event',
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
        
        # Timer for control updates
        self.timer = self.create_timer(0.02, self.control_update)
        
        self.get_logger().info('Longitudinal controller node initialized')
    
    def cmd_vel_callback(self, msg):
        self.target_speed = msg.linear.x
    
    def odometry_callback(self, msg):
        # Extract linear speed from odometry
        self.current_speed = np.sqrt(
            msg.twist.twist.linear.x**2 + 
            msg.twist.twist.linear.y**2
        )
    
    def control_update(self):
        """ current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0:
            return
        
        # Calculate error
        error = self.target_speed - self.current_speed
        
        # Update error sum (with anti-windup)
        self.error_sum += error * dt
        self.error_sum = max(-1.0, min(1.0, self.error_sum))
        
        # Calculate error derivative
        error_derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error
        
        # PID control
        control_output = (
            self.kp * error +
            self.ki * self.error_sum +
            self.kd * error_derivative
        )
        
        # Limit control output
        throttle = max(self.min_throttle, min(self.max_throttle, control_output))
        
        # Publish motor command
        motor_msg = Float32()
        motor_msg.data = float(throttle)
        self.motor_pub.publish(motor_msg)
        
        # Publish stop event if speed is close to zero
        if abs(self.current_speed) < 0.05 and abs(self.target_speed) < 0.05:
            event_msg = Bool()
            event_msg.data = True
            self.event_pub.publish(event_msg)# Calculate time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0:
            return
        
        # Calculate error
        error = self.target_speed - self.current_speed
        
        # Update error sum (with anti-windup)
        self.error_sum += error * dt
        self.error_sum = max(-1.0, min(1.0, self.error_sum))
        
        # Calculate error derivative
        error_derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error
        
        # PID control
        control_output = (
            self.kp * error +
            self.ki * self.error_sum +
            self.kd * error_derivative
        )
        
        # Limit control output
        throttle = max(self.min_throttle, min(self.max_throttle, control_output))
        
        # Publish motor command
        motor_msg = Float32()
        motor_msg.data = float(throttle)
        self.motor_pub.publish(motor_msg)
        
        # Publish stop event if speed is close to zero
        if abs(self.current_speed) < 0.05 and abs(self.target_speed) < 0.05:
            event_msg = Bool()
            event_msg.data = True
            self.event_pub.publish(event_msg) """
            
        # for now send the value as it is
        motor_msg = Float32()
        motor_msg.data = float(self.target_speed)
        self.motor_pub.publish(motor_msg)
        # Publish stop event if speed is close to zero
        if abs(self.current_speed) < 0.05 and abs(self.target_speed) < 0.05:
            event_msg = Bool()
            event_msg.data = True
            self.event_pub.publish(event_msg)
        else:
            event_msg = Bool()
            event_msg.data = False
            self.event_pub.publish(event_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LongitudinalControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
