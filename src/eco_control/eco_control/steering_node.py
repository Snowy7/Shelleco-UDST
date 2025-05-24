#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
from adafruit_servokit import ServoKit

class SteeringInterfaceNode(Node):
    def __init__(self):
        super().__init__('steering_interface')
        
        # Parameters
        self.declare_parameter('center_angle_degrees', 180)
        self.declare_parameter('angle_range_degrees', 180)  # Range on each side of center
        self.declare_parameter('min_angle_degrees', 0)
        self.declare_parameter('max_angle_degrees', 360)
        self.declare_parameter('servo_channel', 0)  # Which servo channel to use
        self.declare_parameter('num_channels', 16)  # Number of channels on the board
        
        self.center_angle_degrees = self.get_parameter('center_angle_degrees').value
        self.angle_range_degrees = self.get_parameter('angle_range_degrees').value
        self.min_angle_degrees = self.get_parameter('min_angle_degrees').value
        self.max_angle_degrees = self.get_parameter('max_angle_degrees').value
        self.servo_channel = self.get_parameter('servo_channel').value
        self.num_channels = self.get_parameter('num_channels').value
        
        self.get_logger().info(f"Steering angle range: {self.min_angle_degrees}° to {self.max_angle_degrees}°")
        
        # State variables
        self.current_angle = self.center_angle_degrees
        self.target_angle = self.center_angle_degrees
        
        # Initialize servo controller
        try:
            self.servo_kit = ServoKit(channels=self.num_channels)
            self.servo_connected = True
            self.get_logger().info(f"Connected to servo controller with {self.num_channels} channels")
            
            # Center the servo immediately
            self.send_angle(self.center_angle_degrees)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize servo controller: {str(e)}")
            self.servo_connected = False
        
        # Subscribers
        self.steering_sub = self.create_subscription(
            Float32,
            '/control/steering_commands',
            self.steering_callback,
            10)
        
        # Publishers
        self.angle_pub = self.create_publisher(
            Float32,
            '/sensor/steering_angle',
            10)
        
        # Timer for periodic operations (10Hz is sufficient)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Steering interface node initialized')
    
    def timer_callback(self):
        """Main timer callback that handles sending commands and publishing feedback"""
        # Publish current angle for feedback
        self.publish_angle_feedback(self.current_angle)
        
        # If not connected, try to reconnect
        if not self.servo_connected:
            try:
                self.servo_kit = ServoKit(channels=self.num_channels)
                self.servo_connected = True
                self.get_logger().info("Reconnected to servo controller")
                self.send_angle(self.target_angle)
            except Exception as e:
                self.get_logger().debug(f"Failed to reconnect to servo controller: {str(e)}")
    
    def steering_callback(self, msg):
        # Received the steering in degrees
        servo_angle = msg.data
        
        # Ensure the angle is within the allowed range
        servo_angle = max(self.min_angle_degrees, min(self.max_angle_degrees, servo_angle))
        
        # Round to integer
        servo_angle = int(round(servo_angle))
        
        # Only process if the angle has changed
        if servo_angle != self.target_angle:
            self.target_angle = servo_angle
            
            # Send the angle immediately
            if self.servo_connected:
                self.send_angle(self.target_angle)
    
    def send_angle(self, angle_degrees):
        """Send angle to servo controller"""
        # Ensure angle is within bounds
        angle_degrees = max(self.min_angle_degrees, min(self.max_angle_degrees, angle_degrees))
        
        try:
            self.servo_kit.servo[self.servo_channel].angle = angle_degrees
            self.current_angle = angle_degrees
            self.get_logger().info(f"Set servo angle to {angle_degrees}°")
            return True
        except Exception as e:
            self.get_logger().error(f"Error setting servo angle: {str(e)}")
            self.servo_connected = False
            return False
    
    def publish_angle_feedback(self, angle_degrees):
        """Publish feedback in degrees"""
        feedback_msg = Float32()
        feedback_msg.data = float(angle_degrees)
        self.angle_pub.publish(feedback_msg)
    
    def destroy_node(self):
        """Clean up when node is shutting down"""
        # Center the steering before shutdown
        if self.servo_connected:
            try:
                self.send_angle(self.center_angle_degrees)
                self.get_logger().info("Steering centered before shutdown")
            except Exception as e:
                self.get_logger().error(f"Error during shutdown: {str(e)}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SteeringInterfaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
