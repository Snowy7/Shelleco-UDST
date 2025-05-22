#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import math
import time

class SteeringInterfaceNode(Node):
    def __init__(self):
        super().__init__('steering_interface')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyTHS1')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('center_angle_degrees', 90)
        self.declare_parameter('angle_range_degrees', 35)  # Range on each side of center
        self.declare_parameter('max_steering_angle_deg', 180)  # Max steering angle in radians
        self.declare_parameter('command_repeat_count', 3)  # Number of times to repeat a command
        self.declare_parameter('reconnect_delay', 1.0)  # Delay between reconnection attempts (seconds)
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.center_angle_degrees = self.get_parameter('center_angle_degrees').value
        self.angle_range_degrees = self.get_parameter('angle_range_degrees').value
        self.max_steering_angle_deg = self.get_parameter('max_steering_angle_deg').value
        self.command_repeat_count = self.get_parameter('command_repeat_count').value
        self.reconnect_delay = self.get_parameter('reconnect_delay').value
        
        # Calculate min and max angles based on center and range
        self.min_angle_degrees = 0
        
        self.get_logger().info(f"Steering angle range: {self.min_angle_degrees}° to {self.max_steering_angle_deg}°")
        
        # State variables
        self.ser = None
        self.serial_connected = False
        self.current_angle = self.center_angle_degrees
        self.target_angle = self.center_angle_degrees
        self.last_reconnect_attempt = self.get_clock().now()
        self.command_repeats_left = 0
        
        # Initialize serial connection
        self.connect_serial()
        
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
        
    def connect_serial(self):
        """Attempt to connect to the serial port"""
        if self.ser is not None:
            try:
                self.ser.close()
            except:
                pass
            self.ser = None
        
        try:
            self.ser = serial.Serial(
                self.serial_port, 
                self.baud_rate, 
                timeout=0.1  # Short timeout for non-blocking reads
            )
            self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud")
            self.serial_connected = True
            
            # Flush any pending data
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            # Send center command immediately after connecting
            self.send_angle_raw(self.center_angle_degrees)
            
            # Set up repeats for initial centering
            self.command_repeats_left = self.command_repeat_count - 1
            
            return True
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {str(e)}")
            self.serial_connected = False
            return False
    
    def timer_callback(self):
        """Main timer callback that handles sending commands, reading feedback, and connection health"""
        current_time = self.get_clock().now()
        
        # Check if we need to reconnect
        if not self.serial_connected:
            # Only attempt reconnection after delay
            if (current_time - self.last_reconnect_attempt).nanoseconds / 1e9 >= self.reconnect_delay:
                self.last_reconnect_attempt = current_time
                self.connect_serial()
            return
        
        # Send command repeats if needed
        if self.command_repeats_left > 0:
            self.send_angle_raw(self.target_angle)
            self.command_repeats_left -= 1
        
        # Read feedback
        # self.read_feedback()
        self.publish_angle_feedback(self.current_angle)
    
    def steering_callback(self, msg):
        # recieved the steering in degrees
        servo_angle = msg.data
        
        # Ensure the angle is within the allowed range
        servo_angle = max(self.min_angle_degrees, min(self.max_steering_angle_deg, servo_angle))
        
        # Round to integer
        servo_angle = int(round(servo_angle))
        
        # self.get_logger().info(f"Received steering command: {servo_angle}°, current angle: {self.target_angle}°, command repeats left: {self.command_repeats_left}, serial connected: {self.serial_connected}")
        
        # Only process if the angle has changed
        if servo_angle != self.target_angle:
            self.target_angle = servo_angle
            
            # Send the angle immediately
            if self.serial_connected:
                self.send_angle(self.target_angle)
                
                # Set up repeats
                self.command_repeats_left = self.command_repeat_count - 1
    
    def send_angle(self, angle_degrees):
        """Send angle with validation and update current angle if successful"""
        # Ensure angle is within bounds
        angle_degrees = max(self.min_angle_degrees, min(self.max_steering_angle_deg, angle_degrees))
        
        self.get_logger().info(f"Sending angle: {angle_degrees}°")
        success = self.send_angle_raw(angle_degrees)
        if success:
            self.current_angle = angle_degrees
            
            # Publish the current steering angle (in degrees) for feedback
            # self.publish_angle_feedback(angle_degrees)
    
    def send_angle_raw(self, angle_degrees):
        """Send angle to Arduino without validation"""
        if not self.serial_connected:
            return False
        
        try:
            # Format with newline and flush to ensure immediate sending
            command = f"{int(angle_degrees)}\n"
            self.ser.write(command.encode())
            self.ser.flush()
            
            self.get_logger().info(f"Sent angle: {angle_degrees} degrees")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error sending angle: {str(e)}")
            self.serial_connected = False
            return False
    
    def read_feedback(self):
        """Read feedback from Arduino"""
        if not self.serial_connected:
            return
        
        try:
            # Check if data is available
            if self.ser.in_waiting > 0:
                # Read a line with timeout
                line = self.ser.readline().decode('utf-8', errors='ignore').rstrip()
                
                if line:
                    self.get_logger().debug(f"Received from Arduino: {line}")
                    
                    # Try to parse as angle feedback
                    try:
                        angle_degrees = float(line)
                        
                        # Validate the angle more strictly
                        if (angle_degrees >= self.min_angle_degrees - 10 and 
                            angle_degrees <= self.max_steering_angle_deg + 10):
                            
                            # Update current angle
                            self.current_angle = angle_degrees
                            
                            # Publish feedback
                            self.publish_angle_feedback(angle_degrees)
                        else:
                            self.get_logger().warn(
                                f"Received angle out of expected range: {angle_degrees}°. "
                                f"Expected range: {self.min_angle_degrees}° to {self.max_steering_angle_deg}°"
                            )
                    except ValueError:
                        # Not a valid angle, just log if it's not empty
                        if line.strip():
                            self.get_logger().debug(f"Non-numeric feedback: {line}")
            
        except Exception as e:
            self.get_logger().error(f"Error reading feedback: {str(e)}")
            self.serial_connected = False

    
    def publish_angle_feedback(self, angle_degrees):
        """Publish feedback in degrees"""
        feedback_msg = Float32()
        feedback_msg.data = float(angle_degrees)  # Publish in degrees
        self.angle_pub.publish(feedback_msg)
    
    def destroy_node(self):
        """Clean up when node is shutting down"""
        # Center the steering and close serial connection
        if self.serial_connected:
            try:
                # Send center command multiple times to ensure it's received
                for _ in range(3):
                    self.send_angle_raw(self.center_angle_degrees)
                    time.sleep(0.05)
                
                self.ser.close()
                self.get_logger().info("Steering centered and serial port closed")
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
