#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from eco_interfaces.msg import StopSign, ParkingArea, SystemState

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        
        # Initialize state
        self.current_state = SystemState.IDLE
        self.stop_sign_detected = False
        self.stop_sign_distance = 0.0
        self.parking_area_detected = False
        self.current_position = None
        self.stop_start_time = None
        self.control_mode = "manual"  # Default to manual control
        self.emergency_stopped = False
        
        # Parameters
        self.declare_parameter('stop_duration', 3.0)  # seconds
        self.stop_duration = self.get_parameter('stop_duration').value
        
        # Publishers
        self.state_pub = self.create_publisher(
            SystemState,
            '/state_machine/current_state',
            10)
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        # Subscribers
        self.stop_sign_sub = self.create_subscription(
            StopSign,
            '/perception/stop_sign_detected',
            self.stop_sign_callback,
            10)
        
        self.parking_area_sub = self.create_subscription(
            ParkingArea,
            '/perception/parking_area_detected',
            self.parking_area_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odometry_callback,
            10)
        
        # Control mode subscriber
        self.control_mode_sub = self.create_subscription(
            String,
            '/control_mode',
            self.control_mode_callback,
            10)
        
        # Manual control subscriber
        self.manual_control_sub = self.create_subscription(
            Twist,
            '/manual_control',
            self.manual_control_callback,
            10)
        
        # Emergency stop subscribers
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_stop_callback,
            10)
        
        self.reset_emergency_sub = self.create_subscription(
            Bool,
            '/reset_emergency',
            self.reset_emergency_callback,
            10)
        
        # Timer for state machine updates
        self.timer = self.create_timer(0.1, self.update_state)
        
        self.get_logger().info('State machine node initialized in IDLE state')
    
    def stop_sign_callback(self, msg):
        self.stop_sign_detected = msg.detected
        self.stop_sign_distance = msg.distance
    
    def parking_area_callback(self, msg):
        self.parking_area_detected = msg.detected
    
    def odometry_callback(self, msg):
        self.current_position = msg.pose.pose
    
    def control_mode_callback(self, msg):
        new_mode = msg.data
        if new_mode != self.control_mode:
            self.get_logger().info(f'Control mode changed: {self.control_mode} -> {new_mode}')
            self.control_mode = new_mode
            
            # If switching to manual, reset to IDLE state
            if new_mode == "manual" and self.current_state != SystemState.IDLE:
                self.transition_to(SystemState.IDLE)
                self.get_logger().info('Switched to manual control, resetting to IDLE state')
    
    def manual_control_callback(self, msg):
        # Only forward manual control commands if in manual mode and not emergency stopped
        if self.control_mode == "manual" and not self.emergency_stopped:
            self.cmd_vel_pub.publish(msg)
    
    def emergency_stop_callback(self, msg):
        if msg.data and not self.emergency_stopped:
            self.emergency_stopped = True
            self.get_logger().warn('Emergency stop activated')
            
            # Send stop command
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
    
    def reset_emergency_callback(self, msg):
        if msg.data and self.emergency_stopped:
            self.emergency_stopped = False
            self.get_logger().info('Emergency stop reset')
            
            # If in autonomous mode, restart from IDLE
            if self.control_mode == "autonomous":
                self.transition_to(SystemState.IDLE)
    
    def update_state(self):
        # Only update state if in autonomous mode and not emergency stopped
        if self.control_mode != "autonomous" or self.emergency_stopped:
            return
        
        # State transition logic
        if self.current_state == SystemState.IDLE:
            # Transition to lane following when ready
            self.transition_to(SystemState.SECTION1_LANE_FOLLOWING)
        
        elif self.current_state == SystemState.SECTION1_LANE_FOLLOWING:
            # Check for stop sign
            if self.stop_sign_detected and self.stop_sign_distance < 2.0:
                self.transition_to(SystemState.APPROACHING_STOP_SIGN_SECTION1)
        
        elif self.current_state == SystemState.APPROACHING_STOP_SIGN_SECTION1:
            # Check if close enough to stop
            if self.stop_sign_distance < 0.5:
                self.stop_start_time = self.get_clock().now()
                self.transition_to(SystemState.STOPPED_SECTION1)
        
        elif self.current_state == SystemState.STOPPED_SECTION1:
            # Check if stopped for long enough
            current_time = self.get_clock().now()
            if (current_time - self.stop_start_time).nanoseconds / 1e9 >= self.stop_duration:
                self.transition_to(SystemState.SECTION2_OBSTACLE_AVOIDANCE)
        
        elif self.current_state == SystemState.SECTION2_OBSTACLE_AVOIDANCE:
            # Check for stop sign
            if self.stop_sign_detected and self.stop_sign_distance < 2.0:
                self.transition_to(SystemState.APPROACHING_STOP_SIGN_SECTION2)
        
        elif self.current_state == SystemState.APPROACHING_STOP_SIGN_SECTION2:
            # Check if close enough to stop
            if self.stop_sign_distance < 0.5:
                self.stop_start_time = self.get_clock().now()
                self.transition_to(SystemState.STOPPED_SECTION2)
        
        elif self.current_state == SystemState.STOPPED_SECTION2:
            # Check if stopped for long enough
            current_time = self.get_clock().now()
            if (current_time - self.stop_start_time).nanoseconds / 1e9 >= self.stop_duration:
                self.transition_to(SystemState.SECTION3_PARKING)
        
        elif self.current_state == SystemState.SECTION3_PARKING:
            # Check if parking area detected and close
            if self.parking_area_detected:
                # Logic to determine if we're in the parking spot would go here
                # For now, just transition after a delay
                self.get_logger().info('Parking area detected, executing parking maneuver')
                self.transition_to(SystemState.PARKED)
        
        # Publish current state
        self.publish_state()
    
    def transition_to(self, new_state):
        if new_state != self.current_state:
            self.get_logger().info(f'State transition: {self.current_state} -> {new_state}')
            self.current_state = new_state
    
    def publish_state(self):
        msg = SystemState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.current_state
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
