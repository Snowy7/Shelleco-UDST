#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from eco_interfaces.msg import LaneDetection, SystemState

class Section1PlannerNode(Node):
    def __init__(self):
        super().__init__('section1_planner_node')
        
        # Parameters
        self.declare_parameter('max_speed', 0.5)  # m/s
        self.declare_parameter('min_speed', 0.1)  # m/s
        self.declare_parameter('max_angular_velocity', 0.5)  # rad/s
        self.declare_parameter('lane_center_gain', 1.0)
        self.declare_parameter('lane_heading_gain', 1.5)
        
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.lane_center_gain = self.get_parameter('lane_center_gain').value
        self.lane_heading_gain = self.get_parameter('lane_heading_gain').value
        
        # State variables
        self.current_state = SystemState.IDLE
        self.lane_data = None
        self.odometry = None
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/planning/cmd_vel',
            10)
        
        # Subscribers
        self.state_sub = self.create_subscription(
            SystemState,
            '/state_machine/current_state',
            self.state_callback,
            10)
        
        self.lane_sub = self.create_subscription(
            LaneDetection,
            '/perception/lane_detections',
            self.lane_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odometry_callback,
            10)
        
        # Timer for planning updates
        self.timer = self.create_timer(0.05, self.plan_update)
        
        self.get_logger().info('Section 1 planner node initialized')
    
    def state_callback(self, msg):
        self.current_state = msg.state
    
    def lane_callback(self, msg):
        self.lane_data = msg
    
    def odometry_callback(self, msg):
        self.odometry = msg
    
    def plan_update(self):
        # Only plan if in Section 1 or approaching stop sign in Section 1
        active_states = [
            SystemState.SECTION1_LANE_FOLLOWING,
            SystemState.APPROACHING_STOP_SIGN_SECTION1
        ]
        
        if self.current_state not in active_states:
            return
        
        if self.lane_data is None:
            self.get_logger().warn('Missing lane data')
            return
        
        # Calculate control commands based on lane detection
        cmd_vel = self.calculate_lane_following_commands()
        
        # Adjust speed if approaching stop sign
        if self.current_state == SystemState.APPROACHING_STOP_SIGN_SECTION1:
            # Reduce speed as we approach the stop sign
            cmd_vel.linear.x = min(cmd_vel.linear.x, self.min_speed)
        
        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)
    
    def calculate_lane_following_commands(self):
        cmd_vel = Twist()
        
        # Calculate normalized steering command (-1 to 1) based on lane center offset and heading error
        # Negative values mean steer left, positive values mean steer right
        normalized_steering = (
            -self.lane_center_gain * self.lane_data.lane_center_offset +
            -self.lane_heading_gain * self.lane_data.lane_heading_error
        )
        
        # Clamp the normalized steering to [-1, 1]
        normalized_steering = max(-1.0, min(1.0, normalized_steering))
        
        # Convert normalized steering to angular velocity for ROS
        # This maps [-1, 1] to [-max_angular_velocity, max_angular_velocity] in radians/sec
        cmd_vel.angular.z = normalized_steering
        
        # Set forward velocity based on curvature
        # Reduce speed in curves (when normalized_steering is far from 0)
        curvature_factor = 1.0 - min(1.0, abs(normalized_steering))
        cmd_vel.linear.x = self.min_speed + (self.max_speed - self.min_speed) * curvature_factor
        
        # For debugging, calculate the equivalent steering angle in degrees
        # Maps [-1, 1] to [90-30, 90+30] degrees (60° to 120°)
        steering_angle_deg = 90.0 + (normalized_steering * 70.0)
        
        # Log periodically
        if self.get_clock().now().nanoseconds % 1e9 < 1e6:
            self.get_logger().info(
                f'Lane offset: {self.lane_data.lane_center_offset:.2f}, '
                f'Heading error: {self.lane_data.lane_heading_error:.2f}, '
                f'Normalized steering: {normalized_steering:.2f}, '
                f'Angular velocity: {cmd_vel.angular.z:.2f} rad/s, '
                f'Equivalent angle: {steering_angle_deg:.1f}°, '
                f'Speed: {cmd_vel.linear.x:.2f} m/s'
            )
        return cmd_vel



def main(args=None):
    rclpy.init(args=args)
    node = Section1PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
