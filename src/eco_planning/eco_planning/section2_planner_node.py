#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from eco_interfaces.msg import Obstacles, SystemState, LaneDetection
from std_msgs.msg import Float64
import math

class Section2PlannerNode(Node):
    def __init__(self):
        super().__init__('section2_planner_node')
        
        # Obstacle Avoidance Parameters
        self.declare_parameter('safety_distance', 2.0)      # Distance to start avoiding
        self.declare_parameter('critical_distance', 0.4)    # Distance for max steering
        self.declare_parameter('max_obstacle_steering', 0.8) # Maximum steering for obstacles
        self.declare_parameter('min_obstacle_steering', 0.1) # Minimum steering when avoiding
        self.declare_parameter('steering_smoothness', 0.7)  # Smoothing factor (0-1)
        
        # Lane Following Parameters (from Section 1)
        self.declare_parameter('lane_center_gain', 1.0)
        self.declare_parameter('lane_heading_gain', 2.5)
        
        # Speed Parameters
        self.declare_parameter('max_speed', 0.3)            # m/s
        self.declare_parameter('min_speed', 0.1)            # m/s
        self.declare_parameter('max_angular_velocity', 0.5) # rad/s
        
        # Control Blending Parameters
        self.declare_parameter('obstacle_priority_threshold', 1.0)  # Distance where obstacles take priority
        self.declare_parameter('lane_weight_base', 0.7)            # Base weight for lane following
        self.declare_parameter('obstacle_weight_base', 0.3)        # Base weight for obstacle avoidance
        
        # Get parameters
        self.safety_distance = self.get_parameter('safety_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.max_obstacle_steering = self.get_parameter('max_obstacle_steering').value
        self.min_obstacle_steering = self.get_parameter('min_obstacle_steering').value
        self.steering_smoothness = self.get_parameter('steering_smoothness').value
        self.lane_center_gain = self.get_parameter('lane_center_gain').value
        self.lane_heading_gain = self.get_parameter('lane_heading_gain').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.obstacle_priority_threshold = self.get_parameter('obstacle_priority_threshold').value
        self.lane_weight_base = self.get_parameter('lane_weight_base').value
        self.obstacle_weight_base = self.get_parameter('obstacle_weight_base').value
        
        # State variables
        self.current_state = SystemState.IDLE
        self.obstacles_data = None
        self.lane_data = None
        self.message_count = 0
        self.last_steering = 0.0
        self.last_lane_steering = 0.0
        self.last_obstacle_steering = 0.0
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/planning/cmd_vel',
            10)
        
        self.steering_pub = self.create_publisher(
            Float64, 
            '/section2/combined_steering', 
            10)
        
        self.debug_pub = self.create_publisher(
            Float64,
            '/section2/debug_info',
            10)
        
        # Subscribers
        self.state_sub = self.create_subscription(
            SystemState,
            '/state_machine/current_state',
            self.state_callback,
            10)
        
        self.obstacles_sub = self.create_subscription(
            Obstacles,
            '/obstacles',
            self.obstacles_callback,
            10)
        
        self.lane_sub = self.create_subscription(
            LaneDetection,
            '/perception/lane_detections',
            self.lane_callback,
            10)
        
        # Timer for planning updates
        self.timer = self.create_timer(0.05, self.plan_update)
        
        self.get_logger().info('Section 2 planner node initialized with lane following and obstacle avoidance')
        self.get_logger().info(f"Lane following gains - Center: {self.lane_center_gain}, Heading: {self.lane_heading_gain}")
        self.get_logger().info(f"Obstacle avoidance - Safety: {self.safety_distance}m, Critical: {self.critical_distance}m")
    
    def state_callback(self, msg):
        self.current_state = msg.state
    
    def obstacles_callback(self, msg):
        self.obstacles_data = msg
    
    def lane_callback(self, msg):
        self.lane_data = msg
    
    def plan_update(self):
        # Only plan if in Section 2 states
        active_states = [
            SystemState.SECTION2_OBSTACLE_AVOIDANCE,
            # Add other section 2 states as needed
        ]
        
        if self.current_state not in active_states:
            return
        
        # Check if we have necessary data
        if self.lane_data is None:
            self.get_logger().warn('Missing lane data', throttle_duration_sec=1.0)
            # If no lane data, fall back to pure obstacle avoidance
            if self.obstacles_data is not None:
                cmd_vel = self.calculate_pure_obstacle_avoidance()
                self.cmd_vel_pub.publish(cmd_vel)
            return
        
        # Calculate combined control commands
        cmd_vel = self.calculate_combined_commands()
        
        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)
    
    def calculate_combined_commands(self):
        """Calculate control commands combining lane following and obstacle avoidance"""
        cmd_vel = Twist()
        
        # 1. Calculate lane following steering (from Section 1 logic)
        lane_steering = self.calculate_lane_following_steering()
        
        # 2. Calculate obstacle avoidance steering
        obstacle_steering, closest_obstacle_distance = self.calculate_obstacle_avoidance_steering()
        
        # 3. Blend the two steering commands based on obstacle proximity
        final_steering, weights = self.blend_steering_commands(
            lane_steering, 
            obstacle_steering, 
            closest_obstacle_distance
        )
        
        # Apply smoothing to prevent jerky movements
        smoothed_steering = (self.steering_smoothness * self.last_steering + 
                           (1 - self.steering_smoothness) * final_steering)
        
        # Clamp to valid range
        smoothed_steering = max(-1.0, min(1.0, smoothed_steering))
        self.last_steering = smoothed_steering
        
        # Convert normalized steering to angular velocity
        cmd_vel.angular.z = smoothed_steering * self.max_angular_velocity
        
        # 4. Calculate forward velocity based on both lane curvature and obstacles
        cmd_vel.linear.x = self.calculate_speed(smoothed_steering, closest_obstacle_distance)
        
        # Publish debug steering info
        steering_msg = Float64()
        steering_msg.data = smoothed_steering
        self.steering_pub.publish(steering_msg)
        
        # Log periodically
        if self.message_count % 20 == 0:
            self.get_logger().info(
                f'Lane steering: {lane_steering:.2f}, '
                f'Obstacle steering: {obstacle_steering:.2f}, '
                f'Weights (L/O): {weights[0]:.2f}/{weights[1]:.2f}, '
                f'Final: {smoothed_steering:.2f}, '
                f'Speed: {cmd_vel.linear.x:.2f} m/s'
            )
        
        self.message_count += 1
        return cmd_vel
    
    def calculate_lane_following_steering(self):
        """Calculate steering based on lane detection (adapted from Section 1)"""
        if self.lane_data is None:
            return 0.0
        
        # Calculate normalized steering command (-1 to 1) based on lane center offset and heading error
        # Negative values mean steer left, positive values mean steer right
        normalized_steering = (
            -self.lane_center_gain * self.lane_data.lane_center_offset +
            -self.lane_heading_gain * self.lane_data.lane_heading_error
        )
        
        # Clamp the normalized steering to [-1, 1]
        normalized_steering = max(-1.0, min(1.0, normalized_steering))
        
        # Store for blending
        self.last_lane_steering = normalized_steering
        
        return normalized_steering
    
    def calculate_obstacle_avoidance_steering(self):
        """Calculate steering to avoid obstacles and return closest distance"""
        if not self.obstacles_data or not self.obstacles_data.obstacles:
            return 0.0, float('inf')
        
        # Find the most threatening obstacle
        best_obstacle = None
        best_threat_score = 0.0
        closest_distance = float('inf')
        
        for obstacle in self.obstacles_data.obstacles:
            obs_x = obstacle.center.x
            obs_y = obstacle.center.y
            
            # Calculate distance to obstacle center
            distance = math.sqrt(obs_x**2 + obs_y**2)
            closest_distance = min(closest_distance, distance)
            
            # Skip obstacles that are too far
            if distance > self.safety_distance:
                continue
            
            # Calculate threat score (closer and more forward = higher threat)
            distance_threat = max(0.0, (self.safety_distance - distance) / self.safety_distance)
            
            # Forward bias - obstacles ahead are more threatening
            angle = abs(math.atan2(obs_y, obs_x))
            forward_threat = math.exp(-angle * 2.0)  # Exponential decay from center
            
            threat_score = distance_threat * forward_threat
            
            if threat_score > best_threat_score:
                best_threat_score = threat_score
                best_obstacle = obstacle
        
        # If no threatening obstacle, return neutral steering
        if best_obstacle is None:
            return 0.0, closest_distance
        
        # Calculate steering for the most threatening obstacle
        obs_x = best_obstacle.center.x
        obs_y = best_obstacle.center.y
        distance = math.sqrt(obs_x**2 + obs_y**2)
        
        # Calculate steering direction
        if obs_y > 0:
            steering_direction = -1.0  # Obstacle on RIGHT, steer LEFT
        elif obs_y < 0:
            steering_direction = +1.0  # Obstacle on LEFT, steer RIGHT
        else:
            # Obstacle dead ahead - check lane data to decide direction
            if self.lane_data and abs(self.lane_data.lane_center_offset) > 0.1:
                # If we're already offset from lane center, continue in that direction
                steering_direction = -math.copysign(1.0, self.lane_data.lane_center_offset)
            else:
                steering_direction = -0.5  # Default to slight left
        
        # Calculate steering magnitude based on distance
        if distance <= self.critical_distance:
            steering_magnitude = self.max_obstacle_steering
        elif distance >= self.safety_distance:
            steering_magnitude = 0.0
        else:
            distance_ratio = (self.safety_distance - distance) / (self.safety_distance - self.critical_distance)
            steering_magnitude = self.min_obstacle_steering + (self.max_obstacle_steering - self.min_obstacle_steering) * distance_ratio
        
        obstacle_steering = steering_direction * steering_magnitude
        
        # Store for blending
        self.last_obstacle_steering = obstacle_steering
        
        return obstacle_steering, distance
    
    def blend_steering_commands(self, lane_steering, obstacle_steering, obstacle_distance):
        """Blend lane following and obstacle avoidance based on context"""
        # Calculate dynamic weights based on obstacle proximity
        if obstacle_distance < self.critical_distance:
            # Very close obstacle - almost pure obstacle avoidance
            obstacle_weight = 0.95
            lane_weight = 0.05
        elif obstacle_distance < self.obstacle_priority_threshold:
            # Obstacle in priority range - weighted blend
            proximity_factor = (self.obstacle_priority_threshold - obstacle_distance) / (self.obstacle_priority_threshold - self.critical_distance)
            obstacle_weight = self.obstacle_weight_base + (0.7 - self.obstacle_weight_base) * proximity_factor
            lane_weight = 1.0 - obstacle_weight
        else:
            # No close obstacles - prioritize lane following
            lane_weight = self.lane_weight_base
            obstacle_weight = self.obstacle_weight_base
        
        # Ensure weights sum to 1.0
        total_weight = lane_weight + obstacle_weight
        if total_weight > 0:
            lane_weight /= total_weight
            obstacle_weight /= total_weight
        
        # Blend the steering commands
        blended_steering = lane_weight * lane_steering + obstacle_weight * obstacle_steering
        
        # If obstacle avoidance and lane following conflict significantly, prioritize safety
        if abs(obstacle_steering) > 0.3 and math.copysign(1, obstacle_steering) != math.copysign(1, lane_steering):
            # They're steering in opposite directions - increase obstacle weight
            safety_factor = min(1.0, abs(obstacle_steering))
            blended_steering = (1 - safety_factor) * blended_steering + safety_factor * obstacle_steering
        
        return blended_steering, (lane_weight, obstacle_weight)
    
    def calculate_speed(self, steering_angle, obstacle_distance):
        """Calculate speed based on both steering angle and obstacle proximity"""
        # Base speed reduction from steering (curves)
        steering_factor = 1.0 - min(1.0, abs(steering_angle))
        
        # Additional speed reduction from obstacles
        if obstacle_distance < self.critical_distance:
            obstacle_factor = 0.3  # Very slow when obstacles are very close
        elif obstacle_distance < self.safety_distance:
            # Linear reduction based on distance
            obstacle_factor = 0.3 + 0.7 * (obstacle_distance - self.critical_distance) / (self.safety_distance - self.critical_distance)
        else:
            obstacle_factor = 1.0  # No reduction from obstacles
        
        # Combine both factors
        combined_factor = steering_factor * obstacle_factor
        
        # Calculate final speed
        speed = self.min_speed + (self.max_speed - self.min_speed) * combined_factor
        
        return speed
    
    def calculate_pure_obstacle_avoidance(self):
        """Fallback method for pure obstacle avoidance when no lane data is available"""
        cmd_vel = Twist()
        
        # Get obstacle avoidance steering
        obstacle_steering, closest_distance = self.calculate_obstacle_avoidance_steering()
        
        # Apply smoothing
        smoothed_steering = (self.steering_smoothness * self.last_steering + 
                           (1 - self.steering_smoothness) * obstacle_steering)
        smoothed_steering = max(-1.0, min(1.0, smoothed_steering))
        self.last_steering = smoothed_steering
        
        # Set angular velocity
        cmd_vel.angular.z = smoothed_steering * self.max_angular_velocity
        
        # Set speed based on obstacles only
        if closest_distance < self.critical_distance:
            cmd_vel.linear.x = self.min_speed * 0.5
        elif closest_distance < self.safety_distance:
            factor = (closest_distance - self.critical_distance) / (self.safety_distance - self.critical_distance)
            cmd_vel.linear.x = self.min_speed + (self.max_speed - self.min_speed) * factor * 0.7
        else:
            cmd_vel.linear.x = self.max_speed
        
        return cmd_vel

def main(args=None):
    rclpy.init(args=args)
    node = Section2PlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Section 2 Planner...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()