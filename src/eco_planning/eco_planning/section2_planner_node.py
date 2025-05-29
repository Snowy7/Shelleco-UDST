#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from eco_interfaces.msg import Obstacles, SystemState
from std_msgs.msg import Float64
import math

class Section2PlannerNode(Node):
    def __init__(self):
        super().__init__('section2_planner_node')
        
        # Parameters
        self.declare_parameter('safety_distance', 2.0)      # Distance to start avoiding
        self.declare_parameter('critical_distance', 0.4)    # Distance for max steering
        self.declare_parameter('max_steering', 0.8)         # Maximum steering output
        self.declare_parameter('min_steering', 0.1)         # Minimum steering when avoiding
        self.declare_parameter('steering_smoothness', 0.7)  # Smoothing factor (0-1)
        self.declare_parameter('max_speed', 0.3)            # m/s
        self.declare_parameter('min_speed', 0.1)            # m/s
        
        self.safety_distance = self.get_parameter('safety_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.max_steering = self.get_parameter('max_steering').value
        self.min_steering = self.get_parameter('min_steering').value
        self.steering_smoothness = self.get_parameter('steering_smoothness').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        
        # State variables
        self.current_state = SystemState.IDLE
        self.obstacles_data = None
        self.message_count = 0
        self.last_steering = 0.0
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/planning/cmd_vel',
            10)
        
        self.steering_pub = self.create_publisher(
            Float64, 
            '/obstacle_avoidance/steering', 
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
        
        # Timer for planning updates
        self.timer = self.create_timer(0.05, self.plan_update)
        
        self.get_logger().info('Section 2 planner node initialized')
        self.get_logger().info("Obstacle avoidance active for front-facing obstacles (-50° to +50°)")
        self.get_logger().info("Steering: -1.0 = Hard Left, +1.0 = Hard Right")
    
    def state_callback(self, msg):
        self.current_state = msg.state
    
    def obstacles_callback(self, msg):
        self.obstacles_data = msg
        self.message_count += 1
        
        # Process obstacles and generate steering commands
        raw_steering = self.calculate_avoidance_steering(msg.obstacles)
        
        # Apply smoothing to prevent jerky movements
        smoothed_steering = (self.steering_smoothness * self.last_steering + 
                           (1 - self.steering_smoothness) * raw_steering)
        
        # Clamp to valid range
        final_steering = max(-1.0, min(1.0, smoothed_steering))
        self.last_steering = final_steering
        
        # Publish steering command
        steering_msg = Float64()
        steering_msg.data = final_steering
        self.steering_pub.publish(steering_msg)
        
        # Debug logging
        if self.message_count % 15 == 0 or abs(final_steering) > 0.1:
            self.get_logger().info(
                f"Obstacles: {len(msg.obstacles)}, "
                f"Raw: {raw_steering:.3f}, Final: {final_steering:.3f}"
            )
    
    def plan_update(self):
        # Only plan if in Section 2 states
        active_states = [
            SystemState.SECTION2_OBSTACLE_AVOIDANCE,
            # Add other section 2 states as needed
        ]
        
        if self.current_state not in active_states:
            return
        
        if self.obstacles_data is None:
            self.get_logger().warn('Missing obstacle data')
            return
        
        # Calculate control commands based on obstacle avoidance
        cmd_vel = self.calculate_obstacle_avoidance_commands()
        
        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)
    
    def calculate_obstacle_avoidance_commands(self):
        cmd_vel = Twist()
        
        # Calculate steering based on obstacles
        raw_steering = self.calculate_avoidance_steering(self.obstacles_data.obstacles)
        
        # Apply smoothing
        smoothed_steering = (self.steering_smoothness * self.last_steering + 
                           (1 - self.steering_smoothness) * raw_steering)
        
        # Set angular velocity
        cmd_vel.angular.z = max(-1.0, min(1.0, smoothed_steering))
        
        # Set forward velocity based on obstacle proximity
        # Reduce speed when obstacles are detected
        if abs(smoothed_steering) > 0.1:
            # Reduce speed when steering to avoid obstacles
            speed_factor = 1.0 - min(1.0, abs(smoothed_steering))
            cmd_vel.linear.x = self.min_speed + (self.max_speed - self.min_speed) * speed_factor
        else:
            cmd_vel.linear.x = self.max_speed
        
        return cmd_vel

    def calculate_avoidance_steering(self, obstacles):
        """Calculate steering to avoid obstacles - gradual and proportional"""
        if not obstacles:
            return 0.0
        
        # Find the most threatening obstacle (closest and most forward)
        best_obstacle = None
        best_threat_score = 0.0
        
        for obstacle in obstacles:
            obs_x = obstacle.center.x
            obs_y = obstacle.center.y
            
            # Calculate distance to obstacle center
            distance = math.sqrt(obs_x**2 + obs_y**2)
            
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
        
        # If no threatening obstacle, return to center
        if best_obstacle is None:
            return 0.0
        
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
            steering_direction = -0.5  # Obstacle dead ahead, slight left
        
        # Calculate steering magnitude based on distance
        if distance <= self.critical_distance:
            # Very close - maximum steering
            steering_magnitude = self.max_steering
        elif distance >= self.safety_distance:
            # Far away - no steering
            steering_magnitude = 0.0
        else:
            # Proportional steering based on distance
            distance_ratio = (self.safety_distance - distance) / (self.safety_distance - self.critical_distance)
            steering_magnitude = self.min_steering + (self.max_steering - self.min_steering) * distance_ratio
        
        final_steering = steering_direction * steering_magnitude
        
        # Debug info
        side = "LEFT" if obs_y < 0 else "RIGHT"
        steer_dir = "RIGHT" if final_steering > 0 else "LEFT"
        self.get_logger().info(
            f"Obstacle on {side} at {distance:.2f}m -> "
            f"Steer {steer_dir} {abs(final_steering):.3f} "
            f"(threat: {best_threat_score:.2f})"
        )
        
        return final_steering

    def calculate_obstacle_force(self, obs_x, obs_y, distance, 
                               safety_dist, critical_dist, gain, forward_bias):
        """Calculate steering force for a single obstacle"""
        
        # Distance-based intensity (closer = stronger)
        if distance <= critical_dist:
            distance_factor = 1.0  # Maximum intensity
        else:
            # Linear falloff from critical to safety distance
            distance_factor = (safety_dist - distance) / (safety_dist - critical_dist)
            distance_factor = max(0.0, min(1.0, distance_factor))
        
        # Direction-based steering (which way to steer)
        # obs_y > 0 means obstacle is on RIGHT, so steer LEFT (negative)
        # obs_y < 0 means obstacle is on LEFT, so steer RIGHT (positive)
        if obs_y > 0:
            base_steering = -1.0  # Steer LEFT
        elif obs_y < 0:
            base_steering = +1.0  # Steer RIGHT
        else:
            # Obstacle directly ahead - choose based on x position or default
            base_steering = -0.5 if obs_x > 0 else 0.5
        
        # Forward bias - obstacles more directly ahead get stronger influence
        angle_to_obstacle = abs(math.atan2(obs_y, obs_x))
        forward_factor = 1.0 + forward_bias * math.exp(-2.0 * angle_to_obstacle)
        
        # Calculate final force
        steering_force = base_steering * distance_factor * forward_factor * gain
        
        return steering_force

    def get_obstacle_urgency(self, obs_x, obs_y, distance):
        """Calculate how urgent it is to avoid this obstacle"""
        # Obstacles directly ahead are more urgent
        angle_to_obstacle = abs(math.atan2(obs_y, obs_x))
        forward_urgency = math.exp(-angle_to_obstacle)
        
        # Closer obstacles are more urgent
        distance_urgency = 1.0 / max(distance, 0.1)
        
        return forward_urgency * distance_urgency

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