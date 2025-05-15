#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from eco_interfaces.msg import SystemState, StopSign, Obstacles, LaneDetection
import threading
import json
import base64
import numpy as np
import cv2
from cv_bridge import CvBridge
from flask import Flask, request, jsonify, render_template_string

class EcoMarathonDashboardNode(Node):
    def __init__(self):
        super().__init__('eco_marathon_dashboard_node')
        
        # Initialize CV bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # Publishers
        self.manual_pub = self.create_publisher(
            Twist,
            '/manual_control',
            10
        )
        
        self.mode_pub = self.create_publisher(
            String,
            '/control_mode',
            10
        )
        
        self.estop_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            10
        )
        
        self.reset_estop_pub = self.create_publisher(
            Bool,
            '/reset_emergency',
            10
        )
        
        # Subscribers
        # State machine state
        self.state_sub = self.create_subscription(
            SystemState,
            '/state_machine/current_state',
            self.state_callback,
            10
        )
        
        # Lane detection
        self.lane_detection_sub = self.create_subscription(
            LaneDetection,
            '/perception/lane_detections',
            self.lane_detection_callback,
            10
        )
        
        # Lane images (from existing system)
        self.lane_mask_sub = self.create_subscription(
            Image,
            '/lane_detection/lane_mask',
            self.lane_mask_callback,
            10
        )
        
        self.debug_image_sub = self.create_subscription(
            Image,
            '/lane_detection/debug_image',
            self.debug_image_callback,
            10
        )
        
        # Stop sign detection
        self.stop_sign_sub = self.create_subscription(
            StopSign,
            '/perception/stop_sign_detected',
            self.stop_sign_callback,
            10
        )
        
        # Obstacle detection
        self.obstacles_sub = self.create_subscription(
            Obstacles,
            '/perception/obstacles',
            self.obstacles_callback,
            10
        )
        
        # Odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odometry_callback,
            10
        )
        
        # LiDAR scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # System data
        self.system_data = {
            "mode": "manual",
            "emergency_stopped": False,
            "state_machine": {
                "current_state": 0,
                "state_name": "IDLE"
            },
            "lane_detection": {
                "center_offset": 0.0,
                "heading_error": 0.0,
                "left_line_detected": False,
                "right_line_detected": False
            },
            "stop_sign": {
                "detected": False,
                "distance": 0.0,
                "position": {"x": 0.0, "y": 0.0, "z": 0.0}
            },
            "obstacles": {
                "count": 0,
                "nearest_distance": 0.0,
                "obstacles": []
            },
            "odometry": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                "linear_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0}
            },
            "lidar": {
                "min_range": 0.0,
                "max_range": 0.0,
                "front_distance": 0.0,
                "left_distance": 0.0,
                "right_distance": 0.0
            },
            "control": {
                "linear_speed": 0.0,
                "angular_speed": 0.0
            }
        }
        
        # Images
        self.lane_mask_image = None
        self.debug_image = None
        
        # Cooldown mechanism for mode switching
        self.last_mode_switch_time = 0
        self.cooldown_period = 1.0  # seconds
        
        self.get_logger().info('Eco Marathon Dashboard Node initialized')
    
    def state_callback(self, msg):
        """Handle state machine state updates"""
        state_names = [
            "IDLE", "SECTION1_LANE_FOLLOWING", "APPROACHING_STOP_SIGN_SECTION1",
            "STOPPED_SECTION1", "SECTION2_OBSTACLE_AVOIDANCE", "APPROACHING_STOP_SIGN_SECTION2",
            "STOPPED_SECTION2", "SECTION3_PARKING", "PARKED", "EMERGENCY_STOP"
        ]
        
        state_num = msg.state
        state_name = state_names[state_num] if state_num < len(state_names) else f"UNKNOWN_{state_num}"
        
        self.system_data["state_machine"]["current_state"] = state_num
        self.system_data["state_machine"]["state_name"] = state_name
    
    def lane_detection_callback(self, msg):
        """Handle lane detection updates"""
        self.system_data["lane_detection"]["center_offset"] = msg.lane_center_offset
        self.system_data["lane_detection"]["heading_error"] = msg.lane_heading_error
        self.system_data["lane_detection"]["left_line_detected"] = len(msg.left_line_x) > 0
        self.system_data["lane_detection"]["right_line_detected"] = len(msg.right_line_x) > 0
    
    def lane_mask_callback(self, msg):
        """Handle lane mask image updates"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            # Convert to RGB for proper display
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            self.lane_mask_image = base64.b64encode(buffer).decode('utf-8')
        except Exception as e:
            self.get_logger().error(f'Error processing lane mask image: {e}')
    
    def debug_image_callback(self, msg):
        """Handle debug image updates"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            self.debug_image = base64.b64encode(buffer).decode('utf-8')
        except Exception as e:
            self.get_logger().error(f'Error processing debug image: {e}')
    
    def stop_sign_callback(self, msg):
        """Handle stop sign detection updates"""
        self.system_data["stop_sign"]["detected"] = msg.detected
        self.system_data["stop_sign"]["distance"] = msg.distance
        self.system_data["stop_sign"]["position"] = {
            "x": msg.position.x,
            "y": msg.position.y,
            "z": msg.position.z
        }
    
    def obstacles_callback(self, msg):
        """Handle obstacle detection updates"""
        obstacles = []
        min_distance = float('inf')
        
        for obstacle in msg.obstacles:
            # Calculate distance to obstacle center
            distance = np.sqrt(obstacle.center.x**2 + obstacle.center.y**2)
            
            if distance < min_distance:
                min_distance = distance
            
            obstacles.append({
                "center": {
                    "x": obstacle.center.x,
                    "y": obstacle.center.y,
                    "z": obstacle.center.z
                },
                "radius": obstacle.radius,
                "distance": distance
            })
        
        self.system_data["obstacles"]["count"] = len(obstacles)
        self.system_data["obstacles"]["nearest_distance"] = min_distance if obstacles else 0.0
        self.system_data["obstacles"]["obstacles"] = obstacles
    
    def odometry_callback(self, msg):
        """Handle odometry updates"""
        # Extract position
        position = msg.pose.pose.position
        
        # Extract orientation (convert quaternion to Euler angles)
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w)
        
        # Extract velocities
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        
        # Update system data
        self.system_data["odometry"]["position"] = {
            "x": position.x, "y": position.y, "z": position.z
        }
        self.system_data["odometry"]["orientation"] = {
            "roll": roll, "pitch": pitch, "yaw": yaw
        }
        self.system_data["odometry"]["linear_velocity"] = {
            "x": linear.x, "y": linear.y, "z": linear.z
        }
        self.system_data["odometry"]["angular_velocity"] = {
            "x": angular.x, "y": angular.y, "z": angular.z
        }
        
        # Update control data (current speeds)
        self.system_data["control"]["linear_speed"] = np.sqrt(linear.x**2 + linear.y**2)
        self.system_data["control"]["angular_speed"] = angular.z
    
    def scan_callback(self, msg):
        """Handle LiDAR scan updates"""
        # Extract basic scan info
        self.system_data["lidar"]["min_range"] = msg.range_min
        self.system_data["lidar"]["max_range"] = msg.range_max
        
        # Calculate distances in different directions
        num_points = len(msg.ranges)
        
        # Front is at 0 degrees (middle of the scan)
        front_idx = num_points // 2
        # Left is at 90 degrees (1/4 of the scan)
        left_idx = front_idx + num_points // 4
        if left_idx >= num_points:
            left_idx -= num_points
        # Right is at -90 degrees (3/4 of the scan)
        right_idx = front_idx - num_points // 4
        if right_idx < 0:
            right_idx += num_points
        
        # Get distances, handling invalid measurements
        front_distance = msg.ranges[front_idx] if msg.ranges[front_idx] < msg.range_max else msg.range_max
        left_distance = msg.ranges[left_idx] if msg.ranges[left_idx] < msg.range_max else msg.range_max
        right_distance = msg.ranges[right_idx] if msg.ranges[right_idx] < msg.range_max else msg.range_max
        
        self.system_data["lidar"]["front_distance"] = front_distance
        self.system_data["lidar"]["left_distance"] = left_distance
        self.system_data["lidar"]["right_distance"] = right_distance
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def send_movement(self, linear_x, angular_z):
        """Send movement command to the vehicle"""
        twist_msg = Twist()
        twist_msg.linear.x = float(linear_x)
        twist_msg.angular.z = float(angular_z)
        self.manual_pub.publish(twist_msg)
        self.get_logger().info(f'Movement command: linear={linear_x}, angular={angular_z}')
        return True
    
    def set_mode(self, mode):
        """Set the control mode (manual/autonomous)"""
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.last_mode_switch_time < self.cooldown_period:
            self.get_logger().warn('Mode switch cooldown in effect')
            return False
        
        self.last_mode_switch_time = current_time
        mode_msg = String()
        mode_msg.data = mode
        self.mode_pub.publish(mode_msg)
        self.system_data["mode"] = mode
        self.get_logger().info(f'Mode change request: {mode}')
        return True
    
    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        estop_msg = Bool()
        estop_msg.data = True
        self.estop_pub.publish(estop_msg)
        self.system_data["emergency_stopped"] = True
        self.get_logger().info('Emergency stop triggered')
        return True
    
    def reset_emergency_stop(self):
        """Reset emergency stop state"""
        reset_msg = Bool()
        reset_msg.data = True
        self.reset_estop_pub.publish(reset_msg)
        self.system_data["emergency_stopped"] = False
        self.get_logger().info('Emergency stop reset requested')
        return True
    
    def get_system_data(self):
        """Get the current system data"""
        return self.system_data
    
    def get_visualization_data(self):
        """Get visualization data (images, etc.)"""
        return {
            "lane_mask": self.lane_mask_image,
            "debug_image": self.debug_image
        }

# Flask application for web server
app = Flask(__name__)
ros_node = None  # Will be initialized in ROS2 thread

# HTML template for the web interface
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Shell Eco-marathon Autonomous Vehicle Dashboard</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <style>
        :root {
            --bg-color: #121212;
            --card-bg: #1e1e1e;
            --card-header: #2a2a2a;
            --text-color: #e0e0e0;
            --accent-color: #4d84ff;
            --danger-color: #ff4d4d;
            --success-color: #4dff88;
            --warning-color: #ffcc4d;
            --disabled-color: #7a7a7a;
            --section1-color: #4d84ff;
            --section2-color: #ffcc4d;
            --section3-color: #4dff88;
        }
        
        body {
            font-family: 'Segoe UI', Roboto, Arial, sans-serif;
            margin: 0;
            padding: 0;
            background-color: var(--bg-color);
            color: var(--text-color);
        }
        
        .container {
            max-width: 1600px;
            margin: 0 auto;
            padding: 20px;
            display: grid;
            grid-template-columns: 1fr;
            gap: 20px;
        }
        
        @media (min-width: 1024px) {
            .container {
                grid-template-columns: 1fr 1fr 1fr;
            }
            
            .full-width {
                grid-column: 1 / -1;
            }
            
            .two-thirds {
                grid-column: span 2;
            }
        }
        
        .card {
            background-color: var(--card-bg);
            border-radius: 10px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.3);
            overflow: hidden;
        }
        
        .card-header {
            background-color: var(--card-header);
            padding: 15px 20px;
            font-weight: bold;
            font-size: 18px;
            color: var(--accent-color);
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        
        .card-content {
            padding: 20px;
        }
        
        h1, h2, h3 {
            margin-top: 0;
            color: var(--accent-color);
        }
        
        .mode-toggle {
            display: flex;
            justify-content: space-between;
            margin-bottom: 20px;
        }
        
        .mode-btn {
            flex: 1;
            padding: 15px;
            margin: 0 5px;
            border: none;
            border-radius: 5px;
            font-size: 16px;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.3s ease;
        }
        
        .mode-btn.active {
            box-shadow: 0 0 10px var(--accent-color);
        }
        
        .mode-btn.manual {
            background-color: var(--accent-color);
            color: white;
        }
        
        .mode-btn.manual.active {
            background-color: #3a6ac5;
        }
        
        .mode-btn.autonomous {
            background-color: var(--warning-color);
            color: #333;
        }
        
        .mode-btn.autonomous.active {
            background-color: #d9ad42;
        }
        
        .joystick-container {
            position: relative;
            width: 280px;
            height: 280px;
            background-color: rgba(255, 255, 255, 0.05);
            border-radius: 50%;
            margin: 0 auto;
            border: 2px solid rgba(255, 255, 255, 0.1);
            touch-action: none;
        }
        
        .joystick-base {
            position: absolute;
            width: 280px;
            height: 280px;
            border-radius: 50%;
            display: flex;
            justify-content: center;
            align-items: center;
        }
        
        .joystick-thumb {
            position: absolute;
            width: 80px;
            height: 80px;
            background-color: var(--accent-color);
            border-radius: 50%;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            box-shadow: 0 2px 8px rgba(0, 0, 0, 0.5);
        }
        
        .status-indicators {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
            margin-top: 20px;
        }
        
        .indicator {
            padding: 10px;
            border-radius: 5px;
            background-color: rgba(255, 255, 255, 0.05);
            text-align: center;
        }
        
        .indicator-value {
            font-size: 24px;
            font-weight: bold;
            margin: 5px 0;
        }
        
        .indicator-label {
            font-size: 12px;
            opacity: 0.7;
        }
        
        .emergency-stop {
            display: block;
            width: 100%;
            padding: 20px;
            margin-top: 20px;
            background-color: var(--danger-color);
            color: white;
            border: none;
            border-radius: 10px;
            font-size: 18px;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.3s ease;
        }
        
        .emergency-stop:active {
            background-color: #c53a3a;
            transform: scale(0.98);
        }
        
        .reset-emergency {
            display: block;
            width: 100%;
            padding: 15px;
            margin-top: 10px;
            background-color: var(--warning-color);
            color: #333;
            border: none;
            border-radius: 10px;
            font-size: 16px;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.3s ease;
        }
        
        .reset-emergency:active {
            background-color: #d9ad42;
            transform: scale(0.98);
        }
        
        .system-status {
            display: flex;
            align-items: center;
            margin-top: 20px;
            padding: 10px;
            border-radius: 5px;
            background-color: rgba(255, 255, 255, 0.05);
        }
        
        .status-indicator {
            width: 15px;
            height: 15px;
            border-radius: 50%;
            margin-right: 10px;
        }
        
        .status-ok {
            background-color: var(--success-color);
        }
        
        .status-warning {
            background-color: var(--warning-color);
        }
        
        .status-error {
            background-color: var(--danger-color);
        }
        
        .status-text {
            flex-grow: 1;
        }
        
        .log-container {
            height: 150px;
            overflow-y: auto;
            background-color: rgba(0, 0, 0, 0.3);
            border-radius: 5px;
            padding: 10px;
            font-family: monospace;
            font-size: 12px;
        }
        
        .log-entry {
            margin-bottom: 5px;
            line-height: 1.3;
        }
        
        .log-time {
            color: var(--accent-color);
        }
        
        .disabled {
            opacity: 0.5;
            pointer-events: none;
        }
        
        .camera-view {
            margin-top: 20px;
            text-align: center;
        }
        
        .camera-image {
            max-width: 100%;
            border-radius: 5px;
            margin-top: 10px;
            background-color: rgba(0, 0, 0, 0.3);
            min-height: 200px;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        
        .camera-image img {
            max-width: 100%;
            max-height: 300px;
            border-radius: 5px;
        }
        
        .camera-image.no-signal {
            color: var(--disabled-color);
            font-style: italic;
        }
        
        .lane-data {
            display: flex;
            align-items: center;
            justify-content: center;
            margin-top: 10px;
            padding: 10px;
            background-color: rgba(255, 255, 255, 0.05);
            border-radius: 5px;
        }
        
        .lane-steering-value {
            font-size: 24px;
            font-weight: bold;
            color: var(--accent-color);
        }
        
        .connection-status {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
            margin-top: 10px;
        }
        
        .connection-item {
            display: flex;
            align-items: center;
            padding: 8px;
            background-color: rgba(255, 255, 255, 0.05);
            border-radius: 5px;
        }
        
        .emergency-banner {
            background-color: var(--danger-color);
            color: white;
            text-align: center;
            padding: 15px;
            margin-bottom: 20px;
            border-radius: 5px;
            font-weight: bold;
            font-size: 18px;
            animation: pulse 2s infinite;
        }
        
        @keyframes pulse {
            0% { opacity: 1; }
            50% { opacity: 0.7; }
            100% { opacity: 1; }
        }
        
        .mode-disabled {
            background-color: var(--disabled-color) !important;
            color: #ddd !important;
            cursor: not-allowed;
        }
        
        .state-machine-display {
            display: flex;
            flex-direction: column;
            margin-top: 15px;
        }
        
        .state-indicator {
            display: flex;
            align-items: center;
            margin-bottom: 10px;
            padding: 10px;
            border-radius: 5px;
            background-color: rgba(255, 255, 255, 0.05);
        }
        
        .state-indicator.active {
            background-color: rgba(77, 132, 255, 0.2);
            border-left: 4px solid var(--accent-color);
        }
        
        .state-indicator.section1 {
            border-left: 4px solid var(--section1-color);
        }
        
        .state-indicator.section2 {
            border-left: 4px solid var(--section2-color);
        }
        
        .state-indicator.section3 {
            border-left: 4px solid var(--section3-color);
        }
        
        .state-name {
            font-weight: bold;
            flex-grow: 1;
        }
        
        .data-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
            margin-top: 15px;
        }
        
        .data-item {
            background-color: rgba(255, 255, 255, 0.05);
            padding: 10px;
            border-radius: 5px;
        }
        
        .data-label {
            font-size: 12px;
            opacity: 0.7;
            margin-bottom: 5px;
        }
        
        .data-value {
            font-size: 18px;
            font-weight: bold;
        }
        
        .data-value.positive {
            color: var(--success-color);
        }
        
        .data-value.negative {
            color: var(--danger-color);
        }
        
        .data-value.warning {
            color: var(--warning-color);
        }
        
        .obstacle-list {
            margin-top: 15px;
            max-height: 200px;
            overflow-y: auto;
            background-color: rgba(0, 0, 0, 0.3);
            border-radius: 5px;
            padding: 10px;
        }
        
        .obstacle-item {
            display: flex;
            justify-content: space-between;
            padding: 8px;
            border-bottom: 1px solid rgba(255, 255, 255, 0.1);
        }
        
        .obstacle-item:last-child {
            border-bottom: none;
        }
        
        .obstacle-distance {
            font-weight: bold;
        }
        
        .obstacle-distance.close {
            color: var(--danger-color);
        }
        
        .obstacle-distance.medium {
            color: var(--warning-color);
        }
        
        .obstacle-distance.far {
            color: var(--success-color);
        }
        
        .visualization-container {
            position: relative;
            width: 100%;
            height: 300px;
            background-color: rgba(0, 0, 0, 0.3);
            border-radius: 5px;
            margin-top: 15px;
            overflow: hidden;
        }
        
        #lidar-visualization {
            width: 100%;
            height: 100%;
        }
        
        .stop-sign-indicator {
            display: flex;
            align-items: center;
            margin-top: 15px;
            padding: 15px;
            border-radius: 5px;
            background-color: rgba(255, 255, 255, 0.05);
        }
        
        .stop-sign-indicator.detected {
            background-color: rgba(255, 77, 77, 0.2);
            border-left: 4px solid var(--danger-color);
            animation: pulse 2s infinite;
        }
        
        .stop-sign-icon {
            width: 40px;
            height: 40px;
            margin-right: 15px;
            background-color: var(--danger-color);
            border-radius: 5px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-weight: bold;
            color: white;
        }
        
        .stop-sign-data {
            flex-grow: 1;
        }
        
        .stop-sign-distance {
            font-size: 24px;
            font-weight: bold;
        }
        
        .tab-container {
            margin-top: 15px;
        }
        
        .tab-buttons {
            display: flex;
            border-bottom: 1px solid rgba(255, 255, 255, 0.1);
        }
        
        .tab-button {
            padding: 10px 15px;
            background: none;
            border: none;
            color: var(--text-color);
            cursor: pointer;
            opacity: 0.7;
            transition: all 0.3s ease;
        }
        
        .tab-button.active {
            opacity: 1;
            border-bottom: 2px solid var(--accent-color);
        }
        
        .tab-content {
            padding: 15px 0;
        }
        
        .tab-panel {
            display: none;
        }
        
        .tab-panel.active {
            display: block;
        }
        
        .progress-bar {
            width: 100%;
            height: 10px;
            background-color: rgba(255, 255, 255, 0.1);
            border-radius: 5px;
            margin-top: 5px;
            overflow: hidden;
        }
        
        .progress-fill {
            height: 100%;
            background-color: var(--accent-color);
            width: 0%;
            transition: width 0.3s ease;
        }
    </style>
</head>
<body>
    <div class="container">
        <div id="emergency-banner" class="emergency-banner full-width" style="display: none;">
            ⚠️ EMERGENCY STOP ACTIVE ⚠️
        </div>
        
        <div class="card">
            <div class="card-header">
                Control Panel
                <div id="control-mode" class="status-indicator status-warning"></div>
            </div>
            <div class="card-content">
                <div class="mode-toggle">
                    <button id="manual-btn" class="mode-btn manual active" onclick="setMode('manual')">Manual Control</button>
                    <button id="auto-btn" class="mode-btn autonomous" onclick="setMode('autonomous')">Autonomous</button>
                </div>
                
                <div id="manual-controls">
                    <h3>Joystick Control</h3>
                    <div id="joystick-container" class="joystick-container">
                        <div class="joystick-base">
                            <div id="joystick-thumb" class="joystick-thumb"></div>
                        </div>
                    </div>
                    
                    <div class="status-indicators">
                        <div class="indicator">
                            <div id="speed-value" class="indicator-value">0.0</div>
                            <div class="indicator-label">SPEED</div>
                        </div>
                        <div class="indicator">
                            <div id="steering-value" class="indicator-value">0.0</div>
                            <div class="indicator-label">STEERING</div>
                        </div>
                    </div>
                </div>
                
                <button id="stop-btn" class="emergency-stop" onclick="emergencyStop()">EMERGENCY STOP</button>
                <button id="reset-stop-btn" class="reset-emergency" onclick="resetEmergencyStop()" style="display: none;">RESET EMERGENCY STOP</button>
            </div>
        </div>
        
        <div class="card">
            <div class="card-header">
                State Machine
                <div id="state-indicator" class="status-indicator status-warning"></div>
            </div>
            <div class="card-content">
                <div class="state-machine-display" id="state-machine-display">
                    <!-- State indicators will be populated dynamically -->
                </div>
            </div>
        </div>
        
        <div class="card">
            <div class="card-header">
                Perception
            </div>
            <div class="card-content">
                <div class="tab-container">
                    <div class="tab-buttons">
                        <button class="tab-button active" onclick="showTab('lane-tab')">Lane Detection</button>
                        <button class="tab-button" onclick="showTab('stop-sign-tab')">Stop Sign</button>
                        <button class="tab-button" onclick="showTab('obstacles-tab')">Obstacles</button>
                    </div>
                    
                    <div class="tab-content">
                        <div id="lane-tab" class="tab-panel active">
                            <div class="data-grid">
                                <div class="data-item">
                                    <div class="data-label">Lane Center Offset</div>
                                    <div id="lane-center-offset" class="data-value">0.00</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Heading Error</div>
                                    <div id="lane-heading-error" class="data-value">0.00</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Left Line</div>
                                    <div id="left-line-status" class="data-value">Not Detected</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Right Line</div>
                                    <div id="right-line-status" class="data-value">Not Detected</div>
                                </div>
                            </div>
                            
                            <div class="camera-view">
                                <h3>Lane Mask</h3>
                                <div id="lane-mask" class="camera-image no-signal">No signal</div>
                            </div>
                            
                            <div class="camera-view">
                                <h3>Debug Image</h3>
                                <div id="debug-image" class="camera-image no-signal">No signal</div>
                            </div>
                        </div>
                        
                        <div id="stop-sign-tab" class="tab-panel">
                            <div id="stop-sign-indicator" class="stop-sign-indicator">
                                <div class="stop-sign-icon">STOP</div>
                                <div class="stop-sign-data">
                                    <div>Status: <span id="stop-sign-status">Not Detected</span></div>
                                    <div>Distance: <span id="stop-sign-distance" class="stop-sign-distance">0.00</span> m</div>
                                    <div>Position: X: <span id="stop-sign-x">0.00</span>, Y: <span id="stop-sign-y">0.00</span>, Z: <span id="stop-sign-z">0.00</span></div>
                                </div>
                            </div>
                        </div>
                        
                        <div id="obstacles-tab" class="tab-panel">
                            <div class="data-grid">
                                <div class="data-item">
                                    <div class="data-label">Obstacle Count</div>
                                    <div id="obstacle-count" class="data-value">0</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Nearest Obstacle</div>
                                    <div id="nearest-obstacle" class="data-value">0.00 m</div>
                                </div>
                            </div>
                            
                            <h3>Detected Obstacles</h3>
                            <div id="obstacle-list" class="obstacle-list">
                                <div class="obstacle-item">No obstacles detected</div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
        
        <div class="card two-thirds">
            <div class="card-header">
                Sensor Visualization
            </div>
            <div class="card-content">
                <div class="tab-container">
                    <div class="tab-buttons">
                        <button class="tab-button active" onclick="showVisualizationTab('lidar-viz-tab')">LiDAR</button>
                        <button class="tab-button" onclick="showVisualizationTab('odometry-viz-tab')">Odometry</button>
                    </div>
                    
                    <div class="tab-content">
                        <div id="lidar-viz-tab" class="tab-panel active">
                            <div class="data-grid">
                                <div class="data-item">
                                    <div class="data-label">Front Distance</div>
                                    <div id="lidar-front" class="data-value">0.00 m</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Left Distance</div>
                                    <div id="lidar-left" class="data-value">0.00 m</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Right Distance</div>
                                    <div id="lidar-right" class="data-value">0.00 m</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Range</div>
                                    <div id="lidar-range" class="data-value">0.00 - 0.00 m</div>
                                </div>
                            </div>
                            
                            <div class="visualization-container">
                                <canvas id="lidar-visualization"></canvas>
                            </div>
                        </div>
                        
                        <div id="odometry-viz-tab" class="tab-panel">
                            <div class="data-grid">
                                <div class="data-item">
                                    <div class="data-label">Position</div>
                                    <div id="odom-position" class="data-value">X: 0.00, Y: 0.00, Z: 0.00</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Orientation</div>
                                    <div id="odom-orientation" class="data-value">R: 0.00, P: 0.00, Y: 0.00</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Linear Velocity</div>
                                    <div id="linear-velocity" class="data-value">0.00 m/s</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Angular Velocity</div>
                                    <div id="angular-velocity" class="data-value">0.00 rad/s</div>
                                </div>
                            </div>
                            
                            <div class="visualization-container">
                                <canvas id="odometry-visualization"></canvas>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
        
        <div class="card">
            <div class="card-header">
                System Log
            </div>
            <div class="card-content">
                <div id="log-container" class="log-container"></div>
            </div>
        </div>
    </div>

    <script>
        // Variables for joystick control
        let joystickActive = false;
        let joystickPos = { x: 0, y: 0 };
        const joystickContainer = document.getElementById('joystick-container');
        const joystickThumb = document.getElementById('joystick-thumb');
        const containerRect = joystickContainer.getBoundingClientRect();
        const containerRadius = containerRect.width / 2;
        const thumbRadius = 40; // Half the thumb width
        let currentMode = 'manual';
        let isEmergencyStopped = false;
        let controlInterval = null;
        let currentState = 0;
        
        // Initialize indicators
        const speedValue = document.getElementById('speed-value');
        const steeringValue = document.getElementById('steering-value');
        const controlModeIndicator = document.getElementById('control-mode');
        const stateIndicator = document.getElementById('state-indicator');
        const logContainer = document.getElementById('log-container');
        const manualBtn = document.getElementById('manual-btn');
        const autoBtn = document.getElementById('auto-btn');
        const manualControls = document.getElementById('manual-controls');
        const emergencyBanner = document.getElementById('emergency-banner');
        const resetStopBtn = document.getElementById('reset-stop-btn');
        const laneMask = document.getElementById('lane-mask');
        const debugImage = document.getElementById('debug-image');
        
        // State machine elements
        const stateMachineDisplay = document.getElementById('state-machine-display');
        
        // Lane detection elements
        const laneCenterOffset = document.getElementById('lane-center-offset');
        const laneHeadingError = document.getElementById('lane-heading-error');
        const leftLineStatus = document.getElementById('left-line-status');
        const rightLineStatus = document.getElementById('right-line-status');
        
        // Stop sign elements
        const stopSignIndicator = document.getElementById('stop-sign-indicator');
        const stopSignStatus = document.getElementById('stop-sign-status');
        const stopSignDistance = document.getElementById('stop-sign-distance');
        const stopSignX = document.getElementById('stop-sign-x');
        const stopSignY = document.getElementById('stop-sign-y');
        const stopSignZ = document.getElementById('stop-sign-z');
        
        // Obstacle elements
        const obstacleCount = document.getElementById('obstacle-count');
        const nearestObstacle = document.getElementById('nearest-obstacle');
        const obstacleList = document.getElementById('obstacle-list');
        
        // LiDAR elements
        const lidarFront = document.getElementById('lidar-front');
        const lidarLeft = document.getElementById('lidar-left');
        const lidarRight = document.getElementById('lidar-right');
        const lidarRange = document.getElementById('lidar-range');
        const lidarCanvas = document.getElementById('lidar-visualization');
        const lidarCtx = lidarCanvas.getContext('2d');
        
        // Odometry elements
        const odomPosition = document.getElementById('odom-position');
        const odomOrientation = document.getElementById('odom-orientation');
        const linearVelocity = document.getElementById('linear-velocity');
        const angularVelocity = document.getElementById('angular-velocity');
        const odometryCanvas = document.getElementById('odometry-visualization');
        const odometryCtx = odometryCanvas.getContext('2d');
        
        // Resize canvas elements
        function resizeCanvas() {
            const container = document.querySelector('.visualization-container');
            const width = container.clientWidth;
            const height = container.clientHeight;
            
            lidarCanvas.width = width;
            lidarCanvas.height = height;
            odometryCanvas.width = width;
            odometryCanvas.height = height;
        }
        
        window.addEventListener('resize', resizeCanvas);
        resizeCanvas();
        
        // Joystick touch/mouse handlers
        joystickContainer.addEventListener('mousedown', startJoystick);
        joystickContainer.addEventListener('touchstart', startJoystick);
        document.addEventListener('mousemove', moveJoystick);
        document.addEventListener('touchmove', moveJoystick, { passive: false });
        document.addEventListener('mouseup', endJoystick);
        document.addEventListener('touchend', endJoystick);
        
        // Initialize state machine display
        initializeStateMachine();
        
        // Initialize status updates
        setInterval(updateSystemData, 100);
        setInterval(updateVisualizationData, 100);
        
        // Set up control interval - send commands immediately on change
        controlInterval = setInterval(sendControlCommand, 50);
        
        // Log initial entry
        addLogEntry('Dashboard initialized');
        
        function initializeStateMachine() {
            const states = [
                { id: 0, name: "IDLE", section: null },
                { id: 1, name: "SECTION1_LANE_FOLLOWING", section: "section1" },
                { id: 2, name: "APPROACHING_STOP_SIGN_SECTION1", section: "section1" },
                { id: 3, name: "STOPPED_SECTION1", section: "section1" },
                { id: 4, name: "SECTION2_OBSTACLE_AVOIDANCE", section: "section2" },
                { id: 5, name: "APPROACHING_STOP_SIGN_SECTION2", section: "section2" },
                { id: 6, name: "STOPPED_SECTION2", section: "section2" },
                { id: 7, name: "SECTION3_PARKING", section: "section3" },
                { id: 8, name: "PARKED", section: "section3" },
                { id: 9, name: "EMERGENCY_STOP", section: null }
            ];
            
            stateMachineDisplay.innerHTML = '';
            
            states.forEach(state => {
                const stateElement = document.createElement('div');
                stateElement.id = `state-${state.id}`;
                stateElement.className = `state-indicator ${state.section || ''}`;
                stateElement.innerHTML = `
                    <div class="state-name">${state.name}</div>
                `;
                stateMachineDisplay.appendChild(stateElement);
            });
        }
        
        function updateStateMachine(currentState) {
            // Remove active class from all states
            document.querySelectorAll('.state-indicator').forEach(el => {
                el.classList.remove('active');
            });
            
            // Add active class to current state
            const currentStateElement = document.getElementById(`state-${currentState}`);
            if (currentStateElement) {
                currentStateElement.classList.add('active');
                
                // Update state indicator color
                if (currentState === 9) { // EMERGENCY_STOP
                    stateIndicator.className = 'status-indicator status-error';
                } else if (currentState === 0) { // IDLE
                    stateIndicator.className = 'status-indicator status-warning';
                } else {
                    stateIndicator.className = 'status-indicator status-ok';
                }
            }
        }
        
        function startJoystick(e) {
            if (currentMode !== 'manual' || isEmergencyStopped) return;
            
            joystickActive = true;
            e.preventDefault();
            updateJoystickPosition(e);
        }
        
        function moveJoystick(e) {
            if (!joystickActive) return;
            
            e.preventDefault();
            updateJoystickPosition(e);
        }
        
        function endJoystick() {
            if (!joystickActive) return;
            
            joystickActive = false;
            
            // Reset joystick position
            joystickThumb.style.left = '50%';
            joystickThumb.style.top = '50%';
            joystickPos = { x: 0, y: 0 };
            
            // Send stop command immediately
            sendControlCommand(true);
            
            // Update display
            updateControlDisplay();
        }
        
        function updateJoystickPosition(e) {
            const rect = joystickContainer.getBoundingClientRect();
            const centerX = rect.left + rect.width / 2;
            const centerY = rect.top + rect.height / 2;
            
            // Get touch/mouse position
            let clientX, clientY;
            if (e.type.startsWith('touch')) {
                clientX = e.touches[0].clientX;
                clientY = e.touches[0].clientY;
            } else {
                clientX = e.clientX;
                clientY = e.clientY;
            }
            
            // Calculate distance from center
            let deltaX = clientX - centerX;
            let deltaY = clientY - centerY;
            const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            
            // Limit to container radius
            if (distance > containerRadius - thumbRadius) {
                const angle = Math.atan2(deltaY, deltaX);
                deltaX = Math.cos(angle) * (containerRadius - thumbRadius);
                deltaY = Math.sin(angle) * (containerRadius - thumbRadius);
            }
            
            // Update thumb position
            joystickThumb.style.left = `calc(50% + ${deltaX}px)`;
            joystickThumb.style.top = `calc(50% + ${deltaY}px)`;
            
            // Map to control values (-1 to 1)
            joystickPos.x = deltaX / (containerRadius - thumbRadius);
            joystickPos.y = -deltaY / (containerRadius - thumbRadius); // Invert Y for forward/backward
            
            // Display values
            updateControlDisplay();
        }
        
        function updateControlDisplay() {
            // Convert joystick position to control values
            // Y axis (forward/backward) maps to speed
            const speed = joystickPos.y;
            // X axis (left/right) maps to steering
            const steering = -joystickPos.x;
            
            speedValue.textContent = speed.toFixed(2);
            steeringValue.textContent = steering.toFixed(2);
        }
        
        function sendControlCommand(forceZero = false) {
            if (currentMode !== 'manual' || isEmergencyStopped) return;
            
            // Calculate speed from Y axis (0 to 1)
            const speed = forceZero ? 0 : Math.max(0, joystickPos.y);
            // Calculate steering from X axis (-1 to 1)
            const steering = forceZero ? 0 : -joystickPos.x;
            
            fetch('/api/move', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ linear_x: speed, angular_z: steering })
            });
        }
        
        function setMode(mode) {
            if (isEmergencyStopped) {
                addLogEntry('Cannot change mode while emergency stopped', 'warning');
                return;
            }
            
            currentMode = mode;
            
            if (mode === 'manual') {
                manualBtn.classList.add('active');
                autoBtn.classList.remove('active');
                manualControls.classList.remove('disabled');
                controlModeIndicator.className = 'status-indicator status-ok';
                addLogEntry('Switched to MANUAL control mode');
            } else {
                manualBtn.classList.remove('active');
                autoBtn.classList.add('active');
                manualControls.classList.add('disabled');
                controlModeIndicator.className = 'status-indicator status-warning';
                addLogEntry('Switched to AUTONOMOUS control mode');
                
                // Send stop command when switching to autonomous
                fetch('/api/move', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ linear_x: 0, angular_z: 0 })
                });
            }
            
            // Send mode change request to server
            fetch('/api/mode', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ mode: mode })
            });
        }
        
        function emergencyStop() {
            fetch('/api/emergency', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' }
            });
            
            addLogEntry('⚠️ EMERGENCY STOP triggered', 'warning');
            joystickPos = { x: 0, y: 0 };
            updateControlDisplay();
        }
        
        function resetEmergencyStop() {
            fetch('/api/reset_emergency', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' }
            });
            
            addLogEntry('Emergency stop reset requested');
        }
        
        function updateSystemData() {
            fetch('/api/system_data')
                .then(response => response.json())
                .then(data => {
                    // Update emergency stop state
                    if (data.emergency_stopped !== isEmergencyStopped) {
                        isEmergencyStopped = data.emergency_stopped;
                        if (isEmergencyStopped) {
                            emergencyBanner.style.display = 'block';
                            resetStopBtn.style.display = 'block';
                            manualControls.classList.add('disabled');
                            manualBtn.classList.add('mode-disabled');
                            autoBtn.classList.add('mode-disabled');
                            addLogEntry('Emergency stop activated', 'warning');
                        } else {
                            emergencyBanner.style.display = 'none';
                            resetStopBtn.style.display = 'none';
                            if (currentMode === 'manual') {
                                manualControls.classList.remove('disabled');
                            }
                            manualBtn.classList.remove('mode-disabled');
                            autoBtn.classList.remove('mode-disabled');
                            addLogEntry('Emergency stop reset', 'info');
                        }
                    }
                    
                    // Update mode buttons
                    if (data.mode !== currentMode && !isEmergencyStopped) {
                        currentMode = data.mode;
                        if (currentMode === 'manual') {
                            manualBtn.classList.add('active');
                            autoBtn.classList.remove('active');
                            manualControls.classList.remove('disabled');
                            controlModeIndicator.className = 'status-indicator status-ok';
                        } else {
                            manualBtn.classList.remove('active');
                            autoBtn.classList.add('active');
                            manualControls.classList.add('disabled');
                            controlModeIndicator.className = 'status-indicator status-warning';
                        }
                    }
                    
                    // Update state machine
                    if (data.state_machine.current_state !== currentState) {
                        currentState = data.state_machine.current_state;
                        updateStateMachine(currentState);
                        addLogEntry(`State changed to: ${data.state_machine.state_name}`);
                    }
                    
                    // Update lane detection data
                    laneCenterOffset.textContent = data.lane_detection.center_offset.toFixed(2);
                    laneHeadingError.textContent = data.lane_detection.heading_error.toFixed(2);
                    
                    // Add positive/negative classes based on values
                    if (Math.abs(data.lane_detection.center_offset) < 0.1) {
                        laneCenterOffset.className = 'data-value positive';
                    } else if (Math.abs(data.lane_detection.center_offset) < 0.3) {
                        laneCenterOffset.className = 'data-value warning';
                    } else {
                        laneCenterOffset.className = 'data-value negative';
                    }
                    
                    if (Math.abs(data.lane_detection.heading_error) < 0.1) {
                        laneHeadingError.className = 'data-value positive';
                    } else if (Math.abs(data.lane_detection.heading_error) < 0.3) {
                        laneHeadingError.className = 'data-value warning';
                    } else {
                        laneHeadingError.className = 'data-value negative';
                    }
                    
                    leftLineStatus.textContent = data.lane_detection.left_line_detected ? 'Detected' : 'Not Detected';
                    leftLineStatus.className = 'data-value ' + (data.lane_detection.left_line_detected ? 'positive' : 'negative');
                    
                    rightLineStatus.textContent = data.lane_detection.right_line_detected ? 'Detected' : 'Not Detected';
                    rightLineStatus.className = 'data-value ' + (data.lane_detection.right_line_detected ? 'positive' : 'negative');
                    
                    // Update stop sign data
                    stopSignStatus.textContent = data.stop_sign.detected ? 'Detected' : 'Not Detected';
                    stopSignDistance.textContent = data.stop_sign.distance.toFixed(2);
                    stopSignX.textContent = data.stop_sign.position.x.toFixed(2);
                    stopSignY.textContent = data.stop_sign.position.y.toFixed(2);
                    stopSignZ.textContent = data.stop_sign.position.z.toFixed(2);
                    
                    if (data.stop_sign.detected) {
                        stopSignIndicator.classList.add('detected');
                    } else {
                        stopSignIndicator.classList.remove('detected');
                    }
                    
                    // Update obstacle data
                    obstacleCount.textContent = data.obstacles.count;
                    nearestObstacle.textContent = data.obstacles.nearest_distance.toFixed(2) + ' m';
                    
                    if (data.obstacles.count > 0) {
                        // Color based on distance
                        if (data.obstacles.nearest_distance < 0.5) {
                            nearestObstacle.className = 'data-value negative';
                        } else if (data.obstacles.nearest_distance < 1.0) {
                            nearestObstacle.className = 'data-value warning';
                        } else {
                            nearestObstacle.className = 'data-value positive';
                        }
                        
                        // Update obstacle list
                        obstacleList.innerHTML = '';
                        data.obstacles.obstacles.forEach((obstacle, index) => {
                            const obstacleElement = document.createElement('div');
                            obstacleElement.className = 'obstacle-item';
                            
                            let distanceClass = 'far';
                            if (obstacle.distance < 0.5) {
                                distanceClass = 'close';
                            } else if (obstacle.distance < 1.0) {
                                distanceClass = 'medium';
                            }
                            
                            obstacleElement.innerHTML = `
                                <div>Obstacle ${index + 1}</div>
                                <div class="obstacle-distance ${distanceClass}">${obstacle.distance.toFixed(2)} m</div>
                            `;
                            obstacleList.appendChild(obstacleElement);
                        });
                    } else {
                        obstacleList.innerHTML = '<div class="obstacle-item">No obstacles detected</div>';
                        nearestObstacle.className = 'data-value';
                    }
                    
                    // Update LiDAR data
                    lidarFront.textContent = data.lidar.front_distance.toFixed(2) + ' m';
                    lidarLeft.textContent = data.lidar.left_distance.toFixed(2) + ' m';
                    lidarRight.textContent = data.lidar.right_distance.toFixed(2) + ' m';
                    lidarRange.textContent = `${data.lidar.min_range.toFixed(2)} - ${data.lidar.max_range.toFixed(2)} m`;
                    
                    // Color based on distance
                    if (data.lidar.front_distance < 0.5) {
                        lidarFront.className = 'data-value negative';
                    } else if (data.lidar.front_distance < 1.0) {
                        lidarFront.className = 'data-value warning';
                    } else {
                        lidarFront.className = 'data-value positive';
                    }
                    
                    // Update odometry data
                    odomPosition.textContent = `X: ${data.odometry.position.x.toFixed(2)}, Y: ${data.odometry.position.y.toFixed(2)}, Z: ${data.odometry.position.z.toFixed(2)}`;
                    odomOrientation.textContent = `R: ${(data.odometry.orientation.roll * 180 / Math.PI).toFixed(1)}°, P: ${(data.odometry.orientation.pitch * 180 / Math.PI).toFixed(1)}°, Y: ${(data.odometry.orientation.yaw * 180 / Math.PI).toFixed(1)}°`;
                    
                    const speed = Math.sqrt(
                        data.odometry.linear_velocity.x ** 2 + 
                        data.odometry.linear_velocity.y ** 2 + 
                        data.odometry.linear_velocity.z ** 2
                    );
                    
                    linearVelocity.textContent = speed.toFixed(2) + ' m/s';
                    angularVelocity.textContent = data.odometry.angular_velocity.z.toFixed(2) + ' rad/s';
                })
                .catch(error => {
                    console.error('Error fetching system data:', error);
                });
        }
        
        function updateVisualizationData() {
            fetch('/api/visualization_data')
                .then(response => response.json())
                .then(data => {
                    // Update lane mask image
                    if (data.lane_mask) {
                        laneMask.innerHTML = `<img src="data:image/jpeg;base64,${data.lane_mask}">`;
                        laneMask.classList.remove('no-signal');
                    } else {
                        laneMask.innerHTML = 'No signal';
                        laneMask.classList.add('no-signal');
                    }
                    
                    // Update debug image
                    if (data.debug_image) {
                        debugImage.innerHTML = `<img src="data:image/jpeg;base64,${data.debug_image}">`;
                        debugImage.classList.remove('no-signal');
                    } else {
                        debugImage.innerHTML = 'No signal';
                        debugImage.classList.add('no-signal');
                    }
                })
                .catch(error => {
                    console.error('Error fetching visualization data:', error);
                });
        }
        
        function showTab(tabId) {
            // Hide all tab panels
            document.querySelectorAll('.tab-panel').forEach(panel => {
                panel.classList.remove('active');
            });
            
            // Deactivate all tab buttons
            document.querySelectorAll('.tab-button').forEach(button => {
                button.classList.remove('active');
            });
            
            // Show the selected tab panel
            document.getElementById(tabId).classList.add('active');
            
            // Activate the clicked tab button
            event.currentTarget.classList.add('active');
        }
        
        function showVisualizationTab(tabId) {
            // Hide all visualization tab panels
            document.querySelectorAll('#lidar-viz-tab, #odometry-viz-tab').forEach(panel => {
                panel.classList.remove('active');
            });
            
            // Deactivate all visualization tab buttons
            document.querySelectorAll('.tab-button').forEach(button => {
                button.classList.remove('active');
            });
            
            // Show the selected tab panel
            document.getElementById(tabId).classList.add('active');
            
            // Activate the clicked tab button
            event.currentTarget.classList.add('active');
            
            // Resize canvas if needed
            resizeCanvas();
        }
        
        function addLogEntry(message, type = 'info') {
            const now = new Date();
            const timeStr = now.toLocaleTimeString();
            const logEntry = document.createElement('div');
            logEntry.className = 'log-entry';
            logEntry.innerHTML = `<span class="log-time">[${timeStr}]</span> ${message}`;
            
            logContainer.appendChild(logEntry);
            logContainer.scrollTop = logContainer.scrollHeight;
            
            // Limit log entries
            while (logContainer.children.length > 50) {
                logContainer.removeChild(logContainer.firstChild);
            }
        }
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    """Serve the control interface"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/move', methods=['POST'])
def handle_movement():
    """API endpoint for movement commands"""
    if not request.json:
        return jsonify({'status': 'error', 'message': 'Invalid request'}), 400
    
    linear_x = request.json.get('linear_x', 0.0)
    angular_z = request.json.get('angular_z', 0.0)
    
    if ros_node:
        ros_node.send_movement(linear_x, angular_z)
        return jsonify({'status': 'success', 'message': 'Movement command sent'}), 200
    else:
        return jsonify({'status': 'error', 'message': 'ROS2 node not initialized'}), 500

@app.route('/api/mode', methods=['POST'])
def handle_mode():
    """API endpoint for mode switching"""
    if not request.json or 'mode' not in request.json:
        return jsonify({'status': 'error', 'message': 'Invalid request'}), 400
    
    mode = request.json['mode']
    if mode not in ['manual', 'autonomous']:
        return jsonify({'status': 'error', 'message': 'Invalid mode'}), 400
    
    if ros_node:
        ros_node.set_mode(mode)
        return jsonify({'status': 'success', 'message': f'Mode set to {mode}'}), 200
    else:
        return jsonify({'status': 'error', 'message': 'ROS2 node not initialized'}), 500

@app.route('/api/emergency', methods=['POST'])
def handle_emergency():
    """API endpoint for emergency stop"""
    if ros_node:
        ros_node.trigger_emergency_stop()
        return jsonify({'status': 'success', 'message': 'Emergency stop triggered'}), 200
    else:
        return jsonify({'status': 'error', 'message': 'ROS2 node not initialized'}), 500

@app.route('/api/reset_emergency', methods=['POST'])
def handle_reset_emergency():
    """API endpoint for resetting emergency stop"""
    if ros_node:
        ros_node.reset_emergency_stop()
        return jsonify({'status': 'success', 'message': 'Emergency stop reset'}), 200
    else:
        return jsonify({'status': 'error', 'message': 'ROS2 node not initialized'}), 500

@app.route('/api/system_data', methods=['GET'])
def handle_system_data():
    """API endpoint for getting system data"""
    if ros_node:
        return jsonify(ros_node.get_system_data()), 200
    else:
        return jsonify({'status': 'error', 'message': 'ROS2 node not initialized'}), 500

@app.route('/api/visualization_data', methods=['GET'])
def handle_visualization_data():
    """API endpoint for getting visualization data"""
    if ros_node:
        return jsonify(ros_node.get_visualization_data()), 200
    else:
        return jsonify({'status': 'error', 'message': 'ROS2 node not initialized'}), 500

def run_flask():
    """Run the Flask server in a separate thread"""
    app.run(host='0.0.0.0', port=8080, debug=False)

def run_ros_node():
    """Run the ROS2 node"""
    global ros_node
    
    rclpy.init()
    ros_node = EcoMarathonDashboardNode()
    
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

def main():
    # Start the Flask server in a separate thread
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    
    # Run the ROS2 node in the main thread
    run_ros_node()

if __name__ == '__main__':
    main()
