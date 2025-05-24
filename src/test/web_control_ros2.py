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
import asyncio
import websockets
import time
from collections import deque
from flask import Flask, render_template, request
import concurrent.futures
import queue
import logging
import subprocess
import signal
import os

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class EcoMarathonDashboardNode(Node):
    def __init__(self):
        super().__init__('eco_marathon_dashboard_node')
        
        # Initialize CV bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # WebSocket clients
        self.websocket_clients = set()
        self.websocket_lock = threading.Lock()
        
        # Message queue for async communication
        self.message_queue = queue.Queue()
        
        # Data change tracking for efficient updates
        self.last_data_hash = {}
        self.image_cache = {}
        self.last_image_time = {}
        
        # Rate limiting for images (reduce frequency for low-end devices)
        self.image_update_interval = 0.2  # 5 FPS instead of 10 FPS
        
        # Publishers
        self.manual_pub = self.create_publisher(Twist, '/manual_control', 10)
        self.mode_pub = self.create_publisher(String, '/control_mode', 10)
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.reset_estop_pub = self.create_publisher(Bool, '/reset_emergency', 10)
        
        # Subscribers with optimized QoS
        self.state_sub = self.create_subscription(
            SystemState, '/state_machine/current_state', 
            self.state_callback, 10)
        
        self.lane_detection_sub = self.create_subscription(
            LaneDetection, '/perception/lane_detections',
            self.lane_detection_callback, 10)
        
        # Reduce image subscription frequency for performance
        self.lane_mask_sub = self.create_subscription(
            Image, '/lane_detection/lane_mask',
            self.lane_mask_callback, 5)  # Reduced queue size
        
        self.debug_image_sub = self.create_subscription(
            Image, '/lane_detection/debug_image',
            self.debug_image_callback, 5)  # Reduced queue size
        
        self.stop_sign_sub = self.create_subscription(
            StopSign, '/perception/stop_sign_detected',
            self.stop_sign_callback, 10)
        
        self.obstacles_sub = self.create_subscription(
            Obstacles, '/perception/obstacles',
            self.obstacles_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered',
            self.odometry_callback, 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback, 5)  # Reduced frequency for LiDAR
        
        # System data with change detection
        self.system_data = {
            "mode": "MANUAL",
            "emergency_stopped": False,
            "state_machine": {"current_state": 0, "state_name": "IDLE"},
            "lane_detection": {
                "center_offset": 0.0, "heading_error": 0.0,
                "left_line_detected": False, "right_line_detected": False
            },
            "stop_sign": {
                "detected": False, "distance": 0.0,
                "position": {"x": 0.0, "y": 0.0, "z": 0.0}
            },
            "obstacles": {"count": 0, "nearest_distance": 0.0, "obstacles": []},
            "odometry": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                "linear_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0}
            },
            "lidar": {
                "min_range": 0.0, "max_range": 0.0,
                "front_distance": 0.0, "left_distance": 0.0, "right_distance": 0.0
            },
            "control": {"linear_speed": 0.0, "angular_speed": 0.0}
        }
        
        # Thread-safe data access
        self.data_lock = threading.Lock()
        
        # Cooldown mechanism
        self.last_mode_switch_time = 0
        self.cooldown_period = 1.0
        
        self.get_logger().info('Eco Marathon Dashboard Node initialized')
    
    def add_websocket_client(self, websocket):
        """Add a WebSocket client"""
        with self.websocket_lock:
            self.websocket_clients.add(websocket)
        self.get_logger().info(f'WebSocket client connected. Total clients: {len(self.websocket_clients)}')
    
    def remove_websocket_client(self, websocket):
        """Remove a WebSocket client"""
        with self.websocket_lock:
            self.websocket_clients.discard(websocket)
        self.get_logger().info(f'WebSocket client disconnected. Total clients: {len(self.websocket_clients)}')
    
    def queue_broadcast(self, data_type, data):
        """Queue a message for broadcasting"""
        try:
            self.message_queue.put({"type": data_type, "data": data}, block=False)
        except queue.Full:
            self.get_logger().warn("Message queue full, dropping message")
    
    def data_changed(self, data_key, new_data):
        """Check if data has changed to avoid unnecessary updates"""
        data_str = json.dumps(new_data, sort_keys=True)
        if self.last_data_hash.get(data_key) != data_str:
            self.last_data_hash[data_key] = data_str
            return True
        return False
    
    def state_callback(self, msg):
        """Handle state machine state updates"""
        state_names = [
            "IDLE", "SECTION1_LANE_FOLLOWING", "APPROACHING_STOP_SIGN_SECTION1",
            "STOPPED_SECTION1", "SECTION2_OBSTACLE_AVOIDANCE", "APPROACHING_STOP_SIGN_SECTION2",
            "STOPPED_SECTION2", "SECTION3_PARKING", "PARKED", "EMERGENCY_STOP"
        ]
        
        state_num = msg.state
        state_name = state_names[state_num] if state_num < len(state_names) else f"UNKNOWN_{state_num}"
        
        new_state_data = {"current_state": state_num, "state_name": state_name}
        
        if self.data_changed("state_machine", new_state_data):
            with self.data_lock:
                self.system_data["state_machine"] = new_state_data
            self.queue_broadcast("state_update", new_state_data)
    
    def lane_detection_callback(self, msg):
        """Handle lane detection updates"""
        new_lane_data = {
            "center_offset": float(msg.lane_center_offset),
            "heading_error": float(msg.lane_heading_error),
            "left_line_detected": len(msg.left_line_x) > 0,
            "right_line_detected": len(msg.right_line_x) > 0
        }
        
        if self.data_changed("lane_detection", new_lane_data):
            with self.data_lock:
                self.system_data["lane_detection"] = new_lane_data
            self.queue_broadcast("lane_update", new_lane_data)
    
    def lane_mask_callback(self, msg):
        """Handle lane mask image updates with rate limiting"""
        current_time = time.time()
        if current_time - self.last_image_time.get("lane_mask", 0) < self.image_update_interval:
            return
        
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            # Resize for performance on low-end devices
            cv_image = cv2.resize(cv_image, (320, 240))
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
            
            # Compress more aggressively for low-end devices
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 60])
            image_b64 = base64.b64encode(buffer).decode('utf-8')
            
            # Only send if image changed significantly
            if self.image_cache.get("lane_mask") != image_b64:
                self.image_cache["lane_mask"] = image_b64
                self.last_image_time["lane_mask"] = current_time
                self.queue_broadcast("image_update", {
                    "type": "lane_mask",
                    "data": image_b64
                })
        except Exception as e:
            self.get_logger().error(f'Error processing lane mask image: {e}')
    
    def debug_image_callback(self, msg):
        """Handle debug image updates with rate limiting"""
        current_time = time.time()
        if current_time - self.last_image_time.get("debug_image", 0) < self.image_update_interval:
            return
        
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # Resize for performance
            cv_image = cv2.resize(cv_image, (320, 240))
            
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 60])
            image_b64 = base64.b64encode(buffer).decode('utf-8')
            
            if self.image_cache.get("debug_image") != image_b64:
                self.image_cache["debug_image"] = image_b64
                self.last_image_time["debug_image"] = current_time
                self.queue_broadcast("image_update", {
                    "type": "debug_image",
                    "data": image_b64
                })
        except Exception as e:
            self.get_logger().error(f'Error processing debug image: {e}')
    
    def stop_sign_callback(self, msg):
        """Handle stop sign detection updates"""
        new_stop_data = {
            "detected": msg.detected,
            "distance": float(msg.distance),
            "position": {"x": float(msg.position.x), "y": float(msg.position.y), "z": float(msg.position.z)}
        }
        
        if self.data_changed("stop_sign", new_stop_data):
            with self.data_lock:
                self.system_data["stop_sign"] = new_stop_data
            self.queue_broadcast("stop_sign_update", new_stop_data)
    
    def obstacles_callback(self, msg):
        """Handle obstacle detection updates"""
        obstacles = []
        min_distance = float('inf')
        
        for obstacle in msg.obstacles:
            distance = np.sqrt(obstacle.center.x**2 + obstacle.center.y**2)
            if distance < min_distance:
                min_distance = distance
            
            obstacles.append({
                "center": {"x": float(obstacle.center.x), "y": float(obstacle.center.y), "z": float(obstacle.center.z)},
                "radius": float(obstacle.radius),
                "distance": float(distance)
            })
        
        new_obstacle_data = {
            "count": len(obstacles),
            "nearest_distance": float(min_distance) if obstacles else 0.0,
            "obstacles": obstacles
        }
        
        if self.data_changed("obstacles", new_obstacle_data):
            with self.data_lock:
                self.system_data["obstacles"] = new_obstacle_data
            self.queue_broadcast("obstacles_update", new_obstacle_data)
    
    def odometry_callback(self, msg):
        """Handle odometry updates"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
        
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        
        new_odom_data = {
            "position": {"x": float(position.x), "y": float(position.y), "z": float(position.z)},
            "orientation": {"roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)},
            "linear_velocity": {"x": float(linear.x), "y": float(linear.y), "z": float(linear.z)},
            "angular_velocity": {"x": float(angular.x), "y": float(angular.y), "z": float(angular.z)}
        }
        
        # Update control data
        speed = np.sqrt(linear.x**2 + linear.y**2)
        new_control_data = {"linear_speed": float(speed), "angular_speed": float(angular.z)}
        
        if self.data_changed("odometry", new_odom_data):
            with self.data_lock:
                self.system_data["odometry"] = new_odom_data
                self.system_data["control"] = new_control_data
            self.queue_broadcast("odometry_update", {
                "odometry": new_odom_data,
                "control": new_control_data
            })
    
    def scan_callback(self, msg):
        """Handle LiDAR scan updates with reduced frequency"""
        num_points = len(msg.ranges)
        front_idx = num_points // 2
        left_idx = (front_idx + num_points // 4) % num_points
        right_idx = (front_idx - num_points // 4) % num_points
        
        front_distance = msg.ranges[front_idx] if msg.ranges[front_idx] < msg.range_max else msg.range_max
        left_distance = msg.ranges[left_idx] if msg.ranges[left_idx] < msg.range_max else msg.range_max
        right_distance = msg.ranges[right_idx] if msg.ranges[right_idx] < msg.range_max else msg.range_max
        
        new_lidar_data = {
            "min_range": float(msg.range_min),
            "max_range": float(msg.range_max),
            "front_distance": float(front_distance),
            "left_distance": float(left_distance),
            "right_distance": float(right_distance)
        }
        
        if self.data_changed("lidar", new_lidar_data):
            with self.data_lock:
                self.system_data["lidar"] = new_lidar_data
            self.queue_broadcast("lidar_update", new_lidar_data)
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles"""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def send_movement(self, linear_x, angular_z):
        """Send movement command"""
        try:
            twist_msg = Twist()
            twist_msg.linear.x = float(linear_x)
            twist_msg.angular.z = float(angular_z)
            self.manual_pub.publish(twist_msg)
            return True
        except Exception as e:
            self.get_logger().error(f'Error sending movement command: {e}')
            return False
    
    def set_mode(self, mode):
        """Set control mode"""
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.last_mode_switch_time < self.cooldown_period:
            return False
        
        try:
            self.last_mode_switch_time = current_time
            mode_msg = String()
            mode_msg.data = mode
            self.mode_pub.publish(mode_msg)
            with self.data_lock:
                self.system_data["mode"] = mode
            self.queue_broadcast("mode_update", {"mode": mode})
            return True
        except Exception as e:
            self.get_logger().error(f'Error setting mode: {e}')
            return False
    
    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        try:
            estop_msg = Bool()
            estop_msg.data = True
            self.estop_pub.publish(estop_msg)
            with self.data_lock:
                self.system_data["emergency_stopped"] = True
            self.queue_broadcast("emergency_update", {"emergency_stopped": True})
            return True
        except Exception as e:
            self.get_logger().error(f'Error triggering emergency stop: {e}')
            return False
    
    def reset_emergency_stop(self):
        """Reset emergency stop"""
        try:
            reset_msg = Bool()
            reset_msg.data = True
            self.reset_estop_pub.publish(reset_msg)
            with self.data_lock:
                self.system_data["emergency_stopped"] = False
            self.queue_broadcast("emergency_update", {"emergency_stopped": False})
            return True
        except Exception as e:
            self.get_logger().error(f'Error resetting emergency stop: {e}')
            return False
    
    def get_system_data(self):
        """Get current system data"""
        with self.data_lock:
            return self.system_data.copy()

# WebSocket server
async def websocket_handler(websocket, path, ros_node):
    """Handle WebSocket connections"""
    try:
        ros_node.add_websocket_client(websocket)
        
        # Send initial data
        initial_data = ros_node.get_system_data()
        await websocket.send(json.dumps({
            "type": "initial_data",
            "data": initial_data
        }))
        
        # Handle incoming messages
        async for message in websocket:
            try:
                data = json.loads(message)
                
                if data["type"] == "move":
                    success = ros_node.send_movement(data.get("linear_x", 0), data.get("angular_z", 0))
                    if not success:
                        await websocket.send(json.dumps({"error": "Failed to send movement command"}))
                        
                elif data["type"] == "mode":
                    success = ros_node.set_mode(data.get("mode", "MANUAL"))
                    if not success:
                        await websocket.send(json.dumps({"error": "Failed to set mode or too soon"}))
                        
                elif data["type"] == "emergency":
                    success = ros_node.trigger_emergency_stop()
                    if not success:
                        await websocket.send(json.dumps({"error": "Failed to trigger emergency stop"}))
                        
                elif data["type"] == "reset_emergency":
                    success = ros_node.reset_emergency_stop()
                    if not success:
                        await websocket.send(json.dumps({"error": "Failed to reset emergency stop"}))
                        
            except json.JSONDecodeError as e:
                await websocket.send(json.dumps({"error": f"Invalid JSON: {str(e)}"}))
            except KeyError as e:
                await websocket.send(json.dumps({"error": f"Missing required field: {str(e)}"}))
            except Exception as e:
                logger.error(f"Error handling WebSocket message: {e}")
                await websocket.send(json.dumps({"error": f"Internal error: {str(e)}"}))
                
    except websockets.exceptions.ConnectionClosed:
        logger.info("WebSocket connection closed normally")
    except Exception as e:
        logger.error(f"WebSocket handler error: {e}")
    finally:
        ros_node.remove_websocket_client(websocket)

async def message_broadcaster(ros_node):
    """Broadcast messages from queue to all WebSocket clients"""
    while True:
        try:
            # Get message from queue with timeout
            try:
                message_data = ros_node.message_queue.get(timeout=0.1)
            except queue.Empty:
                await asyncio.sleep(0.01)
                continue
                
            # Broadcast to all clients
            with ros_node.websocket_lock:
                clients = ros_node.websocket_clients.copy()
            
            if clients:
                message = json.dumps({
                    "type": message_data["type"], 
                    "data": message_data["data"]
                })
                
                # Send to all clients, remove disconnected ones
                disconnected = set()
                for client in clients:
                    try:
                        await client.send(message)
                    except websockets.exceptions.ConnectionClosed:
                        disconnected.add(client)
                    except Exception as e:
                        logger.error(f'Error sending to WebSocket client: {e}')
                        disconnected.add(client)
                
                # Remove disconnected clients
                if disconnected:
                    with ros_node.websocket_lock:
                        for client in disconnected:
                            ros_node.websocket_clients.discard(client)
            
            ros_node.message_queue.task_done()
            
        except Exception as e:
            logger.error(f"Error in message broadcaster: {e}")
            await asyncio.sleep(0.1)

async def run_websocket_server(ros_node):
    """Run WebSocket server with port fallback"""
    ports_to_try = [8765, 8766, 8767, 8768, 8769]
    
    for port in ports_to_try:
        try:
            # Start message broadcaster
            broadcaster_task = asyncio.create_task(message_broadcaster(ros_node))
            
            # Start WebSocket server
            # FIXED: Modified the lambda to handle websockets signature change
            async def handler(websocket):
                await websocket_handler(websocket, None, ros_node)
            
            logger.info(f"Attempting to start WebSocket server on port {port}")
            async with websockets.serve(handler, "0.0.0.0", port, reuse_port=True):
                logger.info(f"WebSocket server successfully started on port {port}")
                await broadcaster_task
                
        except OSError as e:
            if e.errno == 98:  # Address already in use
                logger.warning(f"Port {port} is already in use, trying next port...")
                continue
            else:
                logger.error(f"WebSocket server error on port {port}: {e}")
                raise
        except Exception as e:
            logger.error(f"WebSocket server error on port {port}: {e}")
            raise
    
    raise RuntimeError("Could not start WebSocket server on any available port")

def run_websocket_thread(ros_node):
    """Run WebSocket server in thread with proper event loop"""
    try:
        # Create new event loop for this thread
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        # Run the server
        loop.run_until_complete(run_websocket_server(ros_node))
    except Exception as e:
        logger.error(f"WebSocket thread error: {e}")

# Flask application for web server
app = Flask(__name__)
ros_node = None  # Will be initialized in ROS2 thread

@app.route('/')
def index():
    """Render the main dashboard page"""
    # pass the port of the WebSocket server to the template
    websocket_port = 8765
    return render_template('index.html', websocket_port=websocket_port)

@app.route('/api/status')
def api_status():
    """API endpoint for system status"""
    if ros_node:
        return ros_node.get_system_data()
    else:
        return {"error": "ROS node not initialized"}, 500

def start_flask_server():
    """Start Flask server"""
    try:
        logger.info("Starting Flask server on port 8080")
        app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)
    except Exception as e:
        logger.error(f"Flask server error: {e}")

def check_and_cleanup_ports():
    """Check for and optionally cleanup processes using our ports"""
    ports_to_check = [8765, 8080]
    
    for port in ports_to_check:
        try:
            # Check if port is in use
            result = subprocess.run(['lsof', '-ti', f':{port}'], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0 and result.stdout.strip():
                pids = result.stdout.strip().split('\n')
                logger.warning(f"Port {port} is in use by PIDs: {pids}")
                
                # Optionally kill the processes (be careful with this!)
                # Uncomment the next lines if you want to automatically kill conflicting processes
                # for pid in pids:
                #     try:
                #         os.kill(int(pid), signal.SIGTERM)
                #         logger.info(f"Terminated process {pid} using port {port}")
                #     except:
                #         pass
                        
        except subprocess.TimeoutExpired:
            logger.warning(f"Timeout checking port {port}")
        except FileNotFoundError:
            # lsof not available, skip check
            pass
        except Exception as e:
            logger.warning(f"Error checking port {port}: {e}")

def main():
    global ros_node
    
    # Graceful shutdown flag
    shutdown_complete = False
    
    try:
        # Check for port conflicts before starting
        check_and_cleanup_ports()
        
        rclpy.init()
        ros_node = EcoMarathonDashboardNode()
        
        # Start WebSocket server in separate thread
        websocket_thread = threading.Thread(target=run_websocket_thread, args=(ros_node,), daemon=True)
        websocket_thread.start()
        
        # Start Flask server in separate thread
        flask_thread = threading.Thread(target=start_flask_server, daemon=True)
        flask_thread.start()
        
        logger.info("All servers started, spinning ROS node...")
        
        # Spin ROS node
        rclpy.spin(ros_node)
        
    except KeyboardInterrupt:
        logger.info("Received KeyboardInterrupt, shutting down...")
    except Exception as e:
        logger.error(f"Main thread error: {e}")
    finally:
        if not shutdown_complete:
            shutdown_complete = True
            logger.info("Cleaning up resources...")
            
            # Clear WebSocket clients safely
            if ros_node and hasattr(ros_node, 'websocket_clients'):
                try:
                    with ros_node.websocket_lock:
                        ros_node.websocket_clients.clear()
                except:
                    pass
            
            # Destroy ROS node safely
            if ros_node:
                try:
                    ros_node.destroy_node()
                    logger.info("ROS node destroyed")
                except Exception as e:
                    logger.warning(f"Error destroying node: {e}")
            
            # Shutdown RCL safely
            try:
                if rclpy.ok():
                    rclpy.shutdown()
                    logger.info("RCL shutdown complete")
            except Exception as e:
                logger.warning(f"Error during RCL shutdown: {e}")
            
            logger.info("Shutdown complete")

if __name__ == '__main__':
    main()