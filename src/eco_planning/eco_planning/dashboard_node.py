#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32
from eco_interfaces.msg import LaneDetection, Obstacles, StopSign, ParkingArea, SystemState
from cv_bridge import CvBridge
import numpy as np
import json
import threading
import websockets
import asyncio
import cv2
import base64
import time
from functools import partial
import queue  # Standard Python queue for thread-safe communication

# Global variables for WebSocket server
connected_clients = set()
dashboard_data = {}
command_queue = queue.Queue()  # Thread-safe queue for commands

# Standalone WebSocket server function
async def websocket_server(host="0.0.0.0", port=8765):
    # Updated handler with correct signature - accepts just the websocket
    async def handler(websocket):
        # Register client
        connected_clients.add(websocket)
        try:
            async for message in websocket:
                # Process incoming messages
                try:
                    data = json.loads(message)
                    command_queue.put(data)  # Use standard queue
                except json.JSONDecodeError:
                    print("Received invalid JSON message")
                except Exception as e:
                    print(f"Error processing message: {str(e)}")
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            # Unregister client
            connected_clients.remove(websocket)

    # Updated server creation for newer websockets versions
    server = await websockets.serve(handler, host, port)
    
    # Send updates loop
    while True:
        # Send dashboard updates periodically
        if dashboard_data and connected_clients:
            data_to_send = json.dumps({
                'type': 'dashboard_update',
                'data': dashboard_data
            })
            
            # Send to all clients without using broadcast
            websockets_tasks = []
            for websocket in connected_clients:
                websockets_tasks.append(asyncio.create_task(websocket.send(data_to_send)))
            
            if websockets_tasks:
                await asyncio.gather(*websockets_tasks, return_exceptions=True)
        
        await asyncio.sleep(0.1)  # Send updates at 10Hz

# Run WebSocket server in a separate thread
def run_websocket_server_thread():
    print("WebSocket server thread starting")
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(websocket_server())
    except Exception as e:
        print(f"WebSocket server error: {str(e)}")
    finally:
        print("WebSocket server thread ending")

# ROS Node definition
class EcoMarathonDashboardNode(Node):
    def __init__(self):
        super().__init__('eco_marathon_dashboard_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize data storage
        global dashboard_data
        dashboard_data = {
            'system_state': {
                'state': 'IDLE',
                'state_id': 0,
                'timestamp': 0
            },
            'sensors': {
                'camera': {'connected': False, 'last_update': 0},
                'lidar': {'connected': False, 'last_update': 0},
                'imu': {'connected': False, 'last_update': 0}
            },
            'nodes': {
                'lane_detection': {'active': False, 'last_update': 0},
                'obstacle_detection': {'active': False, 'last_update': 0},
                'stop_sign_detection': {'active': False, 'last_update': 0},
                'state_machine': {'active': False, 'last_update': 0},
                'section1_planner': {'active': False, 'last_update': 0},
                'section2_planner': {'active': False, 'last_update': 0},
                'longitudinal_controller': {'active': False, 'last_update': 0},
                'lateral_controller': {'active': False, 'last_update': 0}
            },
            'control': {
                'mode': 'MANUAL',  # MANUAL or AUTONOMOUS
                'emergency_stop': False,
                'current_speed': 0.0,
                'target_speed': 0.0,
                'steering_angle': 0.0
            },
            'perception': {
                'lane_center_offset': 0.0,
                'lane_heading_error': 0.0,
                'obstacles_count': 0,
                'stop_sign_detected': False,
                'stop_sign_distance': 0.0,
                'parking_area_detected': False
            },
            'images': {
                'lane_detection': None,
                'debug_image': None
            }
        }
        
        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscribers for all relevant topics
        
        # Camera topics
        self.camera_sub = self.create_subscription(
            Image, 
            '/camera/color/image_raw', 
            self.camera_callback, 
            sensor_qos)
        
        # Perception topics
        self.lane_detection_sub = self.create_subscription(
            LaneDetection,
            '/perception/lane_detections',
            self.lane_detection_callback,
            10)
        
        self.lane_mask_sub = self.create_subscription(
            Image,
            '/lane_detection/lane_mask',
            self.lane_mask_callback,
            10)
        
        self.debug_image_sub = self.create_subscription(
            Image,
            '/lane_detection/debug_image',
            self.debug_image_callback,
            10)
        
        self.stop_sign_sub = self.create_subscription(
            StopSign,
            '/perception/stop_sign_detected',
            self.stop_sign_callback,
            10)
        
        self.obstacles_sub = self.create_subscription(
            Obstacles,
            '/perception/obstacles',
            self.obstacles_callback,
            10)
        
        # State machine topic
        self.state_sub = self.create_subscription(
            SystemState,
            '/state_machine/current_state',
            self.state_callback,
            10)
        
        # Control topics
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odometry_callback,
            10)
        
        self.steering_sub = self.create_subscription(
            Float32,
            '/sensor/steering_angle',
            self.steering_callback,
            10)
        
        # LiDAR topic
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos)
        
        # IMU topic
        self.imu_sub = self.create_subscription(
            Odometry,  # Assuming IMU data is published as part of odometry
            '/odometry/filtered',
            self.imu_callback,
            sensor_qos)
        
        # Publishers for control
        self.manual_control_pub = self.create_publisher(
            Twist,
            '/manual_control',
            10)
        
        self.control_mode_pub = self.create_publisher(
            String,
            '/control_mode',
            10)
        
        self.emergency_stop_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            10)
        
        self.reset_emergency_pub = self.create_publisher(
            Bool,
            '/reset_emergency',
            10)
        
        # Timer for checking node health
        self.node_health_timer = self.create_timer(1.0, self.check_node_health)
        
        # Timer for checking command queue
        self.command_timer = self.create_timer(0.05, self.process_commands)
        
        self.get_logger().info('Dashboard node initialized')
    
    def camera_callback(self, msg):
        global dashboard_data
        dashboard_data['sensors']['camera']['connected'] = True
        dashboard_data['sensors']['camera']['last_update'] = time.time()
    
    def scan_callback(self, msg):
        global dashboard_data
        dashboard_data['sensors']['lidar']['connected'] = True
        dashboard_data['sensors']['lidar']['last_update'] = time.time()
    
    def imu_callback(self, msg):
        global dashboard_data
        dashboard_data['sensors']['imu']['connected'] = True
        dashboard_data['sensors']['imu']['last_update'] = time.time()
    
    def lane_detection_callback(self, msg):
        global dashboard_data
        dashboard_data['nodes']['lane_detection']['active'] = True
        dashboard_data['nodes']['lane_detection']['last_update'] = time.time()
        
        dashboard_data['perception']['lane_center_offset'] = msg.lane_center_offset
        dashboard_data['perception']['lane_heading_error'] = msg.lane_heading_error
    
    def lane_mask_callback(self, msg):
        global dashboard_data
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            # Convert to color for better visualization
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            # Resize for efficiency
            cv_image = cv2.resize(cv_image, (320, 240))
            # Convert to base64 for sending over WebSocket
            _, buffer = cv2.imencode('.jpg', cv_image)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            dashboard_data['images']['lane_detection'] = jpg_as_text
        except Exception as e:
            self.get_logger().error(f"Error processing lane mask: {str(e)}")
    
    def debug_image_callback(self, msg):
        global dashboard_data
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Resize for efficiency
            cv_image = cv2.resize(cv_image, (320, 240))
            # Convert to base64 for sending over WebSocket
            _, buffer = cv2.imencode('.jpg', cv_image)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            dashboard_data['images']['debug_image'] = jpg_as_text
        except Exception as e:
            self.get_logger().error(f"Error processing debug image: {str(e)}")
    
    def stop_sign_callback(self, msg):
        global dashboard_data
        dashboard_data['nodes']['stop_sign_detection']['active'] = True
        dashboard_data['nodes']['stop_sign_detection']['last_update'] = time.time()
        
        dashboard_data['perception']['stop_sign_detected'] = msg.detected
        dashboard_data['perception']['stop_sign_distance'] = msg.distance
    
    def obstacles_callback(self, msg):
        global dashboard_data
        dashboard_data['nodes']['obstacle_detection']['active'] = True
        dashboard_data['nodes']['obstacle_detection']['last_update'] = time.time()
        
        dashboard_data['perception']['obstacles_count'] = len(msg.obstacles)
    
    def state_callback(self, msg):
        global dashboard_data
        dashboard_data['nodes']['state_machine']['active'] = True
        dashboard_data['nodes']['state_machine']['last_update'] = time.time()
        
        state_names = [
            'IDLE',
            'SECTION1_LANE_FOLLOWING',
            'APPROACHING_STOP_SIGN_SECTION1',
            'STOPPED_SECTION1',
            'SECTION2_OBSTACLE_AVOIDANCE',
            'APPROACHING_STOP_SIGN_SECTION2',
            'STOPPED_SECTION2',
            'SECTION3_PARKING',
            'PARKED',
            'EMERGENCY_STOP'
        ]
        
        state_id = msg.state
        if 0 <= state_id < len(state_names):
            dashboard_data['system_state']['state'] = state_names[state_id]
        else:
            dashboard_data['system_state']['state'] = 'UNKNOWN'
        
        dashboard_data['system_state']['state_id'] = state_id
        dashboard_data['system_state']['timestamp'] = time.time()
    
    def odometry_callback(self, msg):
        global dashboard_data
        # Extract speed from odometry
        speed = np.sqrt(
            msg.twist.twist.linear.x**2 + 
            msg.twist.twist.linear.y**2
        )
        dashboard_data['control']['current_speed'] = speed
    
    def steering_callback(self, msg):
        global dashboard_data
        dashboard_data['control']['steering_angle'] = msg.data
    
    def check_node_health(self):
        global dashboard_data
        # Check if nodes are active based on last update time
        current_time = time.time()
        timeout = 2.0  # seconds
        
        for node_name, node_data in dashboard_data['nodes'].items():
            if current_time - node_data['last_update'] > timeout:
                node_data['active'] = False
        
        # Check if sensors are connected based on last update time
        for sensor_name, sensor_data in dashboard_data['sensors'].items():
            if current_time - sensor_data['last_update'] > timeout:
                sensor_data['connected'] = False
    
    def process_commands(self):
        # Process commands from the WebSocket queue
        try:
            # Use non-blocking get to check if there are commands
            try:
                data = command_queue.get(block=False)
                self.handle_command(data)
            except queue.Empty:
                pass  # No command available
        except Exception as e:
            self.get_logger().error(f"Error checking command queue: {str(e)}")
    
    def handle_command(self, data):
        global dashboard_data
        try:
            self.get_logger().info(f"Received command: {data} - {type(data)} - mode: {dashboard_data['control']['mode']}")
            if 'type' in data:
                if data['type'] == 'control_mode':
                    # Handle control mode change
                    mode = data['mode']
                    dashboard_data['control']['mode'] = mode
                    
                    # Publish control mode
                    mode_msg = String()
                    mode_msg.data = mode
                    self.control_mode_pub.publish(mode_msg)
                    
                    self.get_logger().info(f"Control mode changed to {mode}")
                
                elif data['type'] == 'manual_control':
                    # Handle manual control commands
                    if dashboard_data['control']['mode'] == 'MANUAL':
                        cmd = Twist()
                        cmd.linear.x = float(data['linear_x'])
                        cmd.angular.z = float(data['angular_z'])
                        self.manual_control_pub.publish(cmd)
                
                elif data['type'] == 'emergency_stop':
                    # Handle emergency stop
                    stop = Bool()
                    stop.data = True
                    self.emergency_stop_pub.publish(stop)
                    dashboard_data['control']['emergency_stop'] = True
                    self.get_logger().warn("Emergency stop activated")
                
                elif data['type'] == 'reset_emergency':
                    # Handle emergency stop reset
                    reset = Bool()
                    reset.data = True
                    self.reset_emergency_pub.publish(reset)
                    dashboard_data['control']['emergency_stop'] = False
                    self.get_logger().info("Emergency stop reset")
        except Exception as e:
            self.get_logger().error(f"Error handling command: {str(e)}")

def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)
    
    # Start WebSocket server in a separate thread
    print("Starting WebSocket server thread")
    ws_thread = threading.Thread(target=run_websocket_server_thread, daemon=True)
    ws_thread.start()
    
    # Create and run the ROS node
    print("Starting dashboard node")
    node = EcoMarathonDashboardNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()