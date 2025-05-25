#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image, LaserScan  # Keep LaserScan
from nav_msgs.msg import Odometry
from eco_interfaces.msg import SystemState, StopSign, Obstacles, LaneDetection
import threading
import json
import base64
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import asyncio
import websockets
import time
import queue
import signal
import sys
import socket
from flask import Flask, render_template
import logging
import os

# Set up logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger(__name__)
ros_logger = None

shutdown_flag = threading.Event()

# Define sensor keys that match HTML IDs (without "sensor-" and "-status")
SENSOR_KEY_CAMERA = "camera"
SENSOR_KEY_LIDAR = "lidar"
SENSOR_KEY_ODOM = "odom"
SENSOR_KEY_LANE_PERCEPTION = "lane-perception"
SENSOR_KEY_OBJECT_PERCEPTION = "object-perception"  # Combined for simplicity
SENSOR_KEY_STATE_MACHINE_NODE = "state-machine-node"

SENSOR_TIMEOUT_S = 3.0  # Seconds before a sensor is considered timed out


class EcoMarathonDashboardNode(Node):
    def __init__(self):
        super().__init__("eco_marathon_dashboard_node")
        global ros_logger
        ros_logger = self.get_logger()

        self.cv_bridge = CvBridge()
        self.websocket_clients = set()
        self.websocket_lock = threading.Lock()
        self.message_queue = queue.Queue(maxsize=200)
        self.last_data_hash = {}
        self.image_cache = {}
        self.last_image_time = {}
        self.image_update_interval = 0.2
        self.websocket_port = 8765

        # ADDED: Sensor health tracking
        self.sensor_last_msg_time = {
            SENSOR_KEY_CAMERA: 0.0,
            SENSOR_KEY_LIDAR: 0.0,
            SENSOR_KEY_ODOM: 0.0,
            SENSOR_KEY_LANE_PERCEPTION: 0.0,
            SENSOR_KEY_OBJECT_PERCEPTION: 0.0,  # Tracks if either stop_sign or obstacles are received
            SENSOR_KEY_STATE_MACHINE_NODE: 0.0,
        }
        self.sensor_health_lock = threading.Lock()  # Lock for sensor_last_msg_time

        self.manual_pub = self.create_publisher(Twist, "/manual_control", 10)
        self.mode_pub = self.create_publisher(String, "/control_mode", 10)
        self.estop_pub = self.create_publisher(Bool, "/emergency_stop", 10)
        self.reset_estop_pub = self.create_publisher(Bool, "/reset_emergency", 10)

        # Subscribers - will update sensor_last_msg_time
        # For camera, we'll use the debug_image as a proxy for camera activity
        self.debug_image_sub = self.create_subscription(
            Image,
            "/lane_detection/debug_image",
            self.debug_image_callback_with_health,
            5,
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback_with_health, 5
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odometry_callback_with_health, 10
        )
        self.lane_detection_sub = self.create_subscription(
            LaneDetection,
            "/perception/lane_detections",
            self.lane_detection_callback_with_health,
            10,
        )
        self.stop_sign_sub = self.create_subscription(
            StopSign,
            "/perception/stop_sign_detected",
            self.stop_sign_callback_with_health,
            10,
        )
        self.obstacles_sub = self.create_subscription(
            Obstacles, "/perception/obstacles", self.obstacles_callback_with_health, 10
        )
        self.state_sub = self.create_subscription(
            SystemState,
            "/state_machine/current_state",
            self.state_callback_with_health,
            10,
        )

        # Original image subs (without health, just for image data)
        self.lane_mask_sub = self.create_subscription(
            Image, "/lane_detection/lane_mask", self.lane_mask_callback, 5
        )

        self.system_data = {
            "mode": "MANUAL",
            "emergency_stopped": False,
            "state_machine": {"current_state": 0, "state_name": "IDLE"},
            "lane_detection": {
                "center_offset": 0.0,
                "heading_error": 0.0,
                "left_line_detected": False,
                "right_line_detected": False,
            },
            "stop_sign": {
                "detected": False,
                "distance": 0.0,
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
            "obstacles": {"count": 0, "nearest_distance": 0.0, "obstacles": []},
            "odometry": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                "linear_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
            "lidar": {
                "min_range": 0.0,
                "max_range": 0.0,
                "front_distance": 0.0,
                "left_distance": 0.0,
                "right_distance": 0.0,
            },
            "control": {"linear_speed": 0.0, "angular_speed": 0.0},
            "sensor_health": {
                key: "UNKNOWN" for key in self.sensor_last_msg_time.keys()
            },  # ADDED
        }
        self.data_lock = threading.Lock()
        self.last_mode_switch_time = 0
        self.cooldown_period = 1.0

        # ADDED: Timer for checking sensor health
        self.sensor_health_timer = self.create_timer(
            1.0, self.check_sensor_health_and_broadcast
        )

        ros_logger.info(
            "Eco Marathon Dashboard Node initialized with sensor health monitoring."
        )

    # --- Sensor Health Update Wrappers for Callbacks ---
    def update_sensor_timestamp(self, sensor_key):
        with self.sensor_health_lock:
            self.sensor_last_msg_time[sensor_key] = time.time()

    def debug_image_callback_with_health(self, msg):
        self.update_sensor_timestamp(SENSOR_KEY_CAMERA)
        self._process_image(
            msg, "debug_image", desired_encoding="bgr8"
        )  # Original processing

    def scan_callback_with_health(self, msg):
        self.update_sensor_timestamp(SENSOR_KEY_LIDAR)
        self.scan_callback(msg)  # Call original

    def odometry_callback_with_health(self, msg):
        self.update_sensor_timestamp(SENSOR_KEY_ODOM)
        self.odometry_callback(msg)  # Call original

    def lane_detection_callback_with_health(self, msg):
        self.update_sensor_timestamp(SENSOR_KEY_LANE_PERCEPTION)
        self.lane_detection_callback(msg)  # Call original

    def stop_sign_callback_with_health(self, msg):
        self.update_sensor_timestamp(
            SENSOR_KEY_OBJECT_PERCEPTION
        )  # Update common object perception key
        self.stop_sign_callback(msg)  # Call original

    def obstacles_callback_with_health(self, msg):
        self.update_sensor_timestamp(
            SENSOR_KEY_OBJECT_PERCEPTION
        )  # Update common object perception key
        self.obstacles_callback(msg)  # Call original

    def state_callback_with_health(self, msg):
        self.update_sensor_timestamp(SENSOR_KEY_STATE_MACHINE_NODE)
        self.state_callback(msg)  # Call original

    # --- Periodic Sensor Health Check ---
    def check_sensor_health_and_broadcast(self):
        current_time = time.time()
        new_health_status = {}
        with self.sensor_health_lock:  # Access timestamps safely
            for key, last_time in self.sensor_last_msg_time.items():
                if last_time == 0.0:  # Never received
                    new_health_status[key] = "UNKNOWN"
                elif (current_time - last_time) > SENSOR_TIMEOUT_S:
                    new_health_status[key] = "TIMEOUT"
                else:
                    new_health_status[key] = "OK"

        # Check if health status actually changed before broadcasting
        if self.data_changed("sensor_health", new_health_status):
            with self.data_lock:
                self.system_data["sensor_health"] = new_health_status
            self.queue_broadcast("sensor_health_update", new_health_status)
            # ros_logger.info(f"Sensor Health Update: {new_health_status}")

    def add_websocket_client(self, websocket):
        with self.websocket_lock:
            self.websocket_clients.add(websocket)
        ros_logger.info(
            f"WS Client connected: {websocket.remote_address}. Total: {len(self.websocket_clients)}"
        )

    def remove_websocket_client(self, websocket):
        with self.websocket_lock:
            self.websocket_clients.discard(websocket)
        ros_logger.info(
            f"WS Client disconnected: {websocket.remote_address}. Total: {len(self.websocket_clients)}"
        )

    def queue_broadcast(self, data_type, data):
        msg_to_queue = {"type": data_type, "data": data, "timestamp": time.time()}
        try:
            self.message_queue.put(msg_to_queue, block=False)
        except queue.Full:
            try:
                self.message_queue.get_nowait()
                self.message_queue.put_nowait(msg_to_queue)
                ros_logger.warn(f"MsgQ full. Discarded oldest, queued: {data_type}")
            except queue.Empty:
                ros_logger.warn(f"MsgQ full, couldn't discard. Dropping: {data_type}")

    def data_changed(self, data_key, new_data):
        try:
            data_str = json.dumps(new_data, sort_keys=True)
            with self.data_lock:
                changed = self.last_data_hash.get(data_key) != data_str
                if changed:
                    self.last_data_hash[data_key] = data_str
                return changed
        except TypeError as e:
            ros_logger.error(f"JSON error in data_changed for {data_key}: {e}")
            return True

    def state_callback(self, msg):  # Original logic
        state_names = [
            "IDLE",
            "SECTION1_LANE_FOLLOWING",
            "APPROACHING_STOP_SIGN_SECTION1",
            "STOPPED_SECTION1",
            "SECTION2_OBSTACLE_AVOIDANCE",
            "APPROACHING_STOP_SIGN_SECTION2",
            "STOPPED_SECTION2",
            "SECTION3_PARKING",
            "PARKED",
            "EMERGENCY_STOP",
        ]
        state_num = msg.state
        state_name = (
            state_names[state_num]
            if 0 <= state_num < len(state_names)
            else f"UNKNOWN_{state_num}"
        )
        new_state_data = {"current_state": state_num, "state_name": state_name}
        if self.data_changed("state_machine", new_state_data):
            with self.data_lock:
                self.system_data["state_machine"] = new_state_data
            self.queue_broadcast("state_update", new_state_data)

    def lane_detection_callback(self, msg):  # Original logic
        new_lane_data = {
            "center_offset": float(msg.lane_center_offset),
            "heading_error": float(msg.lane_heading_error),
            "left_line_detected": len(msg.left_line_x) > 0,
            "right_line_detected": len(msg.right_line_x) > 0,
        }
        if self.data_changed("lane_detection", new_lane_data):
            with self.data_lock:
                self.system_data["lane_detection"] = new_lane_data
            self.queue_broadcast("lane_update", new_lane_data)

    def _process_image(self, msg, image_type_key, desired_encoding="bgr8", color_conversion_code=None):
        current_time = time.time()
        # ros_logger.debug(f"Attempting to process {image_type_key} at {current_time}") # DEBUG
        if current_time - self.last_image_time.get(image_type_key, 0) < self.image_update_interval:
            # ros_logger.debug(f"Skipping {image_type_key} due to rate limit.") # DEBUG
            return
        try:
            # ros_logger.debug(f"Converting {image_type_key} from ROS msg...") # DEBUG
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding=desired_encoding)
            # ros_logger.debug(f"Resizing {image_type_key}...") # DEBUG
            cv_image = cv2.resize(cv_image, (320, 240))
            if color_conversion_code is not None:
                # ros_logger.debug(f"Color converting {image_type_key}...") # DEBUG
                cv_image = cv2.cvtColor(cv_image, color_conversion_code)
            
            # ros_logger.debug(f"Encoding {image_type_key} to JPEG...") # DEBUG
            _, buffer = cv2.imencode(".jpg", cv_image, [cv2.IMWRITE_JPEG_QUALITY, 60])
            image_b64 = base64.b64encode(buffer).decode("utf-8")
            # ros_logger.debug(f"{image_type_key} b64 length: {len(image_b64)}") # DEBUG
            
            with self.data_lock: 
                cached_img_b64 = self.image_cache.get(image_type_key, None)
            
            # Simplified: Always send if different from last fully cached, or if never cached
            # This is more robust than prefix checking for debugging
            if cached_img_b64 != image_b64:
                with self.data_lock: 
                    self.image_cache[image_type_key] = image_b64
                self.last_image_time[image_type_key] = current_time
                self.queue_broadcast("image_update", {"type": image_type_key, "data": image_b64})
                # ros_logger.info(f"Sent updated {image_type_key} (full diff or new)") # DEBUG
            # else:
                # ros_logger.debug(f"Skipped sending identical {image_type_key} (full diff)") # DEBUG

        except CvBridgeError as e:
            ros_logger.error(f"CvBridge Error processing {image_type_key} image: {e}")
        except cv2.error as e:
            ros_logger.error(f"OpenCV Error processing {image_type_key} image: {e}")
        except Exception as e:
            ros_logger.error(f"Generic error processing {image_type_key} image: {e}", exc_info=True)


    def lane_mask_callback(self, msg):  # Original logic
        self._process_image(
            msg,
            "lane_mask",
            desired_encoding="mono8",
            color_conversion_code=cv2.COLOR_GRAY2RGB,
        )

    # debug_image_callback is now debug_image_callback_with_health

    def stop_sign_callback(self, msg):  # Original logic
        new_stop_data = {
            "detected": msg.detected,
            "distance": float(msg.distance),
            "position": {
                "x": float(msg.position.x),
                "y": float(msg.position.y),
                "z": float(msg.position.z),
            },
        }
        if self.data_changed("stop_sign", new_stop_data):
            with self.data_lock:
                self.system_data["stop_sign"] = new_stop_data
            self.queue_broadcast("stop_sign_update", new_stop_data)

    def obstacles_callback(self, msg):  # Original logic
        obstacles, min_distance = [], float("inf")
        for obs in msg.obstacles:
            dist = np.sqrt(obs.center.x**2 + obs.center.y**2)
            min_distance = min(min_distance, dist)
            obstacles.append(
                {
                    "center": {
                        "x": float(obs.center.x),
                        "y": float(obs.center.y),
                        "z": float(obs.center.z),
                    },
                    "radius": float(obs.radius),
                    "distance": float(dist),
                }
            )
        new_obstacle_data = {
            "count": len(obstacles),
            "nearest_distance": float(min_distance) if obstacles else 0.0,
            "obstacles": obstacles,
        }
        if self.data_changed("obstacles", new_obstacle_data):
            with self.data_lock:
                self.system_data["obstacles"] = new_obstacle_data
            self.queue_broadcast("obstacles_update", new_obstacle_data)

    def odometry_callback(self, msg):  # Original logic
        pos, orient = msg.pose.pose.position, msg.pose.pose.orientation
        roll, pitch, yaw = self.quaternion_to_euler(
            orient.x, orient.y, orient.z, orient.w
        )
        linear, angular = msg.twist.twist.linear, msg.twist.twist.angular
        new_odom_data = {
            "position": {"x": float(pos.x), "y": float(pos.y), "z": float(pos.z)},
            "orientation": {
                "roll": float(roll),
                "pitch": float(pitch),
                "yaw": float(yaw),
            },
            "linear_velocity": {
                "x": float(linear.x),
                "y": float(linear.y),
                "z": float(linear.z),
            },
            "angular_velocity": {
                "x": float(angular.x),
                "y": float(angular.y),
                "z": float(angular.z),
            },
        }
        speed = np.sqrt(linear.x**2 + linear.y**2)
        new_control_data = {
            "linear_speed": float(speed),
            "angular_speed": float(angular.z),
        }
        odom_changed = self.data_changed("odometry_data_only", new_odom_data)
        control_changed = self.data_changed("control_data_only", new_control_data)
        if odom_changed or control_changed:
            with self.data_lock:
                if odom_changed:
                    self.system_data["odometry"] = new_odom_data
                if control_changed:
                    self.system_data["control"] = new_control_data
            self.queue_broadcast(
                "odometry_update",
                {"odometry": new_odom_data, "control": new_control_data},
            )

    def scan_callback(self, msg):  # Original logic
        num_points = len(msg.ranges)
        if num_points == 0:
            return
        front_idx, left_idx, right_idx = (
            num_points // 2,
            (num_points // 2 + num_points // 4) % num_points,
            (num_points // 2 - num_points // 4 + num_points) % num_points,
        )
        new_lidar_data = {
            "min_range": float(msg.range_min),
            "max_range": float(msg.range_max),
            "front_distance": (
                float(msg.ranges[front_idx])
                if msg.ranges[front_idx] < msg.range_max
                else msg.range_max
            ),
            "left_distance": (
                float(msg.ranges[left_idx])
                if msg.ranges[left_idx] < msg.range_max
                else msg.range_max
            ),
            "right_distance": (
                float(msg.ranges[right_idx])
                if msg.ranges[right_idx] < msg.range_max
                else msg.range_max
            ),
        }
        if self.data_changed("lidar", new_lidar_data):
            with self.data_lock:
                self.system_data["lidar"] = new_lidar_data
            self.queue_broadcast("lidar_update", new_lidar_data)

    def quaternion_to_euler(self, x, y, z, w):  # Original logic
        sinr_cosp, cosr_cosp = 2 * (w * x + y * z), 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp) if abs(sinp) < 1 else np.copysign(np.pi / 2, sinp)
        siny_cosp, cosy_cosp = 2 * (w * z + x * y), 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def send_movement(self, linear_x, angular_z):  # Original logic
        try:
            self.manual_pub.publish(
                Twist(
                    linear=Twist().linear.__class__(x=float(linear_x)),
                    angular=Twist().angular.__class__(z=float(angular_z)),
                )
            )
            return True
        except Exception as e:
            ros_logger.error(f"Error sending movement: {e}")
            return False

    def set_mode(self, mode):
        current_time_ns = self.get_clock().now().seconds_nanoseconds()
        current_time_sec = float(current_time_ns[0]) + float(current_time_ns[1]) / 1e9 # More precise
        
        ros_logger.info(f"Set_mode called. Current time: {current_time_sec}, Last switch: {self.last_mode_switch_time}, Cooldown: {self.cooldown_period}") # DEBUG

        if (current_time_sec - self.last_mode_switch_time) < self.cooldown_period:
            ros_logger.warn(f"Mode switch cooldown. Wait for {self.cooldown_period - (current_time_sec - self.last_mode_switch_time):.2f}s more.")
            return False
        try:
            self.last_mode_switch_time = current_time_sec # UPDATE last_mode_switch_time HERE
            
            mode_msg = String(data=mode)
            self.mode_pub.publish(mode_msg)
            with self.data_lock: 
                self.system_data["mode"] = mode
            self.queue_broadcast("mode_update", {"mode": mode})
            ros_logger.info(f"Mode set to {mode}")
            return True
        except Exception as e: 
            ros_logger.error(f"Error setting mode: {e}"); 
            return False


    def trigger_emergency_stop(self):  # Original logic
        try:
            self.estop_pub.publish(Bool(data=True))
            with self.data_lock:
                self.system_data["emergency_stopped"] = True
            self.queue_broadcast("emergency_update", {"emergency_stopped": True})
            ros_logger.warn("E-stop triggered")
            return True
        except Exception as e:
            ros_logger.error(f"Error triggering e-stop: {e}")
            return False

    def reset_emergency_stop(self):  # Original logic
        try:
            self.reset_estop_pub.publish(Bool(data=True))
            with self.data_lock:
                self.system_data["emergency_stopped"] = False
            self.queue_broadcast("emergency_update", {"emergency_stopped": False})
            ros_logger.info("E-stop reset")
            return True
        except Exception as e:
            ros_logger.error(f"Error resetting e-stop: {e}")
            return False

    def get_system_data(self):  # Original logic
        with self.data_lock:
            return self.system_data.copy()


def find_free_port(start_port=8765, max_attempts=10):  # Original logic
    for port_offset in range(max_attempts):
        port = start_port + port_offset
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind(("", port))
                logger.info(f"Found free port: {port}")
                return port
        except OSError as e:
            if e.errno == 98:
                logger.warning(f"Port {port} in use, trying next.")
            else:
                logger.error(f"Error binding to port {port}: {e}")
            continue
    logger.error(
        f"Could not find a free port after {max_attempts} attempts from {start_port}."
    )
    return None


async def websocket_actual_handler(
    websocket_connection, ros_node_instance
):  # Original logic
    try:
        ros_node_instance.add_websocket_client(websocket_connection)
        initial_data = ros_node_instance.get_system_data()
        await websocket_connection.send(
            json.dumps({"type": "initial_data", "data": initial_data})
        )
        async for message_str in websocket_connection:
            try:
                data = json.loads(message_str)
                msg_type = data.get("type")
                if msg_type == "move":
                    ros_node_instance.send_movement(
                        data.get("linear_x", 0.0), data.get("angular_z", 0.0)
                    )
                elif msg_type == "mode":
                    ros_node_instance.set_mode(data.get("mode", "MANUAL"))
                elif msg_type == "emergency":
                    ros_node_instance.trigger_emergency_stop()
                elif msg_type == "reset_emergency":
                    ros_node_instance.reset_emergency_stop()
                else:
                    await websocket_connection.send(
                        json.dumps({"error": f"Unknown type: {msg_type}"})
                    )
            except json.JSONDecodeError:
                await websocket_connection.send(json.dumps({"error": "Invalid JSON"}))
            except Exception as e:
                ros_logger.error(f"Error processing WS message: {e}")
                await websocket_connection.send(json.dumps({"error": "Server error"}))
    except websockets.exceptions.ConnectionClosed:
        ros_logger.info(
            f"WS Connection closed for {websocket_connection.remote_address}"
        )
    except Exception as e:
        ros_logger.error(
            f"WS handler error for {websocket_connection.remote_address}: {e}"
        )
    finally:
        ros_node_instance.remove_websocket_client(websocket_connection)


async def message_broadcaster_async(ros_node_instance):  # Original logic
    logger.info("Message broadcaster started.")
    while not shutdown_flag.is_set():
        try:
            message_item = ros_node_instance.message_queue.get(timeout=0.1)
        except queue.Empty:
            await asyncio.sleep(0.01)
            continue
        with ros_node_instance.websocket_lock:
            clients_copy = list(ros_node_instance.websocket_clients)
        if not clients_copy:
            ros_node_instance.message_queue.task_done()
            continue
        payload = json.dumps(
            {"type": message_item["type"], "data": message_item["data"]}
        )
        tasks = [client.send(payload) for client in clients_copy]
        results = await asyncio.gather(*tasks, return_exceptions=True)
        for client, result in zip(clients_copy, results):
            if isinstance(result, Exception):
                ros_logger.warn(
                    f"Send fail to {client.remote_address}: {result}. Removing."
                )
                ros_node_instance.remove_websocket_client(client)
        ros_node_instance.message_queue.task_done()
    logger.info("Message broadcaster stopped.")


def run_websocket_server_thread(ros_node_instance):  # Original logic
    logger.info("WebSocket server thread started.")
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    ws_port = find_free_port(8765)
    if ws_port is None:
        logger.error("WS Port find fail. Exiting WS thread.")
        return
    ros_node_instance.websocket_port = ws_port

    async def main_server_logic():
        async def serve_adapter(websocket_connection):
            await websocket_actual_handler(websocket_connection, ros_node_instance)

        server_object = await websockets.serve(
            serve_adapter, "0.0.0.0", ws_port, reuse_port=True
        )
        logger.info(f"WebSocket server starting on port {ws_port}...")
        broadcaster_task = loop.create_task(
            message_broadcaster_async(ros_node_instance)
        )
        try:
            while not shutdown_flag.is_set():
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            logger.info("WebSocket server main logic cancelled.")
        finally:
            logger.info("Closing WebSocket server...")
            server_object.close()
            await server_object.wait_closed()
            logger.info("WebSocket server closed.")
            if broadcaster_task and not broadcaster_task.done():
                broadcaster_task.cancel()
                try:
                    await asyncio.wait_for(broadcaster_task, timeout=2.0)
                except (asyncio.CancelledError, asyncio.TimeoutError):
                    logger.info("Broadcaster task cancel processed/timed out.")

    try:
        loop.run_until_complete(main_server_logic())
    except KeyboardInterrupt:
        logger.info("WS server thread interrupted.")
    except Exception as e:
        logger.error(f"Critical error in WS server thread: {e}", exc_info=True)
    finally:
        logger.info("WS server thread shutting down loop.")
        for task in asyncio.all_tasks(loop):
            if not task.done():
                task.cancel()
                try:
                    loop.run_until_complete(asyncio.wait_for(task, timeout=0.5))
                except (asyncio.CancelledError, asyncio.TimeoutError, Exception):
                    pass
        loop.close()


app = Flask(__name__)
ros_node_flask_ref = None


@app.route("/")
def index_route():  # Original logic
    ws_port = getattr(ros_node_flask_ref, "websocket_port", 8765)
    return render_template("index.html", websocket_port=ws_port)


def run_flask_server_thread():  # Original logic
    logger.info("Flask server thread started.")
    try:
        app.run(
            host="0.0.0.0", port=8080, debug=False, use_reloader=False, threaded=True
        )
    except Exception as e:
        logger.error(f"Flask server failed: {e}", exc_info=True)
    logger.info("Flask server thread stopped.")


def signal_handler_main(signum, frame):  # Original logic
    logger.info(
        f"Signal {signal.strsignal(signum) if hasattr(signal, 'strsignal') else signum} received. Initiating shutdown..."
    )
    shutdown_flag.set()


def main_node_entry():  # Original logic
    global ros_node_flask_ref
    signal.signal(signal.SIGINT, signal_handler_main)
    signal.signal(signal.SIGTERM, signal_handler_main)
    rclpy.init()
    node = EcoMarathonDashboardNode()
    ros_node_flask_ref = node
    ws_thread = threading.Thread(
        target=run_websocket_server_thread, args=(node,), daemon=True
    )
    flask_thread = threading.Thread(target=run_flask_server_thread, daemon=True)
    ws_thread.start()
    time.sleep(1.5)
    flask_thread.start()
    try:
        while rclpy.ok() and not shutdown_flag.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        logger.info("Spin loop interrupted by KeyboardInterrupt.")
    except Exception as e:
        logger.error(f"Error in ROS spin loop: {e}", exc_info=True)
    finally:
        logger.info("ROS spin loop finished. Cleaning up...")
        if not shutdown_flag.is_set():
            shutdown_flag.set()
        if ws_thread.is_alive():
            logger.info("Waiting for WebSocket thread to join...")
            ws_thread.join(timeout=5.0)
            if ws_thread.is_alive():
                logger.warn("WebSocket thread did not join in time.")
        if node and rclpy.ok():
            try:
                node.destroy_node()
                logger.info("ROS node destroyed")
            except Exception as e:
                logger.error(f"Error destroying ROS node: {e}")
        if rclpy.ok():
            try:
                rclpy.shutdown()
                logger.info("RCLpy shutdown complete")
            except Exception as e:
                logger.error(f"Error during RCLpy shutdown: {e}")
        logger.info("Main thread exiting.")


if __name__ == "__main__":
    main_node_entry()
