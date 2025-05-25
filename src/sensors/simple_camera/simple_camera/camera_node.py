#!/usr/bin/env python3

import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Declare parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('backend', 'v4l2')  # Allow specifying backend (v4l2, any)
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.backend = self.get_parameter('backend').get_parameter_value().string_value
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Set up QoS profile for high-performance publishing
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Create publishers
        self.raw_publisher = self.create_publisher(Image, '/camera/color/image_raw', qos)
        self.compressed_publisher = self.create_publisher(CompressedImage, '/camera/color/image_raw/compressed', qos)
        
        # Initialize camera with backend selection
        self.cap = self.initialize_camera()
        if self.cap is None:
            self.get_logger().error("Camera initialization failed. Shutting down.")
            sys.exit(1)
            
        # Optimize camera settings
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Create timer for high-frequency publishing (100 Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.get_logger().info(f"Camera publisher started with camera index {self.camera_index}, backend {self.backend}")
        
    def initialize_camera(self):
        # Try specified backend or fallback to others
        backends = {
            'v4l2': cv2.CAP_V4L2,
            'any': cv2.CAP_ANY
        }
        
        backend = backends.get(self.backend, cv2.CAP_ANY)
        self.get_logger().info(f"Trying backend: {self.backend}")
        
        cap = cv2.VideoCapture(self.camera_index, backend)
        if not cap.isOpened():
            self.get_logger().warn(f"Failed to open camera with index {self.camera_index} and backend {self.backend}")
            # Try fallback backend if not already using CAP_ANY
            if self.backend != 'any':
                self.get_logger().info("Falling back to CAP_ANY backend")
                cap = cv2.VideoCapture(self.camera_index, cv2.CAP_ANY)
            
            if not cap.isOpened():
                self.get_logger().error(f"Failed to open camera at index {self.camera_index} with any backend")
                return None
                
        return cap
        
    def timer_callback(self):
        # Capture frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return
            
        # Get timestamp
        timestamp = self.get_clock().now().to_msg()
        
        # Publish raw image
        raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        raw_msg.header.stamp = timestamp
        raw_msg.header.frame_id = "camera_frame"
        self.raw_publisher.publish(raw_msg)
        
        # Publish compressed image
        compressed_msg = CompressedImage()
        compressed_msg.header = raw_msg.header
        compressed_msg.format = "jpeg"
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        compressed_msg.data = np.array(cv2.imencode('.jpg', frame, encode_param)[1]).tobytes()
        self.compressed_publisher.publish(compressed_msg)
        
    def destroy_node(self):
        # Release camera when node is destroyed
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()