#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from eco_interfaces.msg import StopSign
from geometry_msgs.msg import Point

class StopSignDetectionNode(Node):
    def __init__(self):
        super().__init__('stop_sign_detection_node')
        
        # Parameters
        self.declare_parameter('stop_sign_cascade_path', 'haarcascades/haarcascade_stop.xml')
        self.declare_parameter('camera_focal_length', 800.0)  # Focal length in pixels (needs calibration)
        self.declare_parameter('stop_sign_real_width', 0.6)  # Actual width of a stop sign in meters (needs verification)
        self.declare_parameter('min_detection_width', 30)  # Minimum pixel width for a detected stop sign
        
        self.cascade_path = self.get_parameter('stop_sign_cascade_path').value
        self.focal_length = self.get_parameter('camera_focal_length').value
        self.stop_sign_real_width = self.get_parameter('stop_sign_real_width').value
        self.min_detection_width = self.get_parameter('min_detection_width').value
        
        # Load the Haar cascade classifier for stop signs
        try:
            self.stop_sign_cascade = cv2.CascadeClassifier(self.cascade_path)
            if self.stop_sign_cascade.empty():
                self.get_logger().error(f"Failed to load stop sign cascade classifier from {self.cascade_path}")
                self.stop_sign_cascade = None
            else:
                self.get_logger().info(f"Loaded stop sign cascade classifier from {self.cascade_path}")
        except Exception as e:
            self.get_logger().error(f"Error loading cascade classifier: {str(e)}")
            self.stop_sign_cascade = None
        
        # Publishers and subscribers
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/color/image_raw',  # Subscribe to the color image topic
            self.image_callback, 
            10)
        
        self.stop_sign_pub = self.create_publisher(
            StopSign, 
            '/perception/stop_sign_detected', 
            10)
        
        # Debug image publisher
        self.debug_image_pub = self.create_publisher(
            Image,
            '/perception/stop_sign_debug_image', # New topic for debug image
            10)
        
        # CV bridge
        self.bridge = CvBridge()
        
        self.get_logger().info('Stop sign detection node initialized')
    
    def image_callback(self, msg):
        if self.stop_sign_cascade is None:
            self.get_logger().warn_once("Stop sign cascade classifier not loaded, skipping detection.")
            self.publish_stop_sign(False, 0.0, (0.0, 0.0, 0.0), msg.header)
            # Publish a blank debug image
            self.publish_debug_image(None, msg.header)
            return

        # Convert ROS Image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
            self.publish_stop_sign(False, 0.0, (0.0, 0.0, 0.0), msg.header)
            self.publish_debug_image(None, msg.header)
            return
        
        # Create a copy of the image for drawing debug information
        debug_image = cv_image.copy()

        # Convert to grayscale for cascade detection
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detect stop signs
        stop_signs = self.stop_sign_cascade.detectMultiScale(
            gray_image, 
            scaleFactor=1.1, 
            minNeighbors=5,
            minSize=(self.min_detection_width, self.min_detection_width)
        )
        
        detected = False
        distance = 0.0
        position = (0.0, 0.0, 0.0)
        
        if len(stop_signs) > 0:
            detected = True
            
            # Find the largest detected stop sign (likely the closest one)
            largest_area = 0
            largest_sign = None
            
            for (x, y, w, h) in stop_signs:
                area = w * h
                if area > largest_area:
                    largest_area = area
                    largest_sign = (x, y, w, h)
            
            if largest_sign:
                x, y, w, h = largest_sign
                
                # Estimate distance based on perceived width
                if w > 0:
                    distance = (self.stop_sign_real_width * self.focal_length) / w
                
                # Calculate the center of the detected stop sign in pixel coordinates
                center_x_pixel = x + w / 2
                center_y_pixel = y + h / 2
                
                # Calculate the position of the stop sign in the camera's frame (x, y, z)
                img_height, img_width = cv_image.shape[:2]
                normalized_x = (center_x_pixel - img_width / 2) / self.focal_length
                normalized_y = (center_y_pixel - img_height / 2) / self.focal_length
                pos_x = normalized_x * distance
                pos_y = normalized_y * distance
                pos_z = distance
                position = (pos_x, pos_y, pos_z)
                
                # Draw a rectangle around the detected stop sign on the debug image
                cv2.rectangle(debug_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                # Put distance text under the detected stop sign
                text = f"{distance:.2f} m"
                text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                text_x = x + (w - text_size[0]) // 2 # Center the text horizontally
                text_y = y + h + text_size[1] + 5 # Position text below the rectangle
                cv2.putText(debug_image, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Publish the stop sign detection results
        self.publish_stop_sign(detected, distance, position, msg.header)
        
        # Publish the debug image
        self.publish_debug_image(debug_image, msg.header)
    
    def publish_stop_sign(self, detected, distance, position, header):
        msg = StopSign()
        msg.header = header
        msg.detected = detected
        msg.distance = float(distance)
        
        pos = Point()
        pos.x = float(position[0])
        pos.y = float(position[1])
        pos.z = float(position[2])
        msg.position = pos
        
        self.stop_sign_pub.publish(msg)

    def publish_debug_image(self, image, header):
        if image is not None:
            try:
                ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
                ros_image.header = header
                self.debug_image_pub.publish(ros_image)
            except Exception as e:
                self.get_logger().error(f"Failed to convert and publish debug image: {str(e)}")
        else:
            # Publish a blank image if no detection is happening
            img_height = 480  # Example dimensions, adjust as needed
            img_width = 640   # Example dimensions, adjust as needed
            blank_image = np.zeros((img_height, img_width, 3), dtype=np.uint8)
            try:
                 ros_image = self.bridge.cv2_to_imgmsg(blank_image, "bgr8")
                 ros_image.header = header
                 self.debug_image_pub.publish(ros_image)
            except Exception as e:
                 self.get_logger().error(f"Failed to publish blank debug image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = StopSignDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
