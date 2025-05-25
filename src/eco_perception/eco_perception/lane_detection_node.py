#!/usr/bin/env python3

import cv2
import torch
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from eco_perception.utils.model_utils import (
    select_device, non_max_suppression, split_for_trace_model,
    driving_area_mask, lane_line_mask
)
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from eco_interfaces.msg import LaneDetection

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        
        # Declare parameters
        self.declare_parameter('weights', 'data/weights/yolopv2.pt')
        self.declare_parameter('img_size', (640, 480))
        self.declare_parameter('conf_thres', 0.3)
        self.declare_parameter('device', '0')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('image_path', '')
        self.declare_parameter('roi_width_lower', 0)
        self.declare_parameter('roi_width_upper', 637)
        self.declare_parameter('roi_height_lower', 380)
        self.declare_parameter('roi_height_upper', 480)
        self.declare_parameter('publish_rate', 60.0)

        # Get parameters
        self.weights = self.get_parameter('weights').value
        self.img_size = self.get_parameter('img_size').value
        self.conf_thres = self.get_parameter('conf_thres').value
        self.device = self.get_parameter('device').value
        self.image_topic = self.get_parameter('image_topic').value
        self.image_path = self.get_parameter('image_path').value
        self.roi_width_lower = self.get_parameter('roi_width_lower').value
        self.roi_width_upper = self.get_parameter('roi_width_upper').value
        self.roi_height_lower = self.get_parameter('roi_height_lower').value
        self.roi_height_upper = self.get_parameter('roi_height_upper').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Initialize
        self.bridge = CvBridge()
        self.device = select_device(self.device)
        self.model = torch.jit.load(self.weights).to(self.device).eval()
        self.half_precision = self.device.type != 'cpu'
        if self.half_precision:
            self.model.half()
        self.smoothed_center = None
        self.original_size = None
        self.last_valid_edges = None

        # Publishers
        self.lane_publisher = self.create_publisher(Image, '/lane_detection/lane_mask', 10)
        self.steering_publisher = self.create_publisher(Float32, '/lane_detection/steering', 10)
        self.debug_publisher = self.create_publisher(Image, '/lane_detection/debug_image', 10)
        self.roi_mask_publisher = self.create_publisher(Image, '/lane_detection/roi_mask', 10)
        self.raw_lane_publisher = self.create_publisher(Image, '/lane_detection/raw_lane_mask', 10)
        self.raw_driving_publisher = self.create_publisher(Image, '/lane_detection/raw_driving_mask', 10)
        
        self.lane_detection_publisher = self.create_publisher(
            LaneDetection, 
            '/perception/lane_detections', 
            10)

        # Initialize mode
        self.image = None
        if self.image_path:
            self.get_logger().info(f'Processing single image: {self.image_path}')
            self.process_single_image()
        else:
            self.create_subscription(Image, self.image_topic, self.image_callback, QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        ))
            self.get_logger().info(f'Subscribed to {self.image_topic}')

    def preprocess_image(self, image):
        """Preprocess image for inference."""
        image = torch.from_numpy(image).permute(2, 0, 1).float().div(255.0).unsqueeze(0).to(self.device)
        if self.half_precision:
            image = image.half()
        return image

    def scale_roi(self, original_size):
        """Scale ROI coordinates to match resized image."""
        orig_width, orig_height = original_size
        target_width, target_height = self.img_size[0], self.img_size[1]
        
        width_ratio = target_width / orig_width
        height_ratio = target_height / orig_height
        
        scaled_roi = {
            'width_lower': int(self.roi_width_lower * width_ratio),
            'width_upper': int(self.roi_width_upper * width_ratio),
            'height_lower': int(self.roi_height_lower * height_ratio),
            'height_upper': int(self.roi_height_upper * height_ratio)
        }
        
        scaled_roi['width_lower'] = max(0, min(scaled_roi['width_lower'], target_width - 1))
        scaled_roi['width_upper'] = max(scaled_roi['width_lower'] + 1, min(scaled_roi['width_upper'], target_width))
        scaled_roi['height_lower'] = max(0, min(scaled_roi['height_lower'], target_height - 1))
        scaled_roi['height_upper'] = max(scaled_roi['height_lower'] + 1, min(scaled_roi['height_upper'], target_height))
        
        self.get_logger().debug(f"Scaled ROI: {scaled_roi}")
        return scaled_roi

    def ensure_mask_size(self, mask, target_size):
        """Ensure mask has the correct size by resizing if necessary."""
        target_width, target_height = target_size
        
        if len(mask.shape) == 3:
            current_height, current_width, _ = mask.shape
        else:
            current_height, current_width = mask.shape
        
        if current_width != target_width or current_height != target_height:
            self.get_logger().debug(f"Resizing mask from {current_width}x{current_height} to {target_width}x{target_height}")
            mask = cv2.resize(mask, (target_width, target_height), interpolation=cv2.INTER_NEAREST)
        
        return mask

    def process_frame(self, frame):
        """Process a frame and return lane and driving area masks."""
        self.original_size = (frame.shape[1], frame.shape[0])
        self.get_logger().debug(f"Original image size: {self.original_size}")
        
        original_image = frame.copy()
        
        # Resize frame to model input size
        frame_resized = cv2.resize(frame, (self.img_size[0], self.img_size[1]), interpolation=cv2.INTER_LINEAR)
        self.get_logger().debug(f"Resized frame size: {frame_resized.shape}")
        
        # Run inference
        image = self.preprocess_image(frame_resized)
        with torch.no_grad():
            [predictions, anchor_grid], segmentation, lane_lines = self.model(image)
        predictions = split_for_trace_model(predictions, anchor_grid)
        predictions = non_max_suppression(predictions, self.conf_thres, 0.45)

        # Get masks from model output
        lane_mask = lane_line_mask(lane_lines, grid_size=6)
        driving_mask = driving_area_mask(segmentation)
        lane_mask = (lane_mask > 0).astype(np.uint8)
        driving_mask = (driving_mask > 0).astype(np.uint8)
        
        # Log original mask sizes
        self.get_logger().debug(f"Lane mask shape from model: {lane_mask.shape}")
        self.get_logger().debug(f"Driving mask shape from model: {driving_mask.shape}")
        
        # Ensure masks match the resized image size
        lane_mask = self.ensure_mask_size(lane_mask, (self.img_size[0], self.img_size[1]))
        driving_mask = self.ensure_mask_size(driving_mask, (self.img_size[0], self.img_size[1]))
        
        self.get_logger().debug(f"Final lane mask shape: {lane_mask.shape}")
        self.get_logger().debug(f"Final driving mask shape: {driving_mask.shape}")
        
        # Publish raw masks for debugging
        self.raw_lane_publisher.publish(self.bridge.cv2_to_imgmsg(lane_mask * 255, encoding='mono8'))
        self.raw_driving_publisher.publish(self.bridge.cv2_to_imgmsg(driving_mask * 255, encoding='mono8'))

        # Calculate scaled ROI
        scaled_roi = self.scale_roi(self.original_size)
        
        # Create ROI visualization
        roi_image = frame_resized.copy()
        cv2.rectangle(roi_image, (scaled_roi['width_lower'], scaled_roi['height_lower']),
                    (scaled_roi['width_upper'], scaled_roi['height_upper']), (255, 255, 255), 2)
        self.roi_mask_publisher.publish(self.bridge.cv2_to_imgmsg(roi_image, encoding='bgr8'))

        return lane_mask, driving_mask, frame_resized, scaled_roi
        
    def get_road_center(self, lane_mask, driving_mask, scaled_roi):
        """Get road center by checking x-axis at the vertical center of the ROI, using only the most left and right lane lines."""
        # Ensure masks are 2D for processing
        if len(lane_mask.shape) == 3:
            lane_mask = lane_mask[:,:,0] if lane_mask.shape[2] > 0 else lane_mask.squeeze()
        if len(driving_mask.shape) == 3:
            driving_mask = driving_mask[:,:,0] if driving_mask.shape[2] > 0 else driving_mask.squeeze()
        
        roi_center_y = (scaled_roi['height_lower'] + scaled_roi['height_upper']) // 2
        roi_width_lower = scaled_roi['width_lower']
        roi_width_upper = scaled_roi['width_upper']

        # Ensure ROI boundaries are within mask dimensions
        roi_center_y = max(0, min(roi_center_y, lane_mask.shape[0] - 1))
        roi_width_lower = max(0, min(roi_width_lower, lane_mask.shape[1] - 1))
        roi_width_upper = max(roi_width_lower + 1, min(roi_width_upper, lane_mask.shape[1]))

        left_edge = right_edge = None
        is_using_fallback = False

        # Check if we have valid slice dimensions
        if roi_center_y < lane_mask.shape[0] and roi_width_upper > roi_width_lower:
            lane_slice = lane_mask[roi_center_y, roi_width_lower:roi_width_upper]
            
            if np.any(lane_slice > 0):
                nonzero_indices = np.where(lane_slice > 0)[0]
                
                if len(nonzero_indices) > 0:
                    segments = []
                    current_segment = [nonzero_indices[0]]
                    
                    for i in range(1, len(nonzero_indices)):
                        if nonzero_indices[i] - nonzero_indices[i-1] <= 5:
                            current_segment.append(nonzero_indices[i])
                        else:
                            if len(current_segment) >= 5:
                                segments.append(current_segment)
                            current_segment = [nonzero_indices[i]]
                    
                    if len(current_segment) >= 5:
                        segments.append(current_segment)
                    
                    valid_segments = [seg for seg in segments if len(seg) >= 5]
                    self.get_logger().debug(f'Valid lane segments: {valid_segments}')
                    if len(valid_segments) >= 2:
                        # Select only the leftmost and rightmost segments
                        leftmost_segment = valid_segments[0]
                        rightmost_segment = valid_segments[-1]
                        
                        left_edge = leftmost_segment[0] + roi_width_lower
                        right_edge = rightmost_segment[-1] + roi_width_lower
                        
                        width = right_edge - left_edge
                        if width > 50:
                            self.get_logger().debug(f'Most left and right lane edges detected at y={roi_center_y}: left={left_edge}, right={right_edge}, width={width}')
                            self.last_valid_edges = (left_edge, right_edge)
                            return left_edge, right_edge, False
                        else:
                            self.get_logger().debug(f'Lane width {width} too narrow, falling back to drivable area')

        self.get_logger().debug(f'Lane edges: left={left_edge}, right={right_edge}')
        if left_edge is None or right_edge is None:
            if roi_center_y < driving_mask.shape[0] and roi_width_upper > roi_width_lower:
                driving_slice = driving_mask[roi_center_y, roi_width_lower:roi_width_upper]
                if np.any(driving_slice > 0):
                    nonzero_cols = np.where(driving_slice > 0)[0]
                    if len(nonzero_cols) > 0:
                        drivable_left = nonzero_cols[0] + roi_width_lower
                        drivable_right = nonzero_cols[-1] + roi_width_lower
                        
                        if left_edge is None:
                            left_edge = drivable_left
                            is_using_fallback = True
                        
                        if right_edge is None:
                            right_edge = drivable_right
                            is_using_fallback = True
                        
                        width = right_edge - left_edge
                        if width > 50:
                            self.get_logger().debug(f'Combined lane and driving area edges: left={left_edge}, right={right_edge}, width={width}')
                            self.last_valid_edges = (left_edge, right_edge)
                            return left_edge, right_edge, is_using_fallback

        if left_edge is None or right_edge is None:
            if roi_center_y < driving_mask.shape[0] and roi_width_upper > roi_width_lower:
                driving_slice = driving_mask[roi_center_y, roi_width_lower:roi_width_upper]
                if np.any(driving_slice > 0):
                    nonzero_cols = np.where(driving_slice > 0)[0]
                    if len(nonzero_cols) > 0:
                        left_edge = nonzero_cols[0] + roi_width_lower
                        right_edge = nonzero_cols[-1] + roi_width_lower
                        width = right_edge - left_edge
                        if width > 50:
                            self.get_logger().debug(f'Driving area edges at y={roi_center_y}: left={left_edge}, right={right_edge}, width={width}')
                            self.last_valid_edges = (left_edge, right_edge)
                            return left_edge, right_edge, True

        if self.last_valid_edges is not None:
            self.get_logger().debug(f'Using last valid edges: left={self.last_valid_edges[0]}, right={self.last_valid_edges[1]}')
            return self.last_valid_edges[0], self.last_valid_edges[1], True

        self.get_logger().debug('No valid edges detected, using image center')
        left_edge = self.img_size[0] // 2 - 100
        right_edge = self.img_size[0] // 2 + 100
        return left_edge, right_edge, True

    def draw_lanes_on_blank_image(self, lane_mask, target_center, steering_angle, left_edge, right_edge, is_using_fallback):
        """Draw lanes, target center, car position, steering angle, and edge lines."""
        # Ensure lane_mask is 2D
        if len(lane_mask.shape) == 3:
            lane_mask = lane_mask[:,:,0] if lane_mask.shape[2] > 0 else lane_mask.squeeze()
        
        lanes_image = np.zeros((lane_mask.shape[0], lane_mask.shape[1], 3), dtype=np.uint8)
        
        lanes_image[:, :, 0] = lane_mask * 255
        
        # Use scaled ROI values
        roi_center_y = (self.roi_height_lower + self.roi_height_upper) // 2
        # Scale ROI center to match current image size
        if lane_mask.shape[0] != self.img_size[1]:
            scale_y = lane_mask.shape[0] / self.img_size[1]
            roi_center_y = int(roi_center_y * scale_y)
        
        bottom_y = lane_mask.shape[0]
        
        if left_edge is not None and right_edge is not None:
            lane_polygon = np.array([
                [left_edge, roi_center_y],
                [right_edge, roi_center_y],
                [right_edge, bottom_y],
                [left_edge, bottom_y]
            ], np.int32)
            
            cv2.fillPoly(lanes_image, [lane_polygon], (0, 128, 0))
        
        if target_center is not None:
            cv2.circle(lanes_image, (int(target_center), roi_center_y), 5, (0, 255, 0), -1)
        
        car_position = lane_mask.shape[1] // 2
        cv2.circle(lanes_image, (car_position, roi_center_y), 5, (0, 0, 255), -1)
        
        if left_edge is not None:
            cv2.line(lanes_image, (left_edge, 0), (left_edge, lane_mask.shape[0]), (255, 0, 0), 2)
        if right_edge is not None:
            cv2.line(lanes_image, (right_edge, 0), (right_edge, lane_mask.shape[0]), (255, 0, 0), 2)
        
        cv2.putText(lanes_image, f"Steering: {steering_angle:.2f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if is_using_fallback:
            cv2.putText(lanes_image, "Using fallback", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        return lanes_image

    def draw_debug_image(self, input_image, lane_mask, driving_mask, target_center, steering_angle, left_edge, right_edge, is_using_fallback):
        """Draw drivable area, lanes, target center, car position, steering angle, and edge lines."""
        debug_image = input_image.copy()
        
        # Ensure masks are 2D
        if len(lane_mask.shape) == 3:
            lane_mask = lane_mask[:,:,0] if lane_mask.shape[2] > 0 else lane_mask.squeeze()
        if len(driving_mask.shape) == 3:
            driving_mask = driving_mask[:,:,0] if driving_mask.shape[2] > 0 else driving_mask.squeeze()
        
        # Resize masks to match debug image if necessary
        if lane_mask.shape[:2] != debug_image.shape[:2]:
            lane_mask = cv2.resize(lane_mask, (debug_image.shape[1], debug_image.shape[0]), interpolation=cv2.INTER_NEAREST)
        
        if driving_mask.shape[:2] != debug_image.shape[:2]:
            driving_mask = cv2.resize(driving_mask, (debug_image.shape[1], debug_image.shape[0]), interpolation=cv2.INTER_NEAREST)
        
        # Add driving area overlay
        driving_overlay = np.zeros_like(debug_image)
        driving_overlay[:, :, 1] = driving_mask * 128
        debug_image = cv2.addWeighted(debug_image, 1.0, driving_overlay, 0.5, 0)
        
        # Add lane overlay
        lane_overlay = np.zeros_like(debug_image)
        lane_overlay[:, :, 2] = lane_mask * 200
        debug_image = cv2.addWeighted(debug_image, 1.0, lane_overlay, 0.5, 0)
        
        # Calculate ROI coordinates for debug image
        roi_height_lower = self.roi_height_lower
        roi_height_upper = self.roi_height_upper
        roi_width_lower = self.roi_width_lower
        roi_width_upper = self.roi_width_upper
        
        # Scale ROI and edges if debug image size differs from model input size
        if debug_image.shape[:2] != (self.img_size[1], self.img_size[0]):
            h_scale = debug_image.shape[0] / self.img_size[1]
            w_scale = debug_image.shape[1] / self.img_size[0]
            roi_height_lower = int(roi_height_lower * h_scale)
            roi_height_upper = int(roi_height_upper * h_scale)
            roi_width_lower = int(roi_width_lower * w_scale)
            roi_width_upper = int(roi_width_upper * w_scale)
            left_edge = int(left_edge * w_scale) if left_edge is not None else None
            right_edge = int(right_edge * w_scale) if right_edge is not None else None
            if target_center is not None:
                target_center = int(target_center * w_scale)
        
        # Draw ROI rectangle
        cv2.rectangle(debug_image, 
                     (roi_width_lower, roi_height_lower),
                     (roi_width_upper, roi_height_upper), 
                     (255, 255, 255), 2)
        
        roi_center_y = (roi_height_lower + roi_height_upper) // 2
        
        # Draw target center
        if target_center is not None:
            cv2.circle(debug_image, (int(target_center), roi_center_y), 5, (0, 255, 0), -1)
        
        # Draw car position
        car_position = debug_image.shape[1] // 2
        cv2.circle(debug_image, (car_position, roi_center_y), 5, (0, 0, 255), -1)
        
        # Draw edge lines
        if left_edge is not None:
            cv2.line(debug_image, (left_edge, 0), (left_edge, debug_image.shape[0]), (255, 255, 0), 2)
        if right_edge is not None:
            cv2.line(debug_image, (right_edge, 0), (right_edge, debug_image.shape[0]), (255, 255, 0), 2)
        
        # Draw steering arrow
        steering_length = 50
        steering_end_x = car_position + int(steering_angle * steering_length)
        steering_end_y = roi_center_y - steering_length
        cv2.line(debug_image, 
                (car_position, roi_center_y), 
                (steering_end_x, steering_end_y), 
                (255, 0, 0), 2)
        
        # Add text information
        cv2.putText(debug_image, f"Steering: {steering_angle:.2f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if is_using_fallback:
            cv2.putText(debug_image, "Using fallback detection", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            cv2.putText(debug_image, "Using lane detection", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return debug_image

    def calculate_steering_angle(self, left_edge, right_edge):
        """Calculate steering angle with smoothed target center."""
        if left_edge == right_edge:
            self.smoothed_center = self.img_size[0] / 2
            self.get_logger().debug(f'No valid edges, target center reset to: {self.smoothed_center}')
            return 0.0, self.smoothed_center

        target_center = (left_edge + right_edge) / 2.0
        alpha = 0.2
        if self.smoothed_center is None:
            self.smoothed_center = target_center
        else:
            self.smoothed_center = alpha * target_center + (1 - alpha) * self.smoothed_center
        self.get_logger().debug(f'Smoothed target center: {self.smoothed_center}')

        img_center = self.img_size[0] / 2.0
        deviation = self.smoothed_center - img_center
        steering_angle = np.clip(deviation / (self.img_size[0] / 2.0), -1.0, 1.0)
        
        # todo: convert the 2.5 to a gain value editable
        return steering_angle, self.smoothed_center

    def create_lane_detection_msg(self, left_edge, right_edge, steering_angle, header):
        """Create LaneDetection message for the eco_interfaces format with normalized values."""
        lane_msg = LaneDetection()
        lane_msg.header = header
        
        if left_edge is not None and right_edge is not None:
            # Create y coordinates from top to bottom of image
            y_coords = np.linspace(0, self.img_size[1]-1, 10).astype(float)
            left_x = np.ones_like(y_coords) * float(left_edge)
            right_x = np.ones_like(y_coords) * float(right_edge)
            
            # Store the raw pixel coordinates for visualization if needed
            lane_msg.left_line_x = left_x.tolist()
            lane_msg.left_line_y = y_coords.tolist()
            lane_msg.right_line_x = right_x.tolist()
            lane_msg.right_line_y = y_coords.tolist()
            
            # Calculate lane center in pixel coordinates
            img_center = self.img_size[0] / 2.0
            lane_center = (left_edge + right_edge) / 2.0
            pixel_offset = lane_center - img_center
            
            # Normalize the offset to range [-1, 1]
            # Where -1 means lane is at the far left of the image
            # and +1 means lane is at the far right of the image
            max_possible_offset = img_center  # Maximum possible offset is half the image width
            normalized_offset = pixel_offset / max_possible_offset
            
            # Clamp to [-1, 1] in case the lane is detected outside the image bounds
            normalized_offset = max(-1.0, min(1.0, normalized_offset))
            
            # Store the normalized offset
            lane_msg.lane_center_offset = float(normalized_offset)
            
            # Log the conversion for debugging
            self.get_logger().debug(
                f"Lane center: {lane_center:.1f}px, Image center: {img_center:.1f}px, "
                f"Pixel offset: {pixel_offset:.1f}px, Normalized offset: {normalized_offset:.2f}"
            )
            
            # For heading error, we could also normalize it if we have a steering angle
            # For now, just set it to 0 or use the provided steering angle if it's normalized
            if steering_angle is not None:
                # Assuming steering_angle is already normalized to [-1, 1]
                lane_msg.lane_heading_error = 0.0 #float(steering_angle)
            else:
                lane_msg.lane_heading_error = 0.0
        
        return lane_msg

    def image_callback(self, msg):
        """Process incoming image messages."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_image(frame, msg.header)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def process_single_image(self):
        """Load and process a single image."""
        try:
            img = cv2.imread(self.image_path, cv2.IMREAD_COLOR)
            if img is None:
                raise ValueError(f"Failed to load image: {self.image_path}")
            if len(img.shape) != 3 or img.shape[2] != 3:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR if len(img.shape) == 2 else cv2.COLOR_RGBA2BGR)
            self.image = img
            self.get_logger().info(f'Loaded image: {self.image_path}')
            timer_period = 1.0 / self.publish_rate
            self.create_timer(timer_period, self.publish_single_image)
        except Exception as e:
            self.get_logger().error(f'Error loading image: {str(e)}')

    def publish_single_image(self):
        """Publish results for single image."""
        if self.image is None:
            self.get_logger().warn('No image loaded')
            return
        self.process_image(self.image)

    def process_image(self, frame, header=None):
        """Process image and publish results."""
        try:
            lane_mask, driving_mask, resized_frame, scaled_roi = self.process_frame(frame)
            left_edge, right_edge, is_using_fallback = self.get_road_center(lane_mask, driving_mask, scaled_roi)
            steering_angle, target_center = self.calculate_steering_angle(left_edge, right_edge)
            
            lanes_image = self.draw_lanes_on_blank_image(lane_mask, target_center, steering_angle, left_edge, right_edge, is_using_fallback)
            
            debug_image = self.draw_debug_image(frame, lane_mask, driving_mask, target_center, steering_angle, left_edge, right_edge, is_using_fallback)
            
            lane_msg = self.bridge.cv2_to_imgmsg(lanes_image, encoding='bgr8')
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            if header:
                lane_msg.header = header
                debug_msg.header = header
            
            steering_msg = Float32(data=float(steering_angle))
            
            lane_detection_msg = self.create_lane_detection_msg(left_edge, right_edge, steering_angle, header)
            self.lane_detection_publisher.publish(lane_detection_msg)
            
            self.lane_publisher.publish(lane_msg)
            self.steering_publisher.publish(steering_msg)
            self.debug_publisher.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in process_image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()