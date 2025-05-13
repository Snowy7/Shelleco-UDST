#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from eco_interfaces.msg import LaneDetection
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        
        # Parameters
        self.declare_parameter('model_path', 'models/lane_detection_model.pt')
        self.declare_parameter('img_size', [640, 360])
        self.declare_parameter('conf_thres', 0.3)
        self.declare_parameter('device', '0')
        self.declare_parameter('roi_height_lower', 180)  # Lower bound for ROI height (vertical)
        self.declare_parameter('roi_height_upper', 300)  # Upper bound for ROI height (vertical)
        self.declare_parameter('roi_width_lower', 100)   # Lower bound for ROI width (horizontal)
        self.declare_parameter('roi_width_upper', 540)   # Upper bound for ROI width (horizontal)
        self.declare_parameter('smoothing_factor', 0.3)  # For smoothing lane center
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.img_size = self.get_parameter('img_size').value
        self.conf_thres = self.get_parameter('conf_thres').value
        self.device_str = self.get_parameter('device').value
        self.roi_height_lower = self.get_parameter('roi_height_lower').value
        self.roi_height_upper = self.get_parameter('roi_height_upper').value
        self.roi_width_lower = self.get_parameter('roi_width_lower').value
        self.roi_width_upper = self.get_parameter('roi_width_upper').value
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        
        # Initialize
        self.bridge = CvBridge()
        self.device = self.select_device(self.device_str)
        self.load_model()
        self.smoothed_center = None
        self.original_size = None
        
        # Publishers and subscribers
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/color/image_raw', 
            self.image_callback, 
            10)
        
        self.lane_pub = self.create_publisher(
            LaneDetection, 
            '/perception/lane_detections', 
            10)
        
        # Debug publishers
        self.debug_image_pub = self.create_publisher(
            Image, 
            '/lane_detection/debug_image', 
            10)
        
        self.lane_mask_pub = self.create_publisher(
            Image, 
            '/lane_detection/lane_mask', 
            10)
        
        self.driving_area_pub = self.create_publisher(
            Image, 
            '/lane_detection/driving_area', 
            10)
        
        self.steering_pub = self.create_publisher(
            Float32, 
            '/lane_detection/steering', 
            10)
        
        self.get_logger().info('Lane detection node initialized')
    
    def select_device(self, device):
        """Select device for inference."""
        if device.lower() == 'cpu':
            return torch.device('cpu')
        else:
            if torch.cuda.is_available():
                return torch.device(f'cuda:{device}')
            else:
                self.get_logger().warn('CUDA not available, using CPU')
                return torch.device('cpu')
    def load_model(self):
        """Load the lane detection model."""
        try:
            self.model = torch.jit.load(self.model_path)
            
            # Print model information
            self.get_logger().info(f"Model loaded from {self.model_path}")
            self.get_logger().info(f"Model device: {self.device}")
            
            # Move model to device
            self.model = self.model.to(self.device)
            
            # Set model to evaluation mode
            self.model.eval()
            
            # Enable half precision if on CUDA
            self.half_precision = self.device.type != 'cpu'
            if self.half_precision:
                self.model.half()
                self.get_logger().info("Using half precision")
            
            # Try to get model input size
            try:
                example_input = torch.zeros((1, 3, self.img_size[1], self.img_size[0]), 
                                        device=self.device)
                if self.half_precision:
                    example_input = example_input.half()
                
                # Log input shape
                self.get_logger().info(f"Example input shape: {example_input.shape}")
                
                # Try a forward pass with example input
                with torch.no_grad():
                    outputs = self.model(example_input)
                    self.get_logger().info(f"Model output types: {[type(o) for o in outputs]}")
                    self.get_logger().info(f"Model output shapes: {[o.shape if isinstance(o, torch.Tensor) else type(o) for o in outputs]}")
            
            except Exception as e:
                self.get_logger().warn(f"Failed to run example inference: {str(e)}")
            
            self.get_logger().info("Model loaded successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {str(e)}")
            self.model = None

    
    def image_callback(self, msg):
        """Process incoming image messages."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image(cv_image, msg.header)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {str(e)}")
    
    def preprocess_image(self, image):
        """Preprocess image for inference."""
        image = torch.from_numpy(image).permute(2, 0, 1).float().div(255.0).unsqueeze(0).to(self.device)
        if self.half_precision:
            image = image.half()
        return image

    def detect_lanes(self, image):
        """Detect lanes and drivable area using the model."""
        try:
            # Preprocess image
            img_tensor = self.preprocess_image(image)
            if img_tensor is None:
                raise ValueError("Failed to preprocess image")
            
            # Run inference
            with torch.no_grad():
                # Get model outputs
                outputs = self.model(img_tensor)
                
                # Check if outputs is a tuple/list with expected number of elements
                if not isinstance(outputs, (tuple, list)) or len(outputs) != 3:
                    raise ValueError(f"Unexpected model output format: {type(outputs)}")
                
                # Unpack outputs based on your model's specific format
                # Modify this part according to your model's actual output structure
                det, da_seg, ll_seg = outputs
                
                # Process detection output (if needed)
                # det = non_max_suppression(det) if det is not None else None
                
                # Process drivable area segmentation
                if da_seg is not None:
                    da_seg = torch.sigmoid(da_seg)
                    driving_mask = da_seg[0, 0].cpu().numpy()
                    driving_mask = (driving_mask > 0.5).astype(np.uint8)
                else:
                    driving_mask = np.zeros((self.img_size[1], self.img_size[0]), dtype=np.uint8)
                
                # Process lane line segmentation
                if ll_seg is not None:
                    ll_seg = torch.sigmoid(ll_seg)
                    lane_mask = ll_seg[0, 0].cpu().numpy()
                    lane_mask = (lane_mask > 0.5).astype(np.uint8)
                else:
                    lane_mask = np.zeros((self.img_size[1], self.img_size[0]), dtype=np.uint8)
                
                # Scale ROI
                scaled_roi = self.scale_roi()
                
                return lane_mask, driving_mask, scaled_roi
                
        except Exception as e:
            self.get_logger().error(f"Error during inference: {str(e)}")
            # Return empty masks on error
            empty_mask = np.zeros((self.img_size[1], self.img_size[0]), dtype=np.uint8)
            scaled_roi = self.scale_roi()
            return empty_mask, empty_mask, scaled_roi
    
    def preprocess_image(self, image):
        """Preprocess image for model inference."""
        # Convert to tensor, normalize, and add batch dimension
        img_tensor = torch.from_numpy(image).permute(2, 0, 1).float().div(255.0).unsqueeze(0)
        img_tensor = img_tensor.to(self.device)
        if self.half_precision:
            img_tensor = img_tensor.half()
        return img_tensor
    
    def split_for_trace_model(self, predictions, anchor_grid):
        """Split predictions for trace model."""
        # This is a placeholder implementation based on your existing code
        # Actual implementation would depend on your model's output format
        return predictions
    
    def non_max_suppression(self, predictions, conf_thres, iou_thres):
        """Apply non-max suppression to predictions."""
        # This is a placeholder implementation based on your existing code
        # Actual implementation would depend on your model's output format
        return predictions
    
    def lane_line_mask(self, lane_lines, grid_size=6):
        """Convert lane line predictions to mask."""
        # This is a placeholder implementation based on your existing code
        # Actual implementation would depend on your model's output format
        h, w = self.img_size[1], self.img_size[0]
        mask = np.zeros((h, w), dtype=np.uint8)
        
        # Example implementation (replace with actual logic for your model)
        try:
            lane_lines = lane_lines[0].cpu().numpy()
            # Process lane lines to create mask
            # This would depend on your model's specific output format
        except Exception as e:
            self.get_logger().error(f"Error creating lane mask: {str(e)}")
        
        return mask
    
    def driving_area_mask(self, segmentation):
        """Convert segmentation predictions to drivable area mask."""
        # This is a placeholder implementation based on your existing code
        # Actual implementation would depend on your model's output format
        h, w = self.img_size[1], self.img_size[0]
        mask = np.zeros((h, w), dtype=np.uint8)
        
        # Example implementation (replace with actual logic for your model)
        try:
            segmentation = segmentation[0].cpu().numpy()
            # Process segmentation to create mask
            # This would depend on your model's specific output format
        except Exception as e:
            self.get_logger().error(f"Error creating driving area mask: {str(e)}")
        
        return mask
    
    def scale_roi(self):
        """Scale ROI to match resized image."""
        scaled_roi = {
            'width_lower': self.roi_width_lower,
            'width_upper': self.roi_width_upper,
            'height_lower': self.roi_height_lower,
            'height_upper': self.roi_height_upper
        }
        
        # Ensure ROI is within image bounds
        scaled_roi['width_lower'] = max(0, min(scaled_roi['width_lower'], self.img_size[0] - 1))
        scaled_roi['width_upper'] = max(scaled_roi['width_lower'] + 1, min(scaled_roi['width_upper'], self.img_size[0]))
        scaled_roi['height_lower'] = max(0, min(scaled_roi['height_lower'], self.img_size[1] - 1))
        scaled_roi['height_upper'] = max(scaled_roi['height_lower'] + 1, min(scaled_roi['height_upper'], self.img_size[1]))
        
        return scaled_roi
    
    def get_lane_edges(self, lane_mask, driving_mask, scaled_roi):
        """Get left and right lane edges using lane mask or drivable area as fallback."""
        # Calculate the vertical center of the ROI for scanning
        roi_center_y = (scaled_roi['height_lower'] + scaled_roi['height_upper']) // 2
        roi_width_lower = scaled_roi['width_lower']
        roi_width_upper = scaled_roi['width_upper']
        
        # Try to find lane edges from lane mask first
        left_edge, right_edge = self.find_edges_in_mask(
            lane_mask, roi_center_y, roi_width_lower, roi_width_upper
        )
        
        is_using_fallback = False
        
        # If lane detection failed or lanes are too narrow, use drivable area as fallback
        if left_edge is None or right_edge is None or (right_edge - left_edge) < 50:
            self.get_logger().debug('Lane detection failed or lanes too narrow, using drivable area as fallback')
            fallback_left, fallback_right = self.find_edges_in_mask(
                driving_mask, roi_center_y, roi_width_lower, roi_width_upper
            )
            
            if fallback_left is not None and fallback_right is not None and (fallback_right - fallback_left) >= 50:
                left_edge, right_edge = fallback_left, fallback_right
                is_using_fallback = True
            else:
                # If both methods fail, use image center
                self.get_logger().debug('Both lane and drivable area detection failed, using image center')
                img_center = self.img_size[0] // 2
                left_edge = img_center - 100  # Arbitrary lane width
                right_edge = img_center + 100
                is_using_fallback = True
        
        return left_edge, right_edge, is_using_fallback
    
    def find_edges_in_mask(self, mask, y_position, x_start, x_end):
        """Find left and right edges in a binary mask at a specific y position."""
        # Extract horizontal slice from mask
        mask_slice = mask[y_position, x_start:x_end]
        
        if np.any(mask_slice > 0):
            # Find non-zero positions (where mask is active)
            nonzero_cols = np.where(mask_slice > 0)[0]
            
            if len(nonzero_cols) > 0:
                # Get leftmost and rightmost active pixels
                left_edge = nonzero_cols[0] + x_start
                right_edge = nonzero_cols[-1] + x_start
                
                return left_edge, right_edge
        
        # Return None if no edges found
        return None, None
    
    def calculate_steering_angle(self, left_edge, right_edge):
        """Calculate steering angle based on lane center with smoothing."""
        # Calculate target center between lanes
        current_center = (left_edge + right_edge) / 2.0
        
        # Apply smoothing to target center
        if self.smoothed_center is None:
            self.smoothed_center = current_center
        else:
            self.smoothed_center = (self.smoothing_factor * current_center + 
                                   (1 - self.smoothing_factor) * self.smoothed_center)
        
        # Calculate deviation from image center
        img_center = self.img_size[0] / 2.0
        deviation = self.smoothed_center - img_center
        
        # Convert deviation to steering angle (-1 to 1)
        # Normalize by half image width to get range approximately -1 to 1
        steering_angle = np.clip(deviation / (self.img_size[0] / 2.0), -1.0, 1.0)
        
        return steering_angle, self.smoothed_center
    
    def create_debug_visualization(self, image, lane_mask, driving_mask, 
                                  left_edge, right_edge, target_center, 
                                  steering_angle, is_using_fallback, scaled_roi):
        """Create a debug visualization image."""
        debug_image = image.copy()
        
        # Add drivable area overlay (green)
        driving_overlay = np.zeros_like(debug_image)
        driving_overlay[:, :, 1] = driving_mask * 128  # Semi-transparent green
        debug_image = cv2.addWeighted(debug_image, 1.0, driving_overlay, 0.5, 0)
        
        # Add lane mask overlay (red)
        lane_overlay = np.zeros_like(debug_image)
        lane_overlay[:, :, 2] = lane_mask * 255  # Red
        debug_image = cv2.addWeighted(debug_image, 1.0, lane_overlay, 0.5, 0)
        
        # Draw ROI rectangle
        cv2.rectangle(debug_image, 
                     (scaled_roi['width_lower'], scaled_roi['height_lower']),
                     (scaled_roi['width_upper'], scaled_roi['height_upper']), 
                     (255, 255, 255), 2)
        
        # Draw lane edges
        if left_edge is not None:
            cv2.line(debug_image, 
                    (left_edge, scaled_roi['height_lower']), 
                    (left_edge, scaled_roi['height_upper']), 
                    (0, 255, 255), 2)  # Yellow
        
        if right_edge is not None:
            cv2.line(debug_image, 
                    (right_edge, scaled_roi['height_lower']), 
                    (right_edge, scaled_roi['height_upper']), 
                    (0, 255, 255), 2)  # Yellow
        
        # Draw target center
        if target_center is not None:
            roi_center_y = (scaled_roi['height_lower'] + scaled_roi['height_upper']) // 2
            cv2.circle(debug_image, (int(target_center), roi_center_y), 5, (0, 255, 0), -1)  # Green
        
        # Draw vehicle center
        vehicle_center_x = self.img_size[0] // 2
        roi_center_y = (scaled_roi['height_lower'] + scaled_roi['height_upper']) // 2
        cv2.circle(debug_image, (vehicle_center_x, roi_center_y), 5, (0, 0, 255), -1)  # Red
        
        # Add steering indicator
        steering_length = 50
        steering_end_x = vehicle_center_x + int(steering_angle * steering_length)
        steering_end_y = roi_center_y - steering_length
        cv2.line(debug_image, 
                (vehicle_center_x, roi_center_y), 
                (steering_end_x, steering_end_y), 
                (255, 0, 0), 2)  # Blue
        
        # Add text information
        cv2.putText(debug_image, f"Steering: {steering_angle:.2f}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if is_using_fallback:
            cv2.putText(debug_image, "Using drivable area (fallback)", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            cv2.putText(debug_image, "Using lane detection", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Resize to original image size if needed
        if self.original_size != (self.img_size[0], self.img_size[1]):
            debug_image = cv2.resize(debug_image, self.original_size)
        
        return debug_image
    
    def create_lane_mask_visualization(self, lane_mask, left_edge, right_edge, target_center):
        """Create a visualization of the lane mask with detected edges."""
        # Create RGB image from lane mask
        lane_viz = np.zeros((self.img_size[1], self.img_size[0], 3), dtype=np.uint8)
        lane_viz[:, :, 2] = lane_mask * 255  # Red channel
        
        # Draw lane edges
        if left_edge is not None and right_edge is not None:
            # Draw filled polygon representing the lane
            roi_center_y = (self.roi_height_lower + self.roi_height_upper) // 2
            bottom_y = self.img_size[1]
            
            # Create polygon points
            lane_polygon = np.array([
                [left_edge, roi_center_y],
                [right_edge, roi_center_y],
                [right_edge, bottom_y],
                [left_edge, bottom_y]
            ], np.int32)
            
            # Draw filled polygon
            cv2.fillPoly(lane_viz, [lane_polygon], (0, 128, 0))  # Green fill
            
            # Draw lane boundaries
            cv2.line(lane_viz, (left_edge, 0), (left_edge, self.img_size[1]), (0, 255, 255), 2)
            cv2.line(lane_viz, (right_edge, 0), (right_edge, self.img_size[1]), (0, 255, 255), 2)
        
        # Draw target center
        if target_center is not None:
            cv2.circle(lane_viz, (int(target_center), self.img_size[1] // 2), 5, (0, 255, 0), -1)
        
        # Draw vehicle center
        vehicle_center = self.img_size[0] // 2
        cv2.circle(lane_viz, (vehicle_center, self.img_size[1] // 2), 5, (0, 0, 255), -1)
        
        return lane_viz
    
    def publish_results(self, lane_mask, driving_mask, debug_image, lane_mask_viz, 
                       left_edge, right_edge, steering_angle, header):
        """Publish all results and visualizations."""
        # Publish lane detection message
        lane_msg = LaneDetection()
        lane_msg.header = header
        
        # Convert lane edges to lane lines (arrays of points)
        if left_edge is not None and right_edge is not None:
            # Create simple vertical lines for left and right edges
            y_coords = np.linspace(0, self.img_size[1]-1, 10).astype(float)
            left_x = np.ones_like(y_coords) * float(left_edge)
            right_x = np.ones_like(y_coords) * float(right_edge)
            
            lane_msg.left_line_x = left_x.tolist()
            lane_msg.left_line_y = y_coords.tolist()
            lane_msg.right_line_x = right_x.tolist()
            lane_msg.right_line_y = y_coords.tolist()
            
            # Calculate lane center offset from vehicle center
            img_center = self.img_size[0] / 2.0
            lane_center = (left_edge + right_edge) / 2.0
            lane_msg.lane_center_offset = float(lane_center - img_center)
            
            # Calculate lane heading error (simplified as 0 for straight lanes)
            lane_msg.lane_heading_error = 0.0
        
        # Publish lane detection
        self.lane_pub.publish(lane_msg)
        
        # Publish steering angle
        steering_msg = Float32()
        steering_msg.data = float(steering_angle)
        self.steering_pub.publish(steering_msg)
        
        # Publish debug visualizations
        debug_img_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
        debug_img_msg.header = header
        self.debug_image_pub.publish(debug_img_msg)
        
        lane_mask_msg = self.bridge.cv2_to_imgmsg(lane_mask_viz, encoding="bgr8")
        lane_mask_msg.header = header
        self.lane_mask_pub.publish(lane_mask_msg)
        
        driving_mask_msg = self.bridge.cv2_to_imgmsg(driving_mask * 255, encoding="mono8")
        driving_mask_msg.header = header
        self.driving_area_pub.publish(driving_mask_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
