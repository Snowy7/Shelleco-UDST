#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
from collections import deque
from eco_interfaces.msg import Obstacle, Obstacles

class MergedLidarObstacleNode(Node):
    def __init__(self):
        super().__init__('merged_lidar_obstacle_node')
        
        # Lidar Filter Parameters
        self.declare_parameter('filter_angle_min', -50.0)  # degrees
        self.declare_parameter('filter_angle_max', 50.0)   # degrees
        self.declare_parameter('max_range', 2.0)           # meters
        self.declare_parameter('min_range', 0.1)           # meters
        
        # Obstacle Detection Parameters
        self.declare_parameter('point_cluster_threshold', 0.15)  # Increased for easier detection
        self.declare_parameter('min_points_per_cluster', 3)     # Reduced minimum points
        self.declare_parameter('target_diameter', 0.45)         # Target obstacle diameter
        self.declare_parameter('diameter_tolerance', 0.25)      # Increased tolerance
        self.declare_parameter('smoothing_alpha', 0.5)          # More responsive
        self.declare_parameter('confidence_threshold', 0.2)     # Lower threshold for debugging
        
        # Define QoS profile with Best Effort reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriber to the raw /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        
        # Publisher for filtered scan (optional, for debugging)
        self.filtered_scan_pub = self.create_publisher(
            LaserScan,
            '/filtered_scan',
            qos_profile
        )
        
        # Obstacle detection publishers
        self.marker_pub = self.create_publisher(Marker, '/obstacle_markers', 10)
        self.obstacles_pub = self.create_publisher(Obstacles, '/obstacles', 10)
        
        # Tracking state for obstacles
        self.tracked_obstacles = {}
        self.next_obstacle_id = 0
        self.scan_count = 0
        
        # Get parameters for logging
        angle_min = self.get_parameter('filter_angle_min').value
        angle_max = self.get_parameter('filter_angle_max').value
        max_range = self.get_parameter('max_range').value
        target_diameter = self.get_parameter('target_diameter').value
        
        self.get_logger().info("=== Merged Lidar Filter and Obstacle Detection Node Started ===")
        self.get_logger().info(f'Filtering angles: {angle_min}° to {angle_max}°')
        self.get_logger().info(f'Max range: {max_range}m')
        self.get_logger().info(f"Target obstacle diameter: {target_diameter}m")
        self.get_logger().info(f"Subscribing to: /scan")
        self.get_logger().info(f"Publishing filtered scan to: /filtered_scan")
        self.get_logger().info(f"Publishing markers to: /obstacle_markers")

    def scan_callback(self, msg):
        self.scan_count += 1
        
        # Step 1: Apply lidar filtering
        filtered_scan = self.filter_lidar_scan(msg)
        
        if filtered_scan is None:
            return
        
        # Publish filtered scan for debugging/visualization
        self.filtered_scan_pub.publish(filtered_scan)
        
        # Step 2: Perform obstacle detection on filtered scan
        self.detect_obstacles(filtered_scan)
        
        # Debug: Print every 50 scans (from original lidar filter)
        if self.scan_count % 50 == 0:
            valid_ranges = [r for r in filtered_scan.ranges if not math.isinf(r) and not math.isnan(r)]
            self.get_logger().info(
                f'Scan #{self.scan_count}: {len(filtered_scan.ranges)} filtered points, '
                f'{len(valid_ranges)} valid readings'
            )

    def filter_lidar_scan(self, msg):
        """Filter the lidar scan based on angle and range parameters"""
        # Get filter parameters
        angle_min_deg = self.get_parameter('filter_angle_min').value
        angle_max_deg = self.get_parameter('filter_angle_max').value
        max_range = self.get_parameter('max_range').value
        min_range = self.get_parameter('min_range').value
        
        # Convert to radians
        angle_min_rad = math.radians(angle_min_deg)
        angle_max_rad = math.radians(angle_max_deg)
        
        # Lists to store filtered data
        filtered_ranges = []
        filtered_intensities = []
        valid_points = 0
        
        # Iterate through the scan data
        for i, range_val in enumerate(msg.ranges):
            # Calculate the angle for this measurement
            angle = msg.angle_min + i * msg.angle_increment
            
            # Check if the angle is within the desired range
            if angle_min_rad <= angle <= angle_max_rad:
                # Apply range filtering
                if (range_val >= min_range and range_val <= max_range and 
                    not math.isinf(range_val) and not math.isnan(range_val)):
                    filtered_ranges.append(range_val)
                    valid_points += 1
                else:
                    # Invalid or out-of-range reading
                    filtered_ranges.append(float('inf'))
                
                # Handle intensities if available
                if msg.intensities and i < len(msg.intensities):
                    filtered_intensities.append(msg.intensities[i])
        
        # Only continue if we have valid filtered data
        if len(filtered_ranges) == 0:
            self.get_logger().warn('No valid points in filtered scan range')
            return None
        
        # Create filtered scan message
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        
        # CRITICAL FIX: Update angle bounds to match filtered data
        filtered_scan.angle_min = angle_min_rad
        filtered_scan.angle_max = angle_max_rad
        
        # Calculate new angle increment for filtered data
        if len(filtered_ranges) > 1:
            filtered_scan.angle_increment = (angle_max_rad - angle_min_rad) / (len(filtered_ranges) - 1)
        else:
            filtered_scan.angle_increment = msg.angle_increment
        
        # Copy other scan parameters
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = min_range
        filtered_scan.range_max = max_range
        
        # Assign filtered data
        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = filtered_intensities if filtered_intensities else []
        
        return filtered_scan

    def detect_obstacles(self, msg):
        """Perform obstacle detection on the filtered scan"""
        # Debug: Print every 30 scans
        if self.scan_count % 30 == 0:
            self.get_logger().info(f"Processing scan #{self.scan_count} for obstacles")
            self.get_logger().info(f"Filtered scan has {len(msg.ranges)} points")
        
        # Convert laser scan to cartesian points
        valid_points = self.extract_valid_points(msg)
        
        if self.scan_count % 30 == 0:
            self.get_logger().info(f"Valid points for clustering: {len(valid_points)}")
        
        # Always publish a basic marker to test visualization
        self.publish_test_marker(msg.header)
        
        if len(valid_points) < self.get_parameter('min_points_per_cluster').value:
            return
        
        # Detect clusters with simpler algorithm
        raw_clusters = self.simple_cluster(valid_points)
        
        if self.scan_count % 30 == 0:
            self.get_logger().info(f"Found {len(raw_clusters)} raw clusters")
        
        # Filter clusters by size
        valid_obstacles = self.filter_by_size(raw_clusters)
        
        if self.scan_count % 30 == 0:
            self.get_logger().info(f"Valid obstacles after filtering: {len(valid_obstacles)}")
        
        # Update tracking
        self.update_obstacle_tracking(valid_obstacles)
        
        # Publish results
        if valid_obstacles:
            self.publish_obstacles(msg.header)
            self.publish_visualization(msg.header)

    def extract_valid_points(self, msg):
        """Convert laser scan to cartesian coordinates"""
        points = []
        # Use the same max_range parameter as the filter
        max_range = self.get_parameter('max_range').value
        min_range = self.get_parameter('min_range').value
        valid_count = 0
        
        for i, distance in enumerate(msg.ranges):
            if min_range < distance < max_range and not math.isinf(distance) and not math.isnan(distance):
                angle = msg.angle_min + i * msg.angle_increment
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                points.append((x, y))
                valid_count += 1
        
        return points

    def simple_cluster(self, points):
        """Simple clustering algorithm for debugging"""
        if not points:
            return []
        
        clusters = []
        visited = [False] * len(points)
        cluster_threshold = self.get_parameter('point_cluster_threshold').value
        min_points = self.get_parameter('min_points_per_cluster').value
        
        for i, point in enumerate(points):
            if visited[i]:
                continue
                
            # Start new cluster
            cluster = []
            queue = deque([i])
            visited[i] = True
            
            while queue:
                current_idx = queue.popleft()
                cluster.append(current_idx)
                current_point = points[current_idx]
                
                # Check all other points for neighbors
                for j, other_point in enumerate(points):
                    if not visited[j]:
                        distance = math.hypot(
                            current_point[0] - other_point[0],
                            current_point[1] - other_point[1]
                        )
                        if distance < cluster_threshold:
                            visited[j] = True
                            queue.append(j)
            
            # Keep cluster if it has enough points
            if len(cluster) >= min_points:
                cluster_points = [points[idx] for idx in cluster]
                clusters.append(cluster_points)
        
        return clusters

    def filter_by_size(self, clusters):
        """Filter clusters by diameter - relaxed for debugging"""
        target_diameter = self.get_parameter('target_diameter').value
        tolerance = self.get_parameter('diameter_tolerance').value
        min_diameter = target_diameter - tolerance
        max_diameter = target_diameter + tolerance
        
        valid_obstacles = []
        
        for cluster_points in clusters:
            if len(cluster_points) < 3:
                continue
                
            # Calculate centroid
            centroid_x = sum(p[0] for p in cluster_points) / len(cluster_points)
            centroid_y = sum(p[1] for p in cluster_points) / len(cluster_points)
            
            # Calculate diameter as 2 * max distance from centroid
            max_radius = 0
            for point in cluster_points:
                radius = math.hypot(point[0] - centroid_x, point[1] - centroid_y)
                max_radius = max(max_radius, radius)
            
            estimated_diameter = 2 * max_radius
            
            # For debugging: accept more sizes
            if 0.1 <= estimated_diameter <= 2.0:  # Very permissive for now
                valid_obstacles.append({
                    'centroid': (centroid_x, centroid_y),
                    'diameter': estimated_diameter,
                    'points': cluster_points
                })
        
        return valid_obstacles

    def update_obstacle_tracking(self, new_obstacles):
        """Simplified tracking"""
        # Clear old obstacles
        self.tracked_obstacles.clear()
        
        # Add new obstacles directly (no smoothing for debugging)
        for i, obs in enumerate(new_obstacles):
            self.tracked_obstacles[i] = {
                'x': obs['centroid'][0],
                'y': obs['centroid'][1],
                'diameter': obs['diameter'],
                'points': obs['points'],
                'confidence': 1.0
            }

    def publish_test_marker(self, header):
        """Publish a simple test marker to verify RViz2 connection"""
        marker = Marker()
        marker.header = header
        marker.header.frame_id = "laser_frame"  # Try common frame names
        marker.ns = "test_marker"
        marker.id = 999
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
        
        # Place a test sphere at origin
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.marker_pub.publish(marker)

    def publish_obstacles(self, header):
        """Publish obstacle message"""
        obstacles_msg = Obstacles()
        obstacles_msg.header = header
        obstacles_msg.obstacles = []
        
        for track in self.tracked_obstacles.values():
            obstacle = Obstacle()
            obstacle.header = header
            obstacle.points = [Point(x=p[0], y=p[1], z=0.0) for p in track['points']]
            obstacle.center = Point(x=track['x'], y=track['y'], z=0.0)
            obstacle.radius = track['diameter'] / 2.0
            obstacles_msg.obstacles.append(obstacle)
        
        if obstacles_msg.obstacles:
            self.obstacles_pub.publish(obstacles_msg)
            self.get_logger().info(f"Published {len(obstacles_msg.obstacles)} obstacles")

    def publish_visualization(self, header):
        """Publish obstacle markers"""
        marker = Marker()
        marker.header = header
        marker.header.frame_id = "laser_frame"  # Force frame_id
        marker.ns = "obstacles"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        
        # Make spheres bigger and more visible
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.points = []
        marker.colors = []
        
        for track in self.tracked_obstacles.values():
            # Add obstacle center
            marker.points.append(Point(x=track['x'], y=track['y'], z=0.0))
            
            # Bright red color
            color = ColorRGBA()
            color.r = 1.0
            color.g = 0.0
            color.b = 0.0
            color.a = 1.0
            marker.colors.append(color)

        # Always publish marker
        self.marker_pub.publish(marker)
        
        if marker.points:
            self.get_logger().info(f"Published {len(marker.points)} obstacle markers")
        
        # Also publish individual spheres for each obstacle
        for i, track in enumerate(self.tracked_obstacles.values()):
            sphere_marker = Marker()
            sphere_marker.header = header
            sphere_marker.header.frame_id = "laser_frame"
            sphere_marker.ns = "individual_obstacles"
            sphere_marker.id = i
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
            
            sphere_marker.pose.position.x = track['x']
            sphere_marker.pose.position.y = track['y']
            sphere_marker.pose.position.z = 0.0
            sphere_marker.pose.orientation.w = 1.0
            
            sphere_marker.scale.x = 0.5
            sphere_marker.scale.y = 0.5
            sphere_marker.scale.z = 0.5
            
            sphere_marker.color.r = 1.0
            sphere_marker.color.g = 0.0
            sphere_marker.color.b = 1.0  # Magenta
            sphere_marker.color.a = 0.8
            
            self.marker_pub.publish(sphere_marker)

def main(args=None):
    rclpy.init(args=args)
    node = MergedLidarObstacleNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down merged lidar obstacle node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()