import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np # For imencode params

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Declare parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('compressed_topic', '/camera/color/image_raw/compressed')
        self.declare_parameter('compression_format', 'jpeg') # 'jpeg' or 'png'
        self.declare_parameter('jpeg_quality', 90) # 0-100
        self.declare_parameter('png_compression', 3) # 0-9
        self.declare_parameter('frame_width', 0) # 0 for auto
        self.declare_parameter('frame_height', 0) # 0 for auto

        # Get parameters
        self.camera_index_ = self.get_parameter('camera_index').get_parameter_value().integer_value
        publish_rate_hz = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_id_ = self.get_parameter('frame_id').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        compressed_topic = self.get_parameter('compressed_topic').get_parameter_value().string_value
        self.compression_format_ = self.get_parameter('compression_format').get_parameter_value().string_value
        self.jpeg_quality_ = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        self.png_compression_ = self.get_parameter('png_compression').get_parameter_value().integer_value
        frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value

        self.get_logger().info(f"Opening camera with index: {self.camera_index_}")
        self.cap_ = cv2.VideoCapture(self.camera_index_)

        if not self.cap_.isOpened():
            self.get_logger().error(f"Could not open video stream on camera index {self.camera_index_}")
            rclpy.shutdown()
            return

        if frame_width > 0 and frame_height > 0:
            self.cap_.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
            self.cap_.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
            self.get_logger().info(f"Attempting to set frame size to {frame_width}x{frame_height}")


        self.bridge_ = CvBridge()

        self.get_logger().info(f"Publishing raw images to: {image_topic}")
        self.get_logger().info(f"Publishing compressed images to: {compressed_topic} (format: {self.compression_format_})")

        self.image_publisher_ = self.create_publisher(Image, image_topic, 10)
        self.compressed_image_publisher_ = self.create_publisher(CompressedImage, compressed_topic, 10)

        timer_period = 1.0 / publish_rate_hz  # seconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Python camera publisher node started.")

    def timer_callback(self):
        ret, frame = self.cap_.read()
        if not ret:
            self.get_logger().warn("Could not read frame from camera.")
            return
        if frame is None:
            self.get_logger().warn("Empty frame captured.")
            return

        now = self.get_clock().now().to_msg()

        # Publish raw image
        try:
            image_msg = self.bridge_.cv2_to_imgmsg(frame, encoding="bgr8")
            image_msg.header.stamp = now
            image_msg.header.frame_id = self.frame_id_
            self.image_publisher_.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f"Error converting/publishing raw image: {e}")


        # Publish compressed image
        try:
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = now
            compressed_msg.header.frame_id = self.frame_id_

            params = []
            if self.compression_format_ == 'jpeg':
                compressed_msg.format = "jpeg"
                params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality_]
                encode_format = ".jpg"
            elif self.compression_format_ == 'png':
                compressed_msg.format = "png"
                params = [cv2.IMWRITE_PNG_COMPRESSION, self.png_compression_]
                encode_format = ".png"
            else:
                self.get_logger().warn_once(f"Unsupported compression format: {self.compression_format_}. Defaulting to JPEG.")
                compressed_msg.format = "jpeg"
                params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality_]
                encode_format = ".jpg"


            result, buffer = cv2.imencode(encode_format, frame, params)
            if result:
                compressed_msg.data = buffer.tobytes() # Use tobytes() for numpy array
                self.compressed_image_publisher_.publish(compressed_msg)
            else:
                self.get_logger().warn(f"cv2.imencode (to {self.compression_format_}) failed!")
        except Exception as e:
            self.get_logger().error(f"Error converting/publishing compressed image: {e}")


    def destroy_node(self):
        if self.cap_.isOpened():
            self.cap_.release()
            self.get_logger().info("Camera released.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"Unhandled exception: {e}")
        else:
            print(f"Unhandled exception before node initialization: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
