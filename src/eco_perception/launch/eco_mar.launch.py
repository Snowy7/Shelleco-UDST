#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('eco_perception')
    
    # Get the path to the ros2_astra_camera package
    astra_camera_pkg_dir = get_package_share_directory('astra_camera')
    # Include the astro_pro_plus.launch.xml file
    astra_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(astra_camera_pkg_dir, 'launch', 'astro_pro_plus.launch.xml')
        )
    )
    
    # Define the path to the models directory
    models_dir = os.path.join(os.path.expanduser('~'), 'eco_marathon_ws', 'models')
    
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(models_dir, 'yolopv2.pt'),
        description='Path to the lane detection model'
    )
    
    return LaunchDescription([
        # Launch arguments
        model_path_arg,
        
        # astra_launch,
        Node(
            package='simple_camera',
            executable='camera_node',
            name='camera_node',
            output='screen',
        ),
        
        # Perception nodes
        Node(
            package='eco_perception',
            executable='stop_sign_detection',
            name='stop_sign_detection',
            output='screen',
            parameters=[{
                'stop_sign_cascade_path': os.path.join(models_dir, 'cascade_stop_sign.xml'),
                'camera_focal_length': 800.0,  # Focal length in pixels (needs calibration)
                'stop_sign_real_width': 0.6,  # Actual width of a stop sign in meters (needs verification)
                'min_detection_width': 30  # Minimum pixel width for a detected stop sign
            }]
        ),
        
        Node(
            package='eco_perception',
            executable='lane_detection',
            name='lane_detection',
            output='screen',
            parameters=[{
                'weights': LaunchConfiguration('model_path'),
                'img_size': [640, 480],
                'conf_thres': 0.3,
                'device': '0',  # Use '0' for first GPU, 'cpu' for CPU
                'roi_width_lower': 0,
                'roi_width_upper': 637,
                'roi_height_lower': 330,
                'roi_height_upper': 400,
                'smoothing_factor': 0.3
            }]
        ),
                
        Node(
            package='eco_control',
            executable='lateral_controller',
            name='lateral_controller',
            output='screen',
            parameters=[{
                'kp': 1.0,
                'ki': 0.0,
                'kd': 0.0,
                'wheelbase': 1.42,  # meters
            }]
        ),
        Node(
            package='eco_control',
            executable='longitudinal_controller',
            name='longitudinal_controller',
            output='screen',
            parameters=[{
                'kp': 1.0,
                'ki': 0.0,
                'kd': 0.0,
                'max_throttle': 0.8,  # degrees
                'min_throttle': -0.5,  # degrees/sec (equivalent to 0.5 rad/s)
            }]
        ),
        
        # planning nodes
        Node(
            package='eco_planning',
            executable='state_machine',
            name='state_machine',
            output='screen',
            parameters=[{
                'stop_duration': 3.0,
            }]
        ),
        
        Node(
            package='eco_planning',
            executable='section1_planner',
            name='section1_planner',
            output='screen',
            parameters=[{
                'max_speed': 0.4, # m/s
                'min_speed': 0.1, # m/s
                'max_angular_velocity': 0.5, # rad/s
                'lane_center_gain': 1.0, # Gain for lane center
                'lane_heading_gain': 2.0 # Gain for lane heading
            }]
        ),
        
        # control nodes
        Node(
            package='eco_control',
            executable='steering',
            name='steering',
            output='screen',
        ),
        
        Node(
            package='eco_control',
            executable='motor',
            name='motor',
            output='screen',
        ),

    ])
