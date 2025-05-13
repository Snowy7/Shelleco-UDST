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
        
        astra_launch,
        
        # Perception nodes
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
                'roi_height_lower': 380,
                'roi_height_upper': 480,
                'smoothing_factor': 0.3
            }]
        ),
    ])
