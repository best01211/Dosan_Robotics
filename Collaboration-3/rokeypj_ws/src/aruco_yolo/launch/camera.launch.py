#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera Frame Capture and JPEG Compression

This script captures frames from a webcam using OpenCV,
retrieves the raw image's width and height, compresses
the image to JPEG format, and also determines the size
of the decoded (compressed) image.

Author: Rujin Kim
Date: 2025-06-16
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('aruco_yolo')
    config_file = os.path.join(pkg_dir, 'config', 'camera_params.yaml')
    calibration_file = os.path.join(pkg_dir, 'config', 'calibration_params.yaml')
    
    # Declare launch arguments
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera device ID'
    )
    
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='30.0',
        description='Camera frame rate'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera name for topics'
    )
    
    use_calibration_arg = DeclareLaunchArgument(
        'use_calibration',
        default_value='true',
        description='Use camera calibration file'
    )
    
    # Camera publisher node
    camera_node = Node(
        package='aruco_yolo',
        executable='camera_pub',
        name='camera_pub',
        parameters=[
            config_file,
            {
                'camera_id': LaunchConfiguration('camera_id'),
                'frame_rate': LaunchConfiguration('frame_rate'),
                'camera_name': LaunchConfiguration('camera_name'),
                'calibration_file': calibration_file,
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        camera_id_arg,
        frame_rate_arg,
        camera_name_arg,
        use_calibration_arg,
        camera_node
    ])
