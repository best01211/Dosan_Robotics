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

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_marker_id', #Error if argument name is videoport
            default_value='0',  # string, but will be interpreted as int
            description='target_marker_id'
        ),        
        Node(
            package='aruco_yolo',
            executable='aruco_move',
            name='aruco_move',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
 
    ])