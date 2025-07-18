#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera Frame Capture and JPEG Compression

This script captures frames from a webcam using OpenCV,
retrieves the raw image's width and height, compresses
the image to JPEG format, and also determines the size
of the decoded (compressed) image.

Author: Rujin Kim
Date: 2025-05-17
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():


    control_node = Node(
            package='turtlebot3_autorace_mission',
            executable='control_lane',
            name='control_lane',
            output='screen',          
            arguments=['--ros-args', '--log-level', 'INFO'],
            remappings=[
                ('/control/lane', '/detect/lane'),
                ('/control/cmd_vel', '/cmd_vel')
            ]
        )
    
    return LaunchDescription([
        control_node
    ])
