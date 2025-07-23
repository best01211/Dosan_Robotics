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
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav2_pkg_share = get_package_share_directory('turtlebot3_navigation2')

    param_file = os.path.join(
        get_package_share_directory('turtlebot3_autorace_mission'),
        'param',
        'navigation.yaml'
    )
    map_file = os.path.join(
        get_package_share_directory('turtlebot3_autorace_mission'),
        'map',
        'map.yaml'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_share, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'map': map_file
        }.items()
    )

    mission_tunnel_node = Node(
        package='turtlebot3_autorace_mission',
        executable='mission_tunnel',
        name='mission_tunnel',
        output='screen',
        parameters=[param_file]
    )

    return LaunchDescription([
        nav2_launch,
        mission_tunnel_node,
    ])
