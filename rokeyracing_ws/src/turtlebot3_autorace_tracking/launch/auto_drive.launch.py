#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Rujin Kim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    calibration_mode_arg = DeclareLaunchArgument(
        'calibration_mode',
        default_value='False',
        description='Mode type [calibration, action]'
    )
    calibration_mode = LaunchConfiguration('calibration_mode')


    video_width_arg = DeclareLaunchArgument(
            'video_width', 
            default_value='640',  
            description='camera frame width'
        )
    video_width = LaunchConfiguration('video_width')

    video_height_arg = DeclareLaunchArgument(
            'video_height', 
            default_value='480',  
            description='camera frame height'
        ) 
    video_height= LaunchConfiguration('video_height')
    
    detect_param = os.path.join(
        get_package_share_directory('turtlebot3_autorace_tracking'),
        'conf',
        'pid.yaml'
        )

    detect_lane_node = Node(
        package='turtlebot3_autorace_tracking',
        executable='auto_drive',
        name='auto_drive',
        output='screen',
        parameters=[
            {'is_detection_calibration_mode': calibration_mode},
            detect_param
        ],
        remappings=[
            ('/detect/image_input', '/camera/image_raw'),
            ('/detect/image_output', '/detect/image_lane'),
            ('/detect/image_output/compressed', '/detect/image_lane/compressed'),
        ]
    )

    return LaunchDescription([
        calibration_mode_arg,
        video_width_arg,
        video_height_arg,
        detect_lane_node,
    ])
