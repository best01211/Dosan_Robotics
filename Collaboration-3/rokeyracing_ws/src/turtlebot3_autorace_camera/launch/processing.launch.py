from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_port', 
            default_value='0', 
            description='video port number'
        ),  
        DeclareLaunchArgument(
            'roi', 
            default_value='Bottom',  
            description='roi'
        ),      

        #DeclareLaunchArgument('namespace', default_value='camera'),
        #PushRosNamespace(LaunchConfiguration('namespace')),    

        Node(
            package='turtlebot3_autorace_camera',
            executable='image_processing',
            name='image_processing',
            parameters=[{
                'video_port': LaunchConfiguration('video_port'),
                'roi': LaunchConfiguration('roi'),                                        
            }],
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO']            
        ),
    ])
