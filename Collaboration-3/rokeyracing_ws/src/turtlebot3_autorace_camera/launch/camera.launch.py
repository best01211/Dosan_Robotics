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
            'image_width', 
            default_value='640',  
            description='camera frame width'
        ), 
        DeclareLaunchArgument(
            'image_height', 
            default_value='480',  
            description='camera frame height'
        ),     
        DeclareLaunchArgument(
            'video_fps', 
            default_value='25',  
            description='camera frame FPS(frames per second)'
        ),          

        DeclareLaunchArgument('namespace', default_value='camera'),
        PushRosNamespace(LaunchConfiguration('namespace')),    

        Node(
            package='turtlebot3_autorace_camera',
            executable='camera_image_pub',
            name='camera_image_pub',
            parameters=[{
                'video_port': LaunchConfiguration('video_port'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),        
                'video_fps': LaunchConfiguration('video_fps'),                                         
            }],
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO']            
        ),
      #  Node(
      #      package='aruco_yolo',
      #      executable='camera_info_pub',
      #      name='camera_info_pub',
      #      output='screen',
      #      arguments=['--ros-args', '--log-level', 'INFO']            
      #  ),
    ])
