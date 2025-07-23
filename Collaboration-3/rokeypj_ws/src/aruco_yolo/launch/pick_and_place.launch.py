from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

"""
getkey.getkey()를 백그라운드 노드에서 쓰지 마세요
ROS 노드가 background process로 실행될 때는 키보드 입력을 직접 받을 수 없습니다. 이런 상황에선 다음 중 하나를 선택하세요:
"""
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_port', #Error if argument name is videoport
            default_value='0',  # string, but will be interpreted as int
            description='/dev/video*'
        ),        
        DeclareLaunchArgument(
            'markerid',
            default_value='0',  # string, but will be interpreted as int
            description='Detecting Aruco Marker ID'
        ),        
        Node(
            package='aruco_yolo',
            executable='compressed_image_pub',
            name='compressed_image_pub',
            output='screen'
        ),
        Node(
            package='aruco_yolo',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen'
        ),
        Node(
            package='aruco_yolo',
            executable='aruco_detector',
            name='pick_and_place',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
 
    ])