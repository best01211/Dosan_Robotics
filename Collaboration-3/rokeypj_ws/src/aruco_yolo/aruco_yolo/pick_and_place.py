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
import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import getkey
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from geometry_msgs.msg import Twist, Pose, PoseArray
from turtlebot_cosmo_interface.srv import MoveitControl
from aruco_yolo.moveit_client import TurtlebotArmClient
import time
import ast


# ANSI 색상 코드 정의
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
MAGENTA = "\033[95m"
CYAN = "\033[96m"
RESET = "\033[0m"  # 색상 초기화


class ArucoMarkerListener(Node):
    def __init__(self):
        super().__init__('aruco_marker_listener')

        # Change this to the desired marker ID from pick_n_place.launch.py file, Declare parameter with default integer value
        self.markerid = self.declare_parameter('markerid', 1).get_parameter_value().integer_value

        self.target_marker_id = self.markerid 

        self.subscription = self.create_subscription(
            MarkerArray,
            'detected_markers',
            self.aruco_listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 2)

        self.twist = Twist()
        self.finish_move = False


        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.get_joint = False
        self.marker = []

        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.trajectory_msg = JointTrajectory()

        self.trajectory_msg.header = Header()
        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
    

        self.point = JointTrajectoryPoint()
        self.point.velocities = [0.0] * 4
        self.point.accelerations = [0.0] * 4
        self.point.time_from_start.sec = 0
        self.point.time_from_start.nanosec = 500

        #sample_pkg/src/simple_manager_node.py
        # 상태 변수
        self.aruco_marker_found = False
        self.task_completed = False
        self.armrun = False
        self.aruco_location_x = 0
        self.aruco_location_y = 0
        self.aruco_location_z = 0       

        self.marker_id = None
        self.state = 'START'  

        self.count = 0
        self.aruco_pose = None  # Aruco marker의 pose 정보를 저장할 변수

        #self.create_timer(1.0, self.run_tasks)
        
        #self.twist = Twist()



    def joint_states_callback(self, msg):

        if self.get_joint == False:
            return
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint1' in name:
                print(f'joint1 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint2' in name:
                print(f'joint2 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint3' in name:
                print(f'joint3 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint4' in name:
                print(f'joint4 : {position}')

    def aruco_listener_callback(self, msg):
        print('aruco_listener_callback called')
        print(f"Number of markers: {len(msg.markers)}")

        for marker in msg.markers:
            print(f"Detected Marker ID: {marker.id}, Target ID: {self.target_marker_id}")
            if marker.id == self.target_marker_id:
                self.get_logger().info(f'Marker ID: {marker.id}, Z: {marker.pose.pose.position.z}')

                self.aruco_position_x = marker.pose.pose.position.x
                self.aruco_position_y = marker.pose.pose.position.y               
                self.aruco_position_z = marker.pose.pose.position.z

                z = marker.pose.pose.position.z

                self.aruco_arm_controll()
                # if z > 0.41:
                #     self.get_logger().info(f'publish_cmd_vel(0.10)')
                #     self.publish_cmd_vel(0.10)
                # elif z > 0.31:
                #     self.get_logger().info(f'publish_cmd_vel(0.06)')                    
                #     self.publish_cmd_vel(0.06)
                # elif z > 0.21:
                #     self.get_logger().info(f'publish_cmd_vel(0.04)')                    
                #     self.publish_cmd_vel(0.04)
                # else:
                #     self.publish_cmd_vel(0.0)
                #     self.get_logger().info('FINISH - calling aruco_arm_controll()')
                #     self.finish_move = True
                #     print("Calling aruco_arm_controll() now")
                #     self.aruco_arm_controll()
                # break


    def publish_cmd_vel(self, linear_x):
        self.twist.linear.x = linear_x
        self.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.twist)  


##################################################################################            
##################################################################################
    def append_pose_init(self, x, y, z):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0  # ✅ 꼭 있어야 함
        pose_array = PoseArray()
        pose_array.poses.append(pose)
        return pose_array


    def aruco_arm_controll(self):
        print("Impossible Mission Start")        
        arm_client = TurtlebotArmClient()

        # ArUco 마커의 위치를 이용해 이동할 위치 계산
        print(f"Mission Aruco marker Location coordinates: {self.aruco_position_x}, {self.aruco_position_y}, {self.aruco_position_z}")

        # 마커가 감지되었으면 이동할 준비
        self.aruco_marker_found = True
        if self.aruco_marker_found:
            self.armrun = True

            # 초기 위치 설정
            print(f"Remove Rock Initial Position")
            time.sleep(2)
            self.point.positions = [0.0, -0.2443, -0.6283, 2.0246]  # 초기 설정 위치
            print("point", self.point.positions)
            self.joint_pub.publish(self.trajectory_msg)

            # ArUco 마커 위치로 이동
            print("Move Aruco Cube(Rock) Mission")
            response = arm_client.send_request(1, "01_home")  # move to home position (side)
            arm_client.get_logger().info(f'Response: {response.response}')

            # 그리퍼 열기
            print("Gripper Open")
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')

            # Cube 위치로 이동: ArUco 마커의 좌표 기반으로 pose_array 설정
            print("Cube Position Start")
            pose_array = self.append_pose_init(
                self.aruco_position_x,   # ArUco 마커의 X 좌표
                self.aruco_position_y,   # ArUco 마커의 Y 좌표
                self.aruco_position_z    # ArUco 마커의 Z 좌표
            )
            self.get_logger().info(f'[DEBUG] Created pose_array: x={pose_array.poses[0].position.x}, y={pose_array.poses[0].position.y}, z={pose_array.poses[0].position.z}')

            # PoseArray를 보내 MoveIt으로 위치 이동 명령
            response = arm_client.send_request(0, "", pose_array)  # cmd == 0, pose_array로 이동
            arm_client.get_logger().info(f'Response: {response.response}')

            # Box Front로 이동
            print("Cube Box Front Start...")
            time.sleep(2)
            response = arm_client.send_request(1, "02_box_front")
            arm_client.get_logger().info(f'Response: {response.response}')

            # Box로 이동
            print("Move to Box Start...")
            time.sleep(2)
            response = arm_client.send_request(1, "03_move_to_box")
            arm_client.get_logger().info(f'Response: {response.response}')

            time.sleep(2)
            print("Gripper Close")
            response = arm_client.send_request(2, "close")
            arm_client.get_logger().info(f'Response: {response.response}')

            # 위로 이동
            print("Move up Start...")
            time.sleep(2)
            response = arm_client.send_request(1, "04_move_up")
            arm_client.get_logger().info(f'Response: {response.response}')

            # 컨베이어 위로 이동
            print("Conveyor up Start...")
            time.sleep(2)
            response = arm_client.send_request(1, "05_conveyor_up")
            arm_client.get_logger().info(f'Response: {response.response}')

            # 컨베이어 아래로 이동
            print("Conveyor down Start...")
            time.sleep(2)
            response = arm_client.send_request(1, "06_conveyor_down")
            arm_client.get_logger().info(f'Response: {response.response}')

            time.sleep(2)
            print("Gripper Open")
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')

            # 컨베이어 다시 위로 이동
            print("Conveyor up Start...")
            time.sleep(2)
            response = arm_client.send_request(1, "07_conveyor_up")
            arm_client.get_logger().info(f'Response: {response.response}')

            # 기본 위치로 복귀
            response = arm_client.send_request(1, "01_home")  # move to home position (side)
            arm_client.get_logger().info(f'Response: {response.response}')

            # 사용자 정의 위치로 복귀
            print("Return to Custom Home Position")
            self.point.positions = [0.0, -0.2443, -0.6283, 2.0246]  # 예시 값
            self.trajectory_msg.points = [self.point]
            print("point:", self.point.positions)
            self.joint_pub.publish(self.trajectory_msg)
            rclpy.spin_once(self)  # force flush
            time.sleep(2)

            # 완료
            self.finish_move = True
            print("Impossible Mission Clear")

            self.armrun = False



def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerListener()

    # 초기 위치 설정
    joint_pub = node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
    trajectory_msg = JointTrajectory()
    trajectory_msg.header = Header()
    trajectory_msg.header.frame_id = ''
    trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

    point = JointTrajectoryPoint()
    point.velocities = [0.0] * 4
    point.accelerations = [0.0] * 4
    point.time_from_start.sec = 0
    point.time_from_start.nanosec = 500
    point.positions = [0.0, -0.2443, -0.6283, 2.0246]   # 초기 자세
    trajectory_msg.points = [point]

    joint_pub.publish(trajectory_msg)
    print("Pick and Place Init done")

    try:
        # 콜백 내부에서 aruco_arm_controll 실행하도록 하고, 여기선 spin만 수행
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()