#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlebot_cosmo_interface.srv import MoveitControl
from geometry_msgs.msg import Pose, PoseArray
from srv_call_test import TurtlebotArmClient
import time


# ANSI 색상 코드 정의
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
MAGENTA = "\033[95m"
CYAN = "\033[96m"
RESET = "\033[0m"  # 색상 초기화

def append_pose_init(x,y,z):
    pose_array = PoseArray()
    pose = Pose()

    pose.position.x = x
    pose.position.y =  y
    pose.position.z =  z

    pose_array.poses.append(pose)
    
    print(f"{CYAN}Pose initialized - x: {x}, y: {y}, z: {z}{RESET}")

    return pose_array

def main(args=None):
    rclpy.init(args=args)
    arm_client = TurtlebotArmClient()

    print ("task start!")
    
    response = arm_client.send_request(1, "camera_home")
    arm_client.get_logger().info(f'Response: {response.response}')

    response = arm_client.send_request(2, "open")
    arm_client.get_logger().info(f'Response: {response.response}')    
    

    response = arm_client.send_request(9, "")
    arm_client.get_logger().info(f'Response: {response.response}')

    pose_array = append_pose_init(0.2203589 ,- 0.0800000   ,0.185779)

    response = arm_client.send_request(0, "", pose_array)
    arm_client.get_logger().info(f'Response: {response.response}')     

    response = arm_client.send_request(9, "")
    arm_client.get_logger().info(f'Response: {response.response}')

    response = arm_client.send_request(2, "close")
    arm_client.get_logger().info(f'Response: {response.response}')
    
    time.sleep(1)

    response = arm_client.send_request(2, "open")
    arm_client.get_logger().info(f'Response: {response.response}')    

    response = arm_client.send_request(1, "camera_home")
    arm_client.get_logger().info(f'Response: {response.response}')





    print ("task end!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()