#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlebot_cosmo_interface.srv import MoveitControl
from geometry_msgs.msg import Pose, PoseArray, Twist
from srv_call_test import TurtlebotArmClient
import time
import getkey

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

    cmd_vel_publisher = arm_client.create_publisher(Twist, '/cmd_vel', 2)
    twist = Twist()

    print ("task start!")

    while(rclpy.ok()):
        # rclpy.spin_once(node)
        key_value = getkey.getkey()
         
        if key_value == '1':
            # ## gripper control
            response = arm_client.send_request(1, "box_up_02")
            arm_client.get_logger().info(f'Response: {response.response}')    

        elif key_value == '2':
            response = arm_client.send_request(1, "box_up_03")
            arm_client.get_logger().info(f'Response: {response.response}')        

        elif key_value == '3':
            response = arm_client.send_request(1, "home2")
            arm_client.get_logger().info(f'Response: {response.response}')        
    
        elif key_value == '4':
            ### pose 기준으로 움직이기
            pose_array = PoseArray()
            pose = Pose()
    
            # camera_setup
            pose.position.x = -0.0821106
            pose.position.y =  -0.0821106
            pose.position.z =  0.206947
    
            pose.orientation.x = 0.0294796
            pose.orientation.y = 0.0290754
            pose.orientation.z =  -0.71136
            pose.orientation.w = 0.701607
    
            pose_array.poses.append(pose)

            response = arm_client.send_request(4, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')
    
        elif key_value == '5':
            twist.linear.x = - 0.01
            twist.angular.z = 0.0
            cmd_vel_publisher.publish(twist)  

        elif key_value == '6':
            twist.linear.x = 0.01
            twist.angular.z = 0.0
            cmd_vel_publisher.publish(twist)  

        elif key_value == 'a':
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')

        elif key_value == 'b':
            response = arm_client.send_request(2, "close")
            arm_client.get_logger().info(f'Response: {response.response}')

        elif key_value == 'c':
            response = arm_client.send_request(9, "")
            arm_client.get_logger().info(f'Response: {response.response}')


        elif key_value == 'q':
            break    

        elif key_value == 'z':
            response = arm_client.send_request(9, "")
            arm_client.get_logger().info(f'Response: {response.response}')

        elif key_value == 'A':
            print ("task start!")
            response = arm_client.send_request(1, "camera_home")
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')    
    

            response = arm_client.send_request(9, "")
            arm_client.get_logger().info(f'Response: {response.response}')

            pose_array = append_pose_init(0.1503589 ,0.0800000   ,0.185779)

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

        elif key_value == 'B':
            pass

        elif key_value == 'C':
            pass

        elif key_value == 'D':
            pass

        elif key_value == 'E':
            pass

    print ("task end!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()