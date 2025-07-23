#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlebot_cosmo_interface.srv import MoveitControl
from geometry_msgs.msg import Pose, PoseArray
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

    print ("task start!")

    #lx = 0.144143
    #ly = 0.0
    #lz = 0.233847

    lx = 0.14
    ly = 0.0
    lz = 0.23

    mul = 1
    pose_array = append_pose_init(lx, ly, lz)

    response = arm_client.send_request(0, "", pose_array)
    #arm_client.get_logger().info(f'Response: {response.response}')
    print(lx, ly, lz)




    while(rclpy.ok()):
        # rclpy.spin_once(node)
        print(" ")
        key_value = getkey.getkey()
        print(" ")
         
        if key_value == '0':
            # ## gripper control
            response = arm_client.send_request(1, "box_up_02")
            arm_client.get_logger().info(f'box_up_02 command : Response: {response.response}')

        elif key_value == '1':
            lx += 0.01 * mul
            pose_array = append_pose_init(lx, ly, lz)
            response = arm_client.send_request(0, "", pose_array)
            #arm_client.get_logger().info(f'Response: {response.response}')
            print(lx, ly, lz)


        elif key_value == '2':
            lx -= 0.01 * mul
            pose_array = append_pose_init(lx, ly, lz)
            response = arm_client.send_request(0, "", pose_array)
            #arm_client.get_logger().info(f'Response: {response.response}')
            print(lx, ly, lz)    

        elif key_value == '3':
            ly += 0.01 * mul
            pose_array = append_pose_init(lx, ly, lz)
            response = arm_client.send_request(0, "", pose_array)
            #arm_client.get_logger().info(f'Response: {response.response}')
            print(lx, ly, lz)    
    
        elif key_value == '4':
            ### pose 기준으로 움직이기
            # camera_setup
            ly -= 0.01 * mul
            pose_array = append_pose_init(lx, ly, lz)
            response = arm_client.send_request(0, "", pose_array)
            #arm_client.get_logger().info(f'Response: {response.response}')
            print(lx, ly, lz)    
    
        elif key_value == '5':
            lz += 0.01 * mul 
            pose_array = append_pose_init(lx, ly, lz)
            response = arm_client.send_request(0, "", pose_array)
            #arm_client.get_logger().info(f'Response: {response.response}')
            print(lx, ly, lz)    

        elif key_value == '6':
            lz -= 0.01 * mul
            pose_array = append_pose_init(lx, ly, lz)
            response = arm_client.send_request(0, "", pose_array)
            #arm_client.get_logger().info(f'Response: {response.response}')
            print(lx, ly, lz)    

        elif key_value == 'a':
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')

        elif key_value == 'b':
            response = arm_client.send_request(2, "close")
            arm_client.get_logger().info(f'Response: {response.response}')

        elif key_value == 'c':
            response = arm_client.send_request(9, "")
            arm_client.get_logger().info(f'Response: {response.response}')

        elif key_value == 'h':
            mul = 1
            print(mul)

        elif key_value == 'i':
            mul = 10
            print(mul)

        elif key_value == 'j':
            mul = 20
            print(mul)

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
            print ("task end!")



        elif key_value == 'B':
            pass

        elif key_value == 'C':
            pass

        elif key_value == 'D':
            print("1. ")

        elif key_value == 'M':
            pass

        elif key_value == 'U':
            response = arm_client.send_request(1, "camera_home")
            arm_client.get_logger().info(f'Response: {response.response}')    

        elif key_value == 'V':
            lx = 0.14
            ly = 0.0
            lz = 0.23

            mul = 1
            pose_array = append_pose_init(lx, ly, lz)
            response = arm_client.send_request(0, "", pose_array)
            #arm_client.get_logger().info(f'Response: {response.response}')
            print(lx, ly, lz)    

        elif key_value == 'W':
            lx = 0.16
            ly =-0.05
            lz = 0.085

            mul = 1
            pose_array = append_pose_init(lx, ly, lz)
            response = arm_client.send_request(0, "", pose_array)
            #arm_client.get_logger().info(f'Response: {response.response}')
            print(lx, ly, lz)    

        elif key_value == 'X':
            lx = 0.15
            ly = 0.08
            lz = 0.085

            mul = 1
            pose_array = append_pose_init(lx, ly, lz)
            response = arm_client.send_request(0, "", pose_array)
            #arm_client.get_logger().info(f'Response: {response.response}')
            print(lx, ly, lz)    

        elif key_value == 'Y':
            #lx = 0.19
            lx = 0.22
            ly =-0.05
            lz = 0.085

            mul = 1
            pose_array = append_pose_init(lx, ly, lz)
            response = arm_client.send_request(0, "", pose_array)
            #arm_client.get_logger().info(f'Response: {response.response}')
            print(lx, ly, lz)    

        elif key_value == 'Z':
            lx = 0.217
            ly = 0.083
            lz = 0.085

            mul = 1
            pose_array = append_pose_init(lx, ly, lz)
            response = arm_client.send_request(0, "", pose_array)
            #arm_client.get_logger().info(f'Response: {response.response}')
            print(lx, ly, lz)    

    print ("task end!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()