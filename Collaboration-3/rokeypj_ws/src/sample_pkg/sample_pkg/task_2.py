#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlebot_cosmo_interface.srv import MoveitControl
from geometry_msgs.msg import Pose, PoseArray
from srv_call_test import TurtlebotArmClient
import time

def main(args=None):
    rclpy.init(args=args)
    arm_client = TurtlebotArmClient()

    print ("task start!")

    ### pose 기준으로 움직이기
    pose_array = PoseArray()
    pose = Pose()
    
    # camera_setup
    pose.position.x = -0.0821106
    pose.position.y =  -0.0821106
    pose.position.z =  0.0
    
    #pose.orientation.x = 0.0
    #pose.orientation.y = 0.0
    #pose.orientation.z =  -0.0
    #pose.orientation.w = 1.0
    
    pose_array.poses.append(pose)

    response = arm_client.send_request(4, "", pose_array)
    arm_client.get_logger().info(f'Response: {response.response}')

    
    #response = arm_client.send_request(9, "")
    #arm_client.get_logger().info(f'Response: {response.response}')


    print ("task end!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()