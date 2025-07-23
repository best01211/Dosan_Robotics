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


class ArucoMarkerListener(Node):
    def __init__(self):
        super().__init__('aruco_marker_listener')

        self.subscription = self.create_subscription(
            MarkerArray,
            'detected_markers',
            self.listener_callback,
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

    def listener_callback(self, msg):
        self.target_marker_id = 0                      # Change this to the desired marker ID
        for marker in msg.markers:
            if self.marker.id == self.target_marker_id:
                self.get_logger().debug(f'Marker ID: {marker.id}, PositionZ: {marker.pose.pose.position.z}')
                print(f'z:[{self.marker.pose.pose.position.z}] x:[{self.marker.pose.pose.position.x}] ')
                # if marker.pose.pose.position.z > 0.30:
                if self.marker.pose.pose.position.z > 0.41:
                    self.publish_cmd_vel(0.10)
                elif self.marker.pose.pose.position.z > 0.31:
                    self.publish_cmd_vel(0.06)
                elif self.marker.pose.pose.position.z > 0.21 :
                    self.publish_cmd_vel(0.04)
                else:
                    self.publish_cmd_vel(0.0)
                    self.finish_move = True
                break
            
    def publish_cmd_vel(self, linear_x):
        self.twist.linear.x = linear_x
        self.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.twist)            

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerListener()
    #rclpy.spin(node)

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

    point.positions = [0.0, -1.052, 1.106, 0.029]

    print("point",point.positions)
    trajectory_msg.points = [point]
    joint_pub.publish(trajectory_msg)

    print("init done")
    rclpy.spin_once(node)

    while(rclpy.ok()):
        key_value = getkey.getkey()
        if key_value == '1':          # cube home
            print(f"No. {key_value}")
            point.positions = [0.0, -0.058, -0.258, 1.94]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)

        elif key_value == '2':        # box home
            print(f"No. {key_value}")
            point.positions = [0.0, -1.052, 1.106, 0.029]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)

        elif key_value == '3':        # move to cube
            print(f"No. {key_value}")
            while 1:
                rclpy.spin_once(node)
                if(node.finish_move == True):
                    node.finish_move = False
                    break
                
        elif key_value == '4':        # forward
            print(f"No. {key_value}")
            node.twist.linear.x = 0.5
            node.twist.angular.z = 0.0
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '5':        # backward
            print(f"No. {key_value}") 
            node.twist.linear.x =-0.5
            node.twist.angular.z = 0.0
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '6':        # forward + left turn
            print(f"No. {key_value}")
            node.twist.linear.x = 0.5
            node.twist.angular.z = 0.1
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '7':        # forward + right turn
            print(f"No. {key_value}")
            node.twist.linear.x = 0.5
            node.twist.angular.z =-0.1
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '8':        # get joint value
            print(f"No. {key_value}")
            node.get_joint = True
            rclpy.spin_once(node)
            node.get_joint = False

        elif key_value == '9':        # distance and offset for marker
            print(f"No. {key_value}")
            rclpy.spin_once(node)

        elif key_value == 'q':
            print(f"No. {key_value}")
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()