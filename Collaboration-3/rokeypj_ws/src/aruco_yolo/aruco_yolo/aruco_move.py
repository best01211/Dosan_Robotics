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
from std_msgs.msg import Float32


class ArucoMarkerListener(Node):
    def __init__(self):
        super().__init__('aruco_marker_listener')
        self.target_marker_id = 0  # Change this to the desired marker ID

        self.move_distance = 0.9        
        self.target_distance = 0.5
        self.aruco_distance = 0.0
        
        self.subscription = self.create_subscription(
            MarkerArray,
            'detected_markers',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.distance_subscription = self.create_subscription(
            Float32,
            '/aruco/distance',
            self.distance_listener_callback,
            10)
        
        self.subscription  # prevent unused variable warning


        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 2)

    def distance_listener_callback(self, msg):
        self.aruco_distance = msg.data
        self.get_logger().info(f'Received distance: {msg.data:.2f} m')

    def listener_callback(self, msg):
        target_marker_id = self.target_marker_id

        decision_value = 'distacne'
        for marker in msg.markers:
            if marker.id == target_marker_id:
                self.get_logger().debug(f'Marker ID: {marker.id}, PositionZ: {marker.pose.pose.position.z}')

                if(decision_value == 'distance'):  
                    if abs(self.aruco_distance) > self.move_distance:
                        self.publish_cmd_vel(0.08)
                    elif abs(self.aruco_distance) > self.target_distance:
                        self.publish_cmd_vel(0.02)
                    else:
                        self.publish_cmd_vel(0.0)
                    break
                elif(decision_value == 'x'):  
                    if abs(marker.pose.pose.position.x) > self.move_distance:
                        self.publish_cmd_vel(0.08)
                    elif abs(marker.pose.pose.position.x) > self.target_distance:
                        self.publish_cmd_vel(0.02)
                    else:
                        self.publish_cmd_vel(0.0)
                    break 
                elif(decision_value == 'z'):  
                    if abs(marker.pose.pose.position.z) > self.move_distance:
                        self.publish_cmd_vel(0.05)
                    elif abs(marker.pose.pose.position.z) > self.target_distance:
                        self.publish_cmd_vel(0.02)
                    else:
                        self.publish_cmd_vel(0.0)
                    break                 
            
    def publish_cmd_vel(self, linear_x):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)            

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()