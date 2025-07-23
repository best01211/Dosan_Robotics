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
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import yaml
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Int32
from aruco_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from std_msgs.msg import Float32


def detect_markers(image, camera_matrix, dist_coeffs, marker_size):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(image)
    detect_data = []
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        rvecs, tvecs, _ = my_estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
        
        if rvecs is not None and tvecs is not None:
            for rvec, tvec, marker_id in zip(rvecs, tvecs, ids):
                rot_mat, _ = cv2.Rodrigues(rvec)
                yaw, pitch, roll = rotationMatrixToEulerAngles(rot_mat)
                marker_pos = np.dot(-rot_mat.T, tvec).flatten()
                distance = np.linalg.norm(tvec)
                detect_data.append([marker_id, marker_pos, (yaw, pitch, roll), distance])
    return image, detect_data

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    rvecs = []
    tvecs = []
    for c in corners:
        _, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
    return rvecs, tvecs, []

def rotationMatrixToEulerAngles(R):
    sy = np.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    return np.degrees(x), np.degrees(y), np.degrees(z)

def load_camera_parameters(yaml_file):
    package_share_directory = get_package_share_directory('aruco_yolo')
    calibration_file = os.path.join(package_share_directory, 'config', yaml_file)

    with open(calibration_file, 'r') as f:
        data = yaml.safe_load(f)
        camera_matrix = np.array(data["camera_matrix"]["data"], dtype=np.float32).reshape(3, 3)
        dist_coeffs = np.array(data["distortion_coefficients"]["data"], dtype=np.float32)
        
    return camera_matrix, dist_coeffs
    
class ArucoMarkerDetector(Node):
    def __init__(self):
        super().__init__('aruco_marker_detector')

        self.get_logger().info(f'ArucoMarkerDetector(Node)')

        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.listener_callback,
            10)

        #Publish Detected markers Info
        self.marker_publisher = self.create_publisher(MarkerArray, 'detected_markers', 10)
        
        #Publish Distance for goal
        self.distance_publisher_ = self.create_publisher(Float32, '/aruco/distance', 10)

        self.bridge = CvBridge()
        self.marker_size = 0.04
        #config/calibration_params.yaml(640x480)
        self.camera_matrix, self.dist_coeffs = load_camera_parameters('calibration_params.yaml')
        
        # self.camera_matrix  = []
        # self.dist_coeffs = []

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        #width, height, channels = frame.shape
        #self.camera_matrix  = np.array([[360, 0, width/2], [0, 360, height/2], [0, 0, 1]])
        #self.dist_coeffs =  np.array([[0, 0, 0, 0, 0]])
        
        #Apply Calibration
        frame, detect_data = detect_markers(frame, self.camera_matrix, self.dist_coeffs, self.marker_size)
        
        #[marker_id, marker_pos, (yaw, pitch, roll), distance]

        if len(detect_data) == 0:
            self.get_logger().debug("No markers detected")
        else:
            closest_marker = min(detect_data, key=lambda x: x[3])
            #self.get_logger().info(f"Closest Marker ID: {closest_marker[0]}, Distance: {closest_marker[3]:.2f}m")

            #self.get_logger().info(f'Position: x:[{closest_marker[1][0]}, y:[{closest_marker[1][1]},z:[{closest_marker[1][2]}]] ')
            #self.get_logger().info(f'Orientation: x:[{closest_marker[2][2]}, y:[{closest_marker[2][1]},z:[{closest_marker[2][0]}]] ')

            distance = f"x:{closest_marker[1][0]:.2f},z:{closest_marker[1][2]:.2f},distance:{closest_marker[3]:.2f}m"


            msg_distance = Float32()
            msg_distance.data = closest_marker[3]
            self.distance_publisher_.publish(msg_distance)
            #self.get_logger().info(f'Publishing: {msg_distance.data}m')

            cv2.putText(frame, distance, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
            (0, 255, 0), 1,                       # 색상 (BGR), 두께
            cv2.LINE_AA)

            marker_array_msg = MarkerArray()
            #Publish Closest Marker position and orientation

            for marker in detect_data:
                if int(closest_marker[0]) != int(marker[0]):
                    continue

                marker_msg = Marker()
                marker_msg.id = int(marker[0])
                marker_msg.pose.pose.position.x = marker[1][0]
                marker_msg.pose.pose.position.y = marker[1][1]
                marker_msg.pose.pose.position.z = marker[1][2]
                marker_msg.pose.pose.orientation.x = marker[2][2]
                marker_msg.pose.pose.orientation.y = marker[2][1]
                marker_msg.pose.pose.orientation.z = marker[2][0]
                marker_array_msg.markers.append(marker_msg)


            self.marker_publisher.publish(marker_array_msg)
          
#######################################################################
# remove cv2...
#######################################################################      
        # Show the image
        
    
        cv2.imshow('Detected Markers', frame)
        cv2.resizeWindow("Detected Markers", 320, 240)  # Error Width x Height in pixels        
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_marker_detector = ArucoMarkerDetector()
    
    rclpy.spin(aruco_marker_detector)
    aruco_marker_detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Detect ArUco markers.')
    parser.add_argument('--marker_size', type=float, default=0.04,help='Size of the ArUco markers in meters.')
    args = parser.parse_args()
    ArucoMarkerDetector.marker_size = args.marker_size
    main()
