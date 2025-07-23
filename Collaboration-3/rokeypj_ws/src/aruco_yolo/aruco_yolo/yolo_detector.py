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
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
import json
import time
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory

# Author: Karl.Kwon
# Email: mrthinks@gmail.com

mtx = np.array(
    [[1381.01926,    0.     ,  695.134361],
        [0.     , 1382.09839,  446.413163],
        [0.     ,    0.     ,    1.     ]]
        )
dist = np.array([[-0.01792346, 0.68685818, 0.0023631, -0.00455134, -2.06831632]])

package_share_directory = get_package_share_directory('aruco_yolo')
#model_path = os.path.join(package_share_directory, 'models', 'yolov8s_trained.pt')
model_path = "/home/rokey12/rokeypj_ws/src/aruco_yolo/models/new_new_new_best.pt"
print(f"Model path: {model_path}")

if not os.path.exists(model_path):
    raise FileNotFoundError(f"Model file not found at {model_path}")

# pix_2_mm = 0.00012 # distance(m) - 175mm
pix_2_mm = 0.000153 # distance(m) - mm

class YoloDetect(Node):
    def __init__(self):
        super().__init__('yolo_detect')
        print('model loading...')
        self.model = YOLO(model_path)
        print('model load done.')

        self.subscription_rgb = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.listener_callback_rgb, 1)
        self.subscription_rgb  # prevent unused variable warning

        self.img_publisher_ = self.create_publisher(CompressedImage, 'yolo/compressed', 1)
        self.info_publisher_ = self.create_publisher(String, 'yolo/detected_info', 5)

        self.last_pub_time = time.time()

    def publish_img(self, frame):
        time_cur = time.time()

        if time_cur - 0.2 < self.last_pub_time:
            return

        self.last_pub_time = time_cur

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]  # 90은 압축 품질
        _, compressed_image = cv2.imencode('.jpg', frame, encode_param)

        # 압축된 이미지를 CompressedImage 메시지로 변환
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
        msg.header.frame_id = "camera"  # 프레임 ID 설정
        msg.format = "jpeg"  # 압축 형식 설정
        msg.data = compressed_image.tobytes()  # 압축된 이미지 데이터

        # CompressedImage 퍼블리시
        self.img_publisher_.publish(msg)
        # self.get_logger().info('Publishing compressed image...')
        
    def convert_box_center_2_mm(self, results, img_size, pix_2_mm):
        cls = results[0].boxes.cls.tolist()
        centers = []
        centers_img = []
        
        for i, r in enumerate(results[0].boxes.xywh):
            d = r.tolist()
            centers.append( (int(cls[i]), (d[0] - img_size[0]/2)*pix_2_mm, (d[1] - img_size[1]/2)*pix_2_mm) )
            centers_img.append( (d[0], d[1]) )
        
        return centers, centers_img

    def inference(self, img_color, pix_2_mm):
        results = self.model(img_color, conf=0.4)
        plots = results[0].plot()
        shp = img_color.shape

        plots = cv2.line(plots, (0, int(shp[0]/2)), (shp[1], int(shp[0]/2)), (0, 255, 255), 3)
        plots = cv2.line(plots, (int(shp[1]/2), 0), (int(shp[1]/2), shp[0]-1), (0, 255, 255), 3)
        info, info_img = self.convert_box_center_2_mm(results, (shp[1], shp[0]), pix_2_mm)

        font = cv2.FONT_ITALIC
        font_scale = 1
        font_thickness = 2
        for i, (x, y) in enumerate(info_img):
            x = int(x)
            y = int(y)
            text = "%d, %d" % (info[i][1]*1000, info[i][2]*1000)

            text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
            text_w, text_h = text_size
            org = (x, y)

            cv2.rectangle(plots, (x - 5, y - 5), (x + text_w + 5, y + text_h + 5), (0, 0, 0), -1)
            cv2.putText(plots, text, (x, y + text_h + font_scale - 1), font, font_scale, (0, 255, 0), font_thickness)

        return plots, info

    def publish_info(self, str_):
        msg = String()
        msg.data = str_
        self.info_publisher_.publish(msg)

    def listener_callback_rgb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode to color image

        h,  w = image_np.shape[:2]
        ####################################################################3    
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

        # undistort
        image_undistort = cv2.undistort(image_np, mtx, dist, None, newcameramtx)

        plots, info = self.inference(image_undistort, pix_2_mm)
        info = json.dumps(info)
        # print(info)
        plots = cv2.resize(plots, (640, 360))

        self.publish_info(info)
        self.publish_img(plots)


# ros2 topic echo /yolo/detected_info

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = YoloDetect()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
