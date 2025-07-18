#!/usr/bin/env python3
#
# Copyright 2025 HumanAI CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors:
#   - Rujin Kim, kimrujin32@gmail.com


import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CompressedImage
from std_msgs.msg import Float32, Int32
import cv2
import numpy as np
from cv_bridge import CvBridge

import threading
import cv2



class LaneTracker(Node):
    def __init__(self):
        super().__init__('lane_tracker')

        self.subscription = self.create_subscription(
            Image, '/camera/image_projected', self.image_callback,10)
        
        self.lane_pub = self.create_publisher(Float32, '/detect/lane', 10)
        self.lane_state_pub = self.create_publisher(Int32, '/detect/lane_state', 10)

        self.last_image = None
        self.last_result = None

        self.bridge = CvBridge()
        self.get_logger().info('Subscribed to /camera/image_raw')

    def image_callback(self, msg):

        try:
            # ROS Image → OpenCV 이미지 변환 (BGR)
            #cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        
        h, w = cv_image.shape[:2]

        yellow_line, white_line = self.detect_lanes(cv_image)

        if yellow_line:
            x_middle = yellow_line[0]
            y_middle = yellow_line[1]
            x_bottom = yellow_line[2]
            y_bottom = yellow_line[3]
            cv2.rectangle(cv_image, (x_middle, y_middle), (x_bottom, y_bottom), (255, 255, 0), 3)

        if white_line:
            x_middle = white_line[0]
            y_middle = white_line[1]
            x_bottom = white_line[2]
            y_bottom = white_line[3]
            cv2.rectangle(cv_image, (x_middle, y_middle), (x_bottom, y_bottom), (255, 0, 0), 3)


        # 차선 중앙 계산
        center_x = w//2
        detected = 0
        left_x = None
        right_x = None
        # 상태
        if yellow_line and white_line:
            detected = 2
            left_x = yellow_line[0]
            right_x = white_line[0]
        elif yellow_line:
            detected = 1
            left_x = yellow_line[0]
        elif white_line:
            detected = 3
            right_x = white_line[0]
        else:
            detected = 0

        # desired_center: (left+right)/2 or fallback

        if left_x is not None and right_x is not None:
            desired_center = int((left_x + right_x) / 2)
        elif left_x is not None:
            desired_center = int(left_x + 150)
        elif right_x is not None:
            desired_center = int(right_x - 150)
        else:
            desired_center = int(center_x)

        h, w = cv_image.shape[:2]
        cv2.line(cv_image,
                (desired_center, h - 1),
                (desired_center, int(h * 0.5)),
                (0, 0, 255),
                5)
                             

        self.lane_pub.publish(Float32(data=float(desired_center)))
        self.lane_state_pub.publish(Int32(data=int(detected)))


        # 결과 저장
        self.last_image = cv_image
        self.last_result = {
            "yellow": yellow_line,
            "white": white_line,
            "center": int(desired_center),
            "state": detected
        }


        # OpenCV로 이미지 처리 또는 표시
        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)  # 짧은 대기 (이 없으면 창이 안 뜰 수 있음)


        
    def get_line(self,mask):

        self.mask = mask
        h, w = self.mask.shape[:2]

        # 컨투어에서 가장 큰 것 선택
        contours, _ = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        cnt = max(contours, key=cv2.contourArea)

        if len(cnt) < 2:
            return None  # fitLine requires at least 2 points

        [vx, vy, x, y] = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)

        if vy == 0:
            return None  # avoid division by zero

        # y = h-1 (bottom) 기준 x좌표
        y_bottom = h - 1
        x_bottom = int(((y_bottom - y) * vx / vy) + x)

        # y = 0.75h (middle) 기준 x좌표
        y_middle = int(h * 0.75)
        x_middle = int(((y_middle - y) * vx / vy) + x)

        return (x_bottom, y_bottom, x_middle, y_middle)

        

    def detect_lanes(self,cv_image):

        self.cv_image = cv_image
        # 색상 범위로 마스킹
        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        # 노란색 범위
        lower_yellow = np.array([15, 60, 100])
        upper_yellow = np.array([40, 255, 255])
        # 흰색 범위
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 40, 255])

        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w = self.cv_image.shape[:2]
        # ROI (하단 부분만)
        mask_roi = np.zeros_like(yellow_mask)
        mask_roi[int(h*0.5):,:] = 1
        yellow_mask = yellow_mask * mask_roi
        white_mask = white_mask * mask_roi

        yellow_line = self.get_line(yellow_mask)
        white_line = self.get_line(white_mask)

        return yellow_line, white_line 

def main(args=None):
    rclpy.init(args=args)
    node = LaneTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()