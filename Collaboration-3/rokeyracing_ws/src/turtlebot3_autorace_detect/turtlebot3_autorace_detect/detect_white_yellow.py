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
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, Int32
import cv2
import numpy as np

from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult


def detect_white_line(cv_image):
    try:
        if cv_image is None or cv_image.size == 0:
            print("입력 이미지가 유효하지 않습니다")
            return None

        #print("error1")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 40, 255])


        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        #print("error2")

   
        kernel = np.ones((5, 5), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        #print("error3")
        h, w = cv_image.shape[:2]
        mask_roi = np.zeros_like(white_mask)
        mask_roi[int(h * 0.7):, :] = 1  # 하단 70%만 사용
        white_mask = white_mask * mask_roi
        #print("error4")
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
       
        if not contours:
            print("error5")
            return None

        cnt = max(contours, key=cv2.contourArea)
        
        #print("cnt:",cnt)

        if len(cnt) < 5:
            return None

        [vx, vy, x, y] = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
        if abs(vy) < 0.001:
            print("error6")
            return None

        y_bottom = h - 1
        x_bottom = int(((y_bottom - y) * vx / vy) + x)
        y_middle = int(h * 0.25)
        x_middle = int(((y_middle - y) * vx / vy) + x)

        if not (0 <= x_bottom < w and 0 <= x_middle < w):
            print("error7")
            return None

        #print("error8")
        return (x_bottom, y_bottom, x_middle, y_middle)

    except Exception as e:
        print(f"흰색 차선 감지 오류: {e}")
        return None

def detect_yellow_line(cv_image):
    try:
        if cv_image is None or cv_image.size == 0:
            print("입력 이미지가 유효하지 않습니다")
            return None

        #print("error1")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)


        lower_yellow = np.array([15, 60, 100])
        upper_yellow = np.array([40, 255, 255])


        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        


        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        #print("error3")
        h, w = cv_image.shape[:2]
        mask_roi = np.zeros_like(yellow_mask)
        mask_roi[int(h * 0.7):, :] = 1 # 하단 70%만 사용
        white_mask = yellow_mask * mask_roi
        #print("error4")
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
       
        if not contours:
            print("error5")
            return None

        cnt = max(contours, key=cv2.contourArea)
        
        #print("cnt:",cnt)

        if len(cnt) < 5:
            return None

        [vx, vy, x, y] = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
        if abs(vy) < 0.001:
            print("error6")
            return None

        y_bottom = h - 1
        x_bottom = int(((y_bottom - y) * vx / vy) + x)
        y_middle = int(h * 0.25)
        x_middle = int(((y_middle - y) * vx / vy) + x)

        if not (0 <= x_bottom < w and 0 <= x_middle < w):
            print("error7")
            return None

        #print("error8")
        return (x_bottom, y_bottom, x_middle, y_middle)

    except Exception as e:
        print(f"흰색 차선 감지 오류: {e}")
        return None

class WhiteLineTracker(Node):
    def __init__(self):
        super().__init__('white_line_tracker')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_projected',
            self.image_callback,
            10)

        self.cvBridge = CvBridge()
        self.sub_image_type = 'raw'

        self.lane_pub = self.create_publisher(Float32, '/detect/lane', 10)
        self.lane_state_pub = self.create_publisher(Int32, '/detect/lane_state', 10)

        self.pub_image_lane = self.create_publisher(
                Image, '/detect/white_image_output', 1
                )

        self.get_logger().info("흰색 차선 추적 노드가 시작되었습니다.")

    def image_callback(self, msg):
        try:
            #np_arr = np.frombuffer(msg.data, np.uint8)
            #img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            self.sub_image_type = 'raw'

            if self.sub_image_type == 'compressed':
                np_arr = np.frombuffer(msg.data, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            elif self.sub_image_type == 'raw':
                img = self.cvBridge.imgmsg_to_cv2(msg, 'bgr8')


            if img is not None:
                height, width = img.shape[:2]
                #print(f"Image shape: height={height}, width={width}")
            else:
                print("이미지를 디코딩할 수 없습니다.")


            h, w = img.shape[:2]
            white_line = detect_white_line(img)
            center_x = w // 2

            if white_line:
                x_bottom = white_line[0]
                desired_center = float(x_bottom)
                state = 2
            else:
                desired_center = float(center_x)
                state = 0

            self.lane_pub.publish(Float32(data=desired_center))
            self.lane_state_pub.publish(Int32(data=state))


            yellow_line = detect_yellow_line(img)

            x_middle = white_line[0]
            y_middle = white_line[1]
            x_bottom = white_line[2]
            y_bottom = white_line[3]
            cv2.rectangle(img, (x_middle, y_middle), (x_bottom, y_bottom), (255, 255,255), 2)

            x_middle = yellow_line[0]
            y_middle = yellow_line[1]
            x_bottom = yellow_line[2]
            y_bottom = yellow_line[3]
            cv2.rectangle(img, (x_middle, y_middle), (x_bottom, y_bottom), (255, 255, 0), 2)


            self.pub_image_lane.publish(self.cvBridge.cv2_to_imgmsg(img, 'bgr8'))

            # OpenCV로 이미지 처리 또는 표시
            cv2.imshow("Camera View", img)
            cv2.waitKey(1)  # 짧은 대기 (이 없으면 창이 안 뜰 수 있음)

        except Exception as e:
            self.get_logger().error(f"이미지 처리 중 오류 발생: {e}")


def main():
    rclpy.init()
    node = WhiteLineTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
