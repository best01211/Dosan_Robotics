#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectLaneOffset(Node):
    def __init__(self):
        super().__init__('detect_lane_offset')
        self.sub_image = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pub_lane = self.create_publisher(Float64, '/detect/lane', 10)

        self.bridge = CvBridge()
        self.left_fitx = None
        self.left_fit = None

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 노란색 차선 HSV 범위 (필요시 조정)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        result = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # 좌측 차선만 추출 (왼쪽 절반)
        h, w, _ = result.shape
        left_mask = np.zeros_like(mask)
        left_mask[:, :w//2] = mask[:, :w//2]

        nonzero = left_mask.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        if len(nonzeroy) > 0:
            fit = np.polyfit(nonzeroy, nonzerox, 2)
            ploty = np.linspace(0, h - 1, h)
            fitx = fit[0] * ploty**2 + fit[1] * ploty + fit[2]
            self.left_fitx = fitx
            self.left_fit = fit

            # 중심선 = 좌측 차선 기준 + offset (픽셀)
            offset = 280  # pixel 기준
            centerx = fitx + offset

            msg = Float64()
            msg.data = float(centerx[350])  # 중앙 근처의 x값 퍼블리시
            self.pub_lane.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectLaneOffset()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
