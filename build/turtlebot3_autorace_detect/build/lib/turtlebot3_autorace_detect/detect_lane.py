# 노란선 offset만 퍼블리시하는 최소 수정 detect_lane.py 예시

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64

import cv2
import numpy as np


class DetectLane(Node):
    def __init__(self):
        super().__init__('detect_lane')
        self.bridge = CvBridge()

        self.sub_image = self.create_subscription(
            Image,
            '/camera/image_projected',
            self.image_callback,
            1
        )

        self.pub_lane = self.create_publisher(
            Float64,
            '/detect/lane',
            1
        )

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 노란색 차선 HSV 범위
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 영역이 너무 작으면 노란선 없다고 판단
        moments = cv2.moments(mask_yellow)
        if moments['m00'] > 5000:
            cx = int(moments['m10'] / moments['m00'])
            self.pub_lane.publish(Float64(data=float(cx)))
        else:
            # 노란선 감지 실패 시 -1 퍼블리시
            self.pub_lane.publish(Float64(data=-1.0))


def main(args=None):
    rclpy.init(args=args)
    node = DetectLane()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
