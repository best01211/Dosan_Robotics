import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge
import numpy as np

class LaneStraightDetector(Node):
    def __init__(self):
        super().__init__('lane_straight_detector')

        self.sub_image = self.create_subscription(
            Image,
            '/detect/image_yellow_lane_marker',
            self.image_callback,
            10
        )
        self.pub_straight = self.create_publisher(
            Bool,
            '/lane/is_straight',
            10
        )

        self.bridge = CvBridge()

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')  # 이진 이미지
        lines = cv2.HoughLines(img, 1, np.pi/180, threshold=100)

        is_straight = Bool()
        if lines is None or len(lines) < 2:
            is_straight.data = False
        else:
            angles = [theta for [[_, theta]] in lines]
            angle_std = np.std(angles)

            is_straight.data = angle_std < 0.1  # 기울기 변화가 작을 때 직선 판단

        self.pub_straight.publish(is_straight)


def main(args=None):
    rclpy.init(args=args)
    node = LaneStraightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
