#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2


class YellowLineDetect(Node):
    def __init__(self):
        super().__init__('yellow_line_node')
        self.bridge = CvBridge()

        # Publisher
        self.publisher = self.create_publisher(
            CompressedImage,
            '/yellow/compressed',
            10
        )

        # Subscriber
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.img_cb,
            10
        )
        self.subscription  # prevent unused variable warning

    def detect_color(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Yellow range in HSV
        yellow_lower = np.array([0, 80, 0])
        yellow_upper = np.array([40, 255, 255])

        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        yellow_color = cv2.bitwise_and(img, img, mask=yellow_mask)

        return yellow_color

    def img_warp(self, img):
        img_x, img_y = img.shape[1], img.shape[0]

        src_center_offset = [200, 315]
        src = np.float32([
            [0, 479],
            [src_center_offset[0], src_center_offset[1]],
            [640 - src_center_offset[0], src_center_offset[1]],
            [639, 479],
        ])
        dst_offset = [round(img_x * 0.125), 0]
        dst = np.float32([
            [dst_offset[0], img_y],
            [dst_offset[0], 0],
            [img_x - dst_offset[0], 0],
            [img_x - dst_offset[0], img_y],
        ])
        matrix = cv2.getPerspectiveTransform(src, dst)
        warp_img = cv2.warpPerspective(img, matrix, (img_x, img_y))

        return warp_img

    def img_cb(self, data):
        img = self.bridge.compressed_imgmsg_to_cv2(data)
        warp_img = self.img_warp(img)
        yellow_line = self.detect_color(warp_img)

        yellow_line_msg = self.bridge.cv2_to_compressed_imgmsg(yellow_line)
        self.publisher.publish(yellow_line_msg)

        # Display (optional)
        cv2.imshow("img", img)
        cv2.imshow("yellow_line", yellow_line)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YellowLineDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
