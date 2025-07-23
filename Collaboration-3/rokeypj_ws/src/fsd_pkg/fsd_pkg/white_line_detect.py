#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2


class WhiteLineDetect(Node):
    def __init__(self):
        super().__init__('white_line_node')
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            CompressedImage,
            '/white/compressed',
            10
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.img_cb,
            10
        )

        # Optional: prevent unused variable warning
        self.subscription  # prevent unused warning

    def detect_color(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        white_lower = np.array([0, 0, 200])
        white_upper = np.array([179, 100, 255])
        white_mask = cv2.inRange(hsv, white_lower, white_upper)
        white_color = cv2.bitwise_and(img, img, mask=white_mask)
        return white_color

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
        white_color = self.detect_color(warp_img)
        white_line_msg = self.bridge.cv2_to_compressed_imgmsg(white_color)
        self.publisher.publish(white_line_msg)

        # Display (optional, for debugging)
        cv2.imshow("White Line", white_color)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = WhiteLineDetect()
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
