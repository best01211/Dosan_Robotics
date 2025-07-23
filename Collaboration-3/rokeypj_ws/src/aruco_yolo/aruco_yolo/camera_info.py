import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        #self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish)

        # Set dummy camera parameters
        self.camera_info = CameraInfo()
        self.camera_info.width = 320
        self.camera_info.height = 240
        self.camera_info.k = [1.0, 0.0, 320.0,
                              0.0, 1.0, 240.0,
                              0.0, 0.0, 1.0]  # dummy intrinsic matrix
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info.r = [1.0, 0.0, 0.0,
                              0.0, 1.0, 0.0,
                              0.0, 0.0, 1.0]
        self.camera_info.p = [1.0, 0.0, 320.0, 0.0,
                              0.0, 1.0, 240.0, 0.0,
                              0.0, 0.0, 1.0, 0.0]
        self.camera_info.distortion_model = 'plumb_bob'

    def publish(self):
        now = self.get_clock().now().to_msg()

        # Dummy image
        #img = np.ones((480, 640, 3), dtype=np.uint8) * 255
        #image_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        #image_msg.header.stamp = now
        #image_msg.header.frame_id = 'camera_frame'

        self.camera_info.header.stamp = now
        self.camera_info.header.frame_id = 'camera_frame'

        #self.image_pub.publish(image_msg)
        self.info_pub.publish(self.camera_info)

        #self.get_logger().info('Published image + camera_info')


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

