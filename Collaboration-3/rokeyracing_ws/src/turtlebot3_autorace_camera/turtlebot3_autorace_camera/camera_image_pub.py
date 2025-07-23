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
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image, CameraInfo


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        self.bridge = CvBridge()
        
        # Declare parameter with default integer value
        self.video_port = self.declare_parameter('video_port', 0).get_parameter_value().integer_value

        self.image_width = self.declare_parameter('image_width', 320).get_parameter_value().integer_value
        self.image_height = self.declare_parameter('image_height', 240).get_parameter_value().integer_value

        self.get_logger().info(f'Get...... video_port parameter : {self.video_port}')

        # image_raw 이미지 퍼블리셔 생성
        self.image_raw_publisher_ = self.create_publisher(Image, 'image_raw', 10)

        #for intrinsic calibration
        self.image_publisher_ = self.create_publisher(Image, 'image', 10)

        # jped compressed 이미지 퍼블리셔 생성
        self.compressed_publisher_ = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
        
        # 주기적인 이미지 전송을 위한 타이머 설정 (주기: 1초)
        self.timer = self.create_timer(0.02, self.publish_image)

        # OpenCV 비디오 캡처 객체 생성 (카메라 0번 장치 사용)
        #self.cap = cv2.VideoCapture('/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0')
        self.cap = cv2.VideoCapture(self.video_port)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)

        # 카메라에서 한 프레임 읽기
        ret, frame = self.cap.read()

        # Get raw image size (before compression)
        height, width = frame.shape[:2]
        self.get_logger().info(f'Raw frame size: width={width}, height={height}')
                               
        if ret:

            self.get_logger().info(f"Frame type:, {type(frame)}")
            self.get_logger().info(f"Shape (Height, Width, Channels):, {frame.shape}")
            self.get_logger().info(f"Height:, {frame.shape[0]}")
            self.get_logger().info(f"Width:, {frame.shape[1]}")
            self.get_logger().info(f"Channels:, {frame.shape[2]}")  # Usually 3 for BGR
            self.get_logger().info(f"Data type:, {frame.dtype}")
        

            #대부분의 카메라 드라이버는 /camera_info 토픽으로 캘리브레이션 데이터를 발행합니다:
            self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
            self.bridge = CvBridge()
            #self.timer = self.create_timer(0.1, self.publish)

            # Set dummy camera parameters
            self.camera_info = CameraInfo()
            self.camera_info.width = 320
            self.camera_info.height = 240
            self.camera_info.k = [525.0, 0.0, 320.0,
                                0.0, 525.0, 240.0,
                                0.0, 0.0, 1.0]  # dummy intrinsic matrix
            self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.camera_info.r = [1.0, 0.0, 0.0,
                                0.0, 1.0, 0.0,
                                0.0, 0.0, 1.0]
            self.camera_info.p = [525.0, 0.0, 320.0, 0.0,
                                0.0, 525.0, 240.0, 0.0,
                                0.0, 0.0, 525.0, 0.0]
            self.camera_info.distortion_model = 'plumb_bob'


    def publish_image(self):
        # 카메라에서 한 프레임 읽기
        ret, frame = self.cap.read()

        frame = cv2.convertScaleAbs(frame, alpha=1.0, beta=-20)
        # alpha: contrast, beta: brightness shift (negative = darker)
        
        if ret:

            ########## 1. Raw Image, Convert OpenCV image (numpy) to ROS Image message
           ############## camera/image_raw topic publish            
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            now = self.get_clock().now().to_msg()
            msg.header.stamp = now
            msg.header.frame_id = 'camera_frame'

            self.image_raw_publisher_.publish(msg)

            self.image_publisher_.publish(msg) #for intrinsic calibration
            #self.get_logger().info('Published image to /image_raw')

            #### camera_info topic publish
            self.camera_info.header.stamp = now
            self.camera_info.header.frame_id = 'camera_frame'
            self.info_pub.publish(self.camera_info)


            # OpenCV 이미지 (BGR)을 JPEG로 압축
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]  # 90은 압축 품질
            _, compressed_image = cv2.imencode('.jpg', frame, encode_param)
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
            msg.header.frame_id = "camera"  # 프레임 ID 설정
            msg.format = "jpeg"  # 압축 형식 설정
            msg.data = compressed_image.tobytes()  # 압축된 이미지 데이터
            self.compressed_publisher_.publish(msg)

        else:
            self.get_logger().error('Video Pen Error!...')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    
    # ROS 2 노드 실행
    rclpy.spin(image_publisher)

    # 종료 시 리소스 해제
    image_publisher.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

