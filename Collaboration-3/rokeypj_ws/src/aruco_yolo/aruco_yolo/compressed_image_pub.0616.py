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
#from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        # Declare parameter with default integer value
        self.video_port = self.declare_parameter('video_port', 0).get_parameter_value().integer_value

        self.get_logger().info(f'Received video_port parameter int): {self.video_port}')

        # 이미지 퍼블리셔 생성
        self.publisher_ = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
        
        # 이미지 퍼블리셔 생성
        self.full_publisher_ = self.create_publisher(CompressedImage, 'image_full_raw/compressed', 10)
        

        # OpenCV와 ROS 간 변환을 위한 CvBridge 초기화, <=Compressed Image : No need to O numpy => ros2 image
 #       self.bridge = CvBridge()

        # 주기적인 이미지 전송을 위한 타이머 설정 (주기: 1초)
        self.timer = self.create_timer(0.2, self.publish_image)

        # OpenCV 비디오 캡처 객체 생성 (카메라 0번 장치 사용)
        #self.cap = cv2.VideoCapture('/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0')
        self.cap = cv2.VideoCapture(self.video_port)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FPS, 25)
        #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        self.get_logger().info('*'*50)
        self.get_logger().info(f'{self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}, {self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}')
        self.get_logger().info('*'*50)

        # 카메라에서 한 프레임 읽기
        ret, frame = self.cap.read()



        # Get raw image size (before compression)
        height, width = frame.shape[:2]
        self.get_logger().info(f'Raw frame size: width={width}, height={height}')
                               

    def publish_image(self):
        # 카메라에서 한 프레임 읽기
        ret, frame = self.cap.read()

        if ret:


            # Draw a 300x300 rectangle in the center of ROI image
            height, width, _ = frame.shape
            rect_w, rect_h = 300, 300
            x1 = (width - rect_w) // 2
            y1 = (height - rect_h) // 2
            x2 = x1 + rect_w
            y2 = y1 + rect_h

            top_left = (x1, y1)
            bottom_right = (x2, y2)
            color = (200, 100,100)  # Green
            thickness = 3        # Outline only
            cv2.rectangle(frame, top_left, bottom_right, color, thickness)


            # Draw ROI rectangle: from (50, 0) to (1250, 720)
            top_left = (50,720-120)
            bottom_right = (50,1280-280)
            color = (0, 255, 0)  # Green
            thickness = 2        # Outline only

            cv2.rectangle(frame, top_left, bottom_right, color, thickness)

             # OpenCV 이미지 (BGR)을 JPEG로 압축
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]  # 90은 압축 품질
            _, compressed_image = cv2.imencode('.jpg', frame, encode_param)

            # 압축된 이미지를 CompressedImage 메시지로 변환
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
            msg.header.frame_id = "camera"  # 프레임 ID 설정
            msg.format = "jpeg"  # 압축 형식 설정
            msg.data = compressed_image.tobytes()  # 압축된 이미지 데이터

            # CompressedImage 퍼블리시
            self.full_publisher_.publish(msg)


            #ROI
            # Crop 50px from each side
            frame = frame[50:720-50, 50:1280-50]  # frame[50:670, 50:1230]


             # OpenCV 이미지 (BGR)을 JPEG로 압축
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]  # 90은 압축 품질
            _, compressed_image = cv2.imencode('.jpg', frame, encode_param)

            # 압축된 이미지를 CompressedImage 메시지로 변환
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
            msg.header.frame_id = "camera"  # 프레임 ID 설정
            msg.format = "jpeg"  # 압축 형식 설정
            msg.data = compressed_image.tobytes()  # 압축된 이미지 데이터

            # CompressedImage 퍼블리시
            self.publisher_.publish(msg)

            
            self.get_logger().info('...')
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

