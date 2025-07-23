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


import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import random
import math
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from cv_bridge import CvBridge
import os

import matplotlib.pyplot as plt
from collections import deque


frame_height = 480
frame_width = 640

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        
        self.frame_count = 0
        self.reliability_yellow_line = 0
        self.reliability_white_line = 0

        self.stack_lpos=deque(maxlen=10)
        self.stack_rpos=deque(maxlen=10)

        self.calibration_mode = self.declare_parameter('calibration_mode', False).get_parameter_value().bool_value
        frame_width = self.width = self.declare_parameter('video_width', 640).get_parameter_value().integer_value
        frame_height = self.height = self.declare_parameter('video_height', 480).get_parameter_value().integer_value
        self.desired_center = 0


        self.offset = 200 #x.top rujin 420=>200

        self.gap = 40
        
        self.get_logger().info(f"Width:, {self.width}")
        self.get_logger().info(f"Height:, {self.height}")
                
        self.bridge = CvBridge()
        
        # Initialize arrow image for steering visualization (optional)
        self.arrow_pic = None
        #self.load_arrow_image()
        
        # Publishers
        self.steer_angle_pub = self.create_publisher(Float32, 'steer_angle', 10)
        self.processed_image_pub = self.create_publisher(Image, 'processed_image', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.pub_lane = self.create_publisher(Float64, '/detect/lane', 10)

        self.pub_yellow_line_reliability = self.create_publisher(
            UInt8, '/detect/yellow_line_reliability', 1
            )

        self.pub_white_line_reliability = self.create_publisher(
            UInt8, '/detect/white_line_reliability', 1
            )

        self.pub_lane_state = self.create_publisher(UInt8, '/detect/lane_state', 1)


        # Timer for processing (if using webcam instead of topic)
        # self.timer = self.create_timer(0.03, self.timer_callback)
        # self.cap = cv2.VideoCapture(0)
        
        self.get_logger().info('Lane Detection Node initialized')

    #YUV 색공간을 사용하여 **명도(Y)**만 히스토그램 평활화(equalizeHist)를 적용
    def auto_exposure(self,frame):
        yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        yuv[:, :, 0] = cv2.equalizeHist(yuv[:, :, 0])
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)
    #LAB 색공간에서 색상 보정을 수행합니다. 특히 **a 채널(녹-적)과 b 채널(청-황)**의 평균값을 기준으로 자동 보정을 수행
    def white_balance(self,frame):
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).astype(np.float32)

        avg_a = np.average(lab[:, :, 1])
        avg_b = np.average(lab[:, :, 2])

        lab[:, :, 1] -= ((avg_a - 128) * (lab[:, :, 0] / 255.0) * 1.1)
        lab[:, :, 2] -= ((avg_b - 128) * (lab[:, :, 0] / 255.0) * 1.1)

        lab = np.clip(lab, 0, 255).astype(np.uint8)
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    
    def load_arrow_image(self): 
        """Load steering arrow image if available"""
        try:
            # Try to load the steering arrow image
            dir_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
            dir_path = os.path.join(dir_path, 'image')

            if os.path.exists(dir_path+"/steer_arrow.png"):
                #self.arrow_pic = cv2.imread(dir_path + '/stop.png', 0)  # trainImage1
                self.arrow_pic = cv2.imread(dir_path + '/steer_arrow.png', cv2.IMREAD_COLOR)
                self.get_logger().info('Steering arrow image loaded successfully')
            else:
                self.get_logger().warn('steer_arrow.png not found, steering visualization disabled')
        except Exception as e:
            self.get_logger().warn(f'Failed to load steering arrow image: {e}')

    #왼쪽 또는 오른쪽 차선을 각각 탐지
    def sliding_windown(self, img_w, left_or_right):
        histogram = np.sum(img_w[int(img_w.shape[0] / 2):, :], axis=0)

        out_img = np.dstack((img_w, img_w, img_w)) * 255

        midpoint = int(histogram.shape[0] / 2)

        if left_or_right == 'left':
            lane_base = np.argmax(histogram[:midpoint])
        elif left_or_right == 'right':
            lane_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 20

        window_height = int(img_w.shape[0] / nwindows)

        nonzero = img_w.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        x_current = lane_base

        margin = 50
        minpix = 50

        lane_inds = []

        for window in range(nwindows):
            win_y_low = img_w.shape[0] - (window + 1) * window_height
            win_y_high = img_w.shape[0] - window * window_height
            win_x_low = x_current - margin
            win_x_high = x_current + margin

            cv2.rectangle(
                out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 2)

            good_lane_inds = (
                (nonzeroy >= win_y_low) &
                (nonzeroy < win_y_high) &
                (nonzerox >= win_x_low) &
                (nonzerox < win_x_high)
                ).nonzero()[0]

            lane_inds.append(good_lane_inds)

            if len(good_lane_inds) > minpix:
                x_current = int(np.mean(nonzerox[good_lane_inds]))

        lane_inds = np.concatenate(lane_inds)

        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        try:
            lane_fit = np.polyfit(y, x, 2)
            self.lane_fit_bef = lane_fit
        except Exception:
            lane_fit = self.lane_fit_bef

        ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]

        return lane_fitx, lane_fit
    
    def image_callback(self, msg):
        """Process incoming image messages"""
        try:


            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            cv2.line(cv_image, (0, self.offset), (cv_image.shape[1], self.offset), (0, 0, 255), 2)


            cv_image = cv2.convertScaleAbs(cv_image, alpha=1.0, beta=-30) #<= Brightness -20
            
            lpos, rpos = self.detect_lane(cv_image)
            
            # Calculate steering angle
            center = (lpos + rpos) / 2
            angle = self.width - center  # 320 is half of 640 (image center)
            steer_angle = angle * 0.4
            
            # Publish steering angle
            steer_msg = Float32()
            steer_msg.data = float(steer_angle)
            self.steer_angle_pub.publish(steer_msg)
            
            # Draw steering visualization if arrow image is available
            #if self.arrow_pic is not None:
            #    cv_image = self.draw_steer(cv_image, steer_angle)
            
            # Publish processed image
            processed_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.processed_image_pub.publish(processed_msg)
            
            # Display image (optional - comment out for headless operation)
            cv2.imshow('Lane Detection', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def publish_center(self, x_value):
        pass
        #msg = Float64()
        #msg.data = float(x_value)
        #self.pub_lane.publish(msg)
        #self.get_logger().info(f"/detect/lane {msg.data:.2f}")

    """
    def timer_callback(self):
        if hasattr(self, 'cap'):
            ret, image = self.cap.read()
            if ret:
                lpos, rpos = self.detect_lane(image)
                center = (lpos + rpos) / 2
                angle = 160 - center
                steer_angle = angle * 0.4
                
                # Publish steering angle
                steer_msg = Float32()
                steer_msg.data = float(steer_angle)
                self.steer_angle_pub.publish(steer_msg)
                
                #if self.arrow_pic is not None:
                #    image = self.draw_steer(image, steer_angle)
                
                # 중심 퍼블리시
                #if center_line is not None:
                self.publish_center(center) #middle of center line's height/2
                #lane_node.publish_center(center_line[int(h*0.7)]) #


                cv2.imshow('Lane Detection', image)
                cv2.waitKey(1)
        """

    def draw_lines(self, img, lines):
        """Draw detected lines on image, only bellow self.offset"""
        for line in lines:
            x1, y1, x2, y2 = line[0]
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            
            img = cv2.line(img, (x1, y1 + self.offset), (x2, y2 + self.offset), color, 2)
            
            # rujin 
            #img = cv2.line(img, (x1, y1), (x2, y2), color, 2)

        return img
    ################## IMPORTENT #################
    def draw_rectangle(self, img, lpos, rpos, offset=0):
        """Draw Left, Right(Sky) and Center(White) lane position, and Image Center(Red) indicators on image"""
        center = int((lpos + rpos) / 2)
        
        # Left position indicator (green)
        cv2.rectangle(img, (lpos - 5, 15 + offset), (lpos + 5, 25 + offset), (255, 255, 0), 3)
       
        # Right position indicator (green)
        cv2.rectangle(img, (rpos - 5, 15 + offset), (rpos + 5, 25 + offset), (255, 255, 0), 3)
       
        # Center of lanes (green)
        cv2.rectangle(img, (center - 5, 15 + offset), (center + 5, 25 + offset), (255, 255, 255), 5)
       
        # Image center reference (red)
        cv2.rectangle(img, (315, 15 + offset), (325, 25 + offset), (0, 0, 255), 5)
        return img, center

    def divide_left_right(self, lines):
        """Separate lines into left and right lane lines"""
        low_slope_threshold = 0
        high_slope_threshold = 10

        slopes = []
        new_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 - x1 == 0:
                slope = 0
            else:
                slope = float(y2 - y1) / float(x2 - x1)
            
            if (abs(slope) > low_slope_threshold) and (abs(slope) < high_slope_threshold):
                slopes.append(slope)
                new_lines.append(line[0])

        left_lines = []
        right_lines = []

        for j in range(len(slopes)):
            line = new_lines[j]
            slope = slopes[j]
            x1, y1, x2, y2 = line

            if (slope < 0) and (x2 < self.width/2 - 90):  # Left lines
                left_lines.append([line.tolist()])
            elif (slope > 0) and (x1 > self.width/2 + 90):  # Right lines
                right_lines.append([line.tolist()])

        return left_lines, right_lines

    def get_line_params(self, lines):
        """Calculate average slope and intercept of lines"""
        x_sum = 0.0
        y_sum = 0.0
        m_sum = 0.0
        count = 0

        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 - x1 == 0:
                continue  # 수직선은 무시 (기울기 무한대)
            m = float(y2 - y1) / float(x2 - x1)
            m_sum += m
            x_sum += x1 + x2
            y_sum += y1 + y2
            count += 1

        if count == 0:
            return 0, 0  # 유효한 선 없음

        x_avg = x_sum / (count * 2)
        y_avg = y_sum / (count * 2)
        m_avg = m_sum / count
        b = y_avg - m_avg * x_avg

        return m_avg, b


    def get_line_pos(self, img, lines, left=False, right=False):
        """Get lane position from detected lines"""
        m, b = self.get_line_params(lines)

        if m == 0 and b == 0:
            # No valid line detected
            pos = 0 if left else self.width
        else:
            y = self.gap / 2
            pos = (y - b) / m

            # Draw the line for visualization
            b += self.offset
            try:
                x1 = (self.height - b) / float(m)
                x2 = ((self.height / 2) - b) / float(m)
                cv2.line(img, (int(x1), self.height), (int(x2), self.height // 2), (255, 0, 0), 3)
            except ZeroDivisionError:
                pos = 0 if left else self.width

        return img, int(pos)



    # Bird's Eye View 변환
    def bird_eye_view(self,image, top_x, top_y, bottom_x, bottom_y):
        center_x = image.shape[1] // 2
        top_y_coord = 80 - top_y
        bottom_y_coord = 120 + bottom_y

        src = np.array([
            [center_x - top_x, top_y_coord],
            [center_x + top_x, top_y_coord],
            [center_x + bottom_x, bottom_y_coord],
            [center_x - bottom_x, bottom_y_coord]
        ], dtype=np.float32)
        dst = np.array([[200,0],[800,0],[800,600],[200,600]], dtype=np.float32)


        H, _ = cv2.findHomography(src, dst)
        warped = cv2.warpPerspective(image, H, (1000,600))
        return warped

    def detect_lane(self, frame):

        tx = cv2.getTrackbarPos('top_x','Control Panel')
        ty = cv2.getTrackbarPos('top_y','Control Panel')
        bx = cv2.getTrackbarPos('bottom_x','Control Panel')
        by = cv2.getTrackbarPos('bottom_y','Control Panel')

        # 좌표 정렬 (x1, y1) = 좌상단 / (x2, y2) = 우하단
        x1 = min(tx, bx)
        y1 = min(ty, by)
        x2 = max(tx, bx)
        y2 = max(ty, by)

        # 사각형 그리기
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        bird = self.bird_eye_view(frame, tx, ty, bx, by)
        hsv = cv2.cvtColor(bird, cv2.COLOR_BGR2HSV)
        # S, V 히스토그램 평탄화
        h, s, v = cv2.split(hsv)
        s_eq = cv2.equalizeHist(s)
        v_eq = cv2.equalizeHist(v)
        
        hsv_eq = cv2.merge([h, s_eq, v_eq])
        bird_eq = cv2.cvtColor(hsv_eq, cv2.COLOR_HSV2BGR)
        # ++++++++++++++++++++++++++++ Equalized Birdview
        #cv2.imshow('Equalized Birdview', bird_eq)


        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        kernel_size = 5
        blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

        # Apply Canny edge detection
        low_threshold = 60
        high_threshold = 70
        edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

        # GOOD: Define ROI and apply Hough transform
        #roi = edge_img[self.offset : self.offset + self.gap, 0 : self.width]
        start_point = (0, self.offset)
        end_point = (self.width, self.offset + self.gap)
        cv2.rectangle(frame, start_point, end_point, (0, 255, 255), 2)
        
        roi = edge_img

        all_lines = cv2.HoughLinesP(roi, 1, math.pi/180, 30, 30, 10)

        if all_lines is None:
            return 0, self.width

        # Separate left and right lines
        left_lines, right_lines = self.divide_left_right(all_lines)

        self.reliability_yellow_line = len(left_lines)  # int 값을 data 필드에 설정
        msg_yellow_line_reliability = UInt8()
        msg_yellow_line_reliability.data = self.reliability_yellow_line
        self.pub_yellow_line_reliability.publish(msg_yellow_line_reliability)


        self.reliability_white_line = len(right_lines)  # int 값을 data 필드에 설정
        pub_white_line_reliability = UInt8()
        pub_white_line_reliability.data = self.reliability_yellow_line
        self.pub_white_line_reliability.publish(msg_yellow_line_reliability)

        # Get lane positions
        frame, lpos = self.get_line_pos(frame, left_lines, left=True)
        frame, rpos = self.get_line_pos(frame, right_lines, right=True)

        # Draw detected lines
        frame = self.draw_lines(frame, left_lines)
        frame = self.draw_lines(frame, right_lines)
        
        # Draw center reference line
        #cv2.line(frame, (230, 235), (410, 235), (255, 255, 255), 3)



        # both lane -> 2, left lane -> 1, right lane -> 3, none -> 0
        lane_state = UInt8()

        if len(self.stack_lpos) > 0:
            average_lpos = int(sum(self.stack_lpos) / len(self.stack_lpos))
        else:
            average_lpos = 0  # 또는 None

        if len(self.stack_rpos) > 0:
            average_rpos = int(sum(self.stack_rpos) / len(self.stack_rpos))
        else:
            average_rpos = 0  # 또는 None


        if self.reliability_yellow_line > 0 and self.reliability_white_line == 0:
            lane_state.data = 1
            # Draw position indicators
            frame,center = self.draw_rectangle(frame, average_lpos, average_rpos, offset=self.offset)            
            line_center = (lpos + rpos)/2
            self.desired_center = self.width/2 - line_center

            return average_lpos,average_rpos
        elif self.reliability_yellow_line > 0 and self.reliability_white_line > 0:
            self.stack_lpos.append(lpos)
            self.stack_rpos.append(rpos)
            # Draw position indicators
            #frame,line_center = self.draw_rectangle(frame, average_lpos, average_rpos, offset=self.offset-5)
            frame,center = self.draw_rectangle(frame, lpos, rpos, offset=self.offset) 

            line_center = (lpos + rpos)/2
              
            if line_center > 0:
                line_center = 500 + line_center
            else:
                line_center = 500 - line_center

            self.desired_center = self.width/2 - line_center

            lane_state.data = 2
        elif self.reliability_yellow_line == 0 and self.reliability_white_line > 0:
            lane_state.data = 3
            # Draw position indicators
            frame,center = self.draw_rectangle(frame, average_lpos, average_rpos, offset=self.offset)

            line_center = (lpos + rpos)/2
            
            self.desired_center = self.width/2 - line_center            
            return average_lpos,average_rpos
        else:
            lane_state.data = 0
            return average_lpos,average_rpos

        # publishes lane center
        msg_desired_center = Float64()
        msg_desired_center.data = self.desired_center

        self.pub_lane.publish(msg_desired_center)

        self.pub_lane_state.publish(lane_state)
        #self.get_logger().info(f'({self.reliability_yellow_line},{self.reliability_white_line}),Lane state: {lane_state.data},desired_center:{self.desired_center}')
        self.get_logger().info(f'({lpos},{rpos}),Lane state: {lane_state.data},desired_center:{self.desired_center}')

        return lpos, rpos

    def draw_text(self,image,text,x,y):
        position = (x,y)  # x, y coordinates (bottom-left corner of the text)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        color = (0, 0, 255)  # Red in BGR
        thickness = 2
        cv2.putText(image, text, position, font, font_scale, color, thickness, cv2.LINE_AA)
        return image

    def draw_angles(self,data):
        x = range(len(data))

        plt.stem(x, data, basefmt=" ", use_line_collection=True)

        plt.title("Stick Graph of Float Values")
        plt.xlabel("Index")
        plt.ylabel("Value")
        plt.grid(True)

        plt.show()

    def draw_steer(self, image, steer_angle):
        """Draw steering wheel visualization"""
        if self.arrow_pic is None:
            return image
        else:
            #height, width = self.arrow_pic.shape[:2]
            #new_width = int(width * 0.5)
            #new_height = int(height * 0.5)

            #self.arrow_pic = cv2.resize(self.arrow_pic, (new_width, new_height))

            data = [3.5, 2.1, 4.8, 1.9, 3.3]
            #self.draw_angles(data)

        try:
            arrow_pic_copy = self.arrow_pic.copy()
            
            origin_height = arrow_pic_copy.shape[0]
            origin_width = arrow_pic_copy.shape[1]
            steer_wheel_center = origin_height * 0.74
            arrow_height = self.height // 2
            arrow_width = int((arrow_height * 462) / 728)

            

            # Rotate steering wheel based on angle
            matrix = cv2.getRotationMatrix2D(
                (origin_width/2, steer_wheel_center), 
                steer_angle * 2.5, 
                0.7
            )

            # Apply rotation and resize
            arrow_pic_copy = cv2.warpAffine(
                arrow_pic_copy, 
                matrix, 
                (origin_width + 60, origin_height)
            )
            arrow_pic_copy = cv2.resize(
                arrow_pic_copy, 
                (arrow_width, arrow_height), 
                interpolation=cv2.INTER_AREA
            )

            # Create mask and overlay
            #gray_arrow = cv2.cvtColor(arrow_pic_copy, cv2.COLOR_BGR2GRAY)
            #_, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

            # Calculate ROI bounds
            y_start = self.height - arrow_height
            y_end = self.height
            x_start = max(0, (self.width//2) - (arrow_width//2))
            x_end = min(self.width, (self.width//2) + (arrow_width//2))

            # Ensure dimensions match
            roi_height = y_end - y_start
            roi_width = x_end - x_start
            
            #cv2.rectangle(image, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)

            if (roi_height > 0 and roi_width > 0 and 
                arrow_height > 0 and arrow_width > 0):
                
                #arrow_pic_resized = cv2.resize(arrow_pic_copy, (roi_width, roi_height))
                #mask_resized = cv2.resize(mask, (roi_width, roi_height))
                
                #arrow_roi = image[y_start:y_end, x_start:x_end]
                #arrow_roi = cv2.add(arrow_pic_resized, arrow_roi, mask=mask_resized)

                #image[y_start:y_end, x_start:x_end] = arrow_roi

                image = self.draw_text(image,str(steer_angle),arrow_height,arrow_width)

        except Exception as e:
            self.get_logger().warn(f'Error drawing steering visualization: {e}')

        return image

    def destroy_node(self):
        """Clean up resources"""
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


# 트랙바 콜백
def nothing(x): pass

def create_trackbars():
    cv2.namedWindow('Control Panel', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Control Panel', 400, 400)
    # White thresholds
    cv2.createTrackbar('White H min', 'Control Panel', 46, 180, nothing)
    cv2.createTrackbar('White S min', 'Control Panel', 56, 255, nothing)
    cv2.createTrackbar('White V min', 'Control Panel', 154, 255, nothing)
    cv2.createTrackbar('White H max', 'Control Panel', 180, 180, nothing)
    cv2.createTrackbar('White S max', 'Control Panel', 250, 255, nothing)
    cv2.createTrackbar('White V max', 'Control Panel', 255, 255, nothing)
    # Yellow thresholds
    cv2.createTrackbar('Yellow H min', 'Control Panel', 11, 180, nothing)
    cv2.createTrackbar('Yellow S min', 'Control Panel', 21, 255, nothing)
    cv2.createTrackbar('Yellow V min', 'Control Panel', 35, 255, nothing)
    cv2.createTrackbar('Yellow H max', 'Control Panel', 60, 180, nothing)
    cv2.createTrackbar('Yellow S max', 'Control Panel', 255, 255, nothing)
    cv2.createTrackbar('Yellow V max', 'Control Panel', 255, 255, nothing)
    # ROI trackbars
    cv2.createTrackbar('top_x', 'Control Panel', 140, frame_width, nothing) #167
    cv2.createTrackbar('top_y', 'Control Panel', 20, frame_height, nothing)
    cv2.createTrackbar('bottom_x', 'Control Panel', 148, frame_width, nothing) #233
    cv2.createTrackbar('bottom_y', 'Control Panel', 147, frame_height, nothing)


def main(args=None):
    rclpy.init(args=args)
    
    node = LaneDetectionNode()
    
    create_trackbars()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
