#!/usr/bin/env python3
#
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

import cv2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import UInt8


class DetectLane(Node):

    def __init__(self):
        super().__init__('detect_lane')

        parameter_descriptor_hue = ParameterDescriptor(
            description='hue parameter range',
            integer_range=[IntegerRange(
                from_value=0,
                to_value=179,
                step=1)]
        )
        parameter_descriptor_saturation_lightness = ParameterDescriptor(
            description='saturation and lightness range',
            integer_range=[IntegerRange(
                from_value=0,
                to_value=255,
                step=1)]
        )
        self.declare_parameters(
            namespace='',
            parameters=[
                ('detect.lane.white.hue_l', 0,
                    parameter_descriptor_hue),
                ('detect.lane.white.hue_h', 179,
                    parameter_descriptor_hue),
                ('detect.lane.white.saturation_l', 0,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.white.saturation_h', 70,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.white.lightness_l', 105,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.white.lightness_h', 255,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.hue_l', 10,
                    parameter_descriptor_hue),
                ('detect.lane.yellow.hue_h', 127,
                    parameter_descriptor_hue),
                ('detect.lane.yellow.saturation_l', 70,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.saturation_h', 255,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.lightness_l', 95,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.lightness_h', 255,
                    parameter_descriptor_saturation_lightness),
                ('is_detection_calibration_mode', False)
            ]
        )

        self.hue_white_l = self.get_parameter(
            'detect.lane.white.hue_l').get_parameter_value().integer_value
        self.hue_white_h = self.get_parameter(
            'detect.lane.white.hue_h').get_parameter_value().integer_value
        self.saturation_white_l = self.get_parameter(
            'detect.lane.white.saturation_l').get_parameter_value().integer_value
        self.saturation_white_h = self.get_parameter(
            'detect.lane.white.saturation_h').get_parameter_value().integer_value
        self.lightness_white_l = self.get_parameter(
            'detect.lane.white.lightness_l').get_parameter_value().integer_value
        self.lightness_white_h = self.get_parameter(
            'detect.lane.white.lightness_h').get_parameter_value().integer_value

        self.hue_yellow_l = self.get_parameter(
            'detect.lane.yellow.hue_l').get_parameter_value().integer_value
        self.hue_yellow_h = self.get_parameter(
            'detect.lane.yellow.hue_h').get_parameter_value().integer_value
        self.saturation_yellow_l = self.get_parameter(
            'detect.lane.yellow.saturation_l').get_parameter_value().integer_value
        self.saturation_yellow_h = self.get_parameter(
            'detect.lane.yellow.saturation_h').get_parameter_value().integer_value
        self.lightness_yellow_l = self.get_parameter(
            'detect.lane.yellow.lightness_l').get_parameter_value().integer_value
        self.lightness_yellow_h = self.get_parameter(
            'detect.lane.yellow.lightness_h').get_parameter_value().integer_value

        self.is_calibration_mode = self.get_parameter(
            'is_detection_calibration_mode').get_parameter_value().bool_value
        if self.is_calibration_mode:
            self.add_on_set_parameters_callback(self.cbGetDetectLaneParam)

        self.sub_image_type = 'raw'         # you can choose image type 'compressed', 'raw'
        self.pub_image_type = 'compressed'  # you can choose image type 'compressed', 'raw'

        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage, '/detect/image_input/compressed', self.FindLane, 1
                )
        elif self.sub_image_type == 'raw':
            self.sub_image_original = self.create_subscription(
                Image, '/detect/image_input', self.FindLane, 1
                )

        if self.pub_image_type == 'compressed':
            self.pub_image_lane = self.create_publisher(
                CompressedImage, '/detect/image_output/compressed', 1
                )
        elif self.pub_image_type == 'raw':
            self.pub_image_lane = self.create_publisher(
                Image, '/detect/image_output', 1
                )

        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_white_lane = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub1/compressed', 1
                    )
                self.pub_image_yellow_lane = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub2/compressed', 1
                    )
            elif self.pub_image_type == 'raw':
                self.pub_image_white_lane = self.create_publisher(
                    Image, '/detect/image_output_sub1', 1
                    )
                self.pub_image_yellow_lane = self.create_publisher(
                    Image, '/detect/image_output_sub2', 1
                    )

        self.pub_lane = self.create_publisher(Float64, '/detect/lane', 1)

        self.pub_yellow_line_reliability = self.create_publisher(
            UInt8, '/detect/yellow_line_reliability', 1
            )

        self.pub_white_line_reliability = self.create_publisher(
            UInt8, '/detect/white_line_reliability', 1
            )

        self.pub_lane_state = self.create_publisher(UInt8, '/detect/lane_state', 1)

        self.cvBridge = CvBridge()

        self.counter = 1

        self.reliability_white_line = 100
        self.reliability_yellow_line = 100

        self.mov_avg_left = np.empty((0, 3))
        self.mov_avg_right = np.empty((0, 3))

        self.left_fit = None
        self.right_fit = None
        self.left_fitx = np.zeros(600)
        self.right_fitx = np.zeros(600)
        self.lane_fit_bef = np.array([0, 0, 0])
        self.is_center_x_exist = False


    def cbGetDetectLaneParam(self, parameters):
        for param in parameters:
            self.get_logger().info(f'Parameter name: {param.name}')
            self.get_logger().info(f'Parameter value: {param.value}')
            self.get_logger().info(f'Parameter type: {param.type_}')
            if param.name == 'detect.lane.white.hue_l':
                self.hue_white_l = param.value
            elif param.name == 'detect.lane.white.hue_h':
                self.hue_white_h = param.value
            elif param.name == 'detect.lane.white.saturation_l':
                self.saturation_white_l = param.value
            elif param.name == 'detect.lane.white.saturation_h':
                self.saturation_white_h = param.value
            elif param.name == 'detect.lane.white.lightness_l':
                self.lightness_white_l = param.value
            elif param.name == 'detect.lane.white.lightness_h':
                self.lightness_white_h = param.value
            elif param.name == 'detect.lane.yellow.hue_l':
                self.hue_yellow_l = param.value
            elif param.name == 'detect.lane.yellow.hue_h':
                self.hue_yellow_h = param.value
            elif param.name == 'detect.lane.yellow.saturation_l':
                self.saturation_yellow_l = param.value
            elif param.name == 'detect.lane.yellow.saturation_h':
                self.saturation_yellow_h = param.value
            elif param.name == 'detect.lane.yellow.lightness_l':
                self.lightness_yellow_l = param.value
            elif param.name == 'detect.lane.yellow.lightness_h':
                self.lightness_yellow_h = param.value
            return SetParametersResult(successful=True)
    ####################################################################### 
    #################### Main Process of Subscription Node ################
    #######################################################################
    def FindLane(self, image_msg):
        #For Debugging...., Comment for real Driving
        #if self.counter % 10 != 0:
        #    self.counter += 1
        #    return
        #else:
        #    self.counter = 1

        if self.sub_image_type == 'compressed':
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == 'raw':
            cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')

        white_fraction, cv_white_lane = self.maskWhiteLane(cv_image)
        yellow_fraction, cv_yellow_lane = self.maskYellowLane(cv_image)

        self.get_logger().info(f'white_fraction:{white_fraction},yellow_fraction:{yellow_fraction}')
        try:
# left_fitx,왼쪽 노란 차선 처리       
            # fit_from_lines 입력: 이전 추정값(self.left_fit)과 이진화된 차선 이미지(cv_yellow_lane)
            # 출력:
            # left_fitx: y좌표(ploty)에 대한 다항식 결과 x값들
            # left_fit: 2차 다항식 계수 [A, B, C] (e.g., 𝐴𝑥2+𝐵𝑥+𝐶)   

            if yellow_fraction > 3000:
                self.left_fitx, self.left_fit = self.fit_from_lines(self.left_fit, cv_yellow_lane)
                #Add new left_fit to mov_avg_left
                self.mov_avg_left = np.append(self.mov_avg_left, np.array([self.left_fit]), axis=0)
# right_fitx
            if white_fraction > 3000:
                self.right_fitx, self.right_fit = self.fit_from_lines(self.right_fit, cv_white_lane)
                #Add new right_fit to mov_avg_right
                self.mov_avg_right = np.append(self.mov_avg_right, np.array([self.right_fit]), axis=0)
        except Exception:
            if yellow_fraction > 3000:
                self.left_fitx, self.left_fit = self.sliding_windown(cv_yellow_lane, 'left')
                self.mov_avg_left = np.array([self.left_fit])

            if white_fraction > 3000:
                self.right_fitx, self.right_fit = self.sliding_windown(cv_white_lane, 'right')
                self.mov_avg_right = np.array([self.right_fit])

        """
        MOV_AVG_LENGTH: 이동 평균을 계산할 때 사용할 프레임 수(5)로, 차선의 2차 다항식 계수를 안정화하는 데 사용
        self.mov_avg_left와 self.mov_avg_right: 각각 왼쪽(노란색)과 오른쪽(흰색) 차선의 최근 다항식 계수들을 
        저장하는 배열(예: (N, 3) 형태, 여기서 N은 저장된 프레임 수, 3은 2차 다항식 계수 [a, b, c]).
        [::-1]: 배열을 시간 역순으로 정렬하여 최신 계수를 먼저 참조.
        [:, 0][0:MOV_AVG_LENGTH]: 각 계수(a, b, c)에 대해 최근 MOV_AVG_LENGTH 프레임의 값을 선택.
        np.mean: 선택된 계수들의 평균을 계산하여 self.left_fit과 self.right_fit에 저장.
        self.left_fit과 self.right_fit: 최종적으로 갱신된 2차 다항식 계수 [a, b, c]로, 차선의 x좌표를 계산하는 데 사용됨 
        (예: x = a*y² + b*y + c).
        """
        MOV_AVG_LENGTH = 5

        self.left_fit = np.array([
            np.mean(self.mov_avg_left[::-1][:, 0][0:MOV_AVG_LENGTH]),
            np.mean(self.mov_avg_left[::-1][:, 1][0:MOV_AVG_LENGTH]),
            np.mean(self.mov_avg_left[::-1][:, 2][0:MOV_AVG_LENGTH])
            ])
        
        self.right_fit = np.array([
            np.mean(self.mov_avg_right[::-1][:, 0][0:MOV_AVG_LENGTH]),
            np.mean(self.mov_avg_right[::-1][:, 1][0:MOV_AVG_LENGTH]),
            np.mean(self.mov_avg_right[::-1][:, 2][0:MOV_AVG_LENGTH])
            ])

        if self.mov_avg_left.shape[0] > 1000:
            self.mov_avg_left = self.mov_avg_left[0:MOV_AVG_LENGTH]

        if self.mov_avg_right.shape[0] > 1000:
            self.mov_avg_right = self.mov_avg_right[0:MOV_AVG_LENGTH]

        ################################### make lane ####################################
        self.make_lane(cv_image, white_fraction, yellow_fraction)
        ##################################################################################


#HSV 범위를 사용해 마스크를 생성, 흰색 차선의 신뢰도(reliability_white_line)를 조정
    def maskWhiteLane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_white_l
        Hue_h = self.hue_white_h
        Saturation_l = self.saturation_white_l
        Saturation_h = self.saturation_white_h
        Lightness_l = self.lightness_white_l
        Lightness_h = self.lightness_white_h

# HSV 값 유효성 검사
        try:
            lower_white = np.array([
                max(0, min(180, self.hue_white_l)),
                max(0, min(255, self.saturation_white_l)),
                max(0, min(255, self.lightness_white_l))
            ])
            upper_white = np.array([
                max(0, min(180, self.hue_white_h)),
                max(0, min(255, self.saturation_white_h)),
                max(0, min(255, self.lightness_white_h))
            ])
        except ValueError as e:
            self.get_logger().error(f"Invalid HSV values: {e}")
            return 0, np.zeros_like(image[:, :, 0])

        mask = cv2.inRange(hsv, lower_white, upper_white)

#fraction_num = np.count_nonzero(mask)를 통해 마스크에서 0이 아닌 픽셀의 개수를 계산합니다. 이는 흰색 차선의 존재를 나타내는 지표
        fraction_num = np.count_nonzero(mask)

#캘리브레이션 모드가 아닐 때, 흰색 차선 감지의 밝기 임계값을 동적으로 조정하여 환경 변화에 적응
        if not self.is_calibration_mode:
            if fraction_num > 35000:
                if self.lightness_white_l < 250:
                    self.lightness_white_l += 5
            elif fraction_num < 5000:
                if self.lightness_white_l > 50:
                    self.lightness_white_l -= 5

        how_much_short = 0

        for i in range(0, 600):
            if np.count_nonzero(mask[i, ::]) > 0:
                how_much_short += 1

        how_much_short = 600 - how_much_short

        if how_much_short > 100:
            if self.reliability_white_line >= 5:
                self.reliability_white_line -= 5
        elif how_much_short <= 100:
            if self.reliability_white_line <= 99:
                self.reliability_white_line += 5

        msg_white_line_reliability = UInt8()
        msg_white_line_reliability.data = self.reliability_white_line
        self.pub_white_line_reliability.publish(msg_white_line_reliability)

        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_white_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, 'jpg'))

            elif self.pub_image_type == 'raw':
                self.pub_image_white_lane.publish(self.cvBridge.cv2_to_imgmsg(mask, 'bgr8'))

        return fraction_num, mask

    def maskYellowLane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_yellow_l
        Hue_h = self.hue_yellow_h
        Saturation_l = self.saturation_yellow_l
        Saturation_h = self.saturation_yellow_h
        Lightness_l = self.lightness_yellow_l
        Lightness_h = self.lightness_yellow_h

# HSV 값 유효성 검사
        try:
            lower_yellow = np.array([
                max(0, min(180, self.hue_yellow_l)),
                max(0, min(255, self.saturation_yellow_l)),
                max(0, min(255, self.lightness_yellow_l))
            ])
            upper_yellow = np.array([
                max(0, min(180, self.hue_yellow_h)),
                max(0, min(255, self.saturation_yellow_h)),
                max(0, min(255, self.lightness_yellow_h))
            ])
        except ValueError as e:
            self.get_logger().error(f"Invalid HSV values: {e}")
            return 0, np.zeros_like(image[:, :, 0])
    
        """
        cv2.inRange(hsv, lower_yellow, upper_yellow): 
           HSV 이미지에서 지정된 색상 범위 내의 픽셀을 255로, 나머지를 0으로 설정하여 이진 마스크 생성.
        fraction_num = np.count_nonzero(mask): 
           마스크에서 비제로 픽셀(즉, 노란색 차선으로 간주되는 픽셀)의 총 개수를 계산. 
        이 값은 yellow_fraction으로 사용되어 차선 감지 신뢰도를 평가하는 데 활용됨
          예: make_lane에서 yellow_fraction > YELLOW_FRACTION_THRESHOLD
        """
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        fraction_num = np.count_nonzero(mask)

        if self.is_calibration_mode:
            if fraction_num > 35000:
                if self.lightness_yellow_l < 250:
                    self.lightness_yellow_l += 20
            elif fraction_num < 5000:
                if self.lightness_yellow_l > 90:
                    self.lightness_yellow_l -= 20

        how_much_short = 0

        """
        np.count_nonzero(mask[i, ::]): 이미지의 i번째 행에서 비제로 픽셀(차선 픽셀)의 개수를 계산.
        how_much_short: 초기에는 비제로 픽셀이 있는 행의 수를 세고, 최종적으로 이미지 높이(600)에서 이를 빼서 차선이 없는 행의 수를 계산.
        """
        for i in range(0, 600):
            if np.count_nonzero(mask[i, ::]) > 0:
                how_much_short += 1

        how_much_short = 600 - how_much_short

        if how_much_short > 100:
            if self.reliability_yellow_line >= 5:
                self.reliability_yellow_line -= 5
        elif how_much_short <= 100:
            if self.reliability_yellow_line <= 99:
                self.reliability_yellow_line += 5

        msg_yellow_line_reliability = UInt8()
        msg_yellow_line_reliability.data = self.reliability_yellow_line
        self.pub_yellow_line_reliability.publish(msg_yellow_line_reliability)

        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_yellow_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, 'jpg'))

            elif self.pub_image_type == 'raw':
                self.pub_image_yellow_lane.publish(self.cvBridge.cv2_to_imgmsg(mask, 'bgr8'))

        return fraction_num, mask
    """
    lane_fit: 2차 다항식의 계수 배열([a, b, c], x = ay² + by + c)이어야 하며, 길이가 3
    image: 이진 이미지로, 차선 픽셀을 포함합니다.
    image.nonzero()를 사용해 이미지에서 0이 아닌 픽셀의 좌표(nonzerox, nonzeroy)를 추출
    비제로 픽셀이 없으면 경고 로그를 출력하고 None, None을 반환
    주어진 lane_fit을 기준으로 ±margin(100 픽셀) 범위 내의 픽셀만 선택
    필터링된 포인트(x, y)를 사용하여 새로운 2차 다항식을 피팅
    np.polyfit(y, x, 2)를 사용해 y좌표에 대해 x좌표를 2차 다항식으로 피팅
    이미지 높이(image.shape[0])에 대해 y값 범위(ploty)를 생성하고, 이를 이용해 x좌표(lane_fitx)를 계산
    """
    def fit_from_lines(self, lane_fit, image):
        try:
            if lane_fit is None or len(lane_fit) != 3:
                self.get_logger().warn("Invalid lane_fit input. Expected array of 3 coefficients.")
                return None, None
            if image is None:
                self.get_logger().warn("Image is None.")
                return None, None

            nonzero = image.nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])

            if len(nonzerox) == 0 or len(nonzeroy) == 0:
                self.get_logger().warn("No non-zero pixels found in the image.")
                return None, None

            margin = 100

            lane_inds = (
                (nonzerox >
                (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] - margin)) &
                (nonzerox <
                (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] + margin))
            )

            x = nonzerox[lane_inds]
            y = nonzeroy[lane_inds]

            if len(x) < 50 or len(y) < 50:  # 최소 포인트 수 제한
                self.get_logger().warn(f"Not enough lane points to fit. Got {len(x)} points.")
                return None, None

            lane_fit = np.polyfit(y, x, 2)
            #Fits a 2nd-degree polynomial to the lane points.
            #x and y should be 1D arrays of the same length.

            ploty = np.linspace(0, image.shape[0] - 1, image.shape[0])
            #Generates a vertical range (ploty) from top to bottom of the image.

            lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]
            #Evaluates the polynomial at each y value (i.e., x = a*y² + b*y + c).

            #print(f'1: {lane_fitx.size,lane_fitx.shape}') # =>(600, (600,))
            
            return lane_fitx, lane_fit

        except Exception as e:
            self.get_logger().error(f"Exception in fit_from_lines: {e}")
            return None, None

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

    def publish_image(self, final, centerx, lane_state):
        if self.is_center_x_exist:
            try:
                msg_desired_center = Float64()
                msg_desired_center.data = centerx.item(350)
                self.pub_lane.publish(msg_desired_center)
            except IndexError:
                self.get_logger().warn("IndexError: centerx does not have index 350")
            except ValueError as e:
                self.get_logger().warn(f"ValueError: {e}")
            except Exception as e:
                self.get_logger().warn(f"Unexpected error while accessing centerx: {e}")

        if self.pub_image_type == 'compressed':
            self.pub_image_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(final, 'jpg'))
        else:
            self.pub_image_lane.publish(self.cvBridge.cv2_to_imgmsg(final, 'bgr8'))

        #self.get_logger().info(f'(Lane state: {lane_state.data},desired_center:{centerx.item(350)}')

    def make_lane(self, cv_image, white_fraction, yellow_fraction):

        # Rujin... Assuming mask is a 2D NumPy array of shape (600, width)
        non_zero_indices = np.nonzero(cv_image)  # Returns (row_indices, col_indices) of non-zero elements
        if len(non_zero_indices[1]) > 0:  # Check if there are any non-zero elements
            centerx = np.mean(non_zero_indices[1])  # Average of x-coordinates (column indices)
        else:
            centerx = 0  # If no non-zero elements, set centerx to 0 (or handle as needed)


        # Create an image to draw the lines on
        warp_zero = np.zeros((cv_image.shape[0], cv_image.shape[1], 1), dtype=np.uint8)

        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        color_warp_lines = np.dstack((warp_zero, warp_zero, warp_zero))

        ploty = np.linspace(0, cv_image.shape[0] - 1, cv_image.shape[0])

        # both lane -> 2, left lane -> 1, right lane -> 3, none -> 0
        lane_state = UInt8()
        """
        self.left_fitx: 노란색 차선(왼쪽 차선)의 x좌표 배열로, fit_from_lines 메서드에서 계산됩니다.
        ploty: 이미지 높이에 걸친 y좌표 배열 (np.linspace(0, cv_image.shape[0] - 1, cv_image.shape[0])).
        np.vstack([self.left_fitx, ploty]): left_fitx와 ploty를 수직으로 쌓아 2xN 배열을 생성.
        np.transpose: 배열을 Nx2 형태로 변환하여 (x, y) 좌표 쌍을 만듦.
        np.flipud: y좌표를 역순으로 정렬하여 이미지 상단부터 하단까지 그리도록 조정.flip up down
        np.array([...]): OpenCV의 cv2.polylines에 필요한 형식으로 점 집합을 변환.
        """
        if yellow_fraction > 3000:
            pts_left = np.array([np.flipud(np.transpose(np.vstack([self.left_fitx, ploty])))])
            cv2.polylines(
                color_warp_lines,
                np.int_([pts_left]),
                isClosed=False,
                color=(0, 0, 255), #BGR
                thickness=25
                )

        if white_fraction > 3000:
            pts_right = np.array([np.transpose(np.vstack([self.right_fitx, ploty]))])
            cv2.polylines(
                color_warp_lines,
                np.int_([pts_right]),
                isClosed=False,
                color=(255, 0, 0), # BGR (파란색)
                thickness=25
                )

        self.is_center_x_exist = True

#YELLOW, WHITE , 차선 중심 경로(centerx)를 계산
# self.left_fitx, self.right_fitx: 슬라이딩 윈도우 또는 다항 근사로 추정된 차선 곡선 x좌표, ploty: y축 좌표들
        if self.reliability_white_line > 50 and self.reliability_yellow_line > 50:
# Good State...  lane_state.data = 2          
            if white_fraction > 3000 and yellow_fraction > 3000:
                centerx = np.mean([self.left_fitx, self.right_fitx], axis=0)
                pts = np.hstack((pts_left, pts_right))
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

                lane_state.data = 2

                # Draw the lane onto the warped blank image
                cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
# More White_fraction lane_state.data = 3, 280 픽셀 보정값은 카메라 시야와 해상도에 따라 조정 필요
            if white_fraction > 3000 and yellow_fraction <= 3000:
                centerx = np.subtract(self.right_fitx, 280)
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

                lane_state.data = 3

# More Yellow_fraction lane_state.data = 1, 280 픽셀 보정값은 카메라 시야와 해상도에 따라 조정 필요
            if white_fraction <= 3000 and yellow_fraction > 3000:
                centerx = np.add(self.left_fitx, 280)
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

                lane_state.data = 1

        #YELLOW ONLY
        elif self.reliability_white_line <= 50 and self.reliability_yellow_line > 50:
            #np.subtract(self.left_fitx, 280)는 Left 차선에서 고정된 오프셋(280 픽셀)을 Add(+) 차선 중심을 추정합니다.
            centerx = np.add(self.left_fitx, 280)
            #차선 감지 시스템에서 차선 중심선을 시각화하기 위해 중심 x좌표(centerx)와 y좌표(ploty)를 조합하여 점 집합을 생성
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            lane_state.data = 1

        # WHITE ONLY , #3
        elif self.reliability_white_line > 50 and self.reliability_yellow_line <= 50:
            #np.subtract(self.right_fitx, 280)는 Right 차선에서 고정된 오프셋(280 픽셀)을 Subtract(-) 차선 중심을 추정
            centerx = np.subtract(self.right_fitx, 280)
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            lane_state.data = 3


        #ELSE, ERROR....
        else:
            self.is_center_x_exist = False

            lane_state.data = 0

            pass

        self.pub_lane_state.publish(lane_state)
        self.get_logger().info(f'Lane state: {lane_state.data}')

        if 'pts_center' in locals() and len(pts_center) > 0:
            cv2.polylines(
                color_warp_lines,
                np.int_([pts_center]),
                isClosed=False,
                color=(0, 255, 255),  # BGR (노란색)
                thickness=12
            )
        else:
            self.get_logger().warn("⚠️ pts_center가 정의되지 않았거나 비어 있습니다. 차선 라인을 그리지 않습니다.")

        
        # Combine the result with the original image
        final = cv2.addWeighted(cv_image, 1, color_warp, 0.2, 0)
        final = cv2.addWeighted(final, 1, color_warp_lines, 1, 0)

        if self.pub_image_type == 'compressed':
            self.publish_image(final, centerx, lane_state)
        elif self.pub_image_type == 'raw':
            self.publish_image(final, centerx, lane_state)


def main(args=None):
    rclpy.init(args=args)
    node = DetectLane()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()