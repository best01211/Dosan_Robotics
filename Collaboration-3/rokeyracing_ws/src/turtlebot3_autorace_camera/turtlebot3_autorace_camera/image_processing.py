import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor_node')

        self.roi = 'Bottom'
        self.roi_ratio = 0.4 #
        self.bridge = CvBridge()

        # Declare parameter with default integer value
        self.video_port = self.declare_parameter('video_port', 0).get_parameter_value().integer_value
        self.roi = self.declare_parameter('roi', 'Left').get_parameter_value().string_value
        

        # Define ROI(region of interest) - (trapezoid)
        self.height = 480
        self.width = 620

        self.vertices = np.array([[
            (0.08 * self.width, self.height),  # Bottom-left
            (0.95 * self.width, self.height),  # Bottom-right
            (0.65 * self.width, 0.4 * self.height),  # Top-right
            (0.35 * self.width, 0.4 * self.height)   # Top-left
        ]], dtype=np.int32)

        # Subscribe to raw image
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for processed image
        self.publisher = self.create_publisher(
            Image,
            '/camera/processed',
            10
        )

        self.get_logger().info(f'Image Processor Node with imshow started. roi={self.roi}')

    def auto_exposure(self,frame):
        yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        yuv[:, :, 0] = cv2.equalizeHist(yuv[:, :, 0])
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)

    def white_balance_simple(self,img):
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB).astype(np.float32)

        avg_a = np.average(lab[:, :, 1])
        avg_b = np.average(lab[:, :, 2])

        lab[:, :, 1] -= ((avg_a - 128) * (lab[:, :, 0] / 255.0) * 1.1)
        lab[:, :, 2] -= ((avg_b - 128) * (lab[:, :, 0] / 255.0) * 1.1)

        lab = np.clip(lab, 0, 255).astype(np.uint8)
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    def draw_center_vertical_line(self,image, color=(0, 255, 0), thickness=2):
        img_copy = image.copy()
        height, width = img_copy.shape[:2]
        center_x = width // 2

        cv2.line(img_copy, (center_x, 0), (center_x, height), color, thickness)
        
        return img_copy

    def region_of_interest(self,img, vertices):
        """Apply a mask to focus on the region of interest."""
        mask = np.zeros_like(img)
        
        if len(img.shape) == 3:
            ignore_mask_color = (255, 255, 255)
        else:
            ignore_mask_color = 255

        cv2.fillPoly(mask, [vertices], ignore_mask_color)

        masked_img = cv2.bitwise_and(img, mask)
        
        return masked_img

    def average_slope_intercept(self,image, lines):
        """Average and extrapolate lines to draw a single line for left and right lanes."""
        left_lines = []
        right_lines = []
        
        if lines is None or len(lines) == 0:
            return None
        
        #HoughLinesP 등으로 검출된 선들을 기울기에 따라 왼쪽 차선 / 오른쪽 차선으로 분류
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            slope = (y2 - y1) / (x2 - x1 + 1e-5)  # Avoid division by zero
            intercept = y1 - slope * x1
            if slope < -0.4:  # Negative slope for left lane
                left_lines.append((slope, intercept))
            elif slope > 0.4:  # Positive slope for right lane
                right_lines.append((slope, intercept))

        #모든 선의 기울기(slope)와 절편(intercept)을 각각 모아 평균값을 계산
        def compute_line(lines, y1, y2):
            if not lines:
                return None
            slopes, intercepts = zip(*lines)
            avg_slope = np.mean(slopes)
            avg_intercept = np.mean(intercepts)
            x1 = int((y1 - avg_intercept) / avg_slope)
            x2 = int((y2 - avg_intercept) / avg_slope)
            return np.array([x1, int(y1), x2, int(y2)])


        y1 = image.shape[0]  # Bottom of image
        y2 = int(image.shape[0] * self.roi_ratio)  # Middle of ROI

        left_line = compute_line(left_lines, y1, y2)
        right_line = compute_line(right_lines, y1, y2)



        return np.array([left_line, right_line]) if left_line is not None and right_line is not None else None

    def draw_lines(self,image, lines, color=(255, 255, 0), thickness=20):
        """Draw lines on the image."""
        line_image = np.zeros_like(image)
        if lines is not None:
            for x1, y1, x2, y2 in lines:
                if x1 is not None and x2 is not None:
                    cv2.line(line_image, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
        return cv2.addWeighted(image, 0.8, line_image, 1, 0)

    def draw_roi_polygon(self,image, vertices, color=(0, 0, 255), thickness=2):
        
        image_copy = image.copy()
        cv2.polylines(image_copy, [vertices], isClosed=True, color=color, thickness=thickness)
    
        return image_copy
    
    def draw_center_rectangle(self,image, left_line, right_line, color=(255, 0, 0), width=50, height=50, ratio=2/3):
        img_copy = image.copy()

        # 왼쪽 선에서 2/3 지점 계산
        lx1, ly1, lx2, ly2 = left_line
        rx1, ry1, rx2, ry2 = right_line

        left_x = int(lx1 + (lx2 - lx1) * ratio)
        left_y = int(ly1 + (ly2 - ly1) * ratio)
        right_x = int(rx1 + (rx2 - rx1) * ratio)
        right_y = int(ry1 + (ry2 - ry1) * ratio)

        # 중심 좌표 계산
        center_x = (left_x + right_x) // 2
        center_y = (left_y + right_y) // 2

        # 사각형 좌표
        top_left = (center_x - width // 4, center_y - height // 4)
        bottom_right = (center_x + width // 4, center_y + height // 4)

        # 사각형 그리기
        cv2.rectangle(img_copy, top_left, bottom_right, color, thickness=2)

        return img_copy, (center_x,center_y)
    
    def process_frame(self,image):
        """Process a single frame to detect and draw lane lines."""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Apply Gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        # Canny edge detection
        edges = cv2.Canny(blur, 50, 150)
        
        # Apply ROI mask
        masked_edges = self.region_of_interest(edges, self.vertices)
        

        # Hough Transform
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=50,
            minLineLength=50,
            maxLineGap=150
        )
        color = (0,255,255)
        # 선 그리기
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                #self.get_logger().info(f'Lines:{x1},{x2},{y1},{y2}')                
                cv2.line(image, (x1, y1), (x2, y2), color, 2)
        

        masked_edges = self.draw_roi_polygon(masked_edges,self.vertices)
        cv2.imshow("masked_edges Image", masked_edges)
        cv2.waitKey(1)  # necessary for OpenCV GUI to update

        # Average and extrapolate lines
        averaged_lines = self.average_slope_intercept(image, lines)
        
        image = self.draw_roi_polygon(image,self.vertices)
    

        # Draw lines on the original image
        result = self.draw_lines(image, averaged_lines)
        result, center = self.draw_center_rectangle(result,averaged_lines[0],averaged_lines[1])
        result = self.draw_center_vertical_line(result)


        return result
    
    def fraction_frame(self,frame,fraction):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Grayscale
        edges = cv2.Canny(gray, 50, 150)               # ✅ Canny Edge Detection

        height = frame.shape[0]
        width = frame.shape[1]
        
        ratio = 0.5
        color = (255,0,255)
        
        if fraction == 'Bottom':
            top_left = np.array([0, height//2])           # (x, y)
            bottom_right = np.array([width, height])  # (x, y)
        elif fraction == 'Top':
            top_left = np.array([0, 0])           # (x, y)
            bottom_right = np.array([width, height//2])  # (x, y)
        elif fraction == 'Left':
            top_left = np.array([0, 0])           # (x, y)
            bottom_right = np.array([width//2, height])  # (x, y)
        elif fraction == 'Right':
            # Define coordinates to keep (left half)
            top_left = np.array([width//2, 0])           # (x, y)
            bottom_right = np.array([width, height])  # (x, y)

        if True:
            x1, y1 = top_left
            x2, y2 = bottom_right
            
            # Crop using coordinates (remember numpy indexing is [y, x])
            fraction_frame = frame[y1:y2, x1:x2]

            cv2.imshow("fraction_frame Image", fraction_frame)
            cv2.waitKey(1)  # necessary for OpenCV GUI to update
       
            #color = (255,0,255)
            #cv2.rectangle(frame, top_left, bottom_right, color, 2)  # 초록색 선

        return fraction_frame
        """
        if self.roi == 'Bottom':
            top_left = np.array([0, height//2])           # (x, y)
            bottom_right = np.array([width, height])  # (x, y)
            
            x1, y1 = top_left
            x2, y2 = bottom_right
            
            # Crop using coordinates (remember numpy indexing is [y, x])
            roi_edges = edges[y1:y2, x1:x2]

       
            color = (255,0,255)
            cv2.rectangle(roi_edges, top_left, bottom_right, color, 2)  # 초록색 선

        elif self.roi == 'Top':
            top_left = np.array([0, 0])           # (x, y)
            bottom_right = np.array([width, height//2])  # (x, y)
            
            x1, y1 = top_left
            x2, y2 = bottom_right
            
            # Crop using coordinates (remember numpy indexing is [y, x])
            roi_edges = edges[y1:y2, x1:x2]

       
            color = (0,255,0)
            cv2.rectangle(roi_edges, top_left, bottom_right, color, 2)  # 초록색 선


        elif self.roi == 'Left':
            top_left = np.array([0, 0])           # (x, y)
            bottom_right = np.array([width//2, height])  # (x, y)
            
            x1, y1 = top_left
            x2, y2 = bottom_right
            
            # Crop using coordinates (remember numpy indexing is [y, x])
            roi_edges = edges[y1:y2, x1:x2]

       
            color = (255,255,0)
            cv2.rectangle(roi_edges, top_left, bottom_right, color, 2)  # 초록색 선

        elif self.roi == 'Right':
            # Define coordinates to keep (left half)
            top_left = np.array([width//2, 0])           # (x, y)
            bottom_right = np.array([width, height])  # (x, y)
       
            x1, y1 = top_left
            x2, y2 = bottom_right
            
            # Crop using coordinates (remember numpy indexing is [y, x])
            roi_edges = edges[y1:y2, x1:x2]

            color = (255,255,255)
            cv2.rectangle(roi_edges, top_left, bottom_right, color, 2)  # 초록색 선
        """



        return frame
    
    def process_image(self,frame):


        # adding Gaussian blur to the image of original
        frame = cv2.GaussianBlur(frame, (5, 5), 0)

        # 자동 노출 보정 + 화이트 밸런싱
        frame = self.auto_exposure(frame)
        #frame = self.white_balance_simple(frame)
        
        frame = self.process_frame(frame)

        cv2.imshow("Full Huffman Frame Image", frame)
        cv2.waitKey(1)  # necessary for OpenCV GUI to update

        # HSV 변환
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 흰색 마스킹
        # 기존 level.yaml 파일 값
        # lower_white = np.array([0, 105, 0])
        # upper_white = np.array([179, 255, 70])
        
        # 흰색 마스킹 새로운 값
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 60, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # 노란색 마스킹
        # 기존 level.yaml 파일 값
        # lower_yellow = np.array([10, 95, 700])
        # upper_yellow = np.array([127, 255, 255])
        
        # 노란색 마스킹 새로운 값
        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([35, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 흰색 노란색 합치기
        mask_combined = cv2.bitwise_or(mask_white, mask_yellow)

        #cv2.imshow("mask_yellow Image", mask_yellow)
        #cv2.waitKey(1)  # necessary for OpenCV GUI to update

        return mask_white, mask_yellow, mask_combined

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            mask_white, mask_yellow, mask_combined = self.process_image(frame)
            # Example: convert to grayscale and back to BGR

            #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            mask_white = cv2.cvtColor(mask_white, cv2.COLOR_GRAY2BGR)
            mask_yellow = cv2.cvtColor(mask_yellow, cv2.COLOR_GRAY2BGR)
            mask_combined = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)

            # Show the processed image using OpenCV
            #cv2.imshow("mask_white Image", mask_white)
            #cv2.waitKey(1)  # necessary for OpenCV GUI to update

            #cv2.imshow("mask_yellow Image", mask_yellow)
            #cv2.waitKey(1)  # necessary for OpenCV GUI to update

            #cv2.imshow("mask_combined Image", mask_combined)
            #cv2.waitKey(1)  # necessary for OpenCV GUI to update


            # Convert back to ROS Image and publish
            out_msg = self.bridge.cv2_to_imgmsg(mask_combined, encoding='bgr8')
            self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
