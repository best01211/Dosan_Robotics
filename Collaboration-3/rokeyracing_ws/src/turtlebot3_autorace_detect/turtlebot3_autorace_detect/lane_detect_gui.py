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


import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import tkinter as tk
from PIL import Image, ImageTk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


class LaneVisualizer(tk.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.master.title("Lane Detection Viewer")
        self.pack()

        # Camera image display
        self.image_label = tk.Label(self)
        self.image_label.pack()

        # Matplotlib histogram figure
        self.hist_figure = Figure(figsize=(4, 2))
        self.hist_canvas = FigureCanvasTkAgg(self.hist_figure, master=self)
        self.hist_canvas.get_tk_widget().pack()

        self.hist_data = []

        # Keep reference to PhotoImage to prevent garbage collection
        self.photo_image = None

    def update_image(self, cv_img, x_left=None, x_right=None, x_center=None):
        """Draw lane markers and show image."""
        if x_left is not None:
            cv2.circle(cv_img, (x_left, 460), 6, (0, 255, 0), -1)
        if x_right is not None:
            cv2.circle(cv_img, (x_right, 460), 6, (0, 255, 255), -1)
        if x_center is not None:
            cv2.circle(cv_img, (x_center, 460), 6, (255, 0, 0), -1)
            self.hist_data.append(x_center)

        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb_image)
        self.photo_image = ImageTk.PhotoImage(image=pil_img)
        self.image_label.config(image=self.photo_image)

        self.update_histogram()

    def update_histogram(self):
        """Update histogram with recent center points."""
        self.hist_figure.clear()
        ax = self.hist_figure.add_subplot(111)
        if len(self.hist_data) > 0:
            ax.hist(self.hist_data[-50:], bins=20, color='blue')
            ax.set_title("Lane Center History")
            ax.set_xlim(0, 640)
        self.hist_canvas.draw()


class LaneGUI(Node):
    def __init__(self, visualizer):
        super().__init__('lane_gui_node')

        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.br = CvBridge()
        self.viewer = visualizer

        self.prev_x_left = 0
        self.prev_x_right = 640

    def image_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        success, x_left, x_right = self.lane_detect(frame)

        if success:
            x_center = (x_left + x_right) // 2
        else:
            x_left = x_right = x_center = None

        self.viewer.update_image(frame, x_left, x_right, x_center)

    def lane_detect(self, img):
        roi = img[300:480, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edge = cv2.Canny(blur, 60, 75)

        lines = cv2.HoughLinesP(edge, 1, np.pi/180, 50, minLineLength=50, maxLineGap=20)
        if lines is None:
            return False, 0, 0

        left_lines = []
        right_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) < 0.2:
                continue
            if slope < 0 and x1 < 320:
                left_lines.append(line[0])
            elif slope > 0 and x1 > 320:
                right_lines.append(line[0])

        def average_line(lines):
            if not lines:
                return 0.0, 0.0
            x_sum = y_sum = m_sum = 0.0
            for x1, y1, x2, y2 in lines:
                m = (y2 - y1) / (x2 - x1) if x2 != x1 else 0.0
                m_sum += m
                x_sum += x1 + x2
                y_sum += y1 + y2
            m_avg = m_sum / len(lines)
            x_avg = x_sum / (2 * len(lines))
            y_avg = y_sum / (2 * len(lines))
            b = y_avg - m_avg * x_avg
            return m_avg, b

        m_left, b_left = average_line(left_lines)
        m_right, b_right = average_line(right_lines)

        def compute_x(m, b):
            if m == 0:
                return None
            return int((180 - b) / m)

        x_left = compute_x(m_left, b_left)
        x_right = compute_x(m_right, b_right)

        if x_left is None:
            x_left = self.prev_x_left
        if x_right is None:
            x_right = self.prev_x_right

        self.prev_x_left = x_left
        self.prev_x_right = x_right

        return True, x_left, x_right


def main():
    rclpy.init()

    root = tk.Tk()
    visualizer = LaneVisualizer(root)
    visualizer.pack()

    node = LaneGUI(visualizer)

    def ros_spin():
        rclpy.spin_once(node, timeout_sec=0.01)
        root.after(10, ros_spin)

    root.after(10, ros_spin)
    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
