import sys
import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QGraphicsView, QGraphicsScene, QGraphicsPixmapItem,
    QLabel, QPushButton, QGridLayout, QTextEdit, QProgressBar
)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

import cv2
import numpy as np

class ROS2ControlGUI(Node, QWidget):
    def __init__(self):
        rclpy.init()
        Node.__init__(self, 'gui_cmdvel_compressed')
        QWidget.__init__(self)

        self.setWindowTitle("ROS2 GUI - Camera + Lane + Move Buttons")
        self.setFixedSize(1300, 850)

        # --- Views ---
        self.camera_view = self.create_view("Camera View")
        self.lane_view = self.create_view("Lane Detection View")

        # --- ROS Subscribers ---
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.camera_callback, 10)
        self.create_subscription(CompressedImage, '/detect/image_lane/compressed', self.lane_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # --- ROS Publisher ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Layout ---
        main_layout = QVBoxLayout()
        top_layout = QHBoxLayout()
        top_layout.addLayout(self.camera_view['layout'], 1)
        top_layout.addLayout(self.lane_view['layout'], 2)

        main_layout.addLayout(top_layout)

        # --- Button Grid ---
        main_layout.addLayout(self.create_button_grid())

        # --- cmd_vel Visualization ---
        self.cmd_label = QLabel("Linear: 0.00, Angular: 0.00")
        self.cmd_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        main_layout.addWidget(self.cmd_label)

        self.linear_bar = QProgressBar()
        self.linear_bar.setRange(-100, 100)
        self.linear_bar.setFormat("Linear Speed: %v")
        self.linear_bar.setValue(0)

        self.angular_bar = QProgressBar()
        self.angular_bar.setRange(-100, 100)
        self.angular_bar.setFormat("Angular Z: %v")
        self.angular_bar.setValue(0)

        main_layout.addWidget(self.linear_bar)
        main_layout.addWidget(self.angular_bar)

        # --- cmd_vel 로그 ---
        main_layout.addWidget(QLabel("📜 cmd_vel History"))
        self.cmd_log = QTextEdit()
        self.cmd_log.setReadOnly(True)
        self.cmd_log.setFixedHeight(150)
        main_layout.addWidget(self.cmd_log)

        self.setLayout(main_layout)

        # --- Timer ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(50)

    def spin_ros(self):
        rclpy.spin_once(self, timeout_sec=0.01)

    def create_view(self, label_text):
        view = QGraphicsView()
        scene = QGraphicsScene()
        item = QGraphicsPixmapItem()
        scene.addItem(item)
        view.setScene(scene)

        label = QLabel(label_text)
        layout = QVBoxLayout()
        layout.addWidget(label)
        layout.addWidget(view)

        return {'view': view, 'scene': scene, 'item': item, 'layout': layout}

    def update_image(self, item, img):
        rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_img.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        item.setPixmap(QPixmap.fromImage(qt_image))

    def camera_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_small = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        h, w, _ = img_small.shape

        center_x, center_y = w // 2, h // 2
        cv2.line(img_small, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 1)
        cv2.line(img_small, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 1)

        roi_x = w // 4
        roi_y = int(h * 0.3)
        roi_w = w // 2
        roi_h = 80
        cv2.rectangle(img_small, (roi_x, roi_y), (roi_x + roi_w, roi_y + roi_h), (0, 0, 255), 2)

        self.update_image(self.camera_view['item'], img_small)

    def lane_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img_small = cv2.resize(img, (0, 0), fx=0.5, fy=0.5)
        self.update_image(self.lane_view['item'], img_small)

    def create_button_grid(self):
        grid = QGridLayout()

        btn_up = QPushButton("▲")
        btn_down = QPushButton("▼")
        btn_left = QPushButton("◀")
        btn_right = QPushButton("▶")
        btn_stop = QPushButton("■ 정지")

        btn_up.clicked.connect(lambda: self.publish_cmd(0.2, 0.0))
        btn_down.clicked.connect(lambda: self.publish_cmd(-0.2, 0.0))
        btn_left.clicked.connect(lambda: self.publish_cmd(0.0, 1.0))
        btn_right.clicked.connect(lambda: self.publish_cmd(0.0, -1.0))
        btn_stop.clicked.connect(lambda: self.publish_cmd(0.0, 0.0))

        grid.addWidget(btn_up, 0, 1)
        grid.addWidget(btn_left, 1, 0)
        grid.addWidget(btn_stop, 1, 1)
        grid.addWidget(btn_right, 1, 2)
        grid.addWidget(btn_down, 2, 1)

        return grid

    def publish_cmd(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)
        self.update_cmd_ui(linear, angular, sent=True)

    def cmd_callback(self, msg):
        self.update_cmd_ui(msg.linear.x, msg.angular.z, sent=False)

    def update_cmd_ui(self, linear, angular, sent=False):
        self.cmd_label.setText(f"Linear: {linear:.2f}, Angular: {angular:.2f}")
        self.linear_bar.setValue(int(linear * 100))
        self.angular_bar.setValue(int(angular * 100))

        prefix = "🔼 Sent →" if sent else "📡 Received ←"
        self.cmd_log.append(f"{prefix} Linear: {linear:.2f}, Angular: {angular:.2f}")

    def closeEvent(self, event):
        self.destroy_node()
        rclpy.shutdown()

def main():
    app = QApplication(sys.argv)
    gui = ROS2ControlGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
