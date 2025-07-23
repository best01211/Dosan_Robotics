import sys
import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer

from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


import cv2
import numpy as np
from ultralytics import YOLO
import threading


#export QT_QPA_PLATFORM=wayland

class ImageViewer(Node, QWidget):
    def __init__(self):
        rclpy.init()
        Node.__init__(self, 'compressed_image_viewer')
        QWidget.__init__(self)

        self.setWindowTitle("Camera Viewer - /camera/image_raw/compressed")
        self.resize(640, 480)

        # PyQt UI
        self.image_label = QLabel("Waiting for image...")
        self.image_label.setScaledContents(True)

        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        self.setLayout(layout)

        # ROS2 Subscriber
        self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            10
        )

        # ROS2 spin timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(50)  # 20 Hz


        # Load YOLOv5 model
        self.model = YOLO('yolov8n.pt')  # You can also use yolov5s.pt or yolov8s.pt etc.
        self.get_logger().info("YOLO model loaded.")

        self.lock = threading.Lock()
        self.image = None

        # Start OpenCV display in another thread
        self.display_thread = threading.Thread(target=self.display_loop)
        self.display_thread.daemon = True
        self.display_thread.start()


        
    def spin_ros(self):
        rclpy.spin_once(self, timeout_sec=0.01)

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Convert to RGB
        rgb_image = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w

        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        self.image_label.setPixmap(pixmap)

    def closeEvent(self, event):
        self.destroy_node()
        rclpy.shutdown()


def main():
    app = QApplication(sys.argv)
    viewer = ImageViewer()
    viewer.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
