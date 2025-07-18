import sys
import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer

from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from ultralytics import YOLO


class YOLOViewer(Node, QWidget):
    def __init__(self):
        rclpy.init()
        Node.__init__(self, 'yolo_image_viewer')
        QWidget.__init__(self)

        self.setWindowTitle("YOLOv8 Object Detection")
        self.resize(640, 480)

        # Load YOLOv8 model
        self.model = YOLO("yolov8n.pt")  # Replace with path if needed

        # PyQt GUI
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

        # Timer for ROS spin
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(50)  # 20 Hz

    def spin_ros(self):
        rclpy.spin_once(self, timeout_sec=0.01)

    def image_callback(self, msg):
        # Decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Run YOLOv8 inference
        results = self.model(img, verbose=False)[0]

        # Draw results
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]
            conf = box.conf[0].item()

            # Draw bounding box and label
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                img, f'{label} {conf:.2f}',
                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2
            )

        # Convert to RGB and display
        rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        qt_image = QImage(rgb_image.data, w, h, ch * w, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        self.image_label.setPixmap(pixmap)

    def closeEvent(self, event):
        self.destroy_node()
        rclpy.shutdown()


def main():
    app = QApplication(sys.argv)
    viewer = YOLOViewer()
    viewer.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
