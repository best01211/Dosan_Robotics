import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QImage,QPixmap
from PyQt5 import uic
import cv2,imutils

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import numpy as np

import queue
import time
from threading import Thread


# Author: Karl.Kwon
# Email: mrthinks@gmail.com

form_class = uic.loadUiType("simple_qt_ros.ui")[0]
job_list = ['{"red": 2, "blue": 1, "goal": 1}', 
			'{"red": 1, "blue": 2, "goal": 2}', 
			'{"red": 1, "goal": 3}']

class WindowClass(QMainWindow, form_class):
	def __init__(self, operation_queue, node):
		super().__init__()
		self.setupUi(self)
		self.node = node

		self.btn_conveyorStart.clicked.connect(self.conveyorStartCallback)
		self.btn_conveyorStop.clicked.connect(self.conveyorStopCallback)
		self.btn_job.clicked.connect(self.btnjobCallback)
		# self.label_1.setText('HAHA')

		self.isClosed = False
		self.operation_queue = operation_queue

		for d in job_list:
			self.comboBox.addItem(d)
		self.selectdStrCb = job_list[0]

		self.comboBox.activated[str].connect(self.onActivate)

	def conveyorStartCallback(self):
		print("conveyorStartCallback clicked")
		self.node.send_start_conveyor()

	def conveyorStopCallback(self):
		print("conveyorStopCallback clicked")
		self.node.send_stop_conveyor()

	def btnjobCallback(self):
		print(self.selectdStrCb)
		self.node.send_job_command(self.selectdStrCb)

	def onActivate(self, text):
		print(text)
		self.selectdStrCb = text

	def editAddFunction(self, text):
		self.edit_1.append(text)

	def showImage(self, image_np):
		image = self.cvimage_to_label(image_np)
		self.label_1.setPixmap(QPixmap.fromImage(image))

	def cvimage_to_label(self,image):
		image = imutils.resize(image,width = 640)
		image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
		image = QImage(image,
			image.shape[1],
			image.shape[0],
			QImage.Format_RGB888)

		return image

	def closeEvent(self, event):
		self.operation_queue.put(' ')
		self.operation_queue.put(' ')

		print("closeEvent")

		time.sleep(0.1)

		self.isClosed = True


class GuiNode(Node):
	def __init__(self):
		super().__init__('image_subscriber')
		self.image_np = None
		self.subscription_rgb = self.create_subscription(
			CompressedImage,
			'yolo/compressed',
			self.listener_callback_rgb,
			10)
		self.subscription_rgb  # prevent unused variable warning

		self.conveyor_pub = self.create_publisher(String, 'conveyor/control', 10)
		self.conveyor_sub = self.create_subscription(String, 'conveyor/status', self.conveyor_status_callback, 10)
		self.conveyor_status = None

		self.gui_pub = self.create_publisher(String, 'gui/command', 10)

	def listener_callback_rgb(self, msg):
		np_arr = np.frombuffer(msg.data, np.uint8)
		self.image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode to color image
		# cv2.imshow('RGB Image', image_np)
		# cv2.waitKey(1)  # Display the image until a keypress

	def send_start_conveyor(self):
		print('send_start_conveyor')
		msg = String()
		msg.data = '{"control":"go", "distance.mm": 1000}'
		self.conveyor_pub.publish(msg)

	def send_stop_conveyor(self):
		print('send_stop_conveyor')
		msg = String()
		msg.data = '{"control":"stop"}'
		self.conveyor_pub.publish(msg)

	def conveyor_status_callback(self, msg):
		self.conveyor_status = "conveyor: " + msg.data

	def get_conveyor_status(self):
		ret = self.conveyor_status
		self.conveyor_status = None
		return ret

	def send_job_command(self, text):
		print('send_job_command')
		msg = String()
		msg.data = text
		self.gui_pub.publish(msg)



def ros_thread(operation_queue, event_queue, sleep_time):
	time_p = time.time()

	while(True):
		try:
			d = operation_queue.get_nowait()
			if d is not None:
				break
		except queue.Empty:
			pass

		time_c = time.time()
		if (time_c - time_p) > (sleep_time):
			event_queue.put('sleep_time: ', sleep_time)
			time_p = time_c

		time.sleep(0.01)
		# time.sleep(sleep_time)

	print('exit : ', sleep_time)


if __name__ == "__main__":
	app = QApplication(sys.argv)
	operation_queue = queue.Queue()

	rclpy.init(args=None)

	image_node = GuiNode()
	myWindow = WindowClass(operation_queue, image_node)

	myWindow.show()

	event_queue1 = queue.Queue()
	Thread(target = ros_thread, args=(operation_queue, event_queue1, 1), daemon=True).start()
	event_queue2 = queue.Queue()
	Thread(target = ros_thread, args=(operation_queue, event_queue2, 2), daemon=True).start()


	while(myWindow.isClosed == False):

		try:
			d1 = event_queue1.get_nowait()
			if d1 is not None:
				myWindow.editAddFunction('1' + d1)
		except queue.Empty:
			pass

		try:
			d2 = event_queue2.get_nowait()
			if d2 is not None:
				myWindow.editAddFunction('2' + d2)
		except queue.Empty:
			pass

		rclpy.spin_once(image_node, timeout_sec=0)
		if(image_node.image_np is not None):
			myWindow.showImage(image_node.image_np)
		if(image_node.conveyor_status is not None):
			myWindow.editAddFunction(image_node.get_conveyor_status())

		app.processEvents()
		# print(a)

	image_node.destroy_node()
	rclpy.shutdown()
	# app.exec_()
