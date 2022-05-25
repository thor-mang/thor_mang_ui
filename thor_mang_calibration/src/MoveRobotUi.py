#! /usr/bin/env python

import rospy
import cv_bridge

from PyQt5.QtWidgets import QWidget, QGridLayout, QPushButton, QLabel, QSizePolicy, QHBoxLayout
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QPixmap, QImage

#import dill
#dill.settings['recurse'] = True

from sensor_msgs.msg import Image

from ImageWidget import ImageWidget


class MoveRobotUi(QWidget):
    move_command = pyqtSignal(bool)

    def __init__(self, parent=None):
        super(MoveRobotUi, self).__init__(parent)

        self.parent = parent

        self.ui = QWidget(parent)

        self.bridge = cv_bridge.CvBridge()

        namespace = rospy.get_namespace()
        self.image_sub = rospy.Subscriber(namespace + 'sensor/head_cam/rgb/image_raw', Image, self.handle_image_sub)

        self.startUi()

        if parent is not None:
            self.parent.move_status.connect(self.handle_move_status)

    def startUi(self):
        self.ui.page_layout = QGridLayout()

        self.ui.camera_feed_label = ImageWidget(self)
        self.ui.camera_feed_label.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        self.ui.page_layout.addWidget(self.ui.camera_feed_label, 0, 0)

        self.ui.static_move_status_label = QLabel('Move Status: ')
        self.ui.move_status_label = QLabel('Not moving.')
        self.ui.move_status_layout = QHBoxLayout()
        self.ui.move_status_layout.addWidget(self.ui.static_move_status_label)
        self.ui.move_status_layout.addWidget(self.ui.move_status_label)
        self.ui.page_layout.addLayout(self.ui.move_status_layout, 1, 0)

        self.ui.move_button = QPushButton('Move')
        self.ui.move_button.clicked.connect(self.handle_move_button)
        self.ui.cancel_button = QPushButton('Cancel')
        self.ui.cancel_button.clicked.connect(self.handle_cancel_button)
        self.ui.cancel_button.setEnabled(False)
        self.ui.button_layout = QHBoxLayout()
        self.ui.button_layout.addWidget(self.ui.move_button)
        self.ui.button_layout.addWidget(self.ui.cancel_button)
        self.ui.page_layout.addLayout(self.ui.button_layout, 2, 0)

        self.ui.setLayout(self.ui.page_layout)

    def handle_image_sub(self, data):
        self.convert_image_to_pixmap(data)

    def convert_image_to_pixmap(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        preview_image = QImage(cv_image.data, image.width, image.height, QImage.Format_RGB888).rgbSwapped()

        pixmap = QPixmap()
        pixmap.convertFromImage(preview_image)
        self.ui.camera_feed_label.setPixmap(pixmap)

    def handle_move_button(self):
        self.move_command.emit(True)

    def handle_cancel_button(self):
        self.move_command.emit(False)

    def handle_move_status(self, status, done):
        self.ui.move_status_label.setText(str(status))

        self.ui.cancel_button.setEnabled(not done)
        self.ui.move_button.setEnabled(done)
