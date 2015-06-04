#!/usr/bin/env python

import os
import QtCore

import rospy
import rospkg
import actionlib

import std_msgs.msg
import sensor_msgs.msg

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QObject
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QComboBox, QColor, QFont

import PyKDL

class CoMAdjustmentDialog(Plugin):

    def __init__(self, context):
        super(CoMAdjustmentDialog, self).__init__(context)
        self.setObjectName('CoMAdjustmentDialog')

        self._parent = QWidget()
        self._widget = CoMAdjustmentWidget(self._parent)
        
        context.add_widget(self._parent)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()

class CoMAdjustmentWidget(QObject):

    def __init__(self, context):
        super(CoMAdjustmentWidget, self).__init__()

        # start widget
        widget = context
        vbox = QVBoxLayout()

        # load from ui
        self.com_adjustment_widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('thor_mang_com_adjustment'), 'resource', 'com_adjustment.ui')
        loadUi(ui_file, self.com_adjustment_widget, {'QWidget': QWidget})
        vbox.addWidget(self.com_adjustment_widget)

        # connect to signals
        self.com_adjustment_widget.pushButton_Send.clicked[bool].connect(self._handle_send_pitch_clicked)
        self.com_adjustment_widget.slider_HipPitch.valueChanged[int].connect(self._handle_slider_hip_changed)
        self.com_adjustment_widget.slider_AnklePitch.valueChanged[int].connect(self._handle_slider_ankle_changed)
        self.com_adjustment_widget.spin_HipPitch.valueChanged[float].connect(self._handle_spin_hip_changed)
        self.com_adjustment_widget.spin_AnklePitch.valueChanged[float].connect(self._handle_spin_ankle_changed)
        
        # end widget
        widget.setLayout(vbox)
       # context.add_widget(widget)

        # init subscribers/action clients
        self.imu_pitch_sub = rospy.Subscriber("/thor_mang/pelvis_imu", sensor_msgs.msg.Imu, self._imu_pitch_callback)
        self.pitch_values_ack_sub = rospy.Subscriber("/thor_mang/step_controller/current_pitch_offset", std_msgs.msg.Float64MultiArray, self._pitch_values_ack_callback)
        
        # init publisher
        self.pitch_values_pub = rospy.Publisher("/thor_mang/step_controller/pitch_offset", std_msgs.msg.Float64MultiArray, queue_size=1)


    def shutdown_plugin(self):
        print "Shutting down ..."
        self.pitch_values_pub.unregister()
        self.pitch_values_ack_sub.unregister()
        self.imu_pitch_sub.unregister()
        print "Done!"

    def _pitch_values_ack_callback(self, current_values_msg):
        if len(current_values_msg.data) != 2:
            return
      
        self.com_adjustment_widget.spin_CurrentHipPitch.setValue(current_values_msg.data[0])
        self.com_adjustment_widget.spin_CurrentAnklePitch.setValue(current_values_msg.data[1])

    def _imu_pitch_callback(self, imu_msg):
        rotation = PyKDL.Rotation.Quaternion(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)
        [roll, pitch, yaw] = rotation.GetRPY()

        self.com_adjustment_widget.lineEdit_CurrentIMUPitch.setText(str(pitch))
        
    def _handle_send_pitch_clicked(self):
        current_pitch_msg = std_msgs.msg.Float64MultiArray(data = [0] * 2)
        current_pitch_msg.data[0] = self.com_adjustment_widget.spin_HipPitch.value()
        current_pitch_msg.data[1] = self.com_adjustment_widget.spin_AnklePitch.value()
        self.pitch_values_pub.publish(current_pitch_msg)
        
    def _handle_slider_hip_changed(self, value):
        self.com_adjustment_widget.spin_HipPitch.setValue(value/100.0)
        
    def _handle_slider_ankle_changed(self, value):
        self.com_adjustment_widget.spin_AnklePitch.setValue(value/100.0)
        
    def _handle_spin_hip_changed(self, value):
        self.com_adjustment_widget.slider_HipPitch.blockSignals(True)
        self.com_adjustment_widget.slider_HipPitch.setValue(value*100.0)
        self.com_adjustment_widget.slider_HipPitch.blockSignals(False)
        
    def _handle_spin_ankle_changed(self, value):
        self.com_adjustment_widget.slider_AnklePitch.blockSignals(True)
        self.com_adjustment_widget.slider_AnklePitch.setValue(value*100.0)
        self.com_adjustment_widget.slider_AnklePitch.blockSignals(False)
