#!/usr/bin/env python

import os

import rospy
import rospkg
import actionlib

import std_msgs.msg

import QtCore

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QObject
from python_qt_binding.QtGui import QWidget, QVBoxLayout, QColor, QFont

from robotis_controller_msgs.msg import SyncWriteItem
from thor_mang_control_msgs.msg import ControlModeStatus, GetControlModesAction, GetControlModesGoal, ChangeControlModeAction, ChangeControlModeGoal


class FeetWristControlDialog(Plugin):

    def __init__(self, context):
        super(FeetWristControlDialog, self).__init__(context)
        self.setObjectName('FeetWristControlDialog')

        self._parent = QWidget()
        self._widget = FeetWristControlWidget(self._parent)

        context.add_widget(self._parent)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()


class FeetWristControlWidget(QObject):

    def __init__(self, context):
        super(FeetWristControlWidget, self).__init__()

        # start widget
        widget = context
        vbox = QVBoxLayout()

        # load from ui
        self.supervisor_widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('thor_mang_supervisor'), 'resource', 'feet_wrist_control.ui')
        loadUi(ui_file, self.supervisor_widget, {'QWidget': QWidget})
        vbox.addWidget(self.supervisor_widget)

        # connect to signals
        self.supervisor_widget.ft_feet_air_button.clicked[bool].connect(lambda: self._handle_ft_command_clicked("feet", "ft_air"))
        self.supervisor_widget.ft_feet_ground_button.clicked[bool].connect(lambda: self._handle_ft_command_clicked("feet", "ft_gnd"))
        self.supervisor_widget.ft_feet_apply_button.clicked[bool].connect(lambda: self._handle_ft_command_clicked("feet", "ft_send"))
        self.supervisor_widget.ft_feet_save_button.clicked[bool].connect(lambda: self._handle_ft_command_clicked("feet", "ft_save"))

        self.supervisor_widget.ft_wrists_air_button.clicked[bool].connect(lambda: self._handle_ft_command_clicked("wrists", "ft_air"))
        self.supervisor_widget.ft_wrists_apply_button.clicked[bool].connect(lambda: self._handle_ft_command_clicked("wrists", "ft_send"))
        self.supervisor_widget.ft_wrists_save_button.clicked[bool].connect(lambda: self._handle_ft_command_clicked("wrists", "ft_save"))

        # end widget
        widget.setLayout(vbox)

        # init publisher
        self.ft_feet_calib_pub = rospy.Publisher("robotis/feet_ft/ft_calib_command", std_msgs.msg.String, queue_size=10)
        self.ft_wrists_calib_pub = rospy.Publisher("robotis/wrists_ft/ft_calib_command", std_msgs.msg.String, queue_size=10)


    def shutdown_plugin(self):
        print "Shutting down ..."
        print "Done!"


    def _handle_ft_command_clicked(self, sensor, command):
        string_msg = std_msgs.msg.String()
        string_msg.data = command
        if sensor == "feet":
            self.ft_feet_calib_pub.publish(command)
        elif sensor == "wrists":
            self.ft_wrists_calib_pub.publish(command)
        else:
            print "Unknown ft sensor ", sensor

