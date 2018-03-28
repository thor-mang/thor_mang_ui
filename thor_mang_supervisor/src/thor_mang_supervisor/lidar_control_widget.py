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

LIDAR_NAME = "head_lidar_spinning_joint"


class LidarControlDialog(Plugin):

    def __init__(self, context):
        super(LidarControlDialog, self).__init__(context)
        self.setObjectName('LidarControlDialog')

        self._parent = QWidget()
        self._widget = LidarControlWidget(self._parent)

        context.add_widget(self._parent)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()


class LidarControlWidget(QObject):

    def __init__(self, context):
        super(LidarControlWidget, self).__init__()

        # start widget
        widget = context
        vbox = QVBoxLayout()

        # load from ui
        self.supervisor_widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('thor_mang_supervisor'), 'resource', 'lidar_control.ui')
        loadUi(ui_file, self.supervisor_widget, {'QWidget': QWidget})
        vbox.addWidget(self.supervisor_widget)

        # connect to signals
        self.supervisor_widget.lidar_send_button.clicked[bool].connect(self._handle_lidar_send_button_clicked)
        self.supervisor_widget.lidar_speed_spin.valueChanged[int].connect(self._handle_lidar_speed_spin_changed)

        # end widget
        widget.setLayout(vbox)

        # init publisher
        self.sync_write_pub = rospy.Publisher("robotis/sync_write_item", SyncWriteItem, queue_size=10)

    def shutdown_plugin(self):
        print "Shutting down ..."
        self.sync_write_pub.unregister()
        print "Done!"


    def _handle_lidar_send_button_clicked(self):
        msg = SyncWriteItem()
        msg.joint_name.append(LIDAR_NAME)
        if self.supervisor_widget.torque_lidar.isChecked():
            # send speed
            msg.item_name = "goal_velocity"
            speed = self.supervisor_widget.lidar_speed_spin.value()
            msg.value.append(speed)
        else:
            # send torque off
            msg.item_name = "torque_enable"
            msg.value.append(0)
        self.sync_write_pub.publish(msg)

    def _handle_lidar_speed_spin_changed(self):
        speed = self.supervisor_widget.lidar_speed_spin.value()
        rpm = speed * 0.114
        radpsec = rpm * 0.104719755

        self.supervisor_widget.lidar_rps_label.setText(truncate(radpsec, 2) + " rad/sec")



def truncate(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    s = '{}'.format(f)
    if 'e' in s or 'E' in s:
        return '{0:.{1}f}'.format(f, n)
    i, p, d = s.partition('.')
    return '.'.join([i, (d+'0'*n)[:n]])
