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


class LightControlDialog(Plugin):

    def __init__(self, context):
        super(LightControlDialog, self).__init__(context)
        self.setObjectName('LightControlDialog')

        self._parent = QWidget()
        self._widget = LightControlWidget(self._parent)

        context.add_widget(self._parent)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()


class LightControlWidget(QObject):

    def __init__(self, context):
        super(LightControlWidget, self).__init__()

        # start widget
        widget = context
        vbox = QVBoxLayout()

        # load from ui
        self.supervisor_widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('thor_mang_supervisor'), 'resource', 'light_control.ui')
        loadUi(ui_file, self.supervisor_widget, {'QWidget': QWidget})
        vbox.addWidget(self.supervisor_widget)

        # connect to signals
        self.supervisor_widget.send_lights_mode.clicked[bool].connect(self._handle_send_lights_mode_clicked)

        # end widget
        widget.setLayout(vbox)


    def shutdown_plugin(self):
        print "Shutting down ..."
        print "Done!"

    def _handle_send_lights_mode_clicked(self):
        #self.enable_lights_pub.publish(self.supervisor_widget.lights_on.isChecked())
        pass


