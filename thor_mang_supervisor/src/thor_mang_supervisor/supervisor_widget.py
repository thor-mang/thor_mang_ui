#!/usr/bin/env python

import rospy
import std_msgs.msg

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Qt, QObject
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QComboBox

class SupervisorDialog(Plugin):

    def __init__(self, context):
        super(SupervisorDialog, self).__init__(context)
        self.setObjectName('SupervisorDialog')

        self._parent = QWidget()
        self._widget = SupervisorWidget(self._parent)
        
        context.add_widget(self._parent)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()

class SupervisorWidget(QObject):

    execute_step_plan_client = None
    step_plan = None

    def __init__(self, context):
        super(SupervisorWidget, self).__init__()

        # start widget
        widget = context

        # add upper part
        vbox = QVBoxLayout()

        # end widget
        widget.setLayout(vbox)
        #context.add_widget(widget)

    def shutdown_plugin(self):
        print "Shutting down ..."
        print "Done!"

