#!/usr/bin/env python

import os

import rospy
import rospkg
import actionlib

import std_msgs.msg

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QObject
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QComboBox

from vigir_humanoid_control_msgs.msg import ChangeControlModeAction, ChangeControlModeGoal, ChangeControlModeResult

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

    def __init__(self, context):
        super(SupervisorWidget, self).__init__()

        # start widget
        widget = context
        vbox = QVBoxLayout()

        # load from ui
        self.supervisor_widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('thor_mang_supervisor'), 'resource', 'supervisor.ui')
        loadUi(ui_file, self.supervisor_widget, {'QWidget': QWidget})
        vbox.addWidget(self.supervisor_widget)

        # connect to signals
        self.supervisor_widget.send_torque_mode.clicked[bool].connect(self._handle_send_torque_mode_clicked)
        self.supervisor_widget.send_lights_mode.clicked[bool].connect(self._handle_send_lights_mode_clicked)
        self.supervisor_widget.send_control_mode.clicked[bool].connect(self._handle_send_control_mode_clicked)

        # end widget
        widget.setLayout(vbox)
        #context.add_widget(widget)

        # init subscribers/action clients
        self.control_mode_sub = rospy.Subscriber("/flor/controller/mode_name", std_msgs.msg.String, self._control_mode_callback)
        self.set_control_mode_client = actionlib.SimpleActionClient("/mode_controllers/control_mode_controller/change_control_mode", ChangeControlModeAction)

        # init publisher
        self.torque_on_pub = rospy.Publisher('/thor_mang/torque_on', std_msgs.msg.Bool, queue_size=1)
        self.enable_lights_pub = rospy.Publisher('/thor_mang/enable_lights', std_msgs.msg.Bool, queue_size=1)

    def shutdown_plugin(self):
        print "Shutting down ..."
        self.control_mode_sub.unregister()
        self.torque_on_pub.unregister()
        self.enable_lights_pub.unregister()
        print "Done!"

    def _control_mode_callback(self, control_mode):
        self.supervisor_widget.robot_mode_status.setText(str(control_mode.data).upper())
    
    def _handle_send_torque_mode_clicked(self):
        self.torque_on_pub.publish(self.supervisor_widget.torque_on.isChecked())

    def _handle_send_lights_mode_clicked(self):
        self.enable_lights_pub.publish(self.supervisor_widget.lights_on.isChecked())

    def _handle_send_control_mode_clicked(self):
        if (self.set_control_mode_client.wait_for_server(rospy.Duration(0.5))):
            goal = ChangeControlModeGoal()
            goal.mode_request = self.supervisor_widget.control_state_list.currentItem().text().lower()
            print("Requesting mode change to " + goal.mode_request + ".")
            self.set_control_mode_client.send_goal(goal)

            # waiting for getting list of parameter set names
            if (self.set_control_mode_client.wait_for_result(rospy.Duration(1.0))):
                control_mode = self.set_control_mode_client.get_result().result.current_control_mode
                self.supervisor_widget.robot_mode_status.setText(control_mode.upper())
            else:
                printf("Didn't received any results. Check communcation!")
        else:
            printf("Can't connect to control mode action server!")

