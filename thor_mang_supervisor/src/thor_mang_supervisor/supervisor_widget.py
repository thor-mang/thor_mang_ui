#!/usr/bin/env python

import os
import QtCore

import rospy
import rospkg
import actionlib

import std_msgs.msg

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QObject
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QComboBox, QColor, QFont

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
        self.supervisor_widget.allow_all_mode_transitions_button.clicked[bool].connect(self._handle_allow_all_mode_transitions_clicked)

        # style settings
        self._allowed_transition_color = QColor(0, 0, 0, 255)
        self._forbidden_transition_color = QColor(0, 0, 0, 100)
        self._active_mode_font = QFont()
        self._active_mode_font.setBold(True)
        self._inactive_mode_font = QFont()
        self._inactive_mode_font.setBold(False)
        self._status_ok_style = "background-color: rgb(200, 255, 150);"
        self._status_wait_style = "background-color: rgb(255, 255, 150);"
        self._status_error_style = "background-color: rgb(255, 220, 150);"

        # end widget
        widget.setLayout(vbox)
        #context.add_widget(widget)

        # init subscribers/action clients
        self.control_mode_sub = rospy.Subscriber("/flor/controller/mode_name", std_msgs.msg.String, self._control_mode_callback)
        self.allow_all_mode_transitions_status_sub = rospy.Subscriber("/mode_controllers/control_mode_controller/allow_all_mode_transitions_acknowledgement", std_msgs.msg.Bool, self._allow_all_mode_transitions_status_callback)
        self.set_control_mode_client = actionlib.SimpleActionClient("/mode_controllers/control_mode_controller/change_control_mode", ChangeControlModeAction)

        # init publisher
        self.torque_on_pub = rospy.Publisher('/thor_mang/torque_on', std_msgs.msg.Bool, queue_size=1)
        self.enable_lights_pub = rospy.Publisher('/thor_mang/enable_lights', std_msgs.msg.Bool, queue_size=1)
        self.allow_all_mode_transitions_pub = rospy.Publisher('/mode_controllers/control_mode_controller/allow_all_mode_transitions', std_msgs.msg.Bool, queue_size=1)

        # load transition parameters
        self._allowed_transitions = None
        self._parse_allowed_transitions()
        self._allow_all_mode_transitions_enabled = False

        # Qt signals
        self.connect(self, QtCore.SIGNAL('setTransitionModeStatusStyle(PyQt_PyObject)'), self._set_transition_mode_status_style)
        self.connect(self, QtCore.SIGNAL('setRobotModeStatusStyle(PyQt_PyObject)'), self._set_robot_mode_status_style)
        self.connect(self, QtCore.SIGNAL('setRobotModeStatusText(PyQt_PyObject)'), self._set_robot_mode_status_text)


    def shutdown_plugin(self):
        print "Shutting down ..."
        self.control_mode_sub.unregister()
        self.allow_all_mode_transitions_status_sub.unregister()
        self.torque_on_pub.unregister()
        self.enable_lights_pub.unregister()
        print "Done!"

    def _control_mode_callback(self, control_mode):
        self.emit(QtCore.SIGNAL('setRobotModeStatusText(PyQt_PyObject)'), str(control_mode.data))
        self.emit(QtCore.SIGNAL('setRobotModeStatusStyle(PyQt_PyObject)'), self._status_ok_style)

    def _allow_all_mode_transitions_status_callback(self, allowed_msg):
        self._allow_all_mode_transitions_enabled = allowed_msg.data
        self.emit(QtCore.SIGNAL('setTransitionModeStatusStyle(PyQt_PyObject)'), self._status_ok_style)

    def _parse_allowed_transitions(self):
        if rospy.has_param("/atlas_controller/control_mode_to_controllers"):
            self._allowed_transitions = dict()
            always_allowed_transitions = rospy.get_param("/atlas_controller/control_mode_to_controllers/all/transitions")
            for i in range(self.supervisor_widget.control_state_list.count()):
                mode = self.supervisor_widget.control_state_list.item(i).text().lower()
                allowed_transitions = rospy.get_param("/atlas_controller/control_mode_to_controllers/%s/transitions" % mode)
                allowed_transitions.extend(always_allowed_transitions)
                self._allowed_transitions[mode] = allowed_transitions
                rospy.loginfo("Loaded mode %s, has transitions to: %s", mode, str(allowed_transitions))
        else:
            rospy.logwarn("Unable to retrieve allowed transitions from control mode switcher, will not highlight allowed transitons and try again on next mode switch.")

    def _set_transition_mode_status_style(self, style_sheet_string):
        self.supervisor_widget.allow_all_mode_transitions_status.setStyleSheet(style_sheet_string)

    def _set_robot_mode_status_style(self, style_sheet_string):
        self.supervisor_widget.robot_mode_status.setStyleSheet(style_sheet_string)

    def _set_robot_mode_status_text(self, new_mode):
        self.supervisor_widget.robot_mode_status.setText(new_mode.upper())
        new_mode = new_mode.lower()
        if self._allowed_transitions is not None:
            has_defined_transitions = new_mode in self._allowed_transitions.keys()
            for i in range(self.supervisor_widget.control_state_list.count()):
                target_mode = self.supervisor_widget.control_state_list.item(i).text().lower()
                if target_mode == new_mode:
                    self.supervisor_widget.control_state_list.item(i).setFont(self._active_mode_font)
                    self.supervisor_widget.control_state_list.item(i).setTextColor(self._allowed_transition_color)
                elif has_defined_transitions and not target_mode in self._allowed_transitions[new_mode]:
                    self.supervisor_widget.control_state_list.item(i).setTextColor(self._forbidden_transition_color)
                    self.supervisor_widget.control_state_list.item(i).setFont(self._inactive_mode_font)
                else:
                    self.supervisor_widget.control_state_list.item(i).setTextColor(self._allowed_transition_color)
                    self.supervisor_widget.control_state_list.item(i).setFont(self._inactive_mode_font)
    
    def _handle_send_torque_mode_clicked(self):
        self.torque_on_pub.publish(self.supervisor_widget.torque_on.isChecked())

    def _handle_send_lights_mode_clicked(self):
        self.enable_lights_pub.publish(self.supervisor_widget.lights_on.isChecked())

    def _handle_send_control_mode_clicked(self):
        if self._allowed_transitions is None:
            self._parse_allowed_transitions()

        if (self.set_control_mode_client.wait_for_server(rospy.Duration(0.5))):
            self.emit(QtCore.SIGNAL('setRobotModeStatusStyle(PyQt_PyObject)'), self._status_wait_style)
            goal = ChangeControlModeGoal()
            goal.mode_request = self.supervisor_widget.control_state_list.currentItem().text().lower()
            rospy.loginfo("Requesting mode change to " + goal.mode_request + ".")
            self.set_control_mode_client.send_goal(goal)

            # waiting for getting list of parameter set names
            action_timeout = rospy.Duration(1.0)
            if (self.set_control_mode_client.wait_for_result(action_timeout)):
                control_mode = self.set_control_mode_client.get_result().result.current_control_mode
                self.emit(QtCore.SIGNAL('setRobotModeStatusText(PyQt_PyObject)'), str(control_mode))
                self.emit(QtCore.SIGNAL('setRobotModeStatusStyle(PyQt_PyObject)'), self._status_ok_style)
            else:
                rospy.logwarn("Didn't receive any results after %.1f sec. Check communcation!" % action_timeout.to_sec())
                self.emit(QtCore.SIGNAL('setRobotModeStatusStyle(PyQt_PyObject)'), self._status_error_style)
        else:
            rospy.logerr("Can't connect to control mode action server!")

    def _handle_allow_all_mode_transitions_clicked(self):
        self._allow_all_mode_transitions_enabled = not self._allow_all_mode_transitions_enabled
        self.supervisor_widget.allow_all_mode_transitions_status.setText("all allowed" if self._allow_all_mode_transitions_enabled else "restricted")
        self.supervisor_widget.allow_all_mode_transitions_button.setText("Restrict" if self._allow_all_mode_transitions_enabled else "Allow All")
        self.emit(QtCore.SIGNAL('setTransitionModeStatusStyle(PyQt_PyObject)'), self._status_wait_style)
        self.allow_all_mode_transitions_pub.publish(std_msgs.msg.Bool(self._allow_all_mode_transitions_enabled))

