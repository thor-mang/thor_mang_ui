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

        self._last_control_mode_status = ControlModeStatus()

        # load transition parameters
        self._allow_all_mode_transitions_enabled = False
        self._allow_falling_controller_enabled = False

        # load joints from parameter server
        self.joint_groups = {"Misc": JointGroup("NONE", "Misc")}
        self.load_groups()
        self.load_joints()

        # start widget
        widget = context
        vbox = QVBoxLayout()

        # load from ui
        self.supervisor_widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('thor_mang_supervisor'), 'resource', 'supervisor.ui')
        loadUi(ui_file, self.supervisor_widget, {'QWidget': QWidget})
        vbox.addWidget(self.supervisor_widget)

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

        # connect to signals
        self.supervisor_widget.send_torque_mode.clicked[bool].connect(self._handle_send_torque_mode_clicked)
        self.supervisor_widget.send_lights_mode.clicked[bool].connect(self._handle_send_lights_mode_clicked)
        self.supervisor_widget.send_control_mode.clicked[bool].connect(self._handle_send_control_mode_clicked)
        self.supervisor_widget.allow_all_mode_transitions_button.clicked[bool].connect(self._handle_allow_all_mode_transitions_clicked)

        self.supervisor_widget.ft_feet_air_button.clicked[bool].connect(lambda: self._handle_ft_command_clicked("feet", "ft_air"))
        self.supervisor_widget.ft_feet_ground_button.clicked[bool].connect(lambda: self._handle_ft_command_clicked("feet", "ft_gnd"))
        self.supervisor_widget.ft_feet_apply_button.clicked[bool].connect(lambda: self._handle_ft_command_clicked("feet", "ft_apply"))
        self.supervisor_widget.ft_feet_save_button.clicked[bool].connect(lambda: self._handle_ft_command_clicked("feet", "ft_save"))

        # Qt signals
        self.connect(self, QtCore.SIGNAL('setAvailableControlStateList(PyQt_PyObject)'), self._set_available_control_state_list)
        self.connect(self, QtCore.SIGNAL('setTransitionModeStatusStyle(PyQt_PyObject)'), self._set_transition_mode_status_style)
        self.connect(self, QtCore.SIGNAL('setRobotModeStatusStyle(PyQt_PyObject)'), self._set_robot_mode_status_style)
        self.connect(self, QtCore.SIGNAL('setRobotModeStatusText(PyQt_PyObject)'), self._set_robot_mode_status_text)

        # end widget
        widget.setLayout(vbox)

        # init subscribers
        self.control_mode_status_sub = rospy.Subscriber("control_mode_switcher/status", ControlModeStatus, self._control_mode_status_callback)
        self.allow_all_mode_transitions_status_sub = rospy.Subscriber("control_mode_switcher/allow_all_mode_transitions_ack", std_msgs.msg.Bool, self._allow_all_mode_transitions_status_callback)

        # init publisher
        self.sync_write_pub = rospy.Publisher("robotis/sync_write_item", SyncWriteItem, queue_size=10)
        self.allow_all_mode_transitions_pub = rospy.Publisher('control_mode_switcher/allow_all_mode_transitions', std_msgs.msg.Bool, queue_size=1)
        self.ft_feet_calib_pub = rospy.Publisher("robotis/feet_ft/ft_calib_command", std_msgs.msg.String, queue_size=10)
        self.ft_wrists_calib_pub = rospy.Publisher("robotis/wrists_ft/ft_calib_command", std_msgs.msg.String, queue_size=10)

        # action clients
        self.get_control_modes_client = actionlib.SimpleActionClient("control_mode_switcher/get_control_modes", GetControlModesAction)
        self.set_control_mode_client = actionlib.SimpleActionClient("control_mode_switcher/change_control_mode", ChangeControlModeAction)

        # get all available modes
        self.obtain_control_modes()

    def shutdown_plugin(self):
        print "Shutting down ..."
        self.control_mode_status_sub.unregister()
        self.allow_all_mode_transitions_status_sub.unregister()
        self.sync_write_pub.unregister()
        self.allow_all_mode_transitions_pub.unregister()
        print "Done!"

    def load_groups(self):
        groups = rospy.get_param("joints/groups", []).keys()
        for group in groups:
            prefix = rospy.get_param("joints/groups/" + group + "/prefix", "not_found")
            name = rospy.get_param("joints/groups/" + group + "/name", "No name found.")
            self.joint_groups[name] = (JointGroup(prefix, name))

    def load_joints(self):
        joint_list = rospy.get_param("joints/joint_list", [])
        for joint in joint_list:
            matched = False
            for group in self.joint_groups.values():
                for prefix in group.prefix:
                    if joint.startswith(prefix):
                        group.joint_list.append(joint)
                        # print "Added joint", joint, "to group", group.name
                        matched = True
                        break
                if matched:
                    break
            if not matched:
                self.joint_groups["Misc"].joint_list.append(joint)
                # print "Added joint", joint, "to group", self.joint_groups["Misc"].name

    def obtain_control_modes(self):
        if self.get_control_modes_client.wait_for_server(rospy.Duration(0.5)):
            self.get_control_modes_client.send_goal(GetControlModesGoal())

            # waiting for getting list of parameter set names
            action_timeout = rospy.Duration(1.0)
            if self.get_control_modes_client.wait_for_result(action_timeout):
                result = self.get_control_modes_client.get_result()

                available_control_modes = result.available_control_modes
                self.emit(QtCore.SIGNAL('setAvailableControlStateList(PyQt_PyObject)'), available_control_modes)

                status = ControlModeStatus()
                status.current_control_mode = result.current_control_mode
                status.allowed_control_modes = result.allowed_control_modes
                self.emit(QtCore.SIGNAL('setRobotModeStatusText(PyQt_PyObject)'), status)
                self.emit(QtCore.SIGNAL('setRobotModeStatusStyle(PyQt_PyObject)'), self._status_ok_style)
            else:
                rospy.logwarn("Didn't receive control modes %.1f sec. Check communcation!" % action_timeout.to_sec())
                self.emit(QtCore.SIGNAL('setRobotModeStatusStyle(PyQt_PyObject)'), self._status_error_style)

    def _control_mode_status_callback(self, status):
        self.emit(QtCore.SIGNAL('setRobotModeStatusText(PyQt_PyObject)'), status)
        if status.status & ControlModeStatus.NO_ERROR or status.status & ControlModeStatus.MODE_ACCEPTED:
            self.emit(QtCore.SIGNAL('setRobotModeStatusStyle(PyQt_PyObject)'), self._status_ok_style)
        else:
            self.emit(QtCore.SIGNAL('setRobotModeStatusStyle(PyQt_PyObject)'), self._status_error_style)
        self._last_control_mode_status = status

    def _allow_all_mode_transitions_status_callback(self, allowed_msg):
        self._allow_all_mode_transitions_enabled = allowed_msg.data
        self.emit(QtCore.SIGNAL('setTransitionModeStatusStyle(PyQt_PyObject)'), self._status_ok_style)
        self.emit(QtCore.SIGNAL('setRobotModeStatusText(PyQt_PyObject)'), self._last_control_mode_status)

    def _set_available_control_state_list(self, available_modes):
        self.supervisor_widget.control_state_list.clear()
        for mode in available_modes:
          self.supervisor_widget.control_state_list.addItem(mode)

    def _set_transition_mode_status_style(self, style_sheet_string):
        self.supervisor_widget.allow_all_mode_transitions_status.setStyleSheet(style_sheet_string)

    def _set_robot_mode_status_style(self, style_sheet_string):
        self.supervisor_widget.robot_mode_status.setStyleSheet(style_sheet_string)

    def _set_robot_mode_status_text(self, status):
        new_mode = str(status.current_control_mode)
        self.supervisor_widget.robot_mode_status.setText(new_mode.upper())
        new_mode = new_mode.lower()
        if status.allowed_control_modes is not None:
            for i in range(self.supervisor_widget.control_state_list.count()):
                target_mode = self.supervisor_widget.control_state_list.item(i).text().lower()
                if target_mode == new_mode:
                    self.supervisor_widget.control_state_list.item(i).setTextColor(self._allowed_transition_color)
                    self.supervisor_widget.control_state_list.item(i).setFont(self._active_mode_font)
                elif not self._allow_all_mode_transitions_enabled and  target_mode not in status.allowed_control_modes:
                    self.supervisor_widget.control_state_list.item(i).setTextColor(self._forbidden_transition_color)
                    self.supervisor_widget.control_state_list.item(i).setFont(self._inactive_mode_font)
                else:
                    self.supervisor_widget.control_state_list.item(i).setTextColor(self._allowed_transition_color)
                    self.supervisor_widget.control_state_list.item(i).setFont(self._inactive_mode_font)

    def _handle_send_torque_mode_clicked(self):
        msg = SyncWriteItem()
        msg.item_name = "torque_enable"

        if self.supervisor_widget.torque_off.isChecked():
            for joint_group in self.joint_groups.values():
                msg.joint_name.extend(joint_group.joint_list)
                msg.value.extend([0] * len(joint_group.joint_list))

        elif self.supervisor_widget.torque_on.isChecked():
            for joint_group in self.joint_groups.values():
                msg.joint_name.extend(joint_group.joint_list)
                msg.value.extend([1] * len(joint_group.joint_list))

        elif self.supervisor_widget.torque_partial.isChecked():
            torque_hands = 1 if self.supervisor_widget.torque_hands.isChecked() else 0
            torque_arms = 1 if self.supervisor_widget.torque_arms.isChecked() else 0
            torque_legs = 1 if self.supervisor_widget.torque_legs.isChecked() else 0
            torque_head = 1 if self.supervisor_widget.torque_head.isChecked() else 0
            torque_torso = 1 if self.supervisor_widget.torque_torso.isChecked() else 0

            if "Fingers" in self.joint_groups:
                msg.joint_name.extend(self.joint_groups["Fingers"].joint_list)
                msg.value.extend([torque_hands] * len(self.joint_groups["Fingers"].joint_list))
            if "Left Arm" in self.joint_groups:
                msg.joint_name.extend(self.joint_groups["Left Arm"].joint_list)
                msg.value.extend([torque_arms] * len(self.joint_groups["Left Arm"].joint_list))
            if "Right Arm" in self.joint_groups:
                msg.joint_name.extend(self.joint_groups["Right Arm"].joint_list)
                msg.value.extend([torque_arms] * len(self.joint_groups["Right Arm"].joint_list))
            if "Left Leg" in self.joint_groups:
                msg.joint_name.extend(self.joint_groups["Left Leg"].joint_list)
                msg.value.extend([torque_legs] * len(self.joint_groups["Left Leg"].joint_list))
            if "Right Leg" in self.joint_groups:
                msg.joint_name.extend(self.joint_groups["Right Leg"].joint_list)
                msg.value.extend([torque_legs] * len(self.joint_groups["Right Leg"].joint_list))
            if "Head" in self.joint_groups:
                msg.joint_name.extend(self.joint_groups["Head"].joint_list)
                msg.value.extend([torque_head] * len(self.joint_groups["Head"].joint_list))
            if "Torso" in self.joint_groups:
                msg.joint_name.extend(self.joint_groups["Torso"].joint_list)
                msg.value.extend([torque_torso] * len(self.joint_groups["Torso"].joint_list))

        self.sync_write_pub.publish(msg)

    def _handle_send_lights_mode_clicked(self):
        #self.enable_lights_pub.publish(self.supervisor_widget.lights_on.isChecked())
        pass

    def _handle_send_control_mode_clicked(self):
        if self.set_control_mode_client.wait_for_server(rospy.Duration(0.5)):
            self.emit(QtCore.SIGNAL('setRobotModeStatusStyle(PyQt_PyObject)'), self._status_wait_style)
            goal = ChangeControlModeGoal()
            goal.mode_request = self.supervisor_widget.control_state_list.currentItem().text().lower()
            rospy.loginfo("Requesting mode change to " + goal.mode_request + ".")
            self.set_control_mode_client.send_goal(goal)

            # waiting for getting list of parameter set names
            action_timeout = rospy.Duration(5.0)
            if self.set_control_mode_client.wait_for_result(action_timeout):
                result = self.set_control_mode_client.get_result()
                self._control_mode_status_callback(result.result)
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

    def _handle_ft_command_clicked(self, sensor, command):
        string_msg = std_msgs.msg.String()
        string_msg.data = command
        if sensor == "feet":
            self.ft_feet_calib_pub.publish(command)
        elif sensor == "wrists":
            self.ft_wrists_calib_pub.publish(command)
        else:
            print "Unknown ft sensor ", sensor


class JointGroup():
    def __init__(self, prefix, name):
        self.prefix = prefix
        self.name = name
        self.joint_list = []