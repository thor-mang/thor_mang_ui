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


class TorqueControlDialog(Plugin):

    def __init__(self, context):
        super(TorqueControlDialog, self).__init__(context)
        self.setObjectName('TorqueControlDialog')

        self._parent = QWidget()
        self._widget = TorqueControlWidget(self._parent)

        context.add_widget(self._parent)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()


class TorqueControlWidget(QObject):

    def __init__(self, context):
        super(TorqueControlWidget, self).__init__()

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
        ui_file = os.path.join(rp.get_path('thor_mang_supervisor'), 'resource', 'torque_control.ui')
        loadUi(ui_file, self.supervisor_widget, {'QWidget': QWidget})
        vbox.addWidget(self.supervisor_widget)

        # style settings
        self._status_ok_style = "background-color: rgb(200, 255, 150);"
        self._status_wait_style = "background-color: rgb(255, 255, 150);"
        self._status_error_style = "background-color: rgb(255, 220, 150);"

        # connect to signals
        self.supervisor_widget.send_torque_mode.clicked[bool].connect(self._handle_send_torque_mode_clicked)

        # end widget
        widget.setLayout(vbox)

        # init publisher
        self.sync_write_pub = rospy.Publisher("robotis/sync_write_item", SyncWriteItem, queue_size=10)
        self.allow_all_mode_transitions_pub = rospy.Publisher('control_mode_switcher/allow_all_mode_transitions', std_msgs.msg.Bool, queue_size=1)
        self.ft_feet_calib_pub = rospy.Publisher("robotis/feet_ft/ft_calib_command", std_msgs.msg.String, queue_size=10)
        self.ft_wrists_calib_pub = rospy.Publisher("robotis/wrists_ft/ft_calib_command", std_msgs.msg.String, queue_size=10)



    def shutdown_plugin(self):
        print "Shutting down ..."
        self.control_mode_status_sub.unregister()
        self.allow_all_mode_transitions_status_sub.unregister()
        self.sync_write_pub.unregister()
        self.allow_all_mode_transitions_pub.unregister()
        print "Done!"

    def load_groups(self):
        groups = rospy.get_param("joints/groups", [])
        if not groups:
            return

        for group in groups.keys():
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

    


class JointGroup():
    def __init__(self, prefix, name):
        self.prefix = prefix
        self.name = name
        self.joint_list = []

