import os
import math
import QtCore

import rospy
import rospkg

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QObject
from python_qt_binding.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QListWidgetItem

from sensor_msgs.msg import JointState
from robotis_controller_msgs.msg import SyncWriteItem, RebootDevice


class TorqueControlDialog(Plugin):

    def __init__(self, context):
        super(TorqueControlDialog, self).__init__(context)
        self.setObjectName('TorqueControlDialog')

        self._parent = QWidget()
        self._widget = TorqueControlWidget(context)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()


class TorqueControlWidget(QObject):

    def __init__(self, context):
        super(TorqueControlWidget, self).__init__()
        self.setObjectName('TorqueControl')

        # start widget
        self._widget = QWidget()

        # load from ui
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('thor_mang_torque_control'), 'resources', 'torque_control.ui')
        loadUi(ui_file, self._widget, {'QWidget': QWidget})
        self._widget.setObjectName('TorqueControlUi')

        # load appendage ui
        self.appendage_widgets = []
        self.appendage_ui_file = os.path.join(rp.get_path('thor_mang_torque_control'), 'resources', 'appendage_widget.ui')

        # load joints from parameter server
        self.joint_groups = [JointGroup(["NONE"], "Misc")]
        self.load_groups()
        if not self.load_joints():
            rospy.logerr("Couldn't determine joint names. Check if joint states are published.")
        self.add_appendage_widgets()

        # connect to signals
        self._widget.send_torque.clicked[bool].connect(self._handle_send_torque_clicked)
        self._widget.send_reboot.clicked[bool].connect(self._handle_send_reboot_clicked)
        self._widget.select_all_button.clicked[bool].connect(self._handle_select_all_button_clicked)
        self._widget.deselect_button.clicked[bool].connect(self._handle_deselect_button_clicked)

        # Qt signals
        # self.connect(self, QtCore.SIGNAL('setTransitionModeStatusStyle(PyQt_PyObject)'), self._set_transition_mode_status_style)

        # set window title
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # add widget to the user interface
        context.add_widget(self._widget)

        # init publishers
        self.torque_pub = rospy.Publisher("robotis/sync_write_item", SyncWriteItem, queue_size=1)
        self.reboot_pub = rospy.Publisher("robotis/reboot_device", RebootDevice, queue_size=1)

    def load_groups(self):
        groups = rospy.get_param("joints/groups", []).keys()
        for group in groups:
            prefix = rospy.get_param("joints/groups/" + group + "/prefix", "not_found")
            name = rospy.get_param("joints/groups/" + group + "/name", "No name found.")
            self.joint_groups.append(JointGroup(prefix, name))

    def load_joints(self):
        try:
            msg = rospy.wait_for_message('joints/joint_states', JointState, 1.0)
        except (rospy.ROSException, rospy.ROSInterruptException):
            return False

        joint_ignore_list = rospy.get_param("joints/joint_ignore_list", [])

        for joint in msg.name:
            if any(joint in s for s in joint_ignore_list):
                # print "Ignored joint", joint
                continue

            matched = False
            for group in self.joint_groups:
                for prefix in group.prefix:
                    if joint.startswith(prefix):
                        group.joint_list.append(joint)
                        # print "Added joint", joint, "to group", group.name
                        matched = True
                        break
                if matched:
                    break
            if not matched:
                self.joint_groups[0].joint_list.append(joint)

        return True

    def add_appendage_widgets(self):
        num_of_widgets = 0

        for group in self.joint_groups:
            if len(group.joint_list) == 0:
                continue

            # create widget
            widget = QWidget()
            self.appendage_widgets.append(widget)
            loadUi(self.appendage_ui_file, widget, {'QWidget': QWidget})
            widget.appendage_group.setTitle(group.name)
            self._widget.appendage_grid.addWidget(widget, math.floor(num_of_widgets / 4), num_of_widgets % 4)
            for joint in group.joint_list:
                self.add_joint_to_list(joint, widget)

            num_of_widgets += 1;

            # connect to signal
            self.connect_select_button_signals(widget)

    def connect_select_button_signals(self, widget):
        widget.select_all_button.clicked[bool].connect(lambda: self._handle_select_all_clicked(widget))
        widget.deselect_button.clicked[bool].connect(lambda: self._handle_deselect_clicked(widget))

    def _handle_send_torque_clicked(self):
        msg = SyncWriteItem()
        msg.item_name = "torque_enable"
        for widget in self.appendage_widgets:
            for item in self.iter_items(widget.list):
                state = item.checkState()
                msg.joint_name.append(item.text())
                if state == Qt.Checked:
                    msg.value.append(1)
                else:
                    msg.value.append(0)
        self.torque_pub.publish(msg)

    def _handle_send_reboot_clicked(self):
        msg = RebootDevice()
        for widget in self.appendage_widgets:
            for item in self.iter_items(widget.list):
                state = item.checkState()
                if state == Qt.Checked:
                    msg.joint_name.append(item.text())
        self.reboot_pub.publish(msg)

    def _handle_select_all_button_clicked(self):
        for widget in self.appendage_widgets:
            self._handle_select_all_clicked(widget)

    def _handle_deselect_button_clicked(self):
        for widget in self.appendage_widgets:
            self._handle_deselect_clicked(widget)

    def _handle_select_all_clicked(self, widget):
        for item in self.iter_items(widget.list):
            item.setCheckState(Qt.Checked)

    def _handle_deselect_clicked(self, widget):
        for item in self.iter_items(widget.list):
            item.setCheckState(Qt.Unchecked)

    @staticmethod
    def add_joint_to_list(joint_name, widget):
        item = QListWidgetItem(joint_name)
        item.setCheckState(Qt.Unchecked)
        widget.list.addItem(item)

    @staticmethod
    def iter_items(list_widget):
        for i in range(list_widget.count()):
            yield list_widget.item(i)

    def shutdown_plugin(self):
        pass


class JointGroup():
    def __init__(self, prefix, name):
        self.prefix = prefix
        self.name = name
        self.joint_list = []