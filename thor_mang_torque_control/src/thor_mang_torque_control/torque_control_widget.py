import os
import QtCore

import rospy
import rospkg

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QObject
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QComboBox, QColor, QFont, QListWidgetItem

from robotis_controller_msgs.msg import SyncWriteItem


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

        # start widget
        widget = context
        vbox = QVBoxLayout()

        # load from ui
        self.torque_control_widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('thor_mang_torque_control'), 'resources', 'torque_control.ui')
        loadUi(ui_file, self.torque_control_widget, {'QWidget': QWidget})
        vbox.addWidget(self.torque_control_widget)

        # load joints from parameter server
        self.load_joints("joint_list")

        # connect to signals
        self.torque_control_widget.sendTorque.clicked[bool].connect(self._handle_send_torque_clicked)

        # Qt signals
        # self.connect(self, QtCore.SIGNAL('setTransitionModeStatusStyle(PyQt_PyObject)'), self._set_transition_mode_status_style)

        # end widget
        widget.setLayout(vbox)

        # init publishers
        self.torque_pub = rospy.Publisher("robotis/sync_write_item", SyncWriteItem, queue_size=1000)

    @staticmethod
    def iter_items(list_widget):
        for i in range(list_widget.count()):
            yield list_widget.item(i)

    def _handle_send_torque_clicked(self):
        msg = SyncWriteItem()
        msg.item_name = "torque_enable"
        for item in self.iter_items(self.torque_control_widget.listWidget):
            state = item.checkState()
            msg.joint_name.append(item.text())
            if state == Qt.Checked:
                msg.value.append(1)
            else:
                msg.value.append(0)
        self.torque_pub.publish(msg)

    def load_joints(self, ns):
        joint_list = rospy.get_param(ns, [])
        for joint in joint_list:
            self.add_joint_to_list(joint)

    def add_joint_to_list(self, joint_name):
        item = QListWidgetItem(joint_name)
        item.setCheckState(Qt.Unchecked)
        self.torque_control_widget.listWidget.addItem(item)

    def shutdown_plugin(self):
        pass