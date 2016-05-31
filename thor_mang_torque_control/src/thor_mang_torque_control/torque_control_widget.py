import os
import QtCore

import rospy
import rospkg

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QObject
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QComboBox, QColor, QFont, QListWidgetItem

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

        # connect to signals
        # self.supervisor_widget.send_torque_mode.clicked[bool].connect(self._handle_send_torque_mode_clicked)

        # style settings
        # self._allowed_transition_color = QColor(0, 0, 0, 255)
        item = QListWidgetItem("test")
        item.setCheckState(Qt.Unchecked)
        self.torque_control_widget.listWidget.addItem(item)

        # Qt signals
        # self.connect(self, QtCore.SIGNAL('setTransitionModeStatusStyle(PyQt_PyObject)'), self._set_transition_mode_status_style)

        # end widget
        widget.setLayout(vbox)

        # init subscribers/action clients

        # init publishers
        
    def shutdown_plugin(self):
        pass
