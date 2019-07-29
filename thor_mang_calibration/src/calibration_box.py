#!/usr/bin/env python

import os

import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QSize
#from python_qt_binding.QtGui import QSizePolicy
from python_qt_binding.QtWidgets import QWidget

class CalibrationBox(QWidget):

    def __init__(self):
        super(CalibrationBox, self).__init__() 
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'ui', 'calibration_box.ui')
        loadUi(ui_file, self, {'QWidget': QWidget})
        
