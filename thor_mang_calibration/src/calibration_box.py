#!/usr/bin/env python

import os

import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QSize, pyqtSignal
from python_qt_binding.QtGui import QPixmap, QValidator, QDoubleValidator
from python_qt_binding.QtWidgets import QWidget, QLabel, QStackedLayout

class CalibrationBox(QWidget):
    step_size = 0.1
    
    # setup own signals
    adjust_rviz = pyqtSignal(int, bool)
    show_axes = pyqtSignal(int, bool, str)
    show_animation = pyqtSignal(int, bool, str)
    
    
    def __init__(self, box_num, box_config, offset, old_offset):
        super(CalibrationBox, self).__init__() 

        self.box_num = box_num        
        self.config = box_config

        self.offset = offset        
        self.old_offset = old_offset
        self.joint = self.config['name']
                        
        self.ui = QWidget()
        
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'ui', 'calibration_box.ui')
        loadUi(ui_file, self.ui, {'QWidget': QWidget})
                
        self._set_image()
        
        self._set_initial_values()
        
        # connect signals
        self.ui.step_size.editingFinished.connect(self._handle_step_size)
        self.ui.inc_button.clicked[bool].connect(lambda state, x=+1: self._handle_inc_and_dec_button(x))
        self.ui.dec_button.clicked[bool].connect(lambda state, x=-1: self._handle_inc_and_dec_button(x))
        self.ui.enable_rviz_check.toggled.connect(self._handle_rviz_check)
        self.ui.show_axes_check.toggled.connect(self._handle_show_axes_check)
        self.ui.show_animation_check.toggled.connect(self._handle_show_animation_check)
        self.ui.reset_button.clicked[bool].connect(self._handle_reset_button)
       

    def _set_initial_values(self):
        joint_help_text = None
        if self.config.has_key('joint_specific_help_text'):
            joint_help_text = self.config['joint_specific_help_text']
            
        self.ui.box.setTitle(self.joint)
        self.ui.tip.setText(joint_help_text)
        self.ui.offset.setText(str(self.offset))
        self.ui.step_size.setText(str(self.step_size))
        
        self.ui.frame = None
        
        
    def _set_image(self):
        rp = rospkg.RosPack()
        img_path = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'img', self.config['pic'])
        self.pixmap = QPixmap(img_path)
        img = QLabel()
        img.setScaledContents(True)
        img.setPixmap(self.pixmap)

        # prepare image/rviz presentation
        layout = QStackedLayout()
        self.ui.display.setLayout(layout)
        layout.addWidget(img)
            
# _______ button functions ____________________________________________________________________________________________

    def _handle_step_size(self):
        valid, value = self._string_to_float(self.ui.step_size.text())

        if valid:
            self.step_size = value
        else:
            self.ui.step_size.setText(str(self.step_size))
            
            
    def _handle_inc_and_dec_button(self, sign):
        self.offset += sign * float(self.step_size)
                
        self.ui.offset.setText(str(self.offset))
        
        
    def _handle_reset_button(self):      
        self.offset = self.old_offset
        self.ui.offset.setText(str(self.offset))
        
        
    def _handle_rviz_check(self, checked):
        if checked:
            self.ui.frame.getManager().getRootDisplayGroup().getDisplayAt(1).setValue(True)
            self.ui.frame.setVisible(True)
                
            self.ui.display.layout().setCurrentIndex(1)
            self.ui.show_axes_check.setEnabled(True)
            
            show_axes = self.ui.show_axes_check.isChecked()
            self._handle_show_axes_check(show_axes)
        else:
            self.ui.frame.getManager().getRootDisplayGroup().getDisplayAt(1).setValue(False)
            self.ui.display.layout().setCurrentIndex(0)
            
        self.adjust_rviz.emit(self.box_num, checked)
        

    def _handle_show_axes_check(self, checked):
        link = ''
        if self.config.has_key('link name'):
            link = self.config['link name']
        else:
            link = self.joint
        
        self.show_axes.emit(self.box_num, checked, link)
        
        
    def _handle_show_animation_check(self, checked):
        self.show_animation.emit(self.box_num, checked, self.joint) #pyqtSignal(int, bool, str)

#________ helper functions ____________________________________________________________________________________________             
        
    def _string_to_float(self, text):
        valid = False
        value = -1
        
        validator = QDoubleValidator()
        result,_,_ = validator.validate(text, 5)

        if result == QValidator.Acceptable:
            text = text.replace(',', '.')
            valid = True
            value = float(text)
        
        return valid, value
    
