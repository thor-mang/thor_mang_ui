#!/usr/bin/env python

import os

import rospy
import rospkg
import actionlib

import math

from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtWidgets import QWidget, QFrame

from calibration_page import CalibrationPage

class WalkingCalibrationPage(CalibrationPage):
    footstep_parameters = pyqtSignal(dict)
    
    
    _id = -1
    
    _noBoxes = -1
    
    _num_steps = 1
    _step_length = 0.1
    _side_step_length = 0.0
    _step_angle = 0.0
    _step_time = 1.0

    def __init__(self, page_id, config,  offset_limits, rviz_frames):
        super(WalkingCalibrationPage, self).__init__(page_id, config, offset_limits, rviz_frames)
        
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'ui', 'walking_panel.ui')
        self.walking_panel = QWidget()
        loadUi(ui_file, self.walking_panel, {'QWidget': QWidget})
        self.walking_panel.setDisabled(True)
        
        self.page_layout.addWidget(self.walking_panel, 0, config['num_joints'])

        self.page_layout.addItem(self.spacer, 0, config['num_joints'] + 1)
        self._set_initial_values()

        # connect signals
        self.walking_panel.num_steps_edit.editingFinished.connect(self._handle_num_steps_edit)
        self.walking_panel.step_length_edit.editingFinished.connect(self._handle_step_length_edit)
        self.walking_panel.side_step_edit.editingFinished.connect(self._handle_side_step_edit)
        self.walking_panel.angle_edit.editingFinished.connect(self._handle_angle_edit)
        self.walking_panel.step_time_edit.editingFinished.connect(self._handle_step_time_edit)
        
        self.walking_panel.forward_button.clicked[bool].connect(self._take_steps)
        self.walking_panel.backward_button.clicked[bool].connect(self._take_steps)
        self.walking_panel.left_button.clicked[bool].connect(self._take_steps)
        self.walking_panel.right_button.clicked[bool].connect(self._take_steps)
        self.walking_panel.stop_button.clicked[bool].connect(self._take_steps)
        self.walking_panel.turn_left_button.clicked[bool].connect(self._take_steps)
        self.walking_panel.turn_right_button.clicked[bool].connect(self._take_steps)
        
        
    def _set_initial_values(self):       
        # initial step parameters
        self.walking_panel.num_steps_edit.setText(str(self._num_steps))
        self.walking_panel.step_length_edit.setText(str(self._step_length))
        self.walking_panel.side_step_edit.setText(str(self._side_step_length))
        self.walking_panel.angle_edit.setText(str(self._step_angle))
        self.walking_panel.step_time_edit.setText(str(self._step_time))

    def update(self):
        if self.isVisible():
            self._set_size_of_widgets()   
            
            show_animation = False
            
            joints = []
            for i in range(1, self._noBoxes + 1):
                self._box_widgets[i].ui.frame = self.rviz_frames[i]
                self._box_widgets[i].ui.display.layout().addWidget(self._box_widgets[i].ui.frame)
                self._hide_all_joint_axes(self._box_widgets[i].ui.frame)
                if self._box_widgets[i].ui.enable_rviz_check.isChecked():
                    self._box_widgets[i].ui.enable_rviz_check.setChecked(False)
                    self._box_widgets[i].ui.enable_rviz_check.setChecked(True)
                
                
                if self._box_widgets[i].ui.show_animation_check.isChecked():
                    show_animation = True
                    self._box_widgets[i].ui.show_animation_check.setChecked(False)
                    self._box_widgets[i].ui.show_animation_check.setChecked(True)
                    
            if not show_animation:
                self.show_animation.emit(False)
                
        else:
            for i in range(1, self._noBoxes + 1):
                if self._box_widgets[i].ui.frame != None:
                    self._rviz_set_update_interval(self._box_widgets[i].ui.frame, 0)
                    self._box_widgets[i].ui.frame = None
            
        
    def _set_size_of_widgets(self):
        width = self.parent().width()
        height = self.parent().height()
        
        max_boxes = self.max_joints
        
        if max_boxes != -1:
            if max_boxes == self._noBoxes:
                max_boxes += 1 # to account for walking panel

            left, top, right, bottom = self.page_layout.getContentsMargins()
            spacing = self.page_layout.spacing()

            box_width = int((width - spacing * max_boxes - left - right)/max_boxes)

            width_left = width - left - right

            for i in range(1, self._noBoxes + 1):
                self._box_widgets[i].setFixedSize(box_width, height - top - bottom)
                
                display = self._box_widgets[i].ui.display
                pixmap = self._box_widgets[i].pixmap

                pixmap = self._resize_with_aspect_ratio(pixmap, display.width(), display.height())
                
                self._box_widgets[i].ui.display.layout().widget(0).setFixedSize(pixmap.width(), pixmap.height())
                self._box_widgets[i].ui.display.layout().widget(0).setPixmap(pixmap)
                
                width_left -= (box_width + spacing)
                
            self.walking_panel.setFixedSize(box_width, height - top - bottom)
            width_left -= (box_width + spacing)

            self.spacer.changeSize(width_left + spacing, 20)


#_______ button functions _________________________________________________________________________   

    def _take_steps(self):
        command = self.sender().text().lower()
        
        params = {'command': command}
        params['num_steps'] = self._num_steps
        params['step_time'] = self._step_time
        params['step_length'] = self._step_length
        params['side_step_length'] = self._side_step_length
        params['step_angle'] = self._step_angle
        
        self.footstep_parameters.emit(params)
        
        
#_______ edit functions ___________________________________________________________________________              
            
    def _handle_num_steps_edit(self):
        valid, value = self._string_to_int(self.walking_panel.num_steps_edit.text())
        
        if valid:
            self._num_steps = value
        else:
            print('Please enter a valid integer for the desired number of steps.')
            self.walking_panel.num_steps_edit.setText(str(self._num_steps))

    def _handle_step_length_edit(self):
        valid, value = self._string_to_float(self.walking_panel.step_length_edit.text())
        
        if valid:
            self._step_length = value
        else:
            print('Please enter a valid float for the desired step length.')
            self.walking_panel.step_length_edit.setText(str(self._step_length))
            
    def _handle_side_step_edit(self):       
        valid, value = self._string_to_float(self.walking_panel.side_step_edit.text())
        
        if valid:
            self._side_step_length = value
        else:
            print('Please enter a valid float for the desired side step length.')
            self.walking_panel.side_step_edit.setText(str(self._side_step_length))
    
    def _handle_angle_edit(self):
        valid, value = self._string_to_float(self.walking_panel.angle_edit.text())
        
        if valid:
            self._step_angle = math.radians(value)
        else:
            print('Please enter a valid float for the desired side step angle.')
            self.walking_panel.angle_edit.setText(str(self._step_angle))
    
    def _handle_step_time_edit(self):
        valid, value = self._string_to_float(self.walking_panel.step_time_edit.text())
        
        if valid:
            self._step_time = value
        else:
            print('Please enter a valid float for the desired step time.')
            self.walking_panel.step_time_edit.setText(str(self._step_time))
        
