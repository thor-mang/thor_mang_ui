#!/usr/bin/env python

import os

import rospy
import rospkg
import actionlib

import math

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFrame

#from rosparam import load_file, upload_params
#from yaml import load, dump

from robotis_controller_msgs.msg import SyncWriteItem

from thormang3_foot_step_generator.msg import FootStepCommand

from thormang3_manipulation_module_msgs.msg import JointPose
from std_msgs.msg import String, Bool

from calibration_page import CalibrationPage

class WalkingCalibrationPage(CalibrationPage):
    _id = -1
    
    _noBoxes = -1
    
    _initial_increment = 0.1
    
    _num_steps = 1
    _step_length = 0.1
    _side_step_length = 0.0
    _step_angle = 0.0
    _step_time = 1.0

    def __init__(self, id, ui_name, wizard = None):
        super(WalkingCalibrationPage, self).__init__(id, ui_name, wizard)
        
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'ui', 'walking_panel.ui')
        self.walking_panel = QWidget()
        loadUi(ui_file, self.walking_panel, {'QWidget': QWidget})
        self.walking_panel.setDisabled(True)
        
        self.page_layout.addWidget(self.walking_panel, 0, 2)
        
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
        
        self._wizard.walking_module_enable_radio_button.toggled[bool].connect(lambda: self._wizard._handle_walking_module_button(True))
        self._wizard.walking_module_disable_radio_button.toggled[bool].connect(lambda: self._wizard._handle_walking_module_button(False))
        
        self._wizard.walking_module_disable_radio_button.setChecked(True)
        
        
    def _set_initial_values(self):       
        # initial step parameters
        self.walking_panel.num_steps_edit.setText(str(self._num_steps))
        self.walking_panel.step_length_edit.setText(str(self._step_length))
        self.walking_panel.side_step_edit.setText(str(self._side_step_length))
        self.walking_panel.angle_edit.setText(str(self._step_angle))
        self.walking_panel.step_time_edit.setText(str(self._step_time))

    def _update(self):
        if self.isVisible():
            self._set_help_text()
            self._hide_buttons()
            self._update_pages_list()
             
            self._set_size_of_widgets()   
                
            joints = []
            for i in range(1, self._noBoxes + 1):
                self._boxes_parts[i]['frame'] = self._wizard.rviz_frames[i]
                self._boxes_parts[i]['display'].layout().addWidget(self._boxes_parts[i]['frame'])
                self._hide_all_joint_axes(self._boxes_parts[i]['frame'])
                if self._boxes_parts[i]['enable_frame'].isChecked():
                    self._handle_rviz_check(i)
                
        else:
            for i in range(1, self._noBoxes + 1):
                if self._boxes_parts[i]['frame'] != None:
                    self._rviz_set_update_interval(self._boxes_parts[i]['frame'], 0)
                    self._boxes_parts[i]['frame'] = None
            

    def _hide_buttons(self):
        self._wizard.finish_button.setVisible(False)
        
    def _set_size_of_widgets(self):
        width = self.parent().width()
        height = self.parent().height()
        
        max_boxes = self._wizard.max_joints
        
        if max_boxes != -1:
        
            if max_boxes == self._noBoxes:
                max_boxes += 1 # to account for walking panel
        
            line_width = 0
            
            if self._lines != {}:
                line_width = self._lines[1].width()
            
            left, top, right, bottom = self.page_layout.getContentsMargins()
            spacing = self.page_layout.spacing()

            box_width = int((width - spacing * max_boxes - left - right)/max_boxes)

            width_left = width - left - right

            for i in range(1, self._noBoxes + 1):
                self._box_widgets[i].setFixedSize(box_width, height - top - bottom)
                
                display = self._boxes_parts[i]['display']
                pixmap = self._boxes_parts[i]['pixmap']

                pixmap = self._resize_with_aspect_ratio(pixmap, display.width(), display.height())
                
                self._boxes_parts[i]['display'].layout().widget(0).setFixedSize(pixmap.width(), pixmap.height())
                self._boxes_parts[i]['display'].layout().widget(0).setPixmap(pixmap)
                
                width_left -= (box_width + spacing)

            self.spacer.changeSize(width_left + spacing, 20)


#_______ button functions _________________________________________________________________________   

    def _take_steps(self):
        if self._wizard.walking_module_on:
            msg = FootStepCommand()
            command = self.sender().text().lower()
            
            self._wizard.publish_footstep_command(command, self._num_steps, self._step_time,
                                    self._step_length, self._side_step_length, self._step_angle)
        else:
            print('Turn walking module on before attempting to take steps.')
            
        
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
        
