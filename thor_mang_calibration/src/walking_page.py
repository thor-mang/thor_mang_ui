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
    
    _noBoxes = 2
    
    _initial_increment = 0.1
    
    _num_steps = 1
    _step_length = 0.1
    _side_step_length = 0.0
    _step_angle = 0.0
    _step_time = 1.0

    _walking_module_on = False
    _ini_pose_taken = False


    def __init__(self, id, ui_name, wizard = None):
        super(WalkingCalibrationPage, self).__init__(id, ui_name, wizard)
        
        line = self._make_line(QFrame.VLine)
        self.page_layout.addWidget(line, 0, 3)
        
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'ui', 'walking_panel.ui')
        self.walking_panel = QWidget()
        loadUi(ui_file, self.walking_panel, {'QWidget': QWidget})
        self.walking_panel.setDisabled(True)
        
        self.page_layout.addWidget(self.walking_panel, 0, 4)
        
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
        
        self._wizard.walking_module_enable_radio_button.toggled[bool].connect(lambda: self._handle_walking_module_button(True))
        self._wizard.walking_module_disable_radio_button.toggled[bool].connect(lambda: self._handle_walking_module_button(False))
        
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
                
            joints = []
            for i in range(1, self._noBoxes + 1):
                if self._boxes[i]['enable_frame'].isChecked():
                    self._handle_rviz_check(i)
                joints.append(self._boxes[i]['joint'])
                
        else:
            self._boxes[1]['frame'] = None
            self._boxes[2]['frame'] = None
            

    def _hide_buttons(self):
        self._wizard.finish_button.setVisible(False)


#_______ button functions _________________________________________________________________________   

    def _handle_take_initial_position(self):
        print('walking page handle take initial position')
        if self._wizard.torque_on:
            print('publishing ini_pose')
            self._wizard.ini_pose_pub.publish("ini_pose")
            self._ini_pose_taken = True
            
            self._wizard.walking_module_group.setEnabled(True)
        else:
            print('Turn torque on before moving the robot into position.')


    def _handle_walking_module_button(self, enable):
        if enable == True and self._walking_module_on == False:
            if self._ini_pose_taken:
                mode = String()
                mode.data = "walking_module"
                self._wizard.module_control_pub.publish(mode)
                self._walking_module_on = True
                
                self.walking_panel.setEnabled(True)
            else:
                print('Go to inital position before enabling the walking module.')
                self._wizard.walking_module_disable_radio_button.setChecked(True)
        if enable == False and self._walking_module_on == True:
            mode = String()
            mode.data = "none"
            self._wizard.module_control_pub.publish(mode)
            self._walking_module_on = False  
            
            self.walking_panel.setDisabled(True) 
        
        
    def _take_steps(self):
        if self._walking_module_on:
            msg = FootStepCommand()
            msg.command = self.sender().text().lower()
            msg.step_num = self._num_steps
            msg.step_time = self._step_time
            msg.step_length = self._step_length
            msg.side_step_length = self._side_step_length
            msg.step_angle_rad = self._step_angle
            self._wizard.walking_command_pub.publish(msg)
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
        
