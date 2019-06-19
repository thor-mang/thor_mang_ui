#!/usr/bin/env python

import os

import rospy
import rospkg
import actionlib

import std_msgs.msg
import dynamic_reconfigure.client

import math

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QObject, pyqtSignal
from python_qt_binding.QtGui import QColor, QFont, QPixmap, QValidator, QDoubleValidator, QIntValidator
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QWizard, QWizardPage, QLabel, QLineEdit, QLayout, QPushButton, QSpacerItem, QSizePolicy, QRadioButton, QGroupBox

from rosparam import load_file, upload_params
from yaml import load, dump

from shutil import copyfile

from robotis_controller_msgs.msg import SyncWriteItem

from thormang3_foot_step_generator.msg import FootStepCommand

from thormang3_walking_module_msgs.srv import SetBalanceParam, IsRunning, StartWalking, SetJointFeedBackGain
from thormang3_walking_module_msgs.msg import BalanceParam, JointFeedBackGain
from thormang3_manipulation_module_msgs.msg import JointPose
from std_msgs.msg import String, Bool

from page_enum import Pages

from page import Page

class WalkingCalibrationPage(Page):
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
        # grab id
        self._id = id  
        
        #self.joint_label.setText(self._joint_designation_to_name(self._id.name))
        
        self.box1_angle.setReadOnly(True)
        self.box2_angle.setReadOnly(True)        
    
        # set images
        rp = rospkg.RosPack()
        self.roll_img = QPixmap(os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'img', 'ankle_roll.png'))
        self.box1_img.setScaledContents(True)
        self.box1_img.setPixmap(self.roll_img)
        self.box2_img.setScaledContents(True)
        self.box2_img.setPixmap(self.roll_img)

        # assembling ui parts & related variables into dicts
        self._box1 = {
            'joint' : 'l_leg_an_r', 'inc_val' : self._initial_increment, 'angle_val' : None, # variables
            'frame' : None, 'enable_frame' : self.box1_enable_rviz_check, 'show_axes' : self.box1_show_axes_check, # rviz
            'inc_ui' : self.box1_incSize, 'angle_ui' : self.box1_angle, 'tip': self.box2_tip,
            'layout_ui' : self.box1_layout, 'img' : self.box1_img # ui parts
            }
        self._box2 = {
            'joint' : 'r_leg_an_r', 'inc_val' : self._initial_increment, 'angle_val' : None, 
            'frame' : None, 'enable_frame' : self.box2_enable_rviz_check, 'show_axes' : self.box2_show_axes_check, 
            'inc_ui' : self.box2_incSize, 'angle_ui' : self.box2_angle, 'tip': self.box2_tip,
            'layout_ui' : self.box2_layout, 'img' : self.box1_img
            }
        
        self._boxes = {1: self._box1, 2: self._box2}
        
        self._set_initial_values()

        # connect signals
        self.num_steps_edit.editingFinished.connect(self._handle_num_steps_edit)
        self.step_length_edit.editingFinished.connect(self._handle_step_length_edit)
        self.side_step_edit.editingFinished.connect(self._handle_side_step_edit)
        self.angle_edit.editingFinished.connect(self._handle_angle_edit)
        self.step_time_edit.editingFinished.connect(self._handle_step_time_edit)
        
        self.forward_button.clicked[bool].connect(self._take_steps)
        self.backward_button.clicked[bool].connect(self._take_steps)
        self.left_button.clicked[bool].connect(self._take_steps)
        self.right_button.clicked[bool].connect(self._take_steps)
        self.stop_button.clicked[bool].connect(self._take_steps)
        self.turn_left_button.clicked[bool].connect(self._take_steps)
        self.turn_right_button.clicked[bool].connect(self._take_steps)
        
        self.box1_incSize.editingFinished.connect(lambda: self._handle_incSize(1))
        self.box1_inc.clicked[bool].connect(lambda: self._handle_button_inc(1))
        self.box1_dec.clicked[bool].connect(lambda: self._handle_button_dec(1))
        self.box1_enable_rviz_check.stateChanged.connect(lambda: self._handle_rviz_check(1))
        self.box1_show_axes_check.stateChanged.connect(lambda: self._handle_show_axes_check(1))
        self.box2_incSize.editingFinished.connect(lambda: self._handle_incSize(2))
        self.box2_inc.clicked[bool].connect(lambda: self._handle_button_inc(2))
        self.box2_dec.clicked[bool].connect(lambda: self._handle_button_dec(2))
        self.box2_enable_rviz_check.stateChanged.connect(lambda: self._handle_rviz_check(2))
        self.box2_show_axes_check.stateChanged.connect(lambda: self._handle_show_axes_check(2))
        self._wizard.walking_module_enable_radio_button.toggled[bool].connect(lambda: self._handle_walking_module_button(True))
        self._wizard.walking_module_disable_radio_button.toggled[bool].connect(lambda: self._handle_walking_module_button(False))
        #self._wizard.take_position.clicked[bool].connect(self._handle_take_initial_position)
        
        self._wizard.walking_module_disable_radio_button.setChecked(True)
        
        
    def _set_initial_values(self):
        # initial offsets & increase sizes    
        for i in range(1, self._noBoxes + 1):
            box = self._boxes[i]
            if rospy.has_param('/johnny5/joint_offsets/' + box['joint']):
                box['angle_val'] = rospy.get_param('/johnny5/joint_offsets/' + box['joint'])
            else:
                print "Error: Parameter " + box['joint'] + " not found."
                box['angle_val'] = '' 
            box['angle_ui'].setText(str(box['angle_val']))
            box['inc_ui'].setText(str(box['inc_val']))
            
            if 'joint_specific_help' in self._wizard.page_config[str(self._id)][i]:
                box['tip'].setText(str(self._wizard.page_config[str(self._id)][i]['joint_specific_help']))
        
        # initial step parameters
        self.num_steps_edit.setText(str(self._num_steps))
        self.step_length_edit.setText(str(self._step_length))
        self.side_step_edit.setText(str(self._side_step_length))
        self.angle_edit.setText(str(self._step_angle))
        self.step_time_edit.setText(str(self._step_time))

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
            
            self._publish_visualized_joints(joints)  
            self._wizard.show_turning_dir_pub.publish(True)
                
        else:
            self._boxes[1]['frame'] = None
            self._boxes[2]['frame'] = None
            

    def _hide_buttons(self):
        self._wizard.finish_button.setVisible(False)


#_______ button functions _________________________________________________________________________   

    def _handle_incSize(self, boxNo):
        box = self._boxes[boxNo]
        valid, value = self._string_to_float(box['inc_ui'].text())
        
        if valid:
            box['inc_val'] = value
        else:
            box['inc_ui'].setText(str(box['inc_val']))
            

    def _handle_button_inc(self, boxNo):
        box = self._boxes[boxNo]
        box['angle_val'] += float(box['inc_val'])
        box['angle_ui'].setText(str(box['angle_val']))
        self._wizard.configuration_client.update_configuration({box['joint']: box['angle_val']})


    def _handle_button_dec(self, boxNo):
        box = self._boxes[boxNo]
        box['angle_val'] -= float(box['inc_val'])
        box['angle_ui'].setText(str(box['angle_val']))
        self._wizard.configuration_client.update_configuration({box['joint']: box['angle_val']})


    def _handle_take_initial_position(self):
        if self._wizard.torque_on:
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
                
                self.step_box.setEnabled(True)
            else:
                print('Go to inital position before enabling the walking module.')
                self._wizard.walking_module_disable_radio_button.setChecked(True)
        if enable == False and self._walking_module_on == True:
            mode = String()
            mode.data = "none"
            self._wizard.module_control_pub.publish(mode)
            self._walking_module_on = False  
            
            self.step_box.setDisabled(True) 
        
        
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
            
            
    def _handle_rviz_check(self, boxNo):   
        box = self._boxes[boxNo]
        previous_joint = ''
        
        if boxNo == 1:
            previous_joint = 'l_leg_an_p'
        else:
            previous_joint = 'r_leg_an_p'
        
        if box['enable_frame'].isChecked():
            if box['frame'] == None:
                box['frame'] = self._wizard.rviz_frames[boxNo]
                box['layout_ui'].addWidget(box['frame'], 1, 0, 1, 2)
                self._hide_all_joint_axes(box['frame'])
            
            box['frame'].getManager().getRootDisplayGroup().getDisplayAt(1).setValue(True)
            box['frame'].setVisible(True)
                
            box['img'].setVisible(False)
            box['show_axes'].setEnabled(True)
            
            self._focus_rviz_view_on_joints(box['frame'], box['joint'], previous_joint)
            self._set_rviz_view(box['frame'], 'Close Front View')
            
            self._handle_show_axes_check(boxNo)
            
        else:
            box['frame'].setVisible(False)
            box['frame'].getManager().getRootDisplayGroup().getDisplayAt(1).setValue(False)
            box['show_axes'].setDisabled(True)
            box['img'].setVisible(True)
            
            self._wizard.enable_all_rviz_check.setChecked(False)
            
        if self._all_frames_enabled():
            self._wizard.enable_all_rviz_check.setChecked(True)
            
            
    def _handle_all_rviz_check(self, no_boxes):
        visible = False
        
        if self._wizard.enable_all_rviz_check.isChecked():
            visible =  True
        
        for i in range(1, no_boxes + 1):
            self._boxes[i]['enable_frame'].setChecked(visible)
            
        
    def _handle_show_axes_check(self, boxNo):
        box = self._boxes[boxNo]
        self._rviz_view_show_joints_axes(box['frame'], box['joint'], box['show_axes'].isChecked())
        
        
    def _all_frames_enabled(self):
        for i in range(1, self._noBoxes + 1):
            if self._boxes[i]['enable_frame'].isChecked() == False:
                return False        
                
        return True
        
#_______ edit functions ___________________________________________________________________________              
            
    def _handle_num_steps_edit(self):
        valid, value = self._string_to_int(self.num_steps_edit.text())
        
        if valid:
            self._num_steps = value
        else:
            print('Please enter a valid integer for the desired number of steps.')
            self.num_steps_edit.setText(str(self._num_steps))

    def _handle_step_length_edit(self):
        valid, value = self._string_to_float(self.step_length_edit.text())
        
        if valid:
            self._step_length = value
        else:
            print('Please enter a valid float for the desired step length.')
            self.step_length_edit.setText(str(self._step_length))
            
    def _handle_side_step_edit(self):       
        valid, value = self._string_to_float(self.side_step_edit.text())
        
        if valid:
            self._side_step_length = value
        else:
            print('Please enter a valid float for the desired side step length.')
            self.side_step_edit.setText(str(self._side_step_length))
    
    def _handle_angle_edit(self):
        valid, value = self._string_to_float(self.angle_edit.text())
        
        if valid:
            self._step_angle = math.radians(value)
        else:
            print('Please enter a valid float for the desired side step angle.')
            self.angle_edit.setText(str(self._step_angle))
    
    def _handle_step_time_edit(self):
        valid, value = self._string_to_float(self.step_time_edit.text())
        
        if valid:
            self._step_time = value
        else:
            print('Please enter a valid float for the desired step time.')
            self.step_time_edit.setText(str(self._step_time))
        
