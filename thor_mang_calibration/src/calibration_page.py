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
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QWizard, QWizardPage, QLabel, QLineEdit, QLayout, QPushButton, QSpacerItem, QSizePolicy

from rosparam import load_file, upload_params
from yaml import load, dump

from shutil import copyfile

from thor_mang_control_msgs.msg import ChangeControlModeGoal
from robotis_controller_msgs.msg import SyncWriteItem
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String, Bool

from page_enum import Pages

import time

from page import Page

class CalibrationPage(Page):
    _id = -1

    _initial_increment = 0.1

    _noBoxes = 3

    def __init__(self, id, ui_name, wizard = None):
        super(CalibrationPage, self).__init__(id, ui_name, wizard)
        # grab id
        self._id = id  
        
        # assembling ui parts & related variables into dicts
        self._box1 = {
            'joint': '', 'inc_val': self._initial_increment, 'angle_val': None, # variables
            'frame': None, 'enable_frame': self.box1_enable_rviz_check, 'show_axes': self.box1_show_axes_check, # rviz
            'inc_ui': self.box1_incSize, 'angle_ui': self.box1_angle, 'tip': self.box1_tip, # ui edits
            'layout_ui': self.box1_layout, 'box': self.box1, 'img': self.box1_img, # other ui parts
            'inc_button': self.box1_inc, 'dec_button': self.box1_dec # ui buttons
            }
        self._box2 = {
            'joint': '', 'inc_val': self._initial_increment, 'angle_val': None, 
            'frame': None, 'enable_frame': self.box2_enable_rviz_check, 'show_axes': self.box2_show_axes_check, 
            'inc_ui': self.box2_incSize, 'angle_ui': self.box2_angle, 'tip': self.box2_tip, 
            'layout_ui': self.box2_layout, 'box': self.box2, 'img': self.box2_img, 
            'inc_button': self.box2_inc, 'dec_button': self.box2_dec 
            }
        self._box3 = {
            'joint': '', 'inc_val': self._initial_increment, 'angle_val': None, 
            'frame': None, 'enable_frame': self.box3_enable_rviz_check, 'show_axes': self.box3_show_axes_check, 
            'inc_ui': self.box3_incSize, 'angle_ui': self.box3_angle, 'tip': self.box3_tip, 
            'layout_ui': self.box3_layout, 'box': self.box3, 'img': self.box3_img, 
            'inc_button': self.box3_inc, 'dec_button': self.box3_dec 
            }

        self._boxes = {1: self._box1, 2: self._box2, 3: self._box3}
        self._lines = {2: self.line_1, 3: self.line_2}
    

        self._setup()


    def _setup(self):
        #self._add_pages_to_list()
    
        info = self._wizard.page_config[str(self._id)]
        
        id = str(self._id)
        
        self._noBoxes = info['num_joints']

        #self.page_label.setText(self._joint_designation_to_name(id))
        
        rp = rospkg.RosPack()
        
        for i in range(1, self._noBoxes + 1):
            box = self._boxes[i]
            
            # get joint designation
            box['joint'] = info[i]['name']
            
            # get current calibration value
            if rospy.has_param('/johnny5/joint_offsets/' + box['joint']):
                box['angle_val'] = rospy.get_param('/johnny5/joint_offsets/' + box['joint'])
            else:
                print "Error: Parameter " + box['joint'] + " not found."
                box['angle_val'] = ''

            # set initial values
            box['box'].setTitle(self._joint_designation_to_name(id) + ' ' + str(info[i]['rotation']).capitalize() + ":")
            if 'joint_specific_help' in info[i]:
                box['tip'].setText(str(info[i]['joint_specific_help']))
            box['inc_ui'].setText(str(box['inc_val']))
            box['angle_ui'].setText(str(box['angle_val']))
            
            # get image
            pix = QPixmap(os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'img', str(info[i]['pic'])))
            box['img'].setScaledContents(True)
            box['img'].setPixmap(pix)
            
            # connect signals
            box['inc_ui'].editingFinished.connect(lambda state, x=i: self._handle_incSize(x))
            box['inc_button'].clicked[bool].connect(lambda state, x=i: self._handle_button_inc(x))
            box['dec_button'].clicked[bool].connect(lambda state, x=i: self._handle_button_dec(x))
            box['enable_frame'].stateChanged.connect(lambda state, x=i: self._handle_rviz_check(x))
            box['show_axes'].stateChanged.connect(lambda state, x=i: self._handle_show_axes_check(x))
    
        # close unneeded parts of the ui
        for i in reversed(range(self._noBoxes + 1, 4)):
            self._boxes[i]['box'].close()
            self._lines[i].close()
        
        
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
            for i in range(1, self._noBoxes + 1):
                self._boxes[i]['frame'] = None
            
            
    def _hide_buttons(self):
        self._wizard.walking_module_group.setVisible(False)
        self._wizard.line_2.setVisible(False)
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
        
    def _handle_rviz_check(self, boxNo):   
        box = self._boxes[boxNo]
        info = self._wizard.page_config[str(self._id)]
        previous_joint = ''
        
        if boxNo == 1:
            previous_joint = info[1]['previous']
        else:
            previous_joint = info[boxNo - 1]['name']
        
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
            self._set_rviz_view(box['frame'], info[boxNo]['rviz view'])
            #self._rviz_view_set_distance(box['frame'], 0.25)
            
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
        
# _______________________ helper functions ________________________________________________________

    def _all_frames_enabled(self):
        for i in range(1, self._noBoxes + 1):
            if self._boxes[i]['enable_frame'].isChecked() == False:
                return False        
                
        return True


