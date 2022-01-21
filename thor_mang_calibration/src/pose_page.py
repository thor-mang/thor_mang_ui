#!/usr/bin/env python

import os

import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QRadioButton, QVBoxLayout, QLineEdit, QLabel, QSpacerItem, QSizePolicy, QFrame
from python_qt_binding.QtGui import QValidator, QDoubleValidator
from python_qt_binding.QtCore import Qt, pyqtSignal

from sensor_msgs.msg import JointState

from yaml import load, dump

from page import Page

from tab_widget import TabWidget

import copy
import math

class PosePage(Page):
    pose_changed = pyqtSignal(dict)
    
    def __init__(self, page_id, config, joint_overview, joint_limits, rviz_frame):
        super(PosePage, self).__init__(page_id, config, 'pose_page.ui')    
        
        self.joint_limits = joint_limits
        
        self.rviz_frame = rviz_frame
        
        self._id = id
        
        self.poses = self._load_calibration_poses()
        
        self.customTab = self._setupCustomTab(joint_overview)

        self._add_poses_to_list()

        for edit in self._lineEdits.values():
            edit.setDisabled(True)
            
        # connect signals
        self.reload_file.clicked[bool].connect(self._handle_reload_file_button)
        self.pose_list.itemSelectionChanged.connect(self._handle_pose_list_changed)
        
        # for functionality of a custom pose setting
        v = QDoubleValidator()
        for key in self._lineEdits.keys():
            edit = self._lineEdits[key]
            edit.setValidator(v)
            #result =
            edit.editingFinished.connect(lambda sender_key=key: self._handle_edits(sender_key), 0x80)
            
        # have a button selected by default
        self.pose_list.setCurrentRow(0)

    def update(self):
        if self.isVisible():
            frame = self.rviz_frame
            self.preview_layout.insertWidget(0, frame)
            
            frame.getManager().getRootDisplayGroup().getDisplayAt(1).setValue(True)
            frame.setVisible(True)
            
            self._focus_rviz_view_on_links(frame, '', 'pelvis')
            self._set_all_alpha_to_one(frame)
            self._set_rviz_view(frame, 'Front View')
            self._hide_all_joint_axes(frame)
            
            self.show_animation.emit(False)

#_________ changes to ui __________________________________________________________________________

    def _setupCustomTab(self, joint_overview):
        self.customTab = TabWidget()
        label_dicts = self.customTab.setup_tab_widget(joint_overview, 1, False)
        self._lineEdits = label_dicts[0]
        
        self.customTab.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)
        
        self.page_layout.addWidget(self.customTab, 0, 1, Qt.AlignTop)
        
        for key in self._lineEdits.keys():
            edit = self._lineEdits[key]
            edit.setReadOnly(False)

    def _add_poses_to_list(self):
        for key in sorted(self.poses.keys()):
            self.pose_list.addItem(str(key))

#_______ button functions _________________________________________________________________________

    def _handle_reload_file_button(self):
        current_selection = self.pose_list.currentItem().text()
        self.poses = self._load_calibration_poses()
        
        self.pose_list.clear()
        self._add_poses_to_list()

        something_selected = False
        for i in range(self.pose_list.count()):
            if self.pose_list.item(i).text() == current_selection:
                self.pose_list.setCurrentRow(i)
                something_selected = True

        if not something_selected:
            self.pose_list.setCurrentRow(0)

    def _handle_pose_list_changed(self):
        current_selection = self.pose_list.currentItem()
        if current_selection == None:
            return
        current_selection = self.pose_list.currentItem().text()
        
        if current_selection != 'Custom':
            self._enable_line_edits(False)
        else:
            self._enable_line_edits(True)
            
        for key in self.poses[current_selection].keys():
            self._lineEdits[key].setText(str(self.poses[current_selection][key]))
            
            
        self.pose_changed.emit(self.poses[current_selection])

    # for functionality of a custom pose setting       
    def _handle_edits(self, sender_key):
        limits = self.joint_limits

        edit = self._lineEdits[sender_key]
        edit.clearFocus()

        editString = edit.text()
        valid, value = self._string_to_float(editString)
        if value != self.poses['Custom'][sender_key]:
            if valid:
                upper_limit = limits[sender_key]['max']
                lower_limit = limits[sender_key]['min']
                if math.radians(value) > upper_limit:
                    value = round(math.degrees(upper_limit), 2)
                    print(str(sender_key) + ' has an upper limit of ' + str(value) + ' degrees!')
                elif math.radians(value) < lower_limit:
                    value = round(math.degrees(lower_limit), 2)
                    print(str(sender_key) + ' has a lower limit of ' + str(value) + ' degrees!')
                self.poses['Custom'][sender_key] = value
            else:
                value = self.poses['Custom'][sender_key]

            edit.setText(str(value))
                
        self.pose_changed.emit(self.poses['Custom'])
        
# ____________________ helper functions ________________________________________________________________________________

    def _enable_line_edits(self, enabled):
            for edit in self._lineEdits.values():
                if enabled:
                    edit.setEnabled(True)
                else:
                    edit.setDisabled(True)

    def _load_calibration_poses(self):
        rp = rospkg.RosPack()
        poses_path = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'config', 'pose_config.yaml')  
        f = open(poses_path, 'r')
        poses = load(f)
        f.close()
        
        poses['Custom'] = copy.deepcopy(poses[poses.keys()[0]])
        
        return poses
