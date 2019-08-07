#!/usr/bin/env python

import os

import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QRadioButton, QVBoxLayout, QLineEdit, QLabel, QSpacerItem, QSizePolicy, QFrame
from python_qt_binding.QtGui import QValidator, QDoubleValidator

from sensor_msgs.msg import JointState

from yaml import load, dump

from page import Page

import copy
import math

class PosePage(Page):

    def __init__(self, id, ui_name, wizard = None):
        super(PosePage, self).__init__(id, ui_name, wizard)    

        self._id = id
        
        self.poses = self._load_calibration_poses()
        
        #self._radioButtons = []
              
        #self._buttonLayout = QVBoxLayout()

        self._add_labels_and_edits_to_tab()

        #self._add_pose_radio_buttons()

        self._add_poses_to_list()

        for edit in self._lineEdits.values():
            edit.setDisabled(True)
            
        # connect signals
        self.reload_file.clicked[bool].connect(self._handle_reload_file_button)
        self.pose_list.itemSelectionChanged.connect(self._handle_pages_list_changed)
        
        # for functionality of a custom pose setting
        v = QDoubleValidator()
        for edit in self._lineEdits.values():
            edit.setValidator(v)
            result = edit.editingFinished.connect(self._handle_edits, 0x80)
            
        # have a button selected by default
        self.pose_list.setCurrentRow(0)
        
                    
    def _update(self):
        if self.isVisible():
            self._set_help_text()
            self._hide_buttons()
            self._update_pages_list()
            
            frame = self._wizard.rviz_frames[1]
            self.verticalLayout.insertWidget(0, frame)
            
            frame.getManager().getRootDisplayGroup().getDisplayAt(1).setValue(True)
            frame.setVisible(True)
            
            self._focus_rviz_view_on_links(frame, '', 'pelvis')
            self._set_all_alpha_to_one()
            self._set_rviz_view(frame, 'Front View')
            self._hide_all_joint_axes(frame)
            
            self._wizard.publish_turning_direction(False)


#_________ changes to ui __________________________________________________________________________

    def _hide_buttons(self):
        self._wizard.walking_module_group.setVisible(False)
        self._wizard.line_2.setVisible(False)
        self._wizard.enable_all_rviz_check.setVisible(False)
        self._wizard.line_3.setVisible(False)
        self._wizard.finish_button.setVisible(False)
        
    def _add_labels_and_edits_to_tab(self):
        self._lineEdits = {}

        arm_counter_left = 1
        arm_counter_right = 1
        leg_counter_left = 1
        leg_counter_right = 1
        other_counter = 0
        
        self.customTab.widget(0).layout().addWidget(QLabel('<b>Left [deg]:</b>'), 0, 0)
        self.customTab.widget(0).layout().addWidget(QLabel('<b>Right [deg]:</b>'), 0, 4)
        self.customTab.widget(1).layout().addWidget(QLabel('<b>Left [deg]:</b>'), 0, 0)
        self.customTab.widget(1).layout().addWidget(QLabel('<b>Right [deg]:</b>'), 0, 4)
        
        groups = self._sort_joints_by_group()
                                       
        for g in self._wizard.page_config['groups']:
            #joint = self._wizard.page_config[key]
            group = groups[g]
            arm, leg, left, right = False, False, False, False
            for i in range(len(group)):
                name = group[i]
                arm, leg, left, right = self._position_from_joint_name(name)
                
                label = QLabel(str(group[i]))
                edit = QLineEdit()
                edit.setMaximumWidth(80)
                
                self._lineEdits[name] = edit
                
                if arm and left:
                    self._add_label_and_edit(label, edit, 0, arm_counter_left, 0)
                    arm_counter_left += 1
                elif arm and right:
                    self._add_label_and_edit(label, edit, 0, arm_counter_right, 4)
                    arm_counter_right += 1
                elif leg and left:
                    self._add_label_and_edit(label, edit, 1, leg_counter_left, 0)
                    leg_counter_left += 1
                elif leg and right:
                    self._add_label_and_edit(label, edit, 1, leg_counter_right, 4)
                    leg_counter_right += 1
                else:
                    self._add_label_and_edit(label, edit, 2, other_counter, 0)
                    other_counter += 1
                    
            line = self._make_line(QFrame.HLine)
            
            if arm and left:
                self.customTab.widget(0).layout().addWidget(line, arm_counter_left, 0, 1, 2)
                arm_counter_left += 1
            elif arm and right:
                self.customTab.widget(0).layout().addWidget(line, arm_counter_right, 4, 1, 2)
                arm_counter_right += 1
            elif leg and left:
                self.customTab.widget(1).layout().addWidget(line, leg_counter_left, 0, 1, 2)
                leg_counter_left += 1
            elif leg and right:
                self.customTab.widget(1).layout().addWidget(line, leg_counter_right, 4, 1, 2)
                leg_counter_right += 1
            else:
                self.customTab.widget(2).layout().addWidget(line, other_counter, 0, 1, 2)
                other_counter += 1
        
        line = self._make_line(QFrame.VLine)
        self.customTab.widget(0).layout().addWidget(line, 0, 3, arm_counter_left, 2)
        line2 = self._make_line(QFrame.VLine)
        self.customTab.widget(1).layout().addWidget(line2, 0, 3, leg_counter_left, 2)
        
        self._add_spacers()

    def _add_label_and_edit(self, label, edit, tab_num, row, column_start):
        self.customTab.widget(tab_num).layout().addWidget(label, row, column_start)
        self.customTab.widget(tab_num).layout().addWidget(edit, row, column_start + 1)

    def _add_spacers(self):
        self.customTab.widget(0).layout().addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        self.customTab.widget(0).layout().addItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum), 0, 6)
        self.customTab.widget(1).layout().addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        self.customTab.widget(1).layout().addItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum), 0, 6)
        self.customTab.widget(2).layout().addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        self.customTab.widget(2).layout().addItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum), 0, 6)
    
    def _add_poses_to_list(self):
        for key in sorted(self.poses.keys()):
            self.pose_list.addItem(str(key))
        
#_______ rviz functions ___________________________________________________________________________

    def _set_all_alpha_to_one(self):
        robot_display = self._get_robot_state_display(self._wizard.rviz_frames[1])
        
        links = robot_display.subProp('Links')
        
        for i in range(0, links.numChildren()):
            # a link has several properties, using this to exclude options
            if links.childAt(i).getName().find('link') != -1:        
                links.childAt(i).subProp('Alpha').setValue(1.0)
        

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

    def _handle_pages_list_changed(self):
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
            
        self._wizard.pose = copy.deepcopy(self.poses[current_selection])
        self._wizard.publish_preview_pose()

    # for functionality of a custom pose setting       
    def _handle_edits(self):
        limits = self._wizard.joint_limits

        for key in self._lineEdits.keys():
            edit = self._lineEdits[key]
            edit.clearFocus()
            
            editString = edit.text()
            valid, value = self._string_to_float(editString)
            if value != self.poses['Custom'][key]:
                if valid == True:
                    upper_limit = limits[key]['max']
                    lower_limit = limits[key]['min']
                    if math.radians(value) > upper_limit:
                        value = round(math.degrees(upper_limit), 2)
                        print(str(key) + ' has an upper limit of ' + str(value) + ' degrees!')
                    elif math.radians(value) < lower_limit:
                        value = round(math.degrees(lower_limit), 2)
                        print(str(key) + ' has a lower limit of ' + str(value) + ' degrees!')
                    self.poses['Custom'][key] = value
                else:
                    value = self.poses['Custom'][key]
                    
                edit.setText(str(value))
                
        self._wizard.pose = copy.deepcopy(self.poses['Custom'])
        self._wizard.publish_preview_pose()
        
# ____________________ helper functions ________________________________________________________________________________

    def _enable_line_edits(self, enabled):
            for edit in self._lineEdits.values():
                if enabled:
                    edit.setEnabled(True)
                else:
                    edit.setDisabled(True)

    def _load_calibration_poses(self):
        rp = rospkg.RosPack()
        poses_path = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'config', 'calibration_poses.yaml')  
        f = open(poses_path, 'r')
        poses = load(f)
        f.close()
        
        poses['Custom'] = copy.deepcopy(poses[poses.keys()[0]])
        
        return poses
