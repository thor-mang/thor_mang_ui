#!/usr/bin/env python

import os

import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QRadioButton, QVBoxLayout, QLineEdit, QLabel, QGridLayout, QSpacerItem, QSizePolicy, QFrame
from python_qt_binding.QtGui import QValidator, QDoubleValidator

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

from yaml import load, dump

from page import Page

import copy
import math

class PosePage(Page):

    _radioButtons = []

    def __init__(self, id, ui_name, wizard = None):
        super(PosePage, self).__init__(id, ui_name, wizard)    

        self._id = id
        
        rp = rospkg.RosPack()
        poses_path = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'config', 'calibration_poses.yaml')  
        f = open(poses_path, 'r')
        self.poses = load(f)
        f.close()
              
        self._buttonLayout = QVBoxLayout()

        self._add_labels_and_edits_to_tab()

        self._add_pose_radio_buttons()

        for edit in self._lineEdits.values():
            edit.setDisabled(True)
            
        # connect signals
        self.reload_file.clicked[bool].connect(self._handle_reload_file_button)
        
        # for functionality of a custom pose setting
        v = QDoubleValidator()
        for edit in self._lineEdits.values():
            edit.setValidator(v)
            edit.editingFinished.connect(self._handle_edits)
            
        # have a button selected by default
        self._radioButtons[0].setChecked(True)
        
                    
    def _update(self):
        if self.isVisible():
            self._set_help_text()
            self._hide_buttons()
            self._update_pages_list()
            
            self.verticalLayout.insertWidget(0, self._wizard.rviz_frame_1)
            
            self._wizard.rviz_frame_1.getManager().getRootDisplayGroup().getDisplayAt(1).setValue(True)
            self._wizard.rviz_frame_1.setVisible(True)
            
            self._focus_rviz_view_on_joints(self._wizard.rviz_frame_1, '', 'pelvis')
            self._set_all_alpha_to_one()
            self._set_rviz_view(self._wizard.rviz_frame_1, 'Front View')
            self._hide_all_joint_axes(self._wizard.rviz_frame_1)
            
            self._wizard.show_turning_dir_pub.publish(False)


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
        
        keys = []
        
        for o in self._wizard.page_config['order']:
            for key in self._wizard.page_config.keys():
                if str(key).lower().find(str(o)) != -1:
                    keys.append(key)
                    
        for key in keys:
            joint = self._wizard.page_config[key]
                
            arm, leg, left, right = self._position_from_key(key)
            
            for i in range(1, 1 + int(joint['num_joints'])):
                label_text = self._get_label_text(key, joint[i]['rotation'])
                
                label = QLabel(label_text)
                edit = QLineEdit()
                edit.setMaximumWidth(80)
                
                self._lineEdits[joint[i]['name']] = edit
                
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
        self.customTab.widget(0).layout().addWidget(line, 0, 3, arm_counter_left, 1)
        line2 = self._make_line(QFrame.VLine)
        self.customTab.widget(1).layout().addWidget(line2, 0, 3, leg_counter_left, 1)
        
        self._add_spacers()
        
    def _get_label_text(self, key, rotation_axis):
        label_text = copy.deepcopy(key)
        label_text = label_text.replace('l_', '')
        label_text = label_text.replace('L_', '')
        label_text = label_text.replace('r_', '')
        label_text = label_text.replace('R_', '')
        label_text = self._joint_designation_to_name(label_text)
        label_text = label_text + ' ' + rotation_axis.capitalize() + ':'
        #label_text = label_text.capitalize()
        
        return label_text
        

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
    

    def _add_pose_radio_buttons(self):
        # for functionality of a custom pose setting
        self.poses['Custom'] = copy.deepcopy(self.poses[self.poses.keys()[0]])

        i = 0

        for key in sorted(self.poses.keys()):
            self._radioButtons.append(QRadioButton(str(key)))
            self._buttonLayout.addWidget(self._radioButtons[i])
            self._radioButtons[i].toggled.connect(self._handle_radioButtons)
            i += 1

        self._buttonLayout.addStretch()
        self.pose_box.setLayout(self._buttonLayout)
        
#_______ rviz functions ___________________________________________________________________________

    def _set_all_alpha_to_one(self):
        robot_display = self._get_robot_state_display(self._wizard.rviz_frame_1)
        
        links = robot_display.subProp('Links')
        
        for i in range(0, links.numChildren()):
            # a link has several properties, using this to exclude options
            if links.childAt(i).getName().find('link') != -1:        
                links.childAt(i).subProp('Alpha').setValue(1.0)
        

#_______ button functions _________________________________________________________________________

    def _handle_reload_file_button(self):
        currently_checked = ''
        for button in self._radioButtons:
            if button.isChecked():
                currently_checked = button.text()

        self._radioButtons = []

        while self._buttonLayout.count():
            child = self._buttonLayout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()
        
        self._add_pose_radio_buttons()

        something_checked = False
        for button in self._radioButtons:
            if button.text() == currently_checked:
                button.setChecked(True)
                something_checked = True
                
        if something_checked == False and len(self._radioButtons) != 0:
            self._radioButtons[0].setChecked(True)

    def _handle_radioButtons(self):
        for button in self._radioButtons:
            if button.isChecked() and button.text() == self.sender().text():
                for edit in self._lineEdits.values():
                    edit.setDisabled(True)
                for key in self.poses[button.text()].keys():
                    self._lineEdits[key].setText(str(self.poses[button.text()][key]))

                self._wizard.pose = copy.deepcopy(self.poses[button.text()])

                # for functionality of a custom pose setting
                if button.text() == 'Custom':
                    for edit in self._lineEdits.values():
                        edit.setEnabled(True)
                        
                self._send_preview_pose_to_joint_state_publisher()   
                return # no need to check further buttons

    # for functionality of a custom pose setting       
    def _handle_edits(self):
        #if self._radioButtons[0].isChecked():
        for key in self._lineEdits.keys():
            edit = self._lineEdits[key]
            edit.clearFocus()
            
            editString = edit.text()
            valid, value = self._string_to_float(editString)
            
            if valid == True:
                self.poses['Custom'][key] = value
            else:
                edit.setText(self.poses['Custom'][key])
                
        self._wizard.pose = copy.deepcopy(self.poses['Custom'])
        self._send_preview_pose_to_joint_state_publisher()
        
# ____________________ message function _______________________________________________________________________________

    def _send_preview_pose_to_joint_state_publisher(self):
        msg = self._generate_joint_state_message()
        self._wizard.preview_pose_pub.publish(msg)

    def _generate_joint_state_message(self):
        msg = JointState()
        
        for joint in self._wizard.pose:
            msg.name.append(joint)           
            msg.position.append(math.radians(self._wizard.pose[joint]))
            
        return msg
