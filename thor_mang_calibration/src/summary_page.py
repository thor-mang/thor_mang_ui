#!/usr/bin/env python

import copy
import os

import rospkg

from yaml import load

from python_qt_binding.QtWidgets import QLabel, QLineEdit, QSpacerItem, QSizePolicy, QFrame

from page import Page

from tab_widget import TabWidget

class SummaryPage(Page):
    def __init__(self, id, ui_name, wizard = None):
        super(SummaryPage, self).__init__(id, ui_name, wizard)    

        self._id = id
        
        self._lineEdits_old_calibration = {}
        self._lineEdits_new_calibration = {}

        self._overview_tab = TabWidget(['Arms', 'Legs', 'Other'])

        self._add_labels_and_edits_to_overview_tab()

        self.verticalLayout.insertWidget(1, self._overview_tab)

        # connect signals
        self.save_button.toggled[bool].connect(self._handle_button_set)
        self.reset_button.toggled[bool].connect(self._handle_button_set)
        self.nosave_button.toggled[bool].connect(self._handle_button_set)
        
        
    def _update(self):
        if self.isVisible():
            self._set_help_text()
            self._hide_buttons()
            self._update_pages_list()    
        
            # get old parameters in order to check for changes
            rp = rospkg.RosPack()
            offset_path = os.path.join(rp.get_path('thormang3_manager'), 'config', 'offset.yaml')
            f = open(offset_path, 'r')
            yamlfile = load(f)
            f.close()
            old_calibration = yamlfile["offset"]
            
            # grab new parameters
            new_calibration = self._wizard.configuration_client.get_configuration()
            
            self._check_and_set_changes(old_calibration, new_calibration)
            
        
    def _check_and_set_changes(self, old_calibration, new_calibration):
        #joints = old_calibration.keys()
        
        for i in range(self._overview_tab.count()):
            self._overview_tab.uncolorTab(i)

        for key in self._lineEdits_old_calibration.keys():
            arm, leg, left, right = self._position_from_joint_name(key)
            self._lineEdits_old_calibration[key].setText(str(old_calibration[key]))
            # float equality check
            if abs(abs(old_calibration[key]) - abs(new_calibration[key])) >= 1e-8:
                self._lineEdits_new_calibration[key].setText(str(new_calibration[key]))
                self._lineEdits_new_calibration[key].setEnabled(True)
                self._lineEdits_new_calibration[key].setStyleSheet("color: red;")
                
                self._lineEdits_old_calibration[key].setDisabled(True)
                
                if arm:
                    self._overview_tab.colorTab(0)
                elif leg:
                    self._overview_tab.colorTab(1)
                else:
                    self._overview_tab.colorTab(2)
                
                #self._lineEdits_new_calibration[key].parent().parent().parent().setStyleSheet("color: red;")
            else:
                self._lineEdits_old_calibration[key].setEnabled(True)
                
                self._lineEdits_new_calibration[key].setText('')
                self._lineEdits_new_calibration[key].setDisabled(True)
                


#_____________________________ ui changes _________________________________________________________

    def _hide_buttons(self):
        self._wizard.header_widget.setVisible(False)
        self._wizard.line_4.setVisible(False)
        self._wizard.next_button.setVisible(False)

    def _add_labels_and_edits_to_overview_tab(self):
        arm_counter_left = 1
        arm_counter_right = 1
        leg_counter_left = 1
        leg_counter_right = 1
        other_counter = 1
        
        self._overview_tab.widget(0).layout().addWidget(QLabel('<b>Left:</b>'), 0, 0)
        self._overview_tab.widget(0).layout().addWidget(QLabel('Old Values:'), 0, 1)
        self._overview_tab.widget(0).layout().addWidget(QLabel('New Values:'), 0, 3)
        self._overview_tab.widget(0).layout().addWidget(QLabel('<b>Right:</b>'), 0, 5)
        self._overview_tab.widget(0).layout().addWidget(QLabel('Old Values:'), 0, 6)
        self._overview_tab.widget(0).layout().addWidget(QLabel('New Values:'), 0, 8)
        
        self._overview_tab.widget(1).layout().addWidget(QLabel('<b>Left:</b>'), 0, 0)
        self._overview_tab.widget(1).layout().addWidget(QLabel('Old Values:'), 0, 1)
        self._overview_tab.widget(1).layout().addWidget(QLabel('New Values:'), 0, 3)
        self._overview_tab.widget(1).layout().addWidget(QLabel('<b>Right:</b>'), 0, 5)
        self._overview_tab.widget(1).layout().addWidget(QLabel('Old Values:'), 0, 6)
        self._overview_tab.widget(1).layout().addWidget(QLabel('New Values:'), 0, 8)
        
        self._overview_tab.widget(2).layout().addWidget(QLabel('Old Values:'), 0, 1)
        self._overview_tab.widget(2).layout().addWidget(QLabel('New Values:'), 0, 3)
        
        
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
                
                edit_old = QLineEdit()
                edit_old.setMaximumWidth(80)
                edit_old.setReadOnly(True)
                
                edit_new = QLineEdit()
                edit_new.setMaximumWidth(80)
                edit_new.setReadOnly(True)
                
                self._lineEdits_old_calibration[name] = edit_old
                self._lineEdits_new_calibration[name] = edit_new
                
                if arm and left:
                    self._add_label_and_edit(label, edit_old, edit_new, 0, arm_counter_left, 0)
                    arm_counter_left += 1
                elif arm and right:
                    self._add_label_and_edit(label, edit_old, edit_new, 0, arm_counter_right, 5)
                    arm_counter_right += 1
                elif leg and left:
                    self._add_label_and_edit(label, edit_old, edit_new, 1, leg_counter_left, 0)
                    leg_counter_left += 1
                elif leg and right:
                    self._add_label_and_edit(label, edit_old, edit_new, 1, leg_counter_right, 5)
                    leg_counter_right += 1
                else:
                    self._add_label_and_edit(label, edit_old, edit_new, 2, other_counter, 0)
                    other_counter += 1
                    
            line = self._make_line(QFrame.HLine)
            
            if arm and left:
                self._overview_tab.widget(0).layout().addWidget(line, arm_counter_left, 0, 1, 4)
                arm_counter_left += 1
            elif arm and right:
                self._overview_tab.widget(0).layout().addWidget(line, arm_counter_right, 5, 1, 4)
                arm_counter_right += 1
            elif leg and left:
                self._overview_tab.widget(1).layout().addWidget(line, leg_counter_left, 0, 1, 4)
                leg_counter_left += 1
            elif leg and right:
                self._overview_tab.widget(1).layout().addWidget(line, leg_counter_right, 5, 1, 4)
                leg_counter_right += 1
            else:
                self._overview_tab.widget(2).layout().addWidget(line, other_counter, 0, 1, 4)
                other_counter += 1
        
        line = self._make_line(QFrame.VLine)
        self._overview_tab.widget(0).layout().addWidget(line, 0, 4, arm_counter_left, 1)
        line2 = self._make_line(QFrame.VLine)
        self._overview_tab.widget(1).layout().addWidget(line2, 0, 4, leg_counter_left, 1)
        
        self._add_spacers()

    def _add_label_and_edit(self, label, edit_old, edit_new, tab_num, row, column_start):
        self._overview_tab.widget(tab_num).layout().addWidget(label, row, column_start)
        self._overview_tab.widget(tab_num).layout().addWidget(edit_old, row, column_start + 1)
        self._overview_tab.widget(tab_num).layout().addWidget(QLabel('->'), row, column_start + 2)
        self._overview_tab.widget(tab_num).layout().addWidget(edit_new, row, column_start + 3)

    def _add_spacers(self):
        self._overview_tab.widget(0).layout().addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        self._overview_tab.widget(0).layout().addItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum), 0, 10)
        self._overview_tab.widget(1).layout().addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        self._overview_tab.widget(1).layout().addItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum), 0, 10)
        self._overview_tab.widget(2).layout().addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        self._overview_tab.widget(2).layout().addItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum), 0, 10)


#_______ button functions _________________________________________________________________________  
            
    def _handle_button_set(self):
        if self.save_button.isChecked():
            self._wizard.save = True
            self._wizard.reset = False
            self._wizard.no_save = False
        elif self.reset_button.isChecked():
            self._wizard.save = False
            self._wizard.reset = True
            self._wizard.no_save = False
        elif self.nosave_button.isChecked():
            self._wizard.save = False
            self._wizard.reset = False
            self._wizard.no_save = True

        
