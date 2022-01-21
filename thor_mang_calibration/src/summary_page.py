#!/usr/bin/env python

import copy
import os

import rospy

from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtWidgets import QLabel, QLineEdit, QSpacerItem, QSizePolicy, QFrame

from page import Page

from tab_widget import TabWidget


class SummaryPage(Page):
    save_state = pyqtSignal(int)    
    
    def __init__(self, page_id, config, joint_overview):
        super(SummaryPage, self).__init__(page_id, config, 'summary_page.ui')    
        
        self._id = id
        
        self._lineEdits_old_calibration = {}
        self._lineEdits_new_calibration = {}

        self._overview_tab = TabWidget()
        label_dicts = self._overview_tab.setup_tab_widget(joint_overview, 2, True)
        
        self._lineEdits_old_calibration = label_dicts[0]
        self._lineEdits_new_calibration = label_dicts[1]

        for key in self._lineEdits_old_calibration.keys():
            edit1 = self._lineEdits_old_calibration[key]
            edit2 = self._lineEdits_new_calibration[key]
            edit1.setReadOnly(True)
            edit2.setReadOnly(True)

        self.page_layout.insertWidget(1, self._overview_tab)

        # connect signals
        self.save_button.toggled[bool].connect(self._handle_button_set)
        self.reset_button.toggled[bool].connect(self._handle_button_set)
        self.nosave_button.toggled[bool].connect(self._handle_button_set)

    def update(self):
        if self.isVisible():
            # get old parameters in order to check for changes
            old_calibration = self.get_old_offset_parameters()
            # get new parameters
            new_calibration = rospy.get_param(self._joint_offset_ns)
            # compare old and new parameters
            self._check_and_set_changes(old_calibration, new_calibration)

    def _check_and_set_changes(self, old_calibration, new_calibration):
        
        for i in range(self._overview_tab.count()):
            self._overview_tab.uncolorTab(i)

        for key in self._lineEdits_old_calibration.keys():
            arm, leg, left, right = self._position_from_joint_name(key)
            self._lineEdits_old_calibration[key].setText(str(round(old_calibration[key], 5))) # 5 decimal digits
            # float equality check
            if abs(abs(old_calibration[key]) - abs(new_calibration[key])) >= 1e-8:
                self._lineEdits_new_calibration[key].setText(str(round(new_calibration[key], 5))) # 5 decimal digits
                self._lineEdits_new_calibration[key].setEnabled(True)
                self._lineEdits_new_calibration[key].setStyleSheet("color: red;")
                
                self._lineEdits_old_calibration[key].setDisabled(True)
                
                if arm:
                    self._overview_tab.colorTab(0)
                elif leg:
                    self._overview_tab.colorTab(1)
                else:
                    self._overview_tab.colorTab(2)
                
            else:
                self._lineEdits_old_calibration[key].setEnabled(True)
                
                self._lineEdits_new_calibration[key].setText('')
                self._lineEdits_new_calibration[key].setDisabled(True)
                

#_______ button functions _________________________________________________________________________  
            
    def _handle_button_set(self):
        if self.save_button.isChecked():
            self.save_state.emit(0)
        elif self.reset_button.isChecked():
            self.save_state.emit(1)
        elif self.nosave_button.isChecked():
            self.save_state.emit(2)
