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

        self._overview_tab = TabWidget()
        label_dicts = self._overview_tab.setup_tab_widget(self._wizard.joint_overview, 2, True)
        
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
                
            else:
                self._lineEdits_old_calibration[key].setEnabled(True)
                
                self._lineEdits_new_calibration[key].setText('')
                self._lineEdits_new_calibration[key].setDisabled(True)
                


#_____________________________ ui changes _________________________________________________________

    def _hide_buttons(self):
        self._wizard.header_widget.setVisible(False)
        self._wizard.line_4.setVisible(False)
        self._wizard.next_button.setVisible(False)

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

        
