#!/usr/bin/env python

import os

#import rospy
import rospkg

import rviz

#from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QRadioButton, QVBoxLayout
from python_qt_binding.QtGui import QPixmap, QColor
from python_qt_binding.QtCore import QVariant

from page_enum import Pages

from page import Page
from pose_page import PosePage
from calibration_page import CalibrationPage
from walking_page import WalkingCalibrationPage
from summary_page import SummaryPage

class IntroPage(Page):
    pix_full = 0
    pix_arm = 0
    pix_leg = 0

    def __init__(self, id, ui_name, wizard = None):
        super(IntroPage, self).__init__(id, ui_name, wizard)        
        
        # add rviz frame to layout
        self.gridLayout_2.addWidget(self._wizard.rviz_frame_1, 0, 1)
        
        # hide buttons not needed on this page
        self._hide_buttons()
        self._set_help_text()
        
        self._radioButtons = []
        self._buttonLayout = QVBoxLayout()
        self._add_path_radio_buttons()

        
        # first button as default calibration mode
        self._radioButtons[0].setChecked(True)
        
        self._wizard.rviz_frame_1.getManager().getRootDisplayGroup().getDisplayAt(1).setValue(True)
        

    def _hide_buttons(self):
        self._wizard.back_button.setVisible(False)
        self._wizard.finish_button.setVisible(False)
        self._wizard.page_list.setVisible(False)
        self._wizard.header_widget.setVisible(False)
        self._wizard.line_4.setVisible(False)
        
    def _add_path_radio_buttons(self):
        paths = self._wizard.paths
        
        keys = paths.keys()
        
        i = 0

        for key in sorted(keys):
            self._radioButtons.append(QRadioButton(str(key)))
            self._buttonLayout.addWidget(self._radioButtons[i])
            self._radioButtons[i].toggled.connect(self._handle_radioButtons)
            i += 1

        self._buttonLayout.addStretch()
        self.paths_groupBox.setLayout(self._buttonLayout)
            
    
    def _setup_pages(self):
        path_picked = self._get_current_button_text()
             
        if path_picked != self._wizard.path_name:
        
            pages = self._wizard.pages
        
            self._clear_pages()
            self._add_pages()
            
            self._wizard.page_list.clear()
            self._add_pages_to_list()
            
        
    def _clear_pages(self):
        pages = self._wizard.pages
        
        if pages.count() > 1:
            for i in reversed(range(2, pages.count() + 1)):
                pages.removeWidget(pages.widget(i))
    
    
    def _add_pages(self):
        pages = self._wizard.pages
        path = self._wizard.paths[self._get_current_button_text()]['path']
        self._wizard.path = ['Intro'] + path + ['Summary']
        
        i = 1
            
        for page in path:
            if str(page).find('Pose') != -1:
                pages.insertWidget(i, PosePage(str(page), 'pose_page.ui', self._wizard))
            elif str(page).find('Walking_Calibration') != -1:
                pages.insertWidget(i, WalkingCalibrationPage(str(page), 'walking_page.ui', self._wizard))
            else:
                pages.insertWidget(i, CalibrationPage(str(page), 'calibration_page.ui', self._wizard))   
            i += 1
                
        pages.insertWidget(i, SummaryPage('Summary', 'summary_page.ui', self._wizard))
        
    def _handle_radioButtons(self):
        paths = self._wizard.paths
        if self.sender().isChecked():
            sender_text = self.sender().text()
            self._set_alpha(paths[sender_text]['show'])
            
        
    def _set_alpha(self, part):
        robot_display = self._get_robot_state_display(self._wizard.rviz_frame_1)
         
        links = robot_display.subProp('Links')
                
        for i in range(0, links.numChildren()):
            # a link has several properties, using this to exclude options
            if links.childAt(i).getName().find('link') != -1:        
                link = links.childAt(i)
                if link.getName().find(part) != -1:
                    link.subProp('Alpha').setValue(1)
                else:
                    link.subProp('Alpha').setValue(0.25)
                
    def _update(self):
        if self.isVisible():
            self._set_help_text()
            self._hide_buttons()
        
            self._wizard.rviz_frame_1.setVisible(True)
            self._wizard.rviz_frame_1.getManager().getRootDisplayGroup().getDisplayAt(1).setValue(True)
            self.gridLayout_2.addWidget(self._wizard.rviz_frame_1, 0, 1)
            self._focus_rviz_view_on_joints(self._wizard.rviz_frame_1, '', 'pelvis')
            self._set_rviz_view(self._wizard.rviz_frame_1, 'Front View')
            self._hide_all_joint_axes(self._wizard.rviz_frame_1)
            
            if self._wizard.path_name != '':
                self._set_alpha(self._wizard.paths[self._wizard.path_name]['show'])
            else:
                self._set_alpha(self._wizard.paths[self._get_current_button_text()]['show'])
                
            self._wizard.show_turning_dir_pub.publish(False)
            
    def _get_current_button_text(self):
        
        for button in self._radioButtons:
            if button.isChecked():
                return button.text()
                
