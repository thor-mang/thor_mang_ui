#!/usr/bin/env python

import os

import rospkg

from python_qt_binding.QtWidgets import QRadioButton, QVBoxLayout

from page import Page

class IntroPage(Page):
    pix_full = 0
    pix_arm = 0
    pix_leg = 0

    def __init__(self, id, ui_name, wizard = None):
        super(IntroPage, self).__init__(id, ui_name, wizard)        
        
        # add rviz frame to layout
        frame = self._wizard.rviz_frames[1]
        self.gridLayout_2.addWidget(frame, 1, 1)
        
        # hide buttons not needed on this page
        self._hide_buttons()
        self._set_help_text()
        
        paths = self._wizard.paths        
        keys = paths.keys()
        
        for key in sorted(keys):
            self.paths_list.addItem(str(key))
        
        # first button as default calibration mode
        self.paths_list.currentItemChanged.connect(self._handle_pages_list_changed)
        frame.getManager().getRootDisplayGroup().getDisplayAt(1).setValue(True)
        
        self.paths_list.setCurrentRow(0)
        
    def _hide_buttons(self):
        self._wizard.back_button.setVisible(False)
        self._wizard.finish_button.setVisible(False)
        self._wizard.page_list.setVisible(False)
        self._wizard.header_widget.setVisible(False)
        self._wizard.line_4.setVisible(False)
        
    def _handle_pages_list_changed(self):
        paths = self._wizard.paths
        self._set_alpha(paths[self.paths_list.currentItem().text()]['show'])
        
    def _set_alpha(self, part):
        frame = self._wizard.rviz_frames[1]
        robot_display = self._get_robot_state_display(frame)
         
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
            
            frame = self._wizard.rviz_frames[1]

            self.gridLayout_2.addWidget(frame, 1, 1)
            frame.setVisible(True)
            frame.getManager().getRootDisplayGroup().getDisplayAt(1).setValue(True)
            self._set_rviz_view(frame, 'Front View')
            self._focus_rviz_view_on_links(frame, '', 'pelvis')
            self._hide_all_joint_axes(frame)
            
            if self._wizard.path_name != '':
                self._set_alpha(self._wizard.paths[self._wizard.path_name]['show'])
            else:
                self._set_alpha(self._wizard.paths[self.paths_list.currentItem().text()]['show'])
                
            self._wizard.show_turning_dir_pub.publish(False)
            
    def get_chosen_path(self):
        return self.paths_list.currentItem().text()
            
