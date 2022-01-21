#!/usr/bin/env python

import os

import rospkg

from python_qt_binding.QtWidgets import QRadioButton, QVBoxLayout

from page import Page

class IntroPage(Page):

    def __init__(self, page_id, config, paths, rviz_frame):
        super(IntroPage, self).__init__(page_id, config, 'intro_page.ui')        
        
        self.rviz_frame = rviz_frame
        
        # add rviz frame to layout
        frame = self.rviz_frame
        self.page_layout.addWidget(frame, 1, 1)
        
        self.chosen_path = ''
        self.paths = paths
        keys = self.paths.keys()
        
        for key in sorted(keys):
            self.path_list.addItem(str(key))
        
        # first button as default calibration mode
        self.path_list.currentItemChanged.connect(self._handle_pages_list_changed)
        frame.getManager().getRootDisplayGroup().getDisplayAt(1).setValue(True)
        
        self.path_list.setCurrentRow(0)
        
        
    def _handle_pages_list_changed(self):
        self.chosen_path = self.path_list.currentItem().text()
        self._set_alpha(self.paths[self.chosen_path]['show'])
        
        
    def _set_alpha(self, part):
        frame = self.rviz_frame
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
                
                
    def update(self):
        if self.isVisible():
            frame = self.rviz_frame

            self.page_layout.addWidget(frame, 1, 1)
            frame.setVisible(True)
            frame.getManager().getRootDisplayGroup().getDisplayAt(1).setValue(True)
            self._set_rviz_view(frame, 'Front View')
            self._focus_rviz_view_on_links(frame, '', 'pelvis')
            self._hide_all_joint_axes(frame)
            
            if self.chosen_path != '':
                self._set_alpha(self.paths[self.chosen_path]['show'])
            else:
                self._set_alpha(self.paths[self.path_list.currentItem().text()]['show'])
                
            self.show_animation.emit(False)

            
    def get_chosen_path(self):
        return self.chosen_path
