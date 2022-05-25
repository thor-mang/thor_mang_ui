#!/usr/bin/env python

import os

import math

import rospy

from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtGui import QPixmap, QResizeEvent
from python_qt_binding.QtWidgets import QStackedLayout, QLabel, QFrame, QSizePolicy, QSpacerItem, QMainWindow, QGridLayout, QWidget

from page import Page

from calibration_box import CalibrationBox


class CalibrationPage(Page):
    update_joint_offset = pyqtSignal(str, float)
    all_rviz_frames_active = pyqtSignal(bool)
    animated_joint = pyqtSignal(str)
    
    _id = -1
    _noBoxes = -1

    def __init__(self, page_id, config, offset_limits, rviz_frames):
        super(CalibrationPage, self).__init__(page_id, config)

        self.max_joints = -1
        
        self.rviz_frames = rviz_frames
        
        self.offset_mins = offset_limits['mins']
        self.offset_maxs = offset_limits['maxs']
        
        self.old_offsets = self.get_old_offset_parameters()

        self.page_layout = QGridLayout()
        self.setLayout(self.page_layout)

        self._box_widgets = {}

        self._setup()
        
        for i in range(1, self._noBoxes + 1):
            continue
            self._box_widgets[i].ui.display.setEnabled(True)

    def _setup(self):
        self._noBoxes = self._config['num_joints']
        
        item_count = 0
        
        for i in range(1, self._noBoxes + 1):
            box_config = self._config[i]
            
            joint = box_config['name']
            old_offset = self.old_offsets[joint]
            offset = None
            
            # get current offset value
            if self._check_joint_exists_in_ns(joint):
                offset = rospy.get_param(self._joint_offset_ns + joint)
            else:
                print "Error: Parameter " + joint + " not found."
            
            box = CalibrationBox(i, box_config, offset, old_offset)
            
            self._box_widgets[i] = box
            
            
            self.page_layout.addWidget(box.ui, 0, item_count)
            item_count += 1
            
            # connect signals
            box.ui.offset.textChanged.connect(lambda state, x=i: self._offset_changed(x))
            box.adjust_rviz.connect(self._handle_adjust_rviz)
            box.show_axes.connect(self._handle_show_axes)
            box.show_animation.connect(self._handle_show_animation)
         
        self.spacer = QSpacerItem(1, 40, QSizePolicy.Minimum, QSizePolicy.Minimum)
        self.page_layout.addItem(self.spacer, 0, item_count)

       
    def update(self):
        if self.isVisible():                 
            self._set_size_of_widgets()

            show_animation = False
            
            for i in range(1, self._noBoxes + 1):
                self._box_widgets[i].ui.frame = self.rviz_frames[i]
                self._box_widgets[i].ui.display.layout().addWidget(self._box_widgets[i].ui.frame)
                self._hide_all_joint_axes(self._box_widgets[i].ui.frame)
                if self._box_widgets[i].ui.enable_rviz_check.isChecked():
                    self._box_widgets[i].ui.enable_rviz_check.setChecked(False)
                    self._box_widgets[i].ui.enable_rviz_check.setChecked(True)
                if self._box_widgets[i].ui.show_animation_check.isChecked():
                    show_animation = True
                    self._box_widgets[i].ui.show_animation_check.setChecked(False)
                    self._box_widgets[i].ui.show_animation_check.setChecked(True)
                    
            if not show_animation:
                self.show_animation.emit(False)
            
        else:
            for i in range(1, self._noBoxes + 1):
                if self._box_widgets[i].ui.frame != None:
                    self._rviz_set_update_interval(self._box_widgets[i].ui.frame, 0)
                    self._box_widgets[i].ui.frame = None
            
                
    def _set_size_of_widgets(self):
        width = self.parent().width()
        height = self.parent().height()
        
        max_boxes = self.max_joints
        
        if max_boxes != -1:
        
            left, top, right, bottom = self.page_layout.getContentsMargins()
            spacing = self.page_layout.spacing()

            box_width = int((width - spacing * (max_boxes) - left - right)/max_boxes)

            width_left = width - left - right

            for i in range(1, self._noBoxes + 1):
                self._box_widgets[i].setFixedSize(box_width, height - top - bottom)
                
                display = self._box_widgets[i].ui.display
                pixmap = self._box_widgets[i].pixmap
                pixmap = self._resize_with_aspect_ratio(pixmap, display.width(), display.height())
                
                self._box_widgets[i].ui.display.layout().widget(0).setFixedSize(pixmap.width(), pixmap.height())
                self._box_widgets[i].ui.display.layout().widget(0).setPixmap(pixmap)
                
                width_left -= (box_width + spacing)

            self.spacer.changeSize(width_left + spacing, 20)
                
                
    def resizeEvent(self, resizeEvent):
        self._set_size_of_widgets()
        QMainWindow.resizeEvent(self, resizeEvent)

#_______ ui functions _________________________________________________________________________________________________        
            
    def _offset_changed(self, box_num):
        box = self._box_widgets[box_num]

        outside_range_str = ('Range of joint offset of ' + str(box.joint) + ': [' + 
                             str(self.offset_mins[box.joint]) + ', ' + 
                             str(self.offset_maxs[box.joint]) + '].')
        if box.offset > self.offset_maxs[box.joint]:
            box.offset = self.offset_maxs[box.joint]
            print(outside_range_str)
        elif box.offset < self.offset_mins[box.joint]:
            box.offset = self.offset_mins[box.joint]
            print(outside_range_str)
                
        box.ui.offset.setText(str(box.offset))
        self.update_joint_offset.emit(box.joint, box.offset)
        
    def _handle_adjust_rviz(self, box_num, show):   
        box = self._box_widgets[box_num]
        previous_link = ''
        
        if 'previous' in self._config[box_num].keys():
            previous_link = self._config[box_num]['previous']
        else:
            previous_link = self._config[box_num - 1]['name']
        
        if show:
            link = box.joint
            if 'link name' in self._config[box_num].keys():
                link = self._config[box_num]['link name']
            
            self._set_rviz_view(box.ui.frame, self._config[box_num]['rviz view'])
            self._focus_rviz_view_on_links(box.ui.frame, link, previous_link)
            
        else:
            self.all_rviz_frames_active.emit(False)
            
        if self._all_frames_enabled():
            self.all_rviz_frames_active.emit(True)
        
        
    def _handle_all_rviz_check(self, no_boxes, checked):         
        for i in range(1, no_boxes + 1):
            self._box_widgets[i].ui.enable_rviz_check.setChecked(checked)
            
        
    def _handle_show_axes(self, box_num, checked, link):
        box = self._box_widgets[box_num]

        self._rviz_view_show_joints_axes(box.ui.frame, link, checked)
        
    def _handle_show_animation(self, box_num, checked, joint):
        if checked:
            for i in range(1, self._noBoxes + 1):
                if not i == box_num and self._box_widgets[i].ui.frame != None:
                    self._box_widgets[i].ui.show_animation_check.setChecked(False)
                    self._rviz_set_update_interval(self._box_widgets[i].ui.frame, 1e100)
                    
            self.animated_joint.emit(joint)
            self.show_animation.emit(True)
            self._rviz_set_update_interval(self._box_widgets[box_num].ui.frame, 0)
        else:
            self.show_animation.emit(False)
            
            animated = False
            for i in range(1, self._noBoxes + 1):
                if self._box_widgets[i].ui.show_animation_check.isChecked():
                    animated = True
            
            if not animated:
                for i in range(1, self._noBoxes + 1):
                    if self._box_widgets[i].ui.frame != None:
                        self._rviz_set_update_interval(self._box_widgets[i].ui.frame, 0)
                
            
        
# _______________________ helper functions ________________________________________________________

    def _check_joint_exists_in_ns(self, joint):
        if rospy.has_param(self._joint_offset_ns + joint):
            return True
    
        return False
    
    def _all_frames_enabled(self):
        for i in range(1, self._noBoxes + 1):
            if self._box_widgets[i].ui.enable_rviz_check.isChecked() == False:
                return False        
                
        return True

    def _resize_with_aspect_ratio(self, pixmap, w, h):
        pixmap_2 = ''
        if w < h:
            pixmap_2 = pixmap.scaledToWidth(w, mode=1)
            h_pix = pixmap_2.height()
            if h_pix > h:
                pixmap_2 = pixmap.scaledToHeight(h, mode=1)
        else:
            pixmap_2 = pixmap.scaledToHeight(h, mode=1)
            w_pix = pixmap_2.width()
            if w_pix > w:
                pixmap_2 = pixmap.scaledToWidth(w, mode=1)
                
        return pixmap_2
