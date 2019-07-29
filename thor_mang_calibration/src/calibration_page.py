#!/usr/bin/env python

import os

import math

import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QPixmap, QResizeEvent
from python_qt_binding.QtWidgets import QStackedLayout, QLabel, QFrame, QSizePolicy, QSpacerItem, QMainWindow, QGridLayout, QWidget

from page import Page

from calibration_box import CalibrationBox

class CalibrationPage(Page):
    _id = -1

    _initial_increment = 0.1

    _noBoxes = 3

    def __init__(self, id, ui_name, wizard = None):
        super(CalibrationPage, self).__init__(id, '', wizard)
        # grab id
        self._id = id  
        
        self.page_layout = QGridLayout()
        self.setLayout(self.page_layout)
        # assembling ui parts & related variables into dicts
        #self._box1 = {
        #    'joint': '', 'inc_val': self._initial_increment, 'angle_val': None, # variables
        #    'frame': None, 'enable_frame': self.box1_enable_rviz_check, 'show_axes': self.box1_show_axes_check, 
        #    'show_animation': self.box1_show_animation_check, # rviz (including line above)
        #    'inc_ui': self.box1_incSize, 'angle_ui': self.box1_angle, 'tip': self.box1_tip, # ui edits
        #    'layout_ui': self.box1_layout, 'box': self.box1, 'display': self.box1_display, # other ui parts
        #    'inc_button': self.box1_inc, 'dec_button': self.box1_dec#, 'reset_button': self.box1_reset_button # ui buttons
        #    }
        #self._box2 = {
        #    'joint': '', 'inc_val': self._initial_increment, 'angle_val': None, 
        #    'frame': None, 'enable_frame': self.box2_enable_rviz_check, 'show_axes': self.box2_show_axes_check,
        #    'show_animation': self.box2_show_animation_check, 
        #    'inc_ui': self.box2_incSize, 'angle_ui': self.box2_angle, 'tip': self.box2_tip, 
        #    'layout_ui': self.box2_layout, 'box': self.box2, 'display': self.box2_display, 
        #    'inc_button': self.box2_inc, 'dec_button': self.box2_dec#, 'reset_button': self.box2_reset_button
        #    }
        #self._box3 = {
        #    'joint': '', 'inc_val': self._initial_increment, 'angle_val': None, 
        #    'frame': None, 'enable_frame': self.box3_enable_rviz_check, 'show_axes': self.box3_show_axes_check, 
        #    'show_animation': self.box3_show_animation_check,
        #    'inc_ui': self.box3_incSize, 'angle_ui': self.box3_angle, 'tip': self.box3_tip, 
        #    'layout_ui': self.box3_layout, 'box': self.box3, 'display': self.box3_display, 
        #    'inc_button': self.box3_inc, 'dec_button': self.box3_dec#, 'reset_button': self.box3_reset_button 
        #    }

        self._box_widgets = {}
        self._boxes_parts = {}#{1: self._box1, 2: self._box2, 3: self._box3}
        self._lines = {}#{2: self.line_1, 3: self.line_2}        

        self._setup()
        #self._set_size_of_widgets() 
        
        #self.resize(self.width(), self.height())
        
        for i in range(1, self._noBoxes + 1):
            self._boxes_parts[i]['display'].setEnabled(True)

    def _setup(self):
        info = self._wizard.page_config[str(self._id)]
        
        id = str(self._id)
        
        self._noBoxes = info['num_joints']

        #self.page_label.setText(self._joint_designation_to_name(id))
        
        rp = rospkg.RosPack()
        
        line_count = 1
        item_count = 0
        
        for i in range(1, self._noBoxes + 1):
            cal_box = CalibrationBox()
            self._box_widgets[i] = cal_box
            
            self._boxes_parts[i] = self._find_box_parts(cal_box)#self._boxes_parts[i]
            if i > 1:
                line = self._make_line(QFrame.VLine)
                #self._lines[line_count] = line
                #self.page_layout.addWidget(line, 0, item_count)
                #line_count += 1
                #item_count += 1
            self.page_layout.addWidget(cal_box, 0, item_count)
            item_count += 1
            
            box = self._boxes_parts[i]
            
            # get joint designation
            box['joint'] = info[i]['name']
            
            # get current calibration value
            if rospy.has_param('/johnny5/joint_offsets/' + box['joint']):
                box['angle_val'] = rospy.get_param('/johnny5/joint_offsets/' + box['joint'])
            else:
                print "Error: Parameter " + box['joint'] + " not found."
                box['angle_val'] = ''

            # set initial values
            box['box'].setTitle(str(box['joint']))#self._joint_designation_to_name(id) + ' ' + str(info[i]['rotation']).capitalize() + ":")
            if 'joint_specific_help' in info[i]:
                box['tip'].setText(str(info[i]['joint_specific_help']))
            box['inc_ui'].setText(str(box['inc_val']))
            box['angle_ui'].setText(str(box['angle_val']))
            
            # get image
            pix = QPixmap(os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'img', str(info[i]['pic'])))
            img = QLabel()
            img.setScaledContents(True)
            img.setPixmap(pix)
            
            # prepare image/rviz presentation
            layout = QStackedLayout()
            layout.addWidget(img)
            box['display'].setLayout(layout)
            
            # connect signals
            box['inc_ui'].editingFinished.connect(lambda state, x=i: self._handle_incSize(x))
            box['inc_button'].clicked[bool].connect(lambda state, x=i: self._handle_button_inc(x))
            box['dec_button'].clicked[bool].connect(lambda state, x=i: self._handle_button_dec(x))
            box['enable_frame'].toggled.connect(lambda state, x=i: self._handle_rviz_check(x))
            box['show_axes'].toggled.connect(lambda state, x=i: self._handle_show_axes_check(x))
            box['show_animation'].toggled.connect(lambda state, x=i: self._handle_show_animation_check(x))
            box['reset_button'].clicked[bool].connect(lambda state, x=i: self._handle_reset_button(x))
        
        self.spacer = QSpacerItem(1, 40, QSizePolicy.Minimum, QSizePolicy.Minimum)
        self.page_layout.addItem(self.spacer, 0, item_count)
        # close unneeded parts of the ui
        #for i in reversed(range(self._noBoxes + 1, 4)):
        #    self._boxes_parts[i]['box'].close()
        #    self._lines[i].close()
        
        
    def _update(self):
        if self.isVisible():                 
            self._set_help_text()
            self._hide_buttons()        
            self._update_pages_list()
            
            show_animation = False
            
            for i in range(1, self._noBoxes + 1):
                self._boxes_parts[i]['frame'] = self._wizard.rviz_frames[i]
                self._boxes_parts[i]['display'].layout().addWidget(self._boxes_parts[i]['frame'])
                self._hide_all_joint_axes(self._boxes_parts[i]['frame'])
                if self._boxes_parts[i]['enable_frame'].isChecked():
                    self._handle_rviz_check(i)
                if self._boxes_parts[i]['show_animation'].isChecked():
                    show_animation = True
                    self._handle_show_animation_check(i)
                    
            if not show_animation:
                self._wizard.show_turning_dir_pub.publish(False)
            
        else:
            #self._wizard.show_turning_dir_pub.publish(False)
            for i in range(1, self._noBoxes + 1):
                if self._boxes_parts[i]['frame'] != None:
                    self._rviz_set_update_interval(self._boxes_parts[i]['frame'], 0)
                    self._boxes_parts[i]['frame'] = None
            
            
    def _hide_buttons(self):
        self._wizard.walking_module_group.setVisible(False)
        self._wizard.line_2.setVisible(False)
        self._wizard.finish_button.setVisible(False)
        
    def _find_box_parts(self, box):
        parts = {
            'joint': '', 'inc_val': self._initial_increment, 'angle_val': None, # variables
            'frame': None, 'enable_frame': box.enable_rviz_check, 'show_axes': box.show_axes_check, 
            'show_animation': box.show_animation_check, # rviz (including line above)
            'inc_ui': box.incSize, 'angle_ui': box.angle, 'tip': box.tip, # ui edits
            'layout_ui': box.box_layout, 'box': box.box, 'display': box.display, # other ui parts
            'inc_button': box.inc, 'dec_button': box.dec, 'reset_button': box.reset_button # ui buttons
            }
        
        return parts        
        
    def _set_size_of_widgets(self):
        width = self.parent().width()
        height = self.parent().height()
        
        max_boxes = self._wizard.max_joints
        
        line_width = 0
        
        if self._lines != {}:
            line_width = self._lines[1].width()
        
        left, top, right, bottom = self.page_layout.getContentsMargins()
        spacing = self.page_layout.spacing()

        box_width = int((width - spacing * (max_boxes) - left - right)/max_boxes)

        width_left = width - left - right

        for i in range(1, self._noBoxes + 1):
            self._box_widgets[i].setFixedSize(box_width, height - top - bottom)
            width_left -= (box_width + spacing)
            #if i < self._noBoxes and self._lines != {}:
            #    width_left -= (line_width + spacing)

        self.spacer.changeSize(width_left + spacing, 20)
                
    def resizeEvent(self, resizeEvent):
        self._set_size_of_widgets()
        QMainWindow.resizeEvent(self, resizeEvent)

#_______ button functions _________________________________________________________________________        
            
    def _handle_incSize(self, boxNo):
        box = self._boxes_parts[boxNo]
        valid, value = self._string_to_float(box['inc_ui'].text())
        
        if valid:
            box['inc_val'] = value
        else:
            box['inc_ui'].setText(str(box['inc_val']))

    def _handle_button_inc(self, boxNo):
        box = self._boxes_parts[boxNo]
        box['angle_val'] += float(box['inc_val'])
        if box['angle_val'] > self._wizard.offset_maxs[box['joint']]:
            box['angle_val'] = self._wizard.offset_maxs[box['joint']]
            print('Maximum value for joint offset of ' + box['box'].title() + ' [' +
                str(self._wizard.offset_mins[box['joint']]) + ', ' + 
                str(self._wizard.offset_maxs[box['joint']]) + '].')
        box['angle_ui'].setText(str(box['angle_val']))
        self._wizard.configuration_client.update_configuration({box['joint']: box['angle_val']})

    def _handle_button_dec(self, boxNo):
        box = self._boxes_parts[boxNo]
        box['angle_val'] -= float(box['inc_val'])
        if box['angle_val'] < self._wizard.offset_mins[box['joint']]:
            box['angle_val'] = self._wizard.offset_mins[box['joint']]
            print('Minimum value for joint offset of ' + box['box'].title() + ' [' + 
                str(self._wizard.offset_mins[box['joint']]) + ', ' + 
                str(self._wizard.offset_maxs[box['joint']]) + '].')
        box['angle_ui'].setText(str(box['angle_val']))
        self._wizard.configuration_client.update_configuration({box['joint']: box['angle_val']})
        
    def _handle_reset_button(self, boxNo):
        box = self._boxes_parts[boxNo]
        joint = box['joint']
        old_offset = self._wizard.stored_offsets[joint]
        
        box['angle_val'] = old_offset
        box['angle_ui'].setText(str(old_offset))
        self._wizard.configuration_client.update_configuration({joint: old_offset})
        
    def _handle_rviz_check(self, boxNo):   
        box = self._boxes_parts[boxNo]
        info = self._wizard.page_config[str(self._id)]
        previous_link = ''
        
        if 'previous' in info[boxNo].keys():
            previous_link = info[boxNo]['previous']
        else:
            previous_link = info[boxNo - 1]['name']
        
        if box['enable_frame'].isChecked():
            box['frame'].getManager().getRootDisplayGroup().getDisplayAt(1).setValue(True)
            box['frame'].setVisible(True)
                
            box['display'].layout().setCurrentIndex(1)
            box['show_axes'].setEnabled(True)
            
            link = box['joint']
            if 'link name' in info[boxNo].keys():
                link = info[boxNo]['link name']
            
            self._set_rviz_view(box['frame'], info[boxNo]['rviz view'])
            self._focus_rviz_view_on_links(box['frame'], link, previous_link)
            
            
            self._handle_show_axes_check(boxNo)
            
        else:
            box['frame'].getManager().getRootDisplayGroup().getDisplayAt(1).setValue(False)
            box['display'].layout().setCurrentIndex(0)
            self._wizard.enable_all_rviz_check.setChecked(False)
            box['display'].setEnabled(True)
            
        if self._all_frames_enabled():
            self._wizard.enable_all_rviz_check.setChecked(True)
        
        
    def _handle_all_rviz_check(self, no_boxes):
        visible = False
            
        if self._wizard.enable_all_rviz_check.isChecked():
            visible =  True
            
        for i in range(1, no_boxes + 1):
            self._boxes_parts[i]['enable_frame'].setChecked(visible)
            
        
    def _handle_show_axes_check(self, boxNo):
        box = self._boxes_parts[boxNo]
        link = box['joint']
        if 'link name' in self._wizard.page_config[str(self._id)][boxNo].keys():
            link = self._wizard.page_config[str(self._id)][boxNo]['link name']
        self._rviz_view_show_joints_axes(box['frame'], link, box['show_axes'].isChecked())
        
    def _handle_show_animation_check(self, boxNo):
        if self._boxes_parts[boxNo]['show_animation'].isChecked():
            for i in range(1, self._noBoxes + 1):
                if not i == boxNo and self._boxes_parts[i]['frame'] != None:
                    self._boxes_parts[i]['show_animation'].setChecked(False)
                    self._rviz_set_update_interval(self._boxes_parts[i]['frame'], 1e100)
            self._publish_visualized_joint(self._boxes_parts[boxNo]['joint'])        
            self._wizard.show_turning_dir_pub.publish(True)
            self._rviz_set_update_interval(self._boxes_parts[boxNo]['frame'], 0)
        else:
            self._wizard.show_turning_dir_pub.publish(False)
            
            animated = False
            for i in range(1, self._noBoxes + 1):
                if self._boxes_parts[i]['show_animation'].isChecked():
                    animated = True
            
            if not animated:
                for i in range(1, self._noBoxes + 1):
                    if self._boxes_parts[i]['frame'] != None:
                        self._rviz_set_update_interval(self._boxes_parts[i]['frame'], 0)
                
            
        
# _______________________ helper functions ________________________________________________________

    def _all_frames_enabled(self):
        for i in range(1, self._noBoxes + 1):
            if self._boxes_parts[i]['enable_frame'].isChecked() == False:
                return False        
                
        return True


