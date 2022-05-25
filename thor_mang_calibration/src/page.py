#!/usr/bin/env python

import os

import rospy
import rospkg

from yaml import load

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QObject, pyqtSignal
from python_qt_binding.QtGui import QValidator, QDoubleValidator, QIntValidator
from python_qt_binding.QtWidgets import QWidget, QFrame, QLayout


class Page(QWidget):
    show_animation = pyqtSignal(bool)

    def __init__(self, page_id, config, ui_name = None):
        super(Page, self).__init__() 
        
        self._page_id = page_id
        self._config = config
        
        # joint offset namespace
        self._joint_offset_ns = str(rospy.get_namespace()) + 'joint_offsets/'
        
        # load wizard ui - header line and page change buttons
        if ui_name != None:
            rp = rospkg.RosPack()
            ui_file = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'ui', ui_name)
            loadUi(ui_file, self, {'QWidget': QWidget})
        
            
#________ common functions ________________________________________________________________________

    def update(self):
        # empty function, should be implemented for each page type
        pass
            
    def _position_from_joint_name(self, key):
            
        left = False
        right = False
        arm = False
        leg = False
        
        if str(key).lower().find('arm') != -1: 
            arm = True
        elif str(key).lower().find('leg') != -1: 
            leg = True
         
        if str(key).lower().find('l_') == 0: 
            left = True
        elif str(key).lower().find('r_') == 0: 
            right = True
        
        
        return arm, leg, left, right
        
    def _make_line(self, frame_shape):
        line = QFrame()
        line.setFrameShape(frame_shape)
        line.setFrameShadow(QFrame.Sunken)
                
        return line


    def get_old_offset_parameters(self):
        # get old parameters in order to check for changes
        rp = rospkg.RosPack()
        offset_path = os.path.join(rp.get_path('thormang3_manager'), 'config', 'offset.yaml')
        f = open(offset_path, 'r')
        yamlfile = load(f)
        f.close()
        old_offsets = yamlfile["offset"]
        
        return old_offsets

#________ rviz functions __________________________________________________________________________  
        
    def _set_rviz_view(self, rviz_frame, view_name):
        view_manager = rviz_frame.getManager().getViewManager()
        for i in range(0, view_manager.getNumViews()):
            if view_manager.getViewAt(i).getName() == view_name:
                view_manager.setCurrentFrom(view_manager.getViewAt(i))
                return
                
        print('Error: view \'' + view_name + '\' not found! View not changed.')
        
        
    def _get_robot_state_display(self, rviz_frame):
        root_display_group = rviz_frame.getManager().getRootDisplayGroup()
        
        for i in range(0, root_display_group.numDisplays()):
            if root_display_group.getDisplayAt(i).getClassId() == 'rviz/RobotModel':
                return root_display_group.getDisplayAt(i)
                
        
    def _focus_rviz_view_on_links(self, rviz_frame, joint, previous_joint):
        joint = joint + '_link'
        previous_joint = previous_joint + '_link'
    
        visualization_manager = rviz_frame.getManager()
        visualization_manager.setFixedFrame('calibration/' + previous_joint)
        visualization_manager.getViewManager().getCurrent().subProp('Target Frame').setValue("calibration/" + joint)
        visualization_manager.getViewManager().getCurrent().subProp('Focal Point').setValue("0; 0; 0")
        
        robot_model = self._get_robot_state_display(rviz_frame)
        
        links = robot_model.subProp('Links')
        
        for i in range(0, links.numChildren()):
            link = links.childAt(i)
            # if the child is actually a link property
            if link.getName().find('link') != -1:
                link_name = link.getName()
                if link_name != (joint) and link_name != (previous_joint):
                    link.subProp('Alpha').setValue(0.1)
                else:
                    link.subProp('Alpha').setValue(1.0)


    def _set_all_alpha_to_one(self, frame):
        robot_display = self._get_robot_state_display(frame)
        
        links = robot_display.subProp('Links')
        
        for i in range(0, links.numChildren()):
            # a link has several properties, using this to exclude options
            if links.childAt(i).getName().find('link') != -1:        
                links.childAt(i).subProp('Alpha').setValue(1.0)
        
        
    def _rviz_view_set_distance(self, rviz_frame, distance):
        visualization_manager = rviz_frame.getManager()
        visualization_manager.getViewManager().getCurrent().subProp('Distance').setValue(distance)
    
    def _rviz_set_update_interval(self, rviz_frame, tf_prefix):
        robot_model = self._get_robot_state_display(rviz_frame)
        robot_model.subProp('Update Interval').setValue(tf_prefix)
        
    def _rviz_view_show_joints_axes(self, rviz_frame, joint, value):
        robot_model = self._get_robot_state_display(rviz_frame)
        robot_model.subProp('Links').subProp(joint + '_link').subProp('Show Axes').setValue(value)
        
    def _hide_all_joint_axes(self, rviz_frame):
        robot_model = self._get_robot_state_display(rviz_frame)
        links = robot_model.subProp('Links')
        
        for i in range(0, links.numChildren()):
            link = links.childAt(i)
            # if the child is actually a link property
            if link.getName().find('link') != -1:
                link.subProp('Show Axes').setValue(False) 
        
        
#________ helper functions ________________________________________________________________________            
                       
    def _string_to_float(self, text):
        valid = False
        value = -1
        
        validator = QDoubleValidator()
        result,_,_ = validator.validate(text, 5)
        if result == QValidator.Acceptable:
            text = text.replace(',', '.')
            valid = True
            value = float(text)
        
        return valid, value
            

    def _string_to_int(self, text):
        valid = False
        value = -1
        
        validator = QIntValidator()
        result,_,_ = validator.validate(text, 5)
        if result == QValidator.Acceptable:
            valid = True
            value = int(text)
        
        return valid, value
    
