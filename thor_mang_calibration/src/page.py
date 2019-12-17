#!/usr/bin/env python

import os

import rospy
import rospkg

import rviz

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QObject
from python_qt_binding.QtGui import QValidator, QDoubleValidator, QIntValidator
from python_qt_binding.QtWidgets import QWidget, QFrame, QLayout

from std_msgs.msg import String


class Page(QWidget):

    def __init__(self, id, ui_name, wizard = None):
        super(Page, self).__init__(wizard) 
        
        self._id = id
        self._wizard = wizard
        
        #self.ui = QWidget()
        
        # load wizard ui - header line and page change buttons
        if ui_name != '':
            rp = rospkg.RosPack()
            ui_file = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'ui', ui_name)
            loadUi(ui_file, self, {'QWidget': QWidget})
        
        self._wizard.pages.currentChanged[int].connect(self._update)
        
            
#________ common functions ________________________________________________________________________

    def _update_pages_list(self):
        path = self._wizard.path
        current = 0
        
        for page in path:
            if page == self._id:
                self._wizard.page_list.setCurrentIndex(current)
                return
            current += 1
            
        return -1
        
    def _sort_joints_by_group(self):
        groups = {}
        for g in self._wizard.page_config['groups']:
            groups[g] = []
        
        
        for key in self._wizard.page_config.keys():
            if key == 'Walking Calibration':
                continue
            entry = self._wizard.page_config[key]
            if type(entry) == dict and 'num_joints' in entry.keys():
                for i in range(1, self._wizard.page_config[key]['num_joints'] + 1):
                    joint_name = self._wizard.page_config[key][i]['name']
                    for group_name in self._wizard.page_config['groups']:
                        if str(joint_name).lower().find(str(group_name)) != -1:
                            groups[group_name] = groups[group_name] + [joint_name]
                            break
        
        return groups
       
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

    def _set_help_text(self):
        help_text = ''
        if 'page_help_text' in self._wizard.page_config[self._id]:
            help_text = self._wizard.page_config[self._id]['page_help_text']
        self._wizard.page_help_text_label.setText(help_text)

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
    
