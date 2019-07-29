#!/usr/bin/env python

import os

import rospy
import rospkg
import actionlib

import copy
import math

import rviz

import dynamic_reconfigure.client
from dynamic_reconfigure.msg import ConfigDescription

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from yaml import load
from shutil import copyfile

from robotis_controller_msgs.msg import SyncWriteItem
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from thormang3_foot_step_generator.msg import FootStepCommand
from thor_mang_control_msgs.msg import ChangeControlModeAction, ControlModeStatus, ChangeControlModeGoal
from std_msgs.msg import String, Bool, Int64
from sensor_msgs.msg import JointState

from intro_page import IntroPage
#from thor_mang_calibration.msg import Joints

class CalibrationDialog(Plugin):
    def __init__(self, context):
        super(CalibrationDialog, self).__init__(context)
        self.setObjectName('CalibrationDialog')

        self._parent = QWidget()
        self._wizard = CalibrationWizard(context)
        
    def shutdown_plugin(self):
        self._wizard.shutdown_plugin()


        
#__________________________________________________________________________________________________________________

class CalibrationWizard(QWidget):

    def __init__(self, context):
        super(CalibrationWizard, self).__init__()
        
        self.setObjectName('Calibration')
        self.setWindowTitle('ThorMangCalibration')
        
        print('Initializing calibration wizard...')

        if rospy.has_param('joint_offsets'):
            
            # load wizard ui - header line and page change buttons
            rp = rospkg.RosPack()
            ui_file = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'ui', 'wizard.ui')
            loadUi(ui_file, self, {'QWidget': QWidget})
            
            self._connect_ui_signals()
            
            self._setup_wizard_variables()
            
            # rviz frames used on multiple pages
            self.rviz_frame_1 = self._create_rviz_frame()
            self.rviz_frame_2 = self._create_rviz_frame()
            self.rviz_frame_3 = self._create_rviz_frame()
            
            self.rviz_frames = {1: self.rviz_frame_1, 2: self.rviz_frame_2, 3: self.rviz_frame_3}
            
            self.pages.insertWidget(0, IntroPage('Intro', 'intro_page.ui', self))#enum.value - 1, IntroPage(enum, 'intro_page.ui', self))
            self.pages.setCurrentIndex(0)

            # set window title
            if context.serial_number() > 1:
                self.setWindowTitle(self.windowTitle() + (' (%d)' % context.serial_number()))
                                    
            # add wizard to the context
            context.add_widget(self)

            self._create_publishers_and_clients()
            
            ns = rospy.get_namespace()
            self.sub = rospy.Subscriber(ns + 'joint_offsets/parameter_descriptions', ConfigDescription, self._parse_offset_config)
            
            msg = JointState()
        
            for joint in self.pose:
                msg.name.append(joint)           
                msg.position.append(math.radians(self.pose[joint]))
            
            self.preview_pose_pub.publish(msg)
            
            self.torque_off_radio_button.setChecked(True)
        else:
            print "Parameter Server not running. Shutting down."
            self.shutdown_plugin()
            
        print('Done!')
        
    def shutdown_plugin(self):
        print "Shutting down ..."
        if rospy.has_param('joint_offsets'):
            self.configuration_client.close()
            self.walking_command_pub.unregister()
            self.module_control_pub.unregister()
            self.sync_write_pub.unregister()
            #self.pose_pub.unregister()
            self.ini_pose_pub.unregister()
            self.allow_all_mode_transitions_pub.unregister()
            #self.set_control_mode_client.close()
            for controller in self.trajectory_controllers:
                controller.unregister()
            self.control_mode_status.unregister()
        print "Done!"


# __________________________ functions for initial setup ______________________________________________________________

    def _create_publishers_and_clients(self):
        
        self.configuration_client = dynamic_reconfigure.client.Client('joint_offsets')
        # ensure that saving doesn't happen after each configuration
        self.configuration_client.update_configuration({'save_config':False})
        
        self.module_control_pub =  rospy.Publisher("robotis/enable_ctrl_module", String, queue_size=10)
        self.walking_command_pub = rospy.Publisher("robotis/thormang3_foot_step_generator/walking_command", FootStepCommand, queue_size=10)
        
        self.sync_write_pub = rospy.Publisher("robotis/sync_write_item", SyncWriteItem, queue_size=10)
        
        self.preview_pose_pub = rospy.Publisher("calibration/preview_pose", JointState, queue_size=10)
        self.show_turning_dir_pub = rospy.Publisher("calibration/show_joint_turning_direction", Bool, queue_size=10)        
        self.turning_joint_pub = rospy.Publisher("calibration/turning_joint", String, queue_size=10)
        
        self.ini_pose_pub = rospy.Publisher("robotis/base/ini_pose", String, queue_size=10)
        
        ns = rospy.get_namespace()
        self.trajectory_controller_names = rospy.get_param(ns + 'control_mode_switcher/control_mode_to_controllers/whole_body/desired_controllers_to_start')
        self.trajectory_controllers = []
        self.trajectory_controller_joints = []
        
        self.allow_all_mode_transitions_pub = rospy.Publisher(ns + 'control_mode_switcher/allow_all_mode_transitions', Bool, queue_size=1)
        self.control_mode_status = rospy.Subscriber(ns + "control_mode_switcher/status", ControlModeStatus, self._log_status)
        self.set_control_mode_client = actionlib.SimpleActionClient(ns + "control_mode_switcher/change_control_mode", ChangeControlModeAction)
        
        for name in self.trajectory_controller_names:
            self.trajectory_controllers.append(rospy.Publisher(ns + 'joints/' + name + '/command', JointTrajectory, queue_size=10))
            self.trajectory_controller_joints.append(rospy.get_param(ns + 'joints/' + name + '/joints'))

    def _parse_offset_config(self, data):
        self._parse_config_to_dict(data.min, self.offset_mins)
        self._parse_config_to_dict(data.max, self.offset_maxs)


    def _parse_config_to_dict(self, conf, dictionary):
        for limits in [conf.bools, conf.ints, conf.strs, conf.doubles]:
            for i in range(0, len(limits)):
                name = limits[i].name
                dictionary[name] = limits[i].value

    
    def _init_pose(self):
        rp = rospkg.RosPack()
        poses_path = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'config', 'calibration_poses.yaml')  
        f = open(poses_path, 'r')
        poses = load(f)
        f.close()
        pose = poses[poses.keys()[0]]

        return pose
        
    def _load_from_config(self, file_name):
        rp = rospkg.RosPack()
        file_path = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'config', file_name)  
        f = open(file_path, 'r')
        file_content = load(f)
        f.close()

        return file_content
        
    def _setup_wizard_variables(self):     
        self.save = True
        self.no_save = False
        self.reset = False
        
        self.pose = self._init_pose()
        self.page_config = self._load_from_config('page_config.yaml')
        
        self.paths = self._load_from_config('paths.yaml')
        self.path_name = ''
        self.path = []          
        
        self.torque_on = False
        
        self.current_control_mode = ''
        
        self.offset_mins = {}
        self.offset_maxs = {}
        
        
    def _connect_ui_signals(self):
        self.next_button.clicked[bool].connect(self._handle_next_button)
        self.back_button.clicked[bool].connect(self._handle_back_button)
        self.finish_button.clicked[bool].connect(self._handle_finish_button)
        self.page_list.activated[int].connect(self._handle_page_list_entry_clicked)
        self.torque_on_radio_button.toggled[bool].connect(self._handle_torque_button)
        self.torque_off_radio_button.toggled[bool].connect(self._handle_torque_button)
        self.take_position.clicked[bool].connect(self._handle_take_position_button)
        self.enable_all_rviz_check.clicked.connect(self._handle_all_rviz_check)
        

# __________________________ other functions __________________________________________________________________________


    def _log_status(self, data):
        self.current_control_mode = data.current_control_mode

    
    def _create_rviz_frame(self):
        rviz_frame = rviz.VisualizationFrame()
        rviz_frame.setSplashPath('')
        rviz_frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        rp = rospkg.RosPack()
        reader.readFile(config, os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'config','config.rviz'))
        rviz_frame.load(config)
        rviz_frame.setMenuBar(None)
        rviz_frame.setStatusBar(None)
        rviz_frame.setHideButtonVisibility(False)
        
        return rviz_frame

# __________________________ ui functions _____________________________________________________________________________
    
    def _handle_next_button(self):
        self._disable_robot_models_in_rviz()
        self._show_all_ui_elements()
        self.enable_all_rviz_check.setChecked(False)
        
        if self.pages.currentIndex() == 0:
            print('Adding pages to wizard...')
            self.pages.currentWidget()._setup_pages()
            print('Done!')
            
        self.pages.setCurrentIndex(self._nextId())

    def _handle_back_button(self):
        self._disable_robot_models_in_rviz()
        self._show_all_ui_elements()
        self.enable_all_rviz_check.setChecked(False)
        
        self.pages.setCurrentIndex(self._previousId())
        
    def _handle_finish_button(self):
        self._close_and_restart_configuration_client()
        self._setup_wizard_variables()
        
        self._show_all_ui_elements()
        self.enable_all_rviz_check.setChecked(False)
        self.torque_on_radio_button.setChecked(False)
        self.torque_off_radio_button.setChecked(True)
        self.walking_module_disable_radio_button.setChecked(False)
        
        self.pages.setCurrentIndex(0)
        
    def _handle_page_list_entry_clicked(self):
        self._show_all_ui_elements()
        self.enable_all_rviz_check.setChecked(False)
        
        self.pages.setCurrentIndex(self.page_list.currentIndex())

    def _handle_torque_button(self):
        msg = self._create_torque_message()

        if len(msg.value) == len(msg.joint_name):

            self.sync_write_pub.publish(msg)

            if self.sender() == self.torque_on_radio_button:
                self.torque_on = True
                self.take_position.setEnabled(True)
            else:
                self.torque_on = False
                self.take_position.setDisabled(True)
        
    def _handle_take_position_button(self):
        if self.torque_on == True:
            if self.pages.currentWidget()._id != 'Walking_Calibration':
                mode = String()
                mode.data = "ros_control_module"
                self.module_control_pub.publish(mode)
            
                if self._check_control_mode_is_whole_body() == False:
                    self._set_control_mode_to_whole_body() 
            
                msgs = self._create_controller_trajectories()
                self._send_controller_trajectories(msgs)
                
            else:
                self.pages.currentWidget()._handle_take_initial_position()

        else:
            print('Turn torque on before moving the robot into position.')
                
    def _handle_all_rviz_check(self):
        page = self.pages.currentWidget()
        page._handle_all_rviz_check(page._noBoxes)
            
#______________________ helper functions for ui _______________________________________________________________________

    def _nextId(self):
        path = self.path
        next = 0
        
        current_id = self.pages.currentWidget()._id
        
        for page in path:
            next += 1
            if page == current_id:
                return next
                
        return -1
            
            
    def _previousId(self):
        path = self.path
        previous = -1
        
        current_id = self.pages.currentWidget()._id
        
        for page in path:
            if page == current_id:
                return previous
            previous += 1
                            
        return -1
                      
                      
    def _close_and_restart_configuration_client(self):
        if self.save:
            self.configuration_client.update_configuration({'save_config':True})
        elif self.reset == True:
            rp = rospkg.RosPack()
            offset_path = os.path.join(rp.get_path('thormang3_manager'), 'config', 'offset.yaml')
            f = open(offset_path, 'r')
            yamlfile = load(f)
            f.close()
            params = yamlfile["offset"]  
            self.configuration_client.update_configuration(params)      
            
        self.configuration_client.close()
        
        self.configuration_client = dynamic_reconfigure.client.Client('joint_offsets')
        self.configuration_client.update_configuration({'save_config':False})
                      
            
    def _create_torque_message(self):
        msg = SyncWriteItem()
        msg.item_name = "torque_enable"
        msg.joint_name.extend(self.pose.keys())
        msg.value = []

        if self.sender() == self.torque_on_radio_button:
            msg.value.extend([1] * len(self.pose.keys()))

        if self.sender() == self.torque_off_radio_button:
            msg.value.extend([0] * len(self.pose.keys()))
            
        return msg
            

    def _create_controller_trajectories(self):
        pose = self.pose
        joint_names = self.trajectory_controller_joints
        
        joint_trajectories = []

        for names in joint_names:

            target_point = JointTrajectoryPoint()
            target_point.time_from_start = rospy.Duration(4, 0)

            for name in names:
                target_point.positions.append(math.radians(pose[name]))

            trajectory = JointTrajectory()
            trajectory.joint_names = names
            trajectory.points = self._get_trajectory_from_target_point(target_point, 1)
            joint_trajectories.append(trajectory)

        return joint_trajectories
    
    def _get_trajectory_from_target_point(self, target_point, num_points_total):
        points = []
        
        for i in range(1, num_points_total + 1):
            point = copy.deepcopy(target_point)
            point.positions[:] = [i * x / num_points_total for x in point.positions]
            point.time_from_start = i * point.time_from_start / num_points_total
            
            points.append(point)
            
        return points
    
    def _send_controller_trajectories(self, joint_trajectories):
        controllers = self.trajectory_controllers
        for i in range(0, len(controllers)):
            joint_trajectories[i].header.stamp = rospy.Time.now()
            controllers[i].publish(joint_trajectories[i])


    def _show_all_ui_elements(self):
        self.header_widget.setVisible(True)
        self.torque_box.setVisible(True)
        self.take_position.setVisible(True)
        self.walking_module_group.setVisible(True)
        self.enable_all_rviz_check.setVisible(True)
        
        self.line_0.setVisible(True)
        self.line_1.setVisible(True)
        self.line_2.setVisible(True)
        self.line_3.setVisible(True)
        self.line_4.setVisible(True)
        self.line_5.setVisible(True)
        
        self.page_help_text_label.setVisible(True)
        self.back_button.setVisible(True)
        self.next_button.setVisible(True)
        self.finish_button.setVisible(True)
        self.page_list.setVisible(True)


    def _disable_robot_models_in_rviz(self):
        for i in range(1, len(self.rviz_frames)):
            self.rviz_frames[i].getManager().getRootDisplayGroup().getDisplayAt(1).setValue(False)
    
#________ control mode functions __________________________________________________________________

    def _check_control_mode_is_whole_body(self):
        if self.current_control_mode == 'whole_body':
            return True
        else:
            return False
            
    def _set_control_mode_to_whole_body(self):
        if self.set_control_mode_client.wait_for_server(rospy.Duration(0.5)):
            self.allow_all_mode_transitions_pub.publish(True)
            
            goal = ChangeControlModeGoal()
            goal.mode_request = 'whole_body'
            rospy.loginfo("Requesting mode change to " + goal.mode_request + ".")

            self.set_control_mode_client.send_goal(goal)

            # waiting for getting list of parameter set names
            action_timeout = rospy.Duration(5.0)
            if self.set_control_mode_client.wait_for_result(action_timeout):
                result = self.set_control_mode_client.get_result()
            else:
                rospy.logwarn("Didn't receive any results after %.1f sec. Check communcation!" % action_timeout.to_sec())
        else:
            rospy.logerr("Can't connect to control mode action server!")
