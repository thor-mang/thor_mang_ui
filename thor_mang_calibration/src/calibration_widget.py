#!/usr/bin/env python

import os

import signal

import rospy
import rospkg
import rosnode
import actionlib
import xml.dom.minidom

from rosgraph_msgs.msg import Clock

import copy
import math

import rviz

import dynamic_reconfigure.client
from dynamic_reconfigure.msg import ConfigDescription

# todo: return PyQt5 to python_qt_binding
from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import pyqtSignal

from yaml import load

from robotis_controller_msgs.msg import SyncWriteItem
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from thormang3_foot_step_generator.msg import FootStepCommand
from thor_mang_control_msgs.msg import ChangeControlModeAction, ControlModeStatus, ChangeControlModeGoal
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState

from intro_page import IntroPage
from pose_page import PosePage
from calibration_page import CalibrationPage
from walking_page import WalkingCalibrationPage
from summary_page import SummaryPage
from auto_page import AutomaticPage
from placeholder_page import PlaceholderPage


CUSTOM_PAGES = ['AutomaticPage', 'PlaceholderPage']


class CalibrationDialog(Plugin):
    def __init__(self, context):
        super(CalibrationDialog, self).__init__(context)
        self.setObjectName('CalibrationDialog')

        self._wizard = CalibrationWizard(context)

        signal.signal(signal.SIGINT, self.signal_handler)

    def shutdown_plugin(self):
        self._wizard.shutdown_plugin()

    def signal_handler(self, sig, frame):
        self.deleteLater()

        
# ______________________________________________________________________________________________________________________

class CalibrationWizard(QWidget):
    torque_status = pyqtSignal(bool)

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
            
            self._create_publishers_and_clients()
            
            ns = rospy.get_namespace()            
            calibration_publishers = rospy.get_published_topics(ns + 'calibration')
            print('waiting for calibration/joint_state_publisher...')
            
            while not [str(ns + 'calibration/joint_states'), 'sensor_msgs/JointState'] in calibration_publishers:
                calibration_publishers = rospy.get_published_topics(ns + 'calibration')
                rospy.sleep(0.1)
            
            self.offset_mins = {}
            self.offset_maxs = {}
            self.sub = rospy.Subscriber(ns + 'joint_offsets/parameter_descriptions', ConfigDescription, self._parse_offset_config)
            
            msg = JointState()
        
            for joint in self.pose:
                msg.name.append(joint)           
                msg.position.append(math.radians(self.pose[joint]))
            
            self.preview_pose_pub.publish(msg)
            
            self._use_sim_time = rospy.get_param('/use_sim_time')
            # setting use_sime_time to False for RVIZ running more smoothly
            rospy.set_param('/use_sim_time', 'False')
            # rviz frames used on multiple pages
            self.rviz_frames = {1: self._create_rviz_frame()}
            rospy.set_param('/use_sim_time', str(self._use_sim_time))
                  
            print('generating pages...')
            self.page_dict = self._generate_page_dict_and_connect_page_signals()
            print('done')

            self.paths = self.unravel_custom_pages_in_path()

            self.pages.insertWidget(0, self.page_dict['Intro'])
            self.page_list.addItem('Intro')
            self._handle_new_pages_ui_changes(0)
            
            self.torque_off_radio_button.setChecked(True)
            self.walking_module_disable_radio_button.setChecked(True)
            
            # set window title
            if context.serial_number() > 1:
                self.setWindowTitle(self.windowTitle() + (' (%d)' % context.serial_number()))
                                    
            # add wizard to the context
            context.add_widget(self)

            
        else:
            print('Missing dynamic reconfigure server from thormang3_manager. Shutting down.')
            self.shutdown_plugin()
            
        print('Done!')
        
    def shutdown_plugin(self):
        print('Shutting down ...')
        
        if rospy.has_param('joint_offsets'):
        
            self.sub.unregister()
            
            ns = rospy.get_namespace() 
            nodes = rosnode.get_node_names(ns + 'calibration')           
            rosnode.kill_nodes(nodes)
            
            self.configuration_client.close()
            
            self.module_control_pub.unregister()
            self.walking_command_pub.unregister()
            
            self.sync_write_pub.unregister()
            
            self.preview_pose_pub.unregister()
            self.show_turning_dir_pub.unregister()    
            self.turning_joint_pub.unregister()
            
            self.ini_pose_pub.unregister()
            
            self.allow_all_mode_transitions_pub.unregister()
            self.control_mode_status.unregister()
            for controller in self.trajectory_controllers:
                controller.unregister()
                
            nodes = rosnode.get_node_names(ns + 'thormang3_foot_step_generator')
            if nodes:
                rosnode.kill_nodes(nodes)

        if hasattr(self, 'page_dict'):
            for key in self.page_dict.keys():
                page = self.page_dict[key]
                try:
                    page.close()
                except:
                    pass

        print('Done!')


# __________________________ functions for initial setup ______________________________________________________________

    def _parse_offset_config(self, data):
        self._parse_config_to_dict(data.min, self.offset_mins)
        self._parse_config_to_dict(data.max, self.offset_maxs)

    def _parse_config_to_dict(self, conf, dictionary):
        for limits in [conf.bools, conf.ints, conf.strs, conf.doubles]:
            for i in range(0, len(limits)):
                name = limits[i].name
                dictionary[name] = limits[i].value

    def _get_joints_information_from_robot(self):
        robot_description = rospy.get_param('calibration/robot_description')
        robot = xml.dom.minidom.parseString(robot_description).getElementsByTagName('robot')[0]
        joint_limits = {}
        
        for child in robot.childNodes:
            if child.localName == 'joint':
                if not child.getAttribute('type') in ['fixed', 'floating', 'planar']:
                    name = child.getAttribute('name')
                    if name in self.pose.keys():
                        limit = child.getElementsByTagName('limit')[0]

                        if child.getAttribute('type') == 'continuous':
                            joint_limits[name] = {'min': -math.pi, 'max': math.pi}
                        else:
                            joint_limits[name] = {'min': float(limit.getAttribute('lower')), 'max': float(limit.getAttribute('upper'))}
                
        return joint_limits
    
    def _init_pose(self):
        rp = rospkg.RosPack()
        poses_path = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'config', 'pose_config.yaml')  
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
        self.save_state = 2 # 0: save 1: reset 2: keep for now and discard after restarting the robot
        
        rp = rospkg.RosPack()
        offset_path = rospy.get_param('calibration/offset_path') 
        f = open(offset_path, 'r')
        self.stored_offsets = (load(f))['offset']
        f.close()
        
        self.pose = self._init_pose()
        self.joint_limits = self._get_joints_information_from_robot()
        self.page_config = self._load_from_config('page_config.yaml')
        self.joint_overview = self._load_from_config('joint_overview_config.yaml')
        
        self.max_joints = -1
        
        self.paths = self._load_from_config('path_config.yaml')
        if not hasattr(self, 'path_name'):
            self.path_name = ''
            self.path = []
        if hasattr(self, 'page_dict'):
            self.paths = self.unravel_custom_pages_in_path()
        
        self.torque_on = False
        
        self.walking_module_on = False
        self.walking_ini_pose_taken = False
        
        self.current_control_mode = ''

    def _connect_ui_signals(self):
        self.next_button.clicked[bool].connect(self._handle_next_button)
        self.back_button.clicked[bool].connect(self._handle_back_button)
        self.finish_button.clicked[bool].connect(self._handle_finish_button)
        
        self.page_list.activated[int].connect(self._handle_page_list_entry_clicked)
        
        self.torque_on_radio_button.toggled[bool].connect(self._handle_torque_button)
        self.torque_off_radio_button.toggled[bool].connect(self._handle_torque_button)
        
        self.take_position.clicked[bool].connect(self._handle_take_position_button)
        
        self.walking_module_enable_radio_button.toggled[bool].connect(self._handle_walking_module_on)
        self.walking_module_disable_radio_button.toggled[bool].connect(self._handle_walking_module_off)
        
        self.enable_all_rviz_check.clicked.connect(self._handle_all_rviz_check)

    def _create_publishers_and_clients(self):
        
        self.configuration_client = dynamic_reconfigure.client.Client('joint_offsets')
        # ensure that saving doesn't happen after each configuration
        self.configuration_client.update_configuration({'save_config': False})
        
        self.module_control_pub = rospy.Publisher('robotis/enable_ctrl_module', String, queue_size=10)
        self.walking_command_pub = rospy.Publisher('robotis/thormang3_foot_step_generator/walking_command', FootStepCommand, queue_size=10)
        
        self.sync_write_pub = rospy.Publisher('robotis/sync_write_item', SyncWriteItem, queue_size=10)
        
        self.preview_pose_pub = rospy.Publisher('calibration/preview_pose', JointState, queue_size=10)
        self.show_turning_dir_pub = rospy.Publisher('calibration/show_joint_turning_direction', Bool, queue_size=10)        
        self.turning_joint_pub = rospy.Publisher('calibration/turning_joint', String, queue_size=10)
        
        self.ini_pose_pub = rospy.Publisher('robotis/base/ini_pose', String, queue_size=10)
        
        self.clock = rospy.Subscriber('/clock', Clock, self._log_clock)
        
        ns = rospy.get_namespace()
        self.trajectory_controller_names = rospy.get_param(ns + 'control_mode_switcher/control_mode_to_controllers/whole_body/desired_controllers_to_start')
        self.trajectory_controllers = []
        self.trajectory_controller_joints = []
        
        self.allow_all_mode_transitions_pub = rospy.Publisher(ns + 'control_mode_switcher/allow_all_mode_transitions', Bool, queue_size=1)
        self.control_mode_status = rospy.Subscriber(ns + 'control_mode_switcher/status', ControlModeStatus, self._log_control_mode)
        self.set_control_mode_client = actionlib.SimpleActionClient(ns + 'control_mode_switcher/change_control_mode', ChangeControlModeAction)
        
        for name in self.trajectory_controller_names:
            self.trajectory_controllers.append(rospy.Publisher(ns + 'joints/' + name + '/command', JointTrajectory, queue_size=10))
            self.trajectory_controller_joints.append(rospy.get_param(ns + 'joints/' + name + '/joints'))
        # adding head_trajectory_controller manually
        self.trajectory_controller_names.append(ns + 'joints/head_traj_controller')
        self.trajectory_controllers.append(rospy.Publisher(ns + 'joints/head_traj_controller/command', JointTrajectory, queue_size=10))
        self.trajectory_controller_joints.append(rospy.get_param(ns + 'joints/head_traj_controller/joints'))

    def setup_rviz_frames(self, max_joints):
        rospy.set_param('/use_sim_time', 'False')
        for i in range(1, max_joints + 1):
            if not i in self.rviz_frames.keys():
                self.rviz_frames[i] = self._create_rviz_frame()
        rospy.set_param('/use_sim_time', str(self._use_sim_time))

    def _create_rviz_frame(self):
        rviz_frame = rviz.VisualizationFrame()
        rviz_frame.setSplashPath('')
        rviz_frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        rp = rospkg.RosPack()
        reader.readFile(config, os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'config','rviz_config.rviz'))
        rviz_frame.load(config)
        rviz_frame.setMenuBar(None)
        rviz_frame.setStatusBar(None)
        rviz_frame.setHideButtonVisibility(False)
        
        return rviz_frame

    def _generate_page_dict_and_connect_page_signals(self):
        page_dict = {}
  
        config = self.page_config
        rviz_frames = self.rviz_frames      
          
        offset_limits = {'mins': self.offset_mins, 'maxs': self.offset_maxs}
          
        for key in self.page_config.keys():
            page_type = config[key]['type']

            if page_type.find('Intro') != -1:
                page = IntroPage(str(key), config[str(key)], self.paths, rviz_frames[1])
                
            elif page_type.find('Pose') != -1:
              page = PosePage(str(key), config[str(key)], self.joint_overview, self.joint_limits, rviz_frames[1])
              
              page.pose_changed.connect(self._handle_pose_change)

            elif page_type.find('Summary') != -1:
             page = SummaryPage(str(key), config[str(key)], self.joint_overview)
             
             page.save_state.connect(self._handle_save_state)
             
            elif page_type.find('WalkingCalibrationPage') != -1:
                page = WalkingCalibrationPage(str(key), config[str(key)], offset_limits, rviz_frames)
                
                page.update_joint_offset.connect(self.update_joint_configuration)
                page.all_rviz_frames_active.connect(self._handle_all_rviz_active_signal)
                page.animated_joint.connect(self.publish_visualized_joint)
                page.show_animation.connect(self.publish_turning_direction)

                page.footstep_parameters.connect(self._handle_footstep_parameters_signal)
                
            elif page_type.find('CalibrationPage') != -1:
                page = CalibrationPage(str(key), config[str(key)], offset_limits, rviz_frames)
                
                page.update_joint_offset.connect(self.update_joint_configuration)
                page.all_rviz_frames_active.connect(self._handle_all_rviz_active_signal)
                page.animated_joint.connect(self.publish_visualized_joint)
                page.show_animation.connect(self.publish_turning_direction)

            #debug
            elif page_type.find('AutomaticPage') != -1:
                #print('debug: calibration_widget key: ' + str(key))
                page = AutomaticPage(str(key), config[str(key)], self)
                page.calibration.poseDefinitionUi.preview_pose.connect(self._handle_pose_change)
                page.calibration.take_pose.connect(self._handle_take_pose_signal_from_auto)
                page.calibration.offset_update.connect(self.update_joint_configuration_multiple)

            elif page_type.find('PlaceholderPage') != -1:
                page = PlaceholderPage(str(key), config[str(key)], self)
                # connect signals with respective functions here

            # enter new page type here
                
            else:
                print('Warning: type of page \'' + key + '\' undefined. Page not generated.')
                continue

            page.setVisible(False)

            # if not an automatic page
            page_dict[str(key)] = page

            if page_type in CUSTOM_PAGES != -1:
                self._unravel_custom_pages_for_dict(page_dict, page.ui_names, page.ui_widgets)

            #if page_type.find('AutomaticPage') == -1:
            #    page_dict[str(key)] = page
            #else:
            #    page_dict[str(key)] = page
            #    self._unravel_custom_pages(page_dict, page.ui_names, page.ui_widgets)

            #    for i in range(0, 0):#len(page.automatic_pages)):
            #        name = str(page.ui_names[i])
            #        #print('debug: name: ' + str(name))
            #        page_dict[name] = page.ui_widgets[i]
            #        page.ui_widgets[i]._page_id = name
            
        return page_dict

    def unravel_custom_pages_in_path(self):
        paths = self.paths

        for key in paths.keys():
            path = paths[key]['path']
            for j in range(0, len(path)):
                page = path[j]
                page_type = self.page_config[page]['type']

                if page_type in CUSTOM_PAGES:
                    path.remove(page)
                    page_ui_names = self.page_dict[page].ui_names

                    for k in range(0, len(page_ui_names)):
                        path.insert(j + k, str(page_ui_names[k]))
                        self.page_config[str(page_ui_names[k])] = {}
                        self.page_config[str(page_ui_names[k])]['type'] = page_type
                        self.page_config[str(page_ui_names[k])]['page_help_text'] = ''

            paths[key]['path'] = path

        return paths

    def _unravel_custom_pages_for_dict(self, page_dict, page_name_list, page_widget_list):
        for i in range(0, len(page_name_list)):
            name = str(page_name_list[i])
            page_dict[name] = page_widget_list[i]
            page_widget_list[i]._page_id = name

    def _setup_pages(self):
        chosen_path = self.pages.currentWidget().get_chosen_path()
             
        if chosen_path != self.path_name:
            print('Adding pages to wizard...')

            self._clear_pages()
            self._add_pages_to_wizard(chosen_path)
            
            self.page_list.clear()
            self._add_pages_to_list()
            
            self.path_name = chosen_path
            
            print('Done!')
        
    def _clear_pages(self):
        if self.pages.count() > 1:
            for i in reversed(range(2, self.pages.count() + 1)):
                self.pages.removeWidget(self.pages.widget(i))

    def _add_pages_to_wizard(self, path):
        path = self.paths[path]['path']
        self.path = path + ['Summary']
        
        i = 1
            
        for page in self.path:
            self.pages.insertWidget(i, self.page_dict[str(page)])
            i += 1

        # intro page has already been added in the init function
        self.path = ['Intro'] + self.path

    def _add_pages_to_list(self):
        self.page_list.setMaxVisibleItems(5)       
        self.page_list.setStyleSheet('QComboBox { combobox-popup: 0; }')

        for page in self.path:
            self.page_list.addItem((str(page)))

    def _get_max_joints(self):
        config = self.page_config
        path = self.path
        
        max_joints = -1
        
        for key in path:
            if key in config.keys():
                page = config[key]
                if 'num_joints' in page.keys():
                    num_joints = page['num_joints']
                    if num_joints > max_joints:
                        max_joints = num_joints
                    
        return max_joints
# __________________________ pages signals' handlers __________________________________________________________________

    def _handle_save_state(self, state):
        self.save_state = state

    def _handle_pose_change(self, pose):
        self.pose = copy.deepcopy(pose)
        self.publish_preview_pose()
        
    def _handle_all_rviz_active_signal(self, active):
        self.enable_all_rviz_check.setChecked(active)
        
    def _handle_footstep_parameters_signal(self, params):
        if self.walking_module_on == True:
            self.publish_footstep_command(params)
        else:
            print('Turn the walking module on before attempting to take steps!')

    def _handle_take_pose_signal_from_auto(self, pose_dict):
        #print('debug: got take_pose signal')
        msgs = self._create_controller_trajectories(pose_dict)
        self.send_controller_trajectories(msgs)

# __________________________ ui functions _____________________________________________________________________________
    
    def _handle_next_button(self):
        self._disable_robot_models_in_rviz()
        self._show_all_ui_elements()
        self.enable_all_rviz_check.setChecked(False)
        
        if self.pages.currentIndex() == 0:
            self._setup_pages()

            self.max_joints = self._get_max_joints()
            
            for i in range(0, self.pages.count()):
                page = self.pages.widget(i)
                if hasattr(page, 'max_joints'):
                    page.max_joints = self.max_joints
            
            self.setup_rviz_frames(self.max_joints)
            
            self.path_name = self.pages.currentWidget().get_chosen_path()
        
        self._handle_new_pages_ui_changes(self._nextId())    

    def _handle_back_button(self):
        self._disable_robot_models_in_rviz()
        self._show_all_ui_elements()
        self.enable_all_rviz_check.setChecked(False)
        
        self._handle_new_pages_ui_changes(self._previousId())

    def _handle_finish_button(self):
        self._close_and_restart_configuration_client()
        self._setup_wizard_variables()
        
        self._show_all_ui_elements()
        self.enable_all_rviz_check.setChecked(False)
        self.torque_on_radio_button.setChecked(False)
        self.torque_off_radio_button.setChecked(True)
        self.walking_module_disable_radio_button.setChecked(False)
        
        self._handle_new_pages_ui_changes(0)        

    def _handle_page_list_entry_clicked(self):
        self._show_all_ui_elements()
        self.enable_all_rviz_check.setChecked(False)
        
        self._handle_new_pages_ui_changes(self.page_list.currentIndex())

    def _handle_torque_button(self, checked):
        if checked:
            msg = self._create_torque_message()

            if len(msg.value) == len(msg.joint_name):

                self.torque_status.emit(msg.value[0])
                self.sync_write_pub.publish(msg)

                if self.sender() == self.torque_on_radio_button:
                    self.torque_on = True
                    self.take_position.setEnabled(True)
                    
                    if self.pages.currentWidget()._page_id != 'Walking Calibration':
                        self.publish_control_mode('ros_control_module')

                        if self._check_control_mode_is_whole_body() == False:
                            self._set_control_mode_to_whole_body()
                    
                else:
                    self.torque_on = False
                    self.take_position.setDisabled(True)
                
    def _handle_take_position_button(self):
        if self.torque_on == True:
            if self.pages.currentWidget()._page_id != 'Walking Calibration':
                msgs = self._create_controller_trajectories(self.pose)
                self.send_controller_trajectories(msgs)
            else:
                self.ini_pose_pub.publish('ini_pose')
                self.walking_ini_pose_taken = True
                
                self.walking_module_group.setEnabled(True)

        else:
            print('Turn torque on before moving the robot into position.')

    def _handle_walking_module_on(self, enabled):
        if enabled == True and self.walking_module_on == False:
            if self.walking_ini_pose_taken:
                self.publish_control_mode('walking_module')
                self.walking_module_on = True
                
                self.pages.currentWidget().walking_panel.setEnabled(True)
            else:
                print('Go to inital position before enabling the walking module.')
                self.walking_module_disable_radio_button.setChecked(True)
    
    def _handle_walking_module_off(self, enabled):
        if enabled == True and self.walking_module_on == True:
            self.publish_control_mode('none')
            self.walking_module_on = False  
            
            self.pages.currentWidget().walking_panel.setDisabled(True)   
                
    def _handle_all_rviz_check(self, checked):
        page = self.pages.currentWidget()
        page._handle_all_rviz_check(page._noBoxes, checked)
            
# ______________________ helper functions for ui _______________________________________________________________________

    def _nextId(self):
        path = self.path
        next = 0
        
        current_id = self.pages.currentWidget()._page_id
        
        for page in path:
            next += 1
            if page == current_id:
                return next
                
        return -1
            
    def _previousId(self):
        path = self.path
        previous = -1
        
        current_id = self.pages.currentWidget()._page_id
        
        for page in path:
            if page == current_id:
                return previous
            previous += 1
                            
        return -1

    def _handle_new_pages_ui_changes(self, new_page_num):
        self.pages.setCurrentIndex(new_page_num)
        self._update_page_list(new_page_num)
        self._hide_ui_elements(new_page_num)
        self._set_help_text(new_page_num)
        
        self._call_new_pages_update(new_page_num)

    def _update_page_list(self, show_page_num):
        self.page_list.setCurrentIndex(show_page_num)
      
    def _set_help_text(self, show_page_num):
        page_id = self.page_list.itemText(show_page_num)
        help_text = self.page_config[page_id]['page_help_text']
        
        self.page_help_text_label.setText(help_text)

    def _call_new_pages_update(self, show_page_num):
        page_id = self.page_list.itemText(show_page_num)
        page = self.page_dict[page_id]
        
        page.update()

    def _close_and_restart_configuration_client(self):
        if self.save_state == 0:
            self.configuration_client.update_configuration({'save_config':True})
        elif self.save_state == 1:
            rp = rospkg.RosPack()
            offset_path = rospy.get_param('calibration/offset_path')
            f = open(offset_path, 'r')
            yamlfile = load(f)
            f.close()
            params = yamlfile['offset']  
            self.configuration_client.update_configuration(params)      
            
        self.configuration_client.close()
        
        self.configuration_client = dynamic_reconfigure.client.Client('joint_offsets')
        self.configuration_client.update_configuration({'save_config':False})
                      
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

    def _hide_ui_elements(self, show_page_num):
        page_id = self.page_list.itemText(show_page_num)
        page_type = self.page_config[page_id]['type']
        
        if page_type.find('Intro') != -1:
            self.back_button.setVisible(False)
            self.finish_button.setVisible(False)
            self.page_list.setVisible(False)
            self.header_widget.setVisible(False)
            self.line_4.setVisible(False)
        elif page_type.find('Pose') != -1:
            self.walking_module_group.setVisible(False)
            self.line_2.setVisible(False)
            self.enable_all_rviz_check.setVisible(False)
            self.line_3.setVisible(False)
            self.finish_button.setVisible(False)
        elif page_type.find('Summary') != -1:
            self.header_widget.setVisible(False)
            self.line_4.setVisible(False)
            self.next_button.setVisible(False)
        elif page_type.find('WalkingCalibrationPage') != -1:
            self.finish_button.setVisible(False)
        elif page_type.find('CalibrationPage') != -1:
            self.walking_module_group.setVisible(False)
            self.line_2.setVisible(False)
            self.finish_button.setVisible(False)
        elif page_type.find('AutomaticPage') != -1:
            self.walking_module_group.setVisible(False)
            self.line_1.setVisible(False)
            self.line_2.setVisible(False)
            self.finish_button.setVisible(False)
            self.enable_all_rviz_check.setVisible(False)
            self.take_position.setVisible(False)
            self.line_3.setVisible(False)
        elif page_type.find('PlaceholderPage') != -1:
            self.header_widget.setVisible(False)
            self.finish_button.setVisible(False)
        #else:
            #print('debug: page_type: ' + str(page_type))

    def _disable_robot_models_in_rviz(self):
        for i in range(1, len(self.rviz_frames)):
            self.rviz_frames[i].getManager().getRootDisplayGroup().getDisplayAt(1).setValue(False)

    def _check_control_mode_is_whole_body(self):
        if self.current_control_mode == 'whole_body':
            return True
        else:
            return False
            
#_________ communication with other nodes functions ___________________________________________________________________
    
    def update_joint_configuration(self, joint, value):
        self.configuration_client.update_configuration({joint: value})

    def update_joint_configuration_multiple(self, joint_value_dict):
        self.configuration_client.update_configuration(joint_value_dict)

    def _log_clock(self, data):
        self._clock = data.clock
    
    # ___ for control mode ____________
    
    def _log_control_mode(self, data):
        self.current_control_mode = data.current_control_mode

    def publish_control_mode(self, mode):
        msg = String()
        msg.data = mode
        self.module_control_pub.publish(msg)

    # ___ for trajectory controllers __

    def _set_control_mode_to_whole_body(self):
        if self.set_control_mode_client.wait_for_server(rospy.Duration(0.5)):
            self.allow_all_mode_transitions_pub.publish(True)
            
            goal = ChangeControlModeGoal()
            goal.mode_request = 'whole_body'
            rospy.loginfo('Requesting mode change to ' + goal.mode_request + '.')

            self.set_control_mode_client.send_goal(goal)

            # waiting for getting list of parameter set names
            action_timeout = rospy.Duration(5.0)
            if self.set_control_mode_client.wait_for_result(action_timeout):
                result = self.set_control_mode_client.get_result()
            else:
                rospy.logwarn('Didn\'t receive any results after %.1f sec. Check communcation!' % action_timeout.to_sec())
        else:
            rospy.logerr('Can\'t connect to control mode action server!')

    def send_controller_trajectories(self, joint_trajectories):
        controllers = self.trajectory_controllers
        for i in range(0, len(controllers)):
            joint_trajectories[i].header.stamp = self._get_time()
            controllers[i].publish(joint_trajectories[i])

    # ___ for preview pose ____________
        
    def publish_preview_pose(self):
        msg = self._generate_preview_pose_msg(self.pose)
        self.preview_pose_pub.publish(msg)
        
    def publish_visualized_joint(self, joint):
        msg = String()
        msg = joint
        self.turning_joint_pub.publish(msg)
        
    def publish_turning_direction(self, on):          
        self.show_turning_dir_pub.publish(on)
        
    # ___ for walking _________________
    
    def publish_footstep_command(self, params):
        msg = self.generate_footstep_message(params)
        self.walking_command_pub.publish(msg)
        
#_________ message generation functions ___________________________________________________________
    
    def _get_time(self):
        if rospy.get_param('/use_sim_time') == True:
            return self._clock
        else:
            return rospy.Time.now()

    def _generate_preview_pose_msg(self, pose):
        msg = JointState()
        
        for joint in pose:
            upper_limit = self.joint_limits[joint]['max']
            lower_limit = self.joint_limits[joint]['min']      
            
            value = math.radians(pose[joint])
            if value > upper_limit:
                value = upper_limit
            elif value < lower_limit:
                value = lower_limit
                
            msg.position.append(value)
            msg.name.append(joint) 
            
        return msg
        
    def _create_torque_message(self):
        msg = SyncWriteItem()
        msg.item_name = 'torque_enable'
        msg.joint_name.extend(self.pose.keys())
        msg.value = []

        if self.sender() == self.torque_on_radio_button:
            msg.value.extend([1] * len(self.pose.keys()))

        if self.sender() == self.torque_off_radio_button:
            msg.value.extend([0] * len(self.pose.keys()))
            
        return msg
        
    def _create_controller_trajectories(self, pose):
        #pose = self.pose
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

    def generate_footstep_message(self, params):        
        msg = FootStepCommand()
              
        msg.command = params['command']
        msg.step_num = params['num_steps']
        msg.step_time = params['step_time']
        msg.step_length = params['step_length']
        msg.side_step_length = params['side_step_length']
        msg.step_angle_rad = params['step_angle']
        
        return msg
