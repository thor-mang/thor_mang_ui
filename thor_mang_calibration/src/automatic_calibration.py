#! /usr/bin/env python

import os
import time
from math import atan2, radians, degrees, pi
import sympy
import sympy.vector
import numpy as np
from scipy.optimize import minimize
import copy
import threading

import rospy
import rospkg
import cv_bridge

import cv2

from python_qt_binding import loadUi
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QValidator, QDoubleValidator, QIntValidator

import xml.dom.minidom
from yaml import load, dump
import dill
dill.settings['recurse'] = True

from sensor_msgs.msg import JointState, Image, CameraInfo
from dynamic_reconfigure.msg import ConfigDescription
from control_msgs.msg import JointTrajectoryControllerState

from model_generator import ModelGenerator
from LimbSelectionUi import LimbSelectionUi
from PoseDefinitionUi import PoseDefinitionUi
from MoveRobotUi import MoveRobotUi
from InitialValueUi import InitialValueUi
from OptimizationUi import OptimizationUi

STOPPED_VELOCITY_THRESHOLD = 1e-2

ONLY_USE_CORNERS = True


class AutomaticCalibration(QWidget):
    loading_status = pyqtSignal(dict)
    optimization_status = pyqtSignal(dict)

    model_status = pyqtSignal(list)
    poses_status = pyqtSignal(list)

    chosen_target_cs = pyqtSignal(list)
    models_for_target_cs = pyqtSignal(list)

    move_status = pyqtSignal(basestring, bool)
    take_pose = pyqtSignal(dict)

    option_value = pyqtSignal(list)
    optimization_status = pyqtSignal(bool, basestring)
    optimization_progress = pyqtSignal(basestring)

    offset_update = pyqtSignal(dict)

    def __init__(self, parent=None):
        super(AutomaticCalibration, self).__init__()

        self.parent = parent

        start_time = time.time() # debug

        # optimiziation parameters
        self.tol = 0.001
        self.maxcor = 50
        self.maxls = 20
        self.ftol = 2.220446049250313e-09
        self.gtol = 1e-05
        self.eps = 1e-08
        self.maxfun = 15000
        self.maxiter = 15000

        #print('debug: top of automatic calibration init: ' + str(time.time() - start_time))
        self.robot_links = self.get_joint_cs()
        self.target_links = []
        self.ui = []
        self.pages = []
        self.torque_status = None

        self.target_offsets = ['roll_offset', 'pitch_offset', 'yaw_offset', 'x_offset', 'y_offset', 'z_offset']
        self.calibrate_target_offsets = {}
        self.target_offsets_initial_values = {}

        self.model_generator_threads = []
        self.move_thread = None
        self.move_thread_running = False
        self.optimization_thread = None
        self.optimization_thread_running = False

        self.bridge = cv_bridge.CvBridge()
        self.current_image = None
        self.current_actual_joint_states = {}
        self.current_data_lock = threading.Lock()

        self.old_offsets = self._load_offset_file()

        ns = rospy.get_namespace()

        self.generate_test_data_bool = rospy.get_param(ns + 'calibration/generate_test_data')

        self.camera_info = rospy.wait_for_message(ns + 'sensor/head_cam/rgb/camera_info', CameraInfo)
        self.P = sympy.Matrix(self.camera_info.P).reshape(3, 4)
        self.K = np.array(self.camera_info.K, dtype=int).reshape([3, 3])

        #print('debug: before loading: ' + str(time.time() - start_time))
        self.calibration_options = self._load_from_config('automatic_options.yaml')
        self.calibration_poses = self._load_from_config('automatic_calibration_poses.yaml')
        self.world_point_definition = self._load_from_config('real_world_measured_vertices.yaml')
        self.existing_models = {}

        #print('debug: before world points: ' + str(time.time() - start_time))
        self.world_points = self.generate_world_points(self.world_point_definition)

        #print('debug: before parsing min max offset: ' + str(time.time() - start_time))
        ns = rospy.get_namespace()
        data = rospy.wait_for_message(ns + 'joint_offsets/parameter_descriptions', ConfigDescription)
        self.offset_mins, self.offset_maxs = self._parse_offset_config(data)

        camera_link, link_to_cam_tf = self.load_camera_config()
        #print('debug: before parsing calibratable joints: ' + str(time.time() - start_time))
        self.joints_to_calibrate, joints, links = self.parse_robot_description()

        joints_to_calibrate, existing_models = self.load_models()

        #print('debug: self.joints_to_calibrate:\n' + str(self.joints_to_calibrate))
        #print('debug: joints_to_calibrate:\n' + str(joints_to_calibrate))

        if joints_to_calibrate == self.joints_to_calibrate:
            #print('debug: joints fit.')
            self.existing_models = existing_models

        self.initial_value_dict = {}
        self.calibrate_joint_dict = {key: True for key in self.joints_to_calibrate}

        self.optimization_input_data = []

        #print('debug: before starting ModelGenerator: ' + str(time.time() - start_time))
        self.model_generator = None
        model_generator_start_thread = threading.Thread(target=self.start_model_generator_thread,
                                                        args=(joints, links, [camera_link, link_to_cam_tf]))
        model_generator_start_thread.start()

        #print('debug: before starting pages: ' + str(time.time() - start_time))
        self.limbSelectionUi = LimbSelectionUi(self.robot_links, self.calibration_options, self)
        self.pages.append(['Limb Selection', self.limbSelectionUi])
        self.limbSelectionUi.target_coordinate_systems.connect(self.handle_chosen_target_links)
        self.limbSelectionUi.save_options.connect(self.handle_save_signal)
        self.limbSelectionUi.save_models.connect(self.save_models_signal)
        #print('debug: limbSelectionUi done at: ' + str(time.time() - start_time))

        self.poseDefinitionUi = PoseDefinitionUi(self.calibration_poses, self.joints_to_calibrate, joints,
                                                 self.world_points, self.world_point_definition, self)
        self.pages.append(['Pose Definition', self.poseDefinitionUi])
        self.poseDefinitionUi.preview_pose.connect(self.handle_preview_pose_signal)
        self.poseDefinitionUi.save_poses.connect(self.handle_save_signal)
        #print('debug: poseDefinitionUi done at: ' + str(time.time() - start_time))

        self.moveRobotUi = MoveRobotUi(self)
        self.moveRobotUi.move_command.connect(self.handle_move_command)
        self.pages.append(['Move Robot', self.moveRobotUi])
        #print('debug: moveRobotUi done at: ' + str(time.time() - start_time))

        self.initialOffsetValueUi = InitialValueUi(self.joints_to_calibrate, self.target_offsets,
                                                   self.offset_mins, self.offset_maxs,
                                                   self.old_offsets, self)
        self.pages.append(['Initial Offset Value', self.initialOffsetValueUi])
        self.initialOffsetValueUi.initial_values.connect(self.handle_initial_value_signal)
        self.initialOffsetValueUi.calibrate_joint.connect(self.handle_calibrate_joint_signal)
        self.initialOffsetValueUi.calibrate_target_offset.connect(self.handle_calibrate_target_offset_signal)
        self.initialOffsetValueUi.initial_values_target_offset.connect(self.handle_target_offset_initial_value_signal)
        #print('debug: initialOffsetValueUi done at: ' + str(time.time() - start_time))

        self.optimizationUi = OptimizationUi(self)
        self.pages.append(['Optimization', self.optimizationUi])
        self.optimizationUi.option_set.connect(self.handle_optimization_option_signal)
        self.optimizationUi.run_optimization.connect(self.handle_optimization_signal)

        self.set_publishers_and_subscribers()

        ##print('debug: before waiting for model: ' + str(time.time() - start_time))
        #model_generator_start_thread.join()

        #print('debug: before assigning limbSelectionUi initial values: ' + str(time.time() - start_time))
        self.limbSelectionUi.set_initial_selections()
        #print('debug: before assigning initialOffsetValueUi initial values: ' + str(time.time() - start_time))
        self.initialOffsetValueUi.set_initial_selections()
        self.send_out_initial_optimization_options()

        self.emit_optimization_status()

        if parent is not None:
            self.parent.torque_status.connect(self.handle_torque_status)
        #print('debug: end: ' + str(time.time() - start_time))

    def set_publishers_and_subscribers(self):
        ns = rospy.get_namespace()

        self.preview_pose_pub = rospy.Publisher(ns + 'calibration/preview_pose', JointState, queue_size=10)
        self.image_sub = rospy.Subscriber(ns + 'sensor/head_cam/rgb/image_raw', Image, self.handle_image_sub)

        self.trajectory_controller_names = rospy.get_param(
            ns + 'control_mode_switcher/control_mode_to_controllers/whole_body/desired_controllers_to_start')
        self.trajectory_controller_joints = []
        self.trajectory_controller_state_subs = []

        for name in self.trajectory_controller_names:
            full_name = ns + 'joints/' + name
            self.trajectory_controller_joints.append(rospy.get_param(full_name + '/joints'))
            self.trajectory_controller_state_subs.append(rospy.Subscriber(full_name + '/state',
                                                                          JointTrajectoryControllerState,
                                                                          self.handle_trajectory_state))

        # adding head_trajectory_controller manually
        self.trajectory_controller_joints.append(rospy.get_param(ns + 'joints/head_traj_controller/joints'))
        self.trajectory_controller_state_subs.append(rospy.Subscriber(ns + 'joints/head_traj_controller/state',
                                                                      JointTrajectoryControllerState,
                                                                      self.handle_trajectory_state))

    def handle_trajectory_state(self, data):
        joint_names = data.joint_names
        actual_positions = data.actual.positions
        actual_velocities = data.actual.velocities

        self.current_data_lock.acquire()
        try:
            self.current_actual_joint_states[str(joint_names)] = {'joint_names': joint_names,
                                                                  'positions': actual_positions,
                                                                  'velocities': actual_velocities}
        finally:
            self.current_data_lock.release()

    def handle_image_sub(self, data):
        #cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.current_data_lock.acquire()
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            #self.current_image = QImage(cv_image.data, data.width, data.height, QImage.Format_RGB888).rgbSwapped()
        finally:
            self.current_data_lock.release()

    def start_model_generator_thread(self, joints, links, camera_info):
        self.model_generator = ModelGenerator(self.joints_to_calibrate, joints, links, camera_info)

    def handle_preview_pose_signal(self, pose):
        self.publish_preview_pose(pose)

    def publish_preview_pose(self, pose):
        msg = JointState()

        for joint in self.joints_to_calibrate:
            if str(joint) in pose.keys():
                msg.name.append(joint)
                msg.position.append(radians(pose[joint]))

        self.preview_pose_pub.publish(msg)

    def _parse_offset_config(self, data):
        offset_mins = self._parse_config_to_dict(data.min)
        offset_maxs = self._parse_config_to_dict(data.max)

        return offset_mins, offset_maxs

    def _parse_config_to_dict(self, conf):
        dictionary = {}
        for limits in [conf.bools, conf.ints, conf.strs, conf.doubles]:
            for i in range(0, len(limits)):
                name = limits[i].name
                dictionary[name] = limits[i].value

        return dictionary

    def parse_robot_description(self):
        robot_description = rospy.get_param('robot_description')
        robot = xml.dom.minidom.parseString(robot_description).getElementsByTagName('robot')[0]

        joints = {}
        links = {}
        joints_to_calibrate = []

        for node in robot.childNodes:
            if node.localName == 'joint':
                name = str(node.getAttribute('name'))
                joint_type = str(node.getAttribute('type'))
                origin = node.getElementsByTagName('origin')[0]
                origin_rpy = origin.getAttribute('rpy')
                origin_xyz = origin.getAttribute('xyz')

                axis_xyz_list = []
                limits = {}
                if joint_type == 'revolute':
                    axis = node.getElementsByTagName('axis')
                    if axis:
                        axis_xyz = axis[0].getAttribute('xyz')
                        axis_xyz_list = [float(i) for i in list(str(axis_xyz).split(' '))]
                        joints_to_calibrate.append(name)

                        limit = node.getElementsByTagName('limit')[0]

                        if node.getAttribute('type') == 'continuous':
                            limits = {'min': -pi, 'max': pi}
                        else:
                            limits = {'min': float(limit.getAttribute('lower')),
                                      'max': float(limit.getAttribute('upper'))}

                parent_link = str(node.getElementsByTagName('parent')[0].getAttribute('link'))
                child_link = str(node.getElementsByTagName('child')[0].getAttribute('link'))

                origin_rpy_list = [float(i) for i in list(str(origin_rpy).split(' '))]
                origin_xyz_list = [float(i) for i in list(str(origin_xyz).split(' '))]

                joints[name] = {'type': joint_type, 'parent_link': parent_link, 'child_link': child_link,
                                'axis': axis_xyz_list, 'origin_xyz': origin_xyz_list, 'origin_rpy': origin_rpy_list,
                                'limits': limits}

                if name in self.offset_mins.keys():
                    joints[name]['min'] = self.offset_mins[name]
                if name in self.offset_maxs.keys():
                    joints[name]['max'] = self.offset_maxs[name]

                links[child_link] = {'parent_joint': name}

        return joints_to_calibrate, joints, links

    def _load_offset_file(self):
        offset_path = rospy.get_param('calibration/offset_path')
        f = open(offset_path, 'r')
        old_offsets = (load(f))['offset']
        f.close()

        return old_offsets

    def _load_from_model(self, file_name):
        rp = rospkg.RosPack()
        file_path = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'model', file_name)
        f = open(file_path, 'r')
        file_content = load(f)
        f.close()

        if file_content is None:
            file_content = {}

        return file_content

    def _load_from_config(self, file_name):
        rp = rospkg.RosPack()
        file_path = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'config', file_name)
        f = open(file_path, 'r')
        file_content = load(f)
        f.close()
        return file_content

    def load_camera_config(self):
        data = self._load_from_config('automatic_camera_link.yaml')
        camera_link = data['link']
        link_to_cam_tf = data['link_to_cam_tf']

        return camera_link, link_to_cam_tf

    def check_model(self):
        limbs = self.limbs_to_calibrate
        model = self.model

        status_msg = {}

        for key in limbs.keys():
            if key in model.keys():
                status_msg[key] = {'type': 'model', 'status': True}
            else:
                status_msg[key] = {'type': 'model', 'status': False}

        self.loading_status.emit(status_msg)

    def check_poses(self):
        limbs = self.limbs_to_calibrate
        poses = self.calibration_poses

        status_msg = {}

        for key in limbs.keys():
            if key in poses.keys():
                number_of_poses = len(poses[key].keys())
                status_msg[key] = {'type': 'poses', 'status': True, 'number': number_of_poses}
            else:
                status_msg[key] = {'type': 'poses', 'status': False}

        self.loading_status.emit(status_msg)

    def get_joint_cs(self):
        links = rospy.get_param('robot_self_filter/self_see_links')

        links_as_list = []
        for entry in links:
            links_as_list.append(entry['name'])

        links_as_list = sorted(links_as_list)

        return links_as_list

    def check_for_model(self, target_links):
        result = []
        for link in target_links:
            if link in self.existing_models.keys():
                result.append([link, self.existing_models[link]['joint_list']])
            else:
                result.append([link, 0])

        return result

    def check_for_poses(self, target_links):
        result = []
        for link in target_links:
            if link in self.calibration_poses.keys():
                result.append([link, len(self.calibration_poses[link].keys())])
            else:
                result.append([link, 0])

        return result

    def send_out_initial_optimization_options(self):
        self.option_value.emit(['tol', self.tol])
        self.option_value.emit(['maxcor', self.maxcor])
        self.option_value.emit(['maxls', self.maxls])
        self.option_value.emit(['ftol', self.ftol])
        self.option_value.emit(['gtol', self.gtol])
        self.option_value.emit(['eps', self.eps])
        self.option_value.emit(['maxfun', self.maxfun])
        self.option_value.emit(['maxiter', self.maxiter])

    def handle_chosen_target_links(self, target_links):
        self.target_links = target_links

        model_result = self.check_for_model(target_links)
        poses_result = self.check_for_poses(target_links)

        self.model_status.emit(model_result)
        self.poses_status.emit(poses_result)
        QApplication.processEvents()

        targets_for_emitting = []
        models_for_emitting = []

        for entry in model_result:
            link = entry[0]
            result = entry[1]

            if not result:
                self.model_status.emit([[link, 2]])
                QApplication.processEvents()

                new_thread = threading.Thread(target=self.generate_model_in_thread, args=[link])
                self.model_generator_threads.append(new_thread)
                new_thread.start()

            else:
                model = self.existing_models[str(link)]
                targets_for_emitting.append([str(link), model['joint_list']])
                models_for_emitting.append([str(link), [model['x'], model['y']]])

        wait_thread = threading.Thread(target=self.wait_for_models, args=[target_links])
        wait_thread.start()

    def generate_model_in_thread(self, link):
        #x, y, joint_list = self.model_generator.create_image_coordinate_formula(link)
        link_to_camera_coords, joint_list = self.model_generator.link_to_camera_coordinates_formula(link)
        camera_to_image_coords = self.model_generator.camera_to_image_coordinates_formula(link_to_camera_coords)
        self.existing_models[link] = {}
        self.existing_models[link]['joint_list'] = joint_list
        self.existing_models[link]['to_camera_coordinates'] = link_to_camera_coords
        self.existing_models[link]['x'] = camera_to_image_coords[0]
        self.existing_models[link]['y'] = camera_to_image_coords[1]

        symbols = self.model_generator.get_symbols_as_list()
        symbolic_vertex_vector = self.model_generator.get_symbolic_vertex_vector()

        vertex_symbols = [symbolic_vertex_vector[0], symbolic_vertex_vector[1], symbolic_vertex_vector[2]]

        #self.existing_models[link]['to_camera_coordinates_function'] = \
        #    sympy.lambdify((symbols, symbolic_vertex_vector[0:2]), link_to_camera_coords)
        #self.existing_models[link]['x_function'] = \
        #    sympy.lambdify((symbols, symbolic_vertex_vector[0:2]), camera_to_image_coords[0])
        #self.existing_models[link]['y_function'] = \
        #    sympy.lambdify((symbols, symbolic_vertex_vector[0:2]), camera_to_image_coords[1])

        x_thread = threading.Thread(target=self.lambdify_in_thread,
                                    args=(link, 'x', symbols, vertex_symbols))
        x_thread.start()
        y_thread = threading.Thread(target=self.lambdify_in_thread,
                                    args=(link, 'y', symbols, vertex_symbols))
        y_thread.start()
        to_camera_coordinates_thread = threading.Thread(target=self.lambdify_in_thread,
                                                        args=(link, 'to_camera_coordinates', symbols, vertex_symbols))
        to_camera_coordinates_thread.start()

        y_thread.join()
        y_thread.join()
        to_camera_coordinates_thread.join()

        self.model_status.emit([[link, 1]])
        QApplication.processEvents()

    def lambdify_in_thread(self, link, model_part_name, symbols, symbolic_vector):
        model_part = self.existing_models[link][model_part_name]

        f = sympy.lambdify((symbols, symbolic_vector), model_part)

        self.existing_models[link][model_part_name + '_function'] = f

    def wait_for_models(self, target_links):
        for thread in self.model_generator_threads:
            thread.join()

        targets_for_emitting = []
        models_for_emitting = []
        for link in target_links:
            model = self.existing_models[str(link)]
            targets_for_emitting.append([str(link), model['joint_list']])
            models_for_emitting.append([str(link), [model['x'], model['y'],
                                                    model['to_camera_coordinates_function']]])

        self.chosen_target_cs.emit(targets_for_emitting)
        self.models_for_target_cs.emit(models_for_emitting)

        self.emit_optimization_status()

    def generate_world_points(self, definition):
        world_points = {}
        for key in definition.keys():
            info = definition[str(key)]
            #world_points[str(key)] = {}

            start = info['start_point']
            start = [float(start[0]), float(start[1]), float(start[2])]
            rows = int(info['checkerboard']['rows'])
            cols = int(info['checkerboard']['cols'])
            dist = float(info['checkerboard']['distance'])

            #world_points[str(key)]['rows'] = rows
            #world_points[str(key)]['cols'] = cols

            this_links_world_points = []
            for i in range(0, cols):
                for j in range(0, rows):
                    this_links_world_points.append([start[0] + dist * j, start[1] + dist * i, start[2]])

            world_points[str(key)] = this_links_world_points

        return world_points

    def handle_initial_value_signal(self, joint_value_dict):
        keys = joint_value_dict.keys()
        for key in keys:
            self.initial_value_dict[key] = joint_value_dict[key]

    def handle_target_offset_initial_value_signal(self, signal_list):
        target = signal_list[0]
        axis = signal_list[1]
        initial_value = signal_list[2]

        if target not in self.target_offsets_initial_values.keys():
            self.target_offsets_initial_values[target] = {}

        self.target_offsets_initial_values[target][axis] = initial_value

        ##print('debug: target_offset_initial_values:\n' + str(self.target_offsets_initial_values))

    def handle_calibrate_joint_signal(self, joint_bool_dict):
        keys = joint_bool_dict.keys()
        for key in keys:
            self.calibrate_joint_dict[key] = joint_bool_dict[key]

    def handle_calibrate_target_offset_signal(self, signal_list):
        target = signal_list[0]
        axis = signal_list[1]
        calibrate = signal_list[2]

        if target not in self.calibrate_target_offsets.keys():
            self.calibrate_target_offsets[target] = {}

        self.calibrate_target_offsets[target][axis] = calibrate

    def handle_torque_status(self, status):
        self.torque_status = status
        if not status:
            self.handle_move_command(False)

    def handle_move_command(self, move_command):
        if move_command and not self.move_thread_running:
            self.move_thread_running = True
            self.move_thread = threading.Thread(target=self.move_robot_in_thread)
            self.move_thread.start()
        elif not move_command and self.move_thread is not None:
            self.move_thread_running = False
            self.move_thread.join()

    def handle_save_signal(self, data):
        content = data[0]
        filename = data[1]

        self.save_to_file(content, filename)

    def wait_for_movement_to_be_finished(self):
        keep_waiting = True
        time.sleep(1)
        while keep_waiting:
            time.sleep(1)

            # if the thread should not be running anymore return
            if not self.move_thread_running:
                return

            # check if robot has reached its pose
            keep_waiting = False
            for key in self.current_actual_joint_states.keys():
                velocities = self.current_actual_joint_states[key]['velocities']

                for velocity in velocities:
                    if abs(velocity) > STOPPED_VELOCITY_THRESHOLD:
                        #print('debug: joint velocity: ' + str(velocity) + ' threshold: ' + str(STOPPED_VELOCITY_THRESHOLD))
                        keep_waiting = True

    def sort_trajectory_controller_results_to_pose(self, current_actual_joint_states):
        all_joint_states = []
        all_joint_names = []

        for key in current_actual_joint_states:
            names = current_actual_joint_states[key]['joint_names']
            joint_states = current_actual_joint_states[key]['positions']

            all_joint_states += joint_states
            all_joint_names += names

        sorted_joint_names = self.joints_to_calibrate
        sorted_joint_states = []

        for joint in sorted_joint_names:
            index = all_joint_names.index(joint)
            sorted_joint_states.append(all_joint_states[index])

        #print('debug: sorted_joint_states:\n' + str(sorted_joint_states))

        return sorted_joint_states

    def find_checkerboard(self, image, num_rows, num_cols):
        greyscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        result, corners = cv2.findChessboardCorners(greyscale, (num_rows, num_cols), None, cv2.CALIB_CB_ADAPTIVE_THRESH)

        if result:
            corners2 = cv2.cornerSubPix(greyscale, corners, (11, 11), (-1, -1),
                                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        else:
            result, corners = cv2.findChessboardCorners(greyscale, (num_cols, num_rows), None,
                                                        cv2.CALIB_CB_ADAPTIVE_THRESH)
            if result:
                corners2 = cv2.cornerSubPix(greyscale, corners, (11, 11), (-1, -1),
                                            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            else:
                #print('debug: Couldn\'t find corners.')
                return None

        return corners2

    def world_point_to_image_point(self, model, pose, world_point):
        camera_coordinates = model(pose, world_point)
        image_coordinates = self.P * camera_coordinates
        image_coordinates = image_coordinates / image_coordinates[2, 0]

        return [image_coordinates[0], image_coordinates[1]]

    def sort_vertices_according_to_world_points(self, pose, detected_vertices, target_link):
        if detected_vertices is None:
            print('Error in sort_vertices_according_to_world_points: detected_vertices are empty.')
            return None

        #vertex_rows = self.world_point_definition[target_link]['checkerboard']['rows']
        #vertex_cols = self.world_point_definition[target_link]['checkerboard']['cols']
        world_points = self.world_points[target_link]

        model = self.existing_models[target_link]['to_camera_coordinates_function']
        analytic_vertices = []

        for point in world_points:
            image_coordinates = self.world_point_to_image_point(model, pose, point)
            analytic_vertices.append(image_coordinates)

        sorted_analytic_vertices, sorted_detected_vertices = self.match_analytic_and_detected_positions(analytic_vertices, detected_vertices)

        sorted_vertices = [-1 for i in range(0, len(world_points))]
        for i in range(0, len(world_points)):
            position_index = sorted_analytic_vertices.index(analytic_vertices[i])
            sorted_vertices[i] = sorted_detected_vertices[position_index]

        return sorted_vertices

    def move_robot_in_thread(self):
        if self.torque_status:

            target_links = self.target_links
            num_poses_per_target_link = []

            for link in target_links:
                link_poses = self.calibration_poses[str(link)]
                num_poses_per_target_link.append(len(link_poses))

            self.optimization_input_data = []

            poses_overall = str(sum(num_poses_per_target_link))
            current_pose = 1
            #for pose in poses:
            for link in target_links:
                link_poses = self.calibration_poses[str(link)]

                vertex_rows = self.world_point_definition[link]['checkerboard']['rows']
                vertex_cols = self.world_point_definition[link]['checkerboard']['cols']

                for key in link_poses.keys():
                    pose = link_poses[key]
                    if self.move_thread_running:
                        self.move_status.emit('moving to pose ' + str(current_pose) + '/' + poses_overall, False)
                        self.take_pose.emit(pose)

                        self.wait_for_movement_to_be_finished()

                        # robot reached pose
                        self.current_data_lock.acquire()
                        try:
                            current_image = copy.deepcopy(self.current_image)
                            current_image = cv2.undistort(current_image, self.K, np.array(self.camera_info.D, dtype=float))
                            current_actual_joint_states = copy.deepcopy(self.current_actual_joint_states)
                        finally:
                            self.current_data_lock.release()

                        joint_state_array = self.sort_trajectory_controller_results_to_pose(current_actual_joint_states)
                        checkerboard_vertices = self.find_checkerboard(current_image, vertex_rows, vertex_cols)

                        sorted_vertices = self.sort_vertices_according_to_world_points(joint_state_array,
                                                                                       checkerboard_vertices, link)

                        # corner_vertices

                        self.optimization_input_data += [[link, joint_state_array, sorted_vertices]]#[[[pose], [vertices]], [[pose2], [vertices2]]]

                        current_pose += 1
                    else:
                        self.move_status.emit('Move Canceled', True)
                        self.optimization_input_data = []
                        return

            self.move_status.emit('Move Done', True)
            self.move_thread_running = False
        else:
            self.move_status.emit('Cannot Move - Robot Not Torqued', True)

        self.emit_optimization_status()

    def handle_optimization_option_signal(self, data):
        option = data[0]
        value_text = data[1]

        #print('debug: data: ' + str(data) + ' option: ' + str(option) + ', value: ' + str(value_text))

        if option == "tol":
            if value_text == '':
                value = None
            else:
                valid, value = self._string_to_float(value_text)

            if value is None or valid:
                self.tol = value
            else:
                value = self.tol

        elif option == "eps":
            valid, value = self._string_to_float(value_text)

            if valid:
                self.eps = value
            else:
                value = self.eps

        elif option == "maxcor":
            valid, value = self._string_to_int(value_text)

            if valid:
                self.maxcor = value
            else:
                value = self.maxcor

        elif option == "maxls":
            valid, value = self._string_to_int(value_text)

            if valid:
                self.maxls = value
            else:
                value = self.maxls

        elif option == "ftol":
            valid, value = self._string_to_float(value_text)

            if valid:
                self.ftol = value
            else:
                value = self.ftol

        elif option == "gtol":
            valid, value = self._string_to_float(value_text)

            if valid:
                self.gtol = value
            else:
                value = self.gtol

        elif option == "maxfun":
            valid, value = self._string_to_int(value_text)

            if valid:
                self.maxfun = value
            else:
                value = self.maxfun

        elif option == "maxiter":
            valid, value = self._string_to_int(value_text)

            if valid:
                self.maxiter = value
            else:
                value = self.maxiter

        #print('debug: option: ' + str(option) + ' value: ' + str(value))

        self.option_value.emit([option, value])

    def emit_optimization_status(self):
        target_links = self.target_links
        models = self.existing_models
        poses = self.calibration_poses
        num_poses = 0

        models_available = True
        data_available = True

        for link in target_links:
            link_poses = poses[str(link)]
            num_poses += len(link_poses)

            if str(link) not in models.keys():
                models_available = False
            else:
                entries = models[link].keys()
                if 'to_camera_coordinates_function' not in entries:
                    models_available = False

        if len(self.optimization_input_data) != num_poses:
            data_available = False

        if not models_available and not data_available:
            self.optimization_status.emit(False, 'Waiting For Data And Model(s)')
        elif not models_available:
            self.optimization_status.emit(False, 'Waiting For Model(s)')
        elif not data_available:
            self.optimization_status.emit(False, 'Waiting For Data')
        else:
            self.optimization_status.emit(True, 'Ready To Run')

    def sort_pose(self, pose):
        a = []
        for joint in self.joints_to_calibrate:
            a.append(radians(pose[joint]))
        return a

    def project_camera_coordinate_vector(self, camera_coordinate_vector):
        image_coordinates = self.P * camera_coordinate_vector
        image_coordinates = image_coordinates / image_coordinates[2, 0]

        return image_coordinates

    def generate_test_data(self):
        self.test_offsets = [radians(0.3) for i in range(0, len(self.joints_to_calibrate))]
        self.optimization_input_data = []

        target_links = self.target_links
        for link in target_links:
            poses = self.calibration_poses[link]
            model_x = self.existing_models[link]['x_function']
            model_y = self.existing_models[link]['y_function']
            world_points = self.world_points[link]

            for key in poses.keys():
                pose = self.sort_pose(poses[key])
                pose = [pose[i] + self.test_offsets[i] for i in range(0, len(pose))]
                #pose_with_offset = list(np.array(pose) + np.array(self.test_offsets))
                pose_with_offset = []

                for i in range(0, len(pose)):
                    pose_with_offset += [pose[i] + self.test_offsets[i]]

                image_points = []

                for world_point in world_points:
                    image_points.append([round(model_x(pose_with_offset, world_point), 0),
                                         round(model_y(pose_with_offset, world_point), 0)])

                #if len(world_points) > 4:
                #    corner1 = world_points[0]
                #    corner2 = world_points[4]
                #    corner3 = world_points[29]
                #    corner4 = world_points[34]
                #else:
                #    corner1 = world_points[0]
                #    corner2 = world_points[1]
                #    corner3 = world_points[2]
                #    corner4 = world_points[3]

                #image_points.append([model_x(pose_with_offset, corner1), model_y(pose_with_offset, corner1)])
                #image_points.append([model_x(pose_with_offset, corner2), model_y(pose_with_offset, corner2)])
                #image_points.append([model_x(pose_with_offset, corner3), model_y(pose_with_offset, corner3)])
                #image_points.append([model_x(pose_with_offset, corner4), model_y(pose_with_offset, corner4)])

                #only_corners = [corner1, corner2, corner3, corner4]

                #self.world_points[link] = only_corners

                self.optimization_input_data += [[link, pose, image_points]]

    def handle_optimization_signal(self, run):
        #print('debug: run optimization: ' + str(run))

        if self.generate_test_data_bool and (self.optimization_input_data == [] or self.optimization_input_data is None):
            self.generate_test_data()

        if run and not self.optimization_thread_running:
            self.optimization_thread_running = True
            self.optimization_thread = threading.Thread(target=self.run_optimization_in_thread)
            self.optimization_thread.start()
        elif not run and self.optimization_thread is not None:
            self.optimization_thread_running = False
            self.optimization_thread.join()

    def run_optimization_in_thread(self):
        self.optimization_status.emit(False, 'Running...')

        #print('debug: self.target_links: ' + str(self.target_links))

        if self.optimization_thread_running:
            #print('debug: entered if self.optimization_thread_running')
            self.all_joints_from_joint_lists = []
            for link in self.target_links:
                joint_list = self.existing_models[link]['joint_list']
                self.all_joints_from_joint_lists += joint_list

            #print('debug: got joint_list')

            # remove duplicates
            self.all_joints_from_joint_lists = list(dict.fromkeys(self.all_joints_from_joint_lists))

            #print('debug: removed duplicates')

            initial_values = []
            bounds = []
            for joint in self.joints_to_calibrate:
                if joint in self.all_joints_from_joint_lists:
                    if self.calibrate_joint_dict[joint]:
                        initial_values.append(self.initial_value_dict[joint])
                        bounds.append((self.offset_mins[joint], self.offset_maxs[joint]))  # parentheses NOT redundant
                    else:
                        self.all_joints_from_joint_lists.remove(joint)

            #print('debug: removed joints not the be calibrated')

            #print('debug: target_links: ' + str(self.target_links))
            #print('debug: target_offsets: ' + str(self.target_offsets))

            for limb in self.target_links:
                possible_target_offsets = self.calibrate_target_offsets[limb]
                target_offset_initial_values = self.target_offsets_initial_values[limb]

                for offset_axis in self.target_offsets:
                    if possible_target_offsets[offset_axis]:
                        initial_values.append(target_offset_initial_values[offset_axis])
                        bounds.append((-100, 100)) # @TODO: set actually sensible bounds for target offsets

            initial_values = np.array(initial_values)

            #print('debug: got initial values')

            start_time = time.time()
            #print('debug: options: tol=' + str(self.tol) + '\noptions: ' + str({'disp': True, 'maxcor': self.maxcor, 'ftol': self.ftol, 'gtol': self.gtol,
            #                            'eps': self.eps, 'maxfun': self.maxfun, 'maxiter': self.maxiter,
            #                            'maxls': self.maxls}))
            try:
                #print('debug: trying minimization')
                results = minimize(self.least_squares, initial_values, method='L-BFGS-B', bounds=bounds,
                                   callback=self.minimize_callback_function,
                                   options={'disp': True, 'maxcor': self.maxcor, 'ftol': self.ftol, 'gtol': self.gtol,
                                            'eps': self.eps, 'maxfun': self.maxfun, 'maxiter': self.maxiter,
                                            'maxls': self.maxls})

                end_time = time.time()
                #print('debug: results ' + str((end_time - start_time)/60) + ' minutes:\n' + str(results))

                offsets_in_radians = results.x

                offsets = [degrees(offsets_in_radians[i]) for i in range(0, len(offsets_in_radians))]
                #print('debug: offsets:\n' + str(offsets))

                offsets_to_publish = {}
                offset_index = 0
                for joint in self.joints_to_calibrate:
                    if joint in self.all_joints_from_joint_lists:
                        offsets_to_publish[joint] = offsets[offset_index]
                        offset_index += 1

                #print('debug: emitting offsets')
                self.offset_update.emit(offsets_to_publish)

            except Exception as e:
                print('Exception raised: ' + str(e))
                self.optimization_status.emit(True, 'Ready To Run')
                self.optimization_progress.emit('\nCanceled - Progress Lost\n')
                return
        else:
            self.optimization_status.emit(True, 'Ready To Run')
            self.optimization_progress.emit('Canceled - Progress Lost')
            return

        self.optimization_thread_running = False
        self.optimization_status.emit(True, 'Done - See Offsets On Next Page')

    def minimize_callback_function(self, func_input):
        #print('debug: from minimize callback function with input?\n' + str(func_input))

        if not self.optimization_thread_running:
            #print('debug: thread should stop')
            raise Exception("Terminating optimization: Optimization was manually stopped.")

        return self.optimization_thread_running

    def least_squares(self, offsets):
        if not self.optimization_thread_running:
            #print('debug: thread should stop')
            raise Exception("Terminating optimization: Optimization was manually stopped.")

        residuum = 0

        time_start = time.time()

        adjusted_world_points_per_link = {}
        corner_indices_per_link = {}
        zero_index_of_target_offsets = len(self.all_joints_from_joint_lists)
        count = 0

        #print('debug: in least_squares: target_links: ' + str(self.target_links))

        for link in self.target_links:
            #print('debug: in least_squares: link: ' + str(link))
            rows = self.world_point_definition[link]['checkerboard']['rows']
            cols = self.world_point_definition[link]['checkerboard']['cols']
            corner_indices_per_link[link] = [0, rows - 1, rows * (cols - 1), rows * cols - 1]

            target_offsets = np.array([0 for i in range(0, len(self.target_offsets))])
            axis_count = 0
            for offset_axis in self.target_offsets:
                if self.calibrate_target_offsets[link][offset_axis]:
                    target_offsets[axis_count] = offsets[zero_index_of_target_offsets + count]
                    count += 1
                axis_count += 1

            #print('debug: in least_squares: got target_offsets')

            tf_target_offsets = np.array(self.model_generator.get_tf_mat_translation_first(target_offsets[0], target_offsets[1],
                                                                                           target_offsets[2], target_offsets[3],
                                                                                           target_offsets[4], target_offsets[5]),
                                         dtype=float)# @TODO enter function parameters!! - done?

            #print('debug: in least_squares: and their tf')

            link_world_points = self.world_points[link]
            #adjusted_world_points = [0 for i in range(0, len(link_world_points))]
            adjusted_world_points = np.zeros([len(link_world_points), 3])
            if not ONLY_USE_CORNERS:
                for i in range(0, len(link_world_points)):
                    world_point_hom_array = np.array([link_world_points[i] + [1.0]]).T
                    adjusted_world_point = tf_target_offsets.dot(world_point_hom_array)
                    ##print('debug: adjusted_world_point: ' + str(adjusted_world_point))
                    adjusted_world_points[i][:] = adjusted_world_point[:-1].T
            else:
                corner_indices = corner_indices_per_link[link]
                for i in range(0, 4):
                    index = corner_indices[i]
                    corner = np.array([link_world_points[index] + [1.0]]).T
                    adjusted_corner = tf_target_offsets.dot(corner)
                    adjusted_world_points[index][:] = adjusted_corner[:-1].T

            #print('debug: in least_squares: adjusted world points')

            adjusted_world_points_per_link[link] = adjusted_world_points

        for i in range(0, len(self.optimization_input_data)):
            data = self.optimization_input_data[i]
            link = data[0]
            if link not in self.target_links:
                continue

            pose = data[1]
            image_points_detected = data[2]
            world_points = adjusted_world_points_per_link[link]

            model_x = self.existing_models[link]['x_function']
            model_y = self.existing_models[link]['y_function']

            pose_with_offset = []
            offset_index = 0
            for j in range(0, len(pose)):
                joint = self.joints_to_calibrate[j]
                if joint in self.all_joints_from_joint_lists:
                    pose_with_offset += [pose[j] + offsets[offset_index]]
                    offset_index += 1
                else:
                    pose_with_offset += [pose[j]]

            if not ONLY_USE_CORNERS:
                for j in range(0, len(world_points)):
                    world_point = world_points[j]

                    image_point = image_points_detected[j]

                    x = model_x(pose_with_offset, world_point)
                    y = model_y(pose_with_offset, world_point)

                    residuum += (image_point[0] - x) ** 2
                    residuum += (image_point[1] - y) ** 2
                    if self.optimization_thread_running:
                        self.optimization_progress.emit('Residuum: ' + str(residuum) + ' \nAverage per image: ' +
                                                    str(residuum / len(self.optimization_input_data)) +
                                                    ' \nAverage per checkerboard corner: ' +
                                                    str(residuum / (len(self.optimization_input_data) * len(world_points))))
            else:
                corner_indices = corner_indices_per_link[link]
                for j in range(0, 4):
                    index = corner_indices[j]

                    world_point = world_points[index]

                    image_point = image_points_detected[index]

                    x = model_x(pose_with_offset, world_point)
                    y = model_y(pose_with_offset, world_point)

                    residuum += (image_point[0] - x) ** 2
                    residuum += (image_point[1] - y) ** 2

                    if self.optimization_thread_running:
                        self.optimization_progress.emit('Residuum: ' + str(residuum) + '\nAverage per image: '
                                                    + str(residuum / len(self.optimization_input_data)) +
                                                    '\nAverage per checkerboard corner: ' +
                                                    str(residuum / (len(self.optimization_input_data) * 4)))

        ##print('debug: done after: ' + str(time.time() - time_start))
        #self.optimization_progress.emit('Residuum: ' + str(residuum) + ' \nAverage per image: ' + str(residuum/len(self.optimization_input_data))
        #                                + ' \nAverage per checkerboard corner: ' + str(residuum/(len(self.optimization_input_data) * len(world_points))))

        #if residuum/(len(self.optimization_input_data) * len(world_points)) < 0.005:
        #    return 0.0

        return float(residuum)

    def save_to_file(self, content, filename):
        rp = rospkg.RosPack()
        file_path = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'config', filename)
        f = open(file_path, 'w')
        dump(content, f, default_flow_style=False)

    def save_models_signal(self, save):
        if save:
            rp = rospkg.RosPack()
            file_path = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'model')

            # to avoid file weirdness
            os.remove(file_path + '/joints.txt')
            os.remove(file_path + '/models.txt')

            with open(file_path + '/joints.txt', 'w') as outf:
                dill.dump(self.joints_to_calibrate, outf)

            with open(file_path + '/models.txt', 'w') as outf:
                dill.dump(self.existing_models, outf)

    def load_models(self):
        rp = rospkg.RosPack()
        file_path = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'model')
        if os.path.exists(file_path + '/joints.txt'):
            with open(file_path + '/joints.txt') as inf:
                joints_to_calibrate = dill.load(inf)
        else:
            joints_to_calibrate = None

        if os.path.exists(file_path + '/models.txt'):
            with open(file_path + '/models.txt') as inf:
                existing_models = dill.load(inf)
        else:
            existing_models = None

        return joints_to_calibrate, existing_models

    def _string_to_float(self, text):
        valid = False
        value = -1

        validator = QDoubleValidator()
        result, _, _ = validator.validate(text, 5)
        if result == QValidator.Acceptable:
            text = text.replace(',', '.')
            valid = True
            value = float(text)

        return valid, value

    def _string_to_int(self, text):
        valid = False
        value = -1

        validator = QIntValidator(0, 1000000)
        result, _, _ = validator.validate(text, 0)
        if result == QValidator.Acceptable:
            #text = text.replace(',', '.')
            valid = True
            value = int(text)

        return valid, value

    def match_analytic_and_detected_positions(self, analytic_points, detected_points, image):
        if analytic_points is None:
            print('Error in match_analytic_and_detected_positions: analytic_points are empty.')
            return None
        if detected_points is None:
            print('Error in match_analytic_and_detected_positions: detected_points are empty.')
            return None
        if len(analytic_points) != len(detected_points):
            print('Error in match_analytic_and_detected_positions: len(analytic_points) != len(detected_points): ' +
                  str(len(analytic_points)) + ' != ' + str(len(detected_points)))
            return None

        matches = {}

        #print('debug: type(detected_points) = ' + str(type(detected_points)) + ' \n' + str(detected_points[0]))
        detected_vertex_grid_corners, detected_edge_vertices = self.find_corners_and_edges_of_vertex_grid(
            detected_points, image)

        analytic_points_as_numpy = np.array(analytic_points).astype(np.float32)
        #print('debug: type(analytic_points) = ' + str(type(analytic_points_as_numpy)) + ' \n' + str(
        #    analytic_points_as_numpy[0]))

        analytic_vertex_grid_corners, analytic_edge_vertices = self.find_corners_and_edges_of_vertex_grid(
            analytic_points_as_numpy, image)

        #print('debug: got vertex corners for both')
        anchor = analytic_vertex_grid_corners[0]

        detected_anchor = self.get_detected_anchor_corner(anchor[0], detected_vertex_grid_corners)

        detected_sorted_vertices = self.sort_vertex_grid_from_anchor(detected_points, detected_edge_vertices,
                                                                     detected_vertex_grid_corners, detected_anchor)
        analytic_sorted_vertices = self.sort_vertex_grid_from_anchor(analytic_points_as_numpy, analytic_edge_vertices,
                                                                     analytic_vertex_grid_corners, anchor)
        img2 = self.vertex_detector.mark_corners(image, detected_sorted_vertices)
        img2 = self.vertex_detector.mark_corners(img2, analytic_sorted_vertices, [0, 255, 0])
        cv2.imwrite('./img/res/img_0_all_sorted_vertices.jpg', img2)

        for i in range(0, len(analytic_sorted_vertices)):
            matches[i] = {0: analytic_sorted_vertices[i][0], 1: detected_sorted_vertices[i][0]}

        return matches

    def sort_vertex_grid_from_anchor(self, unsorted_vertices, sorted_edges, grid_corners, anchor):
        #print('debug: anchor = ' + str(anchor))
        #print('debug: sorted_edges:\n' + str(sorted_edges))
        anchor_index_in_grid_corners = grid_corners.index(anchor)
        second_corner_index = anchor_index_in_grid_corners + 1
        third_corner_index = anchor_index_in_grid_corners + 2
        if second_corner_index >= len(grid_corners):
            second_corner_index = 0
            third_corner_index = 1
        elif third_corner_index >= len(grid_corners):
            third_corner_index = 0
        second_corner = grid_corners[second_corner_index]
        third_corner = grid_corners[third_corner_index]

        width = sorted_edges.index(second_corner) - sorted_edges.index(anchor) + 1
        height = sorted_edges.index(third_corner) - sorted_edges.index(second_corner) + 1

        sorted_vertices = []
        distance_threshold = 5

        for i in range(0, height):
            start_corner_index = sorted_edges.index(anchor) - i
            end_corner_index = sorted_edges.index(second_corner) + i

            start_corner = sorted_edges[start_corner_index]
            end_corner = sorted_edges[end_corner_index]

            vertices_on_line = []
            for vertex in unsorted_vertices:
                distance_to_line = self.distance_between_point_and_line(vertex, start_corner, end_corner)

                if distance_to_line < distance_threshold:
                    distance_to_start = (vertex[0][0] - start_corner[0][0]) ** 2 + (
                                vertex[0][1] - start_corner[0][1]) ** 2
                    vertices_on_line.append([distance_to_start, vertex])

            vertices_on_line.sort()
            for entry in vertices_on_line:
                sorted_vertices.append(entry[1])

        return sorted_vertices

    def get_detected_anchor_corner(self, anchor, detected_grid_corners):
        min_dist = float('inf')
        detected_anchor = []
        for i in range(0, len(detected_grid_corners)):
            detected_corner = detected_grid_corners[i][0]

            distance = (anchor[0] - detected_corner[0]) ** 2 + (anchor[1] - detected_corner[1]) ** 2

            if distance < min_dist:
                min_dist = distance
                detected_anchor = detected_corner

        return [detected_anchor]

    def find_corners_and_edges_of_vertex_grid(self, vertices, image):
        #print('debug: entered find corners of vertex grid')
        detected_hull = cv2.convexHull(vertices)
        #print('debug: got hull')

        #img_detected_hull = self.paint_image_of_hull(detected_hull, image)
        #cv2.imwrite('./img/res/img_detected_hull.jpg', img_detected_hull)
        #cv2.imwrite('./img/res/img_detected_vertices.jpg',
        #            self.vertex_detector.mark_corners(image, vertices, [0, 255, 0]))

        detected_edge_vertices = self.find_edge_vertices(vertices, detected_hull)

        #img2 = self.vertex_detector.mark_corners(image, detected_edge_vertices, [0, 255, 0])
        #cv2.imwrite('./img/res/img_detected_edge_corners.jpg', img2)

        center_of_mass = self.find_center_of_mass(detected_edge_vertices)
        sorted_edge_vertices = self.sort_edge_corners_around_center_of_mass(center_of_mass, detected_edge_vertices)

        #img2 = self.vertex_detector.mark_corners(image, sorted_edge_vertices, [0, 255, 0])
        #img2 = self.vertex_detector.mark_corners(img2, [[center_of_mass]], [0, 255, 255])
        #cv2.imwrite('./img/res/img_sorted_edge_corners.jpg', img2)

        detected_corners_of_vertex_grid = self.get_corners_from_sorted_edge_vertices(center_of_mass,
                                                                                     sorted_edge_vertices)

        #img2 = self.vertex_detector.mark_corners(image, detected_corners_of_vertex_grid, [0, 255, 0])
        #cv2.imwrite('./img/res/img_vertex_corners.jpg', img2)

        return detected_corners_of_vertex_grid, sorted_edge_vertices

    def distance_between_point_and_line(self, point, line_start_point, line_end_point):
        point_as_numpy = np.array(point)
        line_start_as_numpy = np.array(line_start_point)
        line_end_as_numpy = np.array(line_end_point)

        distance = np.linalg.norm(np.cross(line_end_as_numpy - line_start_as_numpy,
                                           point_as_numpy - line_start_as_numpy)) /\
                   np.linalg.norm(line_end_as_numpy - line_start_as_numpy)

        return distance

    def find_edge_vertices(self, vertices, convex_hull):
        leftover_vertices = copy.deepcopy(vertices.tolist())
        edge_vertices = []

        distance_threshold = 5  # in pixels

        for i in range(0, len(convex_hull)):
            if i < len(convex_hull) - 1:
                line_start_point = convex_hull[i][0]
                line_end_point = convex_hull[i + 1][0]
            else:
                line_end_point = convex_hull[0][0]
                line_start_point = convex_hull[i][0]

            for j in range(0, len(leftover_vertices)):
                vertex = leftover_vertices[j]
                if vertex in edge_vertices:
                    continue
                distance = self.distance_between_point_and_line(vertex[0], line_start_point, line_end_point)

                # #print('debug: vertex: ' + str(debug_list.index(vertex) + 1) + " to line between: " + str((i + 1, i+2)) + ' has distance = ' + str(distance))

                if abs(distance) < distance_threshold:
                    edge_vertices.append(vertex)
                    # leftover_vertices.remove(vertex)

        return edge_vertices

    def find_center_of_mass(self, edge_vertices):
        center_x = 0
        center_y = 0

        for i in range(0, len(edge_vertices)):
            center_x += edge_vertices[i][0][0]
            center_y += edge_vertices[i][0][1]

        center_x = center_x / len(edge_vertices)
        center_y = center_y / len(edge_vertices)

        return [center_x, center_y]

    def sort_edge_corners_around_center_of_mass(self, center_of_mass, edge_vertices):
        angles = []

        for i in range(0, len(edge_vertices)):
            x = edge_vertices[i][0][0] - center_of_mass[0]
            y = edge_vertices[i][0][1] - center_of_mass[1]
            angle = atan2(y, x)

            if angle <= 0:
                angle = 2 * pi + angle

            angles.append(angle)

        sorted_angles, sorted_vertices = zip(*sorted(zip(angles, edge_vertices)))

        return sorted_vertices

    def get_corners_from_sorted_edge_vertices(self, center_of_mass, sorted_edge_vertices):
        corners = []

        distances = []
        for i in range(0, len(sorted_edge_vertices)):
            vertex = sorted_edge_vertices[i][0]
            distance = ((vertex[0] - center_of_mass[0]) ** 2 + (vertex[1] - center_of_mass[1]) ** 2) ** 0.5
            distances.append(distance)

        if distances[0] > distances[1] and distances[0] > distances[-1]:
            corners.append(sorted_edge_vertices[0])

        for i in range(1, len(distances) - 1):
            if distances[i] > distances[i - 1] and distances[i] > distances[i + 1]:
                corners.append(sorted_edge_vertices[i])

        if distances[-1] > distances[0] and distances[-1] > distances[-2]:
            corners.append(sorted_edge_vertices[-1])

        return corners

    def paint_image_of_hull(self, hull, image):
        # img2 = self.vertex_detector.mark_corners(image, vertices, (255, 0, 0))
        for i in range(0, len(hull)):
            if i < len(hull) - 1:
                img2 = cv2.line(image, (hull[i][0][0], hull[i][0][1]), (hull[i + 1][0][0], hull[i + 1][0][1]),
                                (0, 255 / len(hull) * i, 255 / len(hull) * (len(hull) - i)), 1)
            else:
                img2 = cv2.line(img2, (hull[i][0][0], hull[i][0][1]), (hull[0][0][0], hull[0][0][1]),
                                (0, 255 / len(hull) * i, 255 / len(hull) * (len(hull) - i)), 1)

        img2 = self.vertex_detector.mark_corners(img2, hull, (0, 255, 0))

        return img2

    def match_world_points_to_detected(self, joint_state, world_points, detected_points, image):
        tf_matrix = self.model.evaluate_matrix(self.world_to_cam_tf, joint_state)

        analytic_point_dict = {}
        analytic_point_list = []

        for key in world_points.keys():
            world_point = world_points[key]
            world_point_vector = sympy.Matrix([world_point['x'], world_point['y'], world_point['z'], 1])
            analytic_point = self.world_to_pixel_coordinates(tf_matrix, world_point_vector)
            analytic_point = [analytic_point[0], analytic_point[1]]
            analytic_point_dict[str(analytic_point)] = key
            analytic_point_list = analytic_point_list + [[analytic_point]]

        match_tmp = self.match_analytic_and_detected_positions(analytic_point_list, detected_points, image)

        matches = {}
        for key in match_tmp.keys():
            match = match_tmp[key]

            analytic = match[0]
            detected = match[1]
            world_point_key = analytic_point_dict[str(analytic)]

            matches[world_point_key] = detected

        return matches


class ManualShutdown(Exception):
    pass