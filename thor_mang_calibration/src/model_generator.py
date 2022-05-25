#! /usr/bin/env python

import sympy

import rospy

import threading
import time

from sensor_msgs.msg import CameraInfo


class Translation:
    x = None
    y = None
    z = None

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def _make_string(self):
        return '[' + str(self.x) + ', ' + str(self.y) + ', ' + str(self.z) + ']'

    def __str__(self):
        return self._make_string()

    def __repr__(self):
        return self._make_string()


class ModelGenerator:
    def __init__(self, joint_names, joint_info, links, camera_link_info):
        self.joint_names = joint_names
        self.joint_info = joint_info
        self.links = links
        self.camera_link = camera_link_info[0]
        self.camera_link_tf = camera_link_info[1]

        self.symbols = self.create_symbols_for_joints(self.joint_names)
        self.symbolic_vertex_vector = sympy.Matrix([sympy.Symbol('x'), sympy.Symbol('y'), sympy.Symbol('z'), 1])

        self.tfs = None
        self.tf_generating_thread_running = False
        self.tf_generating_thread = threading.Thread(target=self.create_tfs_for_joints)
        self.tf_generating_thread.start()

        ns = rospy.get_namespace()
        self.camera_info = rospy.wait_for_message(ns + 'sensor/head_cam/rgb/camera_info', CameraInfo)
        self.P = sympy.Matrix(self.camera_info.P).reshape(3, 4)

    def get_symbols(self):
        return self.symbols

    def get_symbols_as_list(self):
        symbol_list = []

        for joint in self.joint_names:
            symbol_list.append(self.symbols[joint])

        return symbol_list

    def get_symbolic_vertex_vector(self):
        return self.symbolic_vertex_vector

    def create_tfs_for_joints(self):
        self.tf_generating_thread_running = True
        tf_dict = {}
        for joint in self.joint_info.keys():
            info = self.joint_info[str(joint)]
            type = info['type']
            origin = info['origin_xyz']
            translation = Translation(origin[0], origin[1], origin[2])
            tf = None
            #print('debug: joint: ' + str(joint))
            if type == 'revolute':
                axis = info['axis']
                if axis:
                    if axis[0] != 0:
                        tf = self._roll_tf_mat(self.symbols[str(joint)], translation, axis[0])
                    elif axis[1] != 0:
                        tf = self._pitch_tf_mat(self.symbols[str(joint)], translation, axis[1])
                    elif axis[2] != 0:
                        tf = self._yaw_tf_mat(self.symbols[str(joint)], translation, axis[2])
                else:
                    tf = self._fixed_tf_mat(info['origin_rpy'], translation)
                    #tf = self._suppress_innaccuracies(tf)

            if type == 'fixed':
                origin_rpy = info['origin_rpy']
                tf = self._fixed_tf_mat(origin_rpy, translation)
                #tf = self._suppress_innaccuracies(tf)

            if tf is not None:
                tf_dict[str(joint)] = {}
                tf_dict[str(joint)]['from_child_to_parent'] = tf
                tf_dict[str(joint)]['from_parent_to_child'] = tf ** -1

        tf = None
        if self.camera_link_tf is not None:
            if 'translation' in self.camera_link_tf.keys():
                trans = self.camera_link_tf['translation']
                translation = Translation(trans[0], trans[1], trans[2])
            else:
                translation = Translation(0, 0, 0)

            if 'rotation' in self.camera_link_tf.keys():
                rot = self.camera_link_tf['rotation']
                if 'rpy' in rot.keys():
                    rpy = rot['rpy']
                    tf = self._fixed_tf_mat(rpy, translation)
                # @TODO add quaternion option

        if tf is not None:
            #tf = self._suppress_innaccuracies(tf)
            tf_dict['camera'] = {}
            tf_dict['camera']['from_child_to_parent'] = tf
            tf_dict['camera']['from_parent_to_child'] = tf ** -1

        #print('debug: tf_dict: ' + str(tf_dict))

        self.tfs = tf_dict
        self.tf_generating_thread_running = False

    def transform_link1_to_link2(self, link1, link2):
        if self.tf_generating_thread_running:
            self.tf_generating_thread.join()

        path_link1_to_pelvis = self.find_path(link1)
        path_link2_to_pelvis = self.find_path(link2)
        path_link2_to_pelvis.reverse()
        path_conjunction = self.find_conjunction(path_link1_to_pelvis, path_link2_to_pelvis)

        tf_list = []
        final_path_links = []
        final_path_joints = []
        for i in range(0, len(path_link1_to_pelvis)):
            link = path_link1_to_pelvis[i]

            if str(link) == path_conjunction:
                break

            joint = self.links[str(link)]['parent_joint']
            tf = self.tfs[str(joint)]['from_child_to_parent']
            tf_list.append(tf)
            final_path_links.append(link)
            final_path_joints.append(joint)

        in_path = False
        for i in range(0, len(path_link2_to_pelvis)):
            link = path_link2_to_pelvis[i]
            if str(link) == path_conjunction:
                in_path = True
                continue

            if in_path:
                joint = self.links[str(link)]['parent_joint']
                tf = self.tfs[str(joint)]['from_parent_to_child']
                tf_list.append(tf)
                final_path_links.append(link)
                final_path_joints.append(joint)

        tf = tf_list[0]

        for i in range(1, len(tf_list)):
            tf = tf_list[i] * tf

        return tf

    def get_list_of_tfs_from_link_to_camera(self, link):
        if self.tf_generating_thread_running:
            self.tf_generating_thread.join()

        path_pelvis_to_camera = self.find_path(self.camera_link)
        path_pelvis_to_camera.reverse()
        path_link_to_pelvis = self.find_path(link)
        path_conjunction = self.find_conjunction(path_link_to_pelvis, path_pelvis_to_camera)

        tf_list = []
        joint_list = []
        for i in range(0, len(path_link_to_pelvis)):
            link = path_link_to_pelvis[i]
            if str(link) == path_conjunction:
                break

            joint = self.links[str(link)]['parent_joint']
            tf = self.tfs[str(joint)]['from_child_to_parent']
            tf_list.append(tf)
            if self.joint_info[joint]['type'] != 'fixed':
                if self.joint_info[joint]['axis']:
                    joint_list.append(joint)

        in_path = False
        for i in range(0, len(path_pelvis_to_camera)):
            link = path_pelvis_to_camera[i]
            if str(link) == path_conjunction:
                in_path = True
                continue

            if in_path:
                joint = self.links[str(link)]['parent_joint']
                tf = self.tfs[str(joint)]['from_parent_to_child']
                tf_list.append(tf)
                if self.joint_info[joint]['type'] != 'fixed':
                    if self.joint_info[joint]['axis']:
                        joint_list.append(joint)

        if 'camera' in self.tfs.keys():
            tf_list.append(self.tfs['camera']['from_parent_to_child'])

        return tf_list, joint_list

    def tf_matrix_from_tf_list(self, tf_list):
        if self.tf_generating_thread_running:
            self.tf_generating_thread.join()

        tf = tf_list[0]
        for i in range(1, len(tf_list)):
            tf = tf_list[i] * tf

        return tf

    def find_path(self, link):
        if self.tf_generating_thread_running:
            self.tf_generating_thread.join()

        path_link_to_imu = []

        current_link = link
        search = True
        while search:
            path_link_to_imu.append(current_link)
            if self.links.has_key(current_link):
                parent_joint = self.links[current_link]['parent_joint']
                current_link = self.joint_info[parent_joint]['parent_link']
            else:
                search = False

        return path_link_to_imu

    def find_conjunction(self, path1, path2):
        paths_meet_in_link = ''
        for i in range(0, len(path1)):
            to_pelvis_link = path1[i]
            for j in range(0, len(path2)):
                if to_pelvis_link == path2[j]:
                    paths_meet_in_link = to_pelvis_link
                    break
            if paths_meet_in_link != '':
                break

        if paths_meet_in_link == '':
            print('debug: no path found from ' + str(path1[0]) + ' to ' + str(path2[-1]))
            return None

        return paths_meet_in_link

    def link_to_camera_coordinates_formula(self, link):
        if self.tf_generating_thread_running:
            self.tf_generating_thread.join()

        tf_list, joint_list = self.get_list_of_tfs_from_link_to_camera(link)

        # symbolic vector points (homogeneous) in link's coordinate system
        formula = self.symbolic_vertex_vector #sympy.Matrix([sympy.Symbol('x'), sympy.Symbol('y'), sympy.Symbol('z'), 1])

        # multiply symbolic vector with the tf from each link up to the camera
        # to gain symbolic vector points in camera coordinates
        for i in range(0, len(tf_list)):
            formula = tf_list[i] * formula

        return formula, joint_list

    def camera_to_image_coordinates_formula(self, camera_coordinates):
        image_coordinates = self.P * camera_coordinates

        image_coordinates = image_coordinates / image_coordinates[2, 0]

        return image_coordinates

    def create_image_coordinate_formula(self, link):
        if self.tf_generating_thread_running:
            self.tf_generating_thread.join()

        formula, joint_list = self.link_to_camera_coordinates_formula(link)

        # multiplication with camera matrix for projection
        #formula = self.P * formula

        # division for image coordinates x, y
        #formula = formula / formula[2, 0]

        formula = self.camera_to_image_coordinates_formula(formula)

        return formula[0], formula[1], joint_list

    def get_tf_mat_translation_first(self, roll, pitch, yaw, x, y, z):
        translation = Translation(x, y, z)
        zero_trans = Translation(0, 0, 0)

        trans_tf = self._roll_tf_mat(0, translation, 1)  # axis irrelevant with 0 deg rotation

        roll_tf = self._roll_tf_mat(roll, zero_trans, 1)
        pitch_tf = self._pitch_tf_mat(pitch, zero_trans, 1)
        yaw_tf = self._yaw_tf_mat(yaw, zero_trans, 1)

        all_tf = yaw_tf * pitch_tf * roll_tf * trans_tf

        return all_tf

    def get_tf_mat_rotation_first(self, roll, pitch, yaw, x, y, z):
        translation = Translation(x, y, z)
        zero_trans = Translation(0, 0, 0)

        trans_tf = self._roll_tf_mat(0, translation, 1)  # axis irrelevant with 0 deg rotation

        roll_tf = self._roll_tf_mat(roll, zero_trans, 1)
        pitch_tf = self._pitch_tf_mat(pitch, zero_trans, 1)
        yaw_tf = self._yaw_tf_mat(yaw, zero_trans, 1)

        all_tf = trans_tf * yaw_tf * pitch_tf * roll_tf

        return all_tf

# _____________________________________ helper functions _______________________________________________________________

    def _suppress_innaccuracies(self, matrix, threshold_for_zero=1e-3, threshold_for_one=0.999):
        for i in range(0, matrix.shape[0]):
            for j in range(0, matrix.shape[1]):
                entry = matrix[i, j]
                if abs(entry) < threshold_for_zero:
                    matrix[i, j] = 0
                elif threshold_for_one < entry < 1.0:
                    matrix[i, j] = 1
                elif -threshold_for_one > entry > -1.0:
                    matrix[i, j] = -1
        return matrix

    def matrix_to_string(self, matrix):
        string = '['
        rows = matrix.shape[1]
        for i in range(0, rows):
            string = string + str(matrix.row(i))[8:-2] + '\n '
        string = string[:-2] + ']'

        return string

    def create_symbols_for_joints(self, joints):
        symbols = {}
        count = 0
        for joint in joints:
            symbols[joint] = sympy.Symbol('a[' + str(count) + ']')
            count += 1

        return symbols

    def _fixed_tf_mat(self, rot, translation):
        x = translation.x
        y = translation.y
        z = translation.z

        roll = rot[0]
        pitch = rot[1]
        yaw = rot[2]

        tf = sympy.Matrix([[+sympy.cos(yaw), -sympy.sin(yaw), 0, 0],
                           [+sympy.sin(yaw), +sympy.cos(yaw), 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

        tmp = sympy.Matrix([[+sympy.cos(pitch), 0, +sympy.sin(pitch), 0],
                            [0, 1, 0, 0],
                            [-sympy.sin(pitch), 0, +sympy.cos(pitch), 0],
                            [0, 0, 0, 1]])

        tf = tf * tmp

        tmp = sympy.Matrix([[1, 0, 0, 0],
                            [0, sympy.cos(roll), -sympy.sin(roll), 0],
                            [0, sympy.sin(roll), sympy.cos(roll), 0],
                            [0, 0, 0, 1]])

        tf = tf * tmp

        tmp = sympy.Matrix([[1, 0, 0, x],
                            [0, 1, 0, y],
                            [0, 0, 1, z],
                            [0, 0, 0, 1]])

        tf = tmp * tf

        return tf

    def _yaw_tf_mat(self, symbol, translation, sign):
        x = translation.x
        y = translation.y
        z = translation.z
        tf = sympy.Matrix([[+sympy.cos(sign * symbol), -sympy.sin(sign * symbol), 0, x],
                           [+sympy.sin(sign * symbol), +sympy.cos(sign * symbol), 0, y],
                           [0, 0, 1, z],
                           [0, 0, 0, 1]])

        return tf

    def _pitch_tf_mat(self, symbol, translation, sign):
        # theta = sympy.Symbol(str(symbol))
        x = translation.x
        y = translation.y
        z = translation.z

        tf = sympy.Matrix([[+sympy.cos(sign * symbol), 0, +sympy.sin(sign * symbol), x],
                           [0, 1, 0, y],
                           [-sympy.sin(sign * symbol), 0, +sympy.cos(sign * symbol), z],
                           [0, 0, 0, 1]])

        return tf

    def _roll_tf_mat(self, symbol, translation, sign):
        # theta = sympy.Symbol(str(symbol))
        x = translation.x
        y = translation.y
        z = translation.z
        tf = sympy.Matrix([[1, 0, 0, x],
                           [0, +sympy.cos(sign * symbol), -sympy.sin(sign * symbol), y],
                           [0, +sympy.sin(sign * symbol), +sympy.cos(sign * symbol), z],
                           [0, 0, 0, 1]])

        return tf
