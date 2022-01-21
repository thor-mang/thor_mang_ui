#! /usr/bin/env python

import os

from math import radians, degrees
import sympy
import sympy.vector
import numpy as np
import copy
import threading

import rospy
import rospkg
import rviz

from PyQt5.QtWidgets import QWidget, QComboBox, QGridLayout, QPushButton, QLabel, QWidgetItem, QSizePolicy, QMessageBox
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QValidator, QDoubleValidator, QPixmap, QImage

from tab_widget import TabWidget
from ImageWidget import ImageWidget

from yaml import load
#import dill
#dill.settings['recurse'] = True

from sensor_msgs.msg import CameraInfo


class PoseDefinitionUi(QWidget):
    preview_pose = pyqtSignal(dict)
    save_poses = pyqtSignal(list)

    def __init__(self, existing_poses, calibratable_joints, joint_info, world_points, world_point_definition, parent=None):
        super(PoseDefinitionUi, self).__init__(parent)

        self.parent = parent

        self.show_save_window = False

        self.poses = existing_poses

        self.calibratable_joints = calibratable_joints
        self.joint_info = joint_info

        self.world_points = copy.deepcopy(world_points)
        self.world_point_definition = world_point_definition

        self.camera_info = None
        self.P = None
        self.wait_for_camera_info_thread = threading.Thread(target=self.wait_for_camera_info)
        self.wait_for_camera_info_thread.start()

        self.zero_pose = {}
        for joint in self.calibratable_joints:
            self.zero_pose[str(joint)] = 0

        self.current_pose = {}

        self.target_cs = []
        self.involved_joints = {}
        self.models = {}

        self.ui = QWidget()

        rp = rospkg.RosPack()
        file_path = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'config',
                                 'joint_overview_config.yaml')
        f = open(file_path, 'r')
        self.joint_tab_config = load(f)
        f.close()

        self.startUi()

        self.connect_to_signals()
        self.wait_for_camera_info_thread.join()

    def wait_for_camera_info(self):
        ns = rospy.get_namespace()
        self.camera_info = rospy.wait_for_message(ns + 'sensor/head_cam/rgb/camera_info', CameraInfo)
        self.P = sympy.Matrix(self.camera_info.P).reshape(3, 4)

    def connect_to_signals(self):
        self.parent.chosen_target_cs.connect(self.log_target_cs)
        self.parent.models_for_target_cs.connect(self.log_models)

    def startUi(self):
        self.ui.joint_tab_widget = TabWidget()
        self.ui.tab_line_edits = self.ui.joint_tab_widget.setup_tab_widget(self.joint_tab_config, 1, False)[0]
        self.ui.tab_labels = self.match_labels_to_edits(self.ui.tab_line_edits)

        self.ui.page_layout = QGridLayout()

        self.ui.rviz_frame = self._create_rviz_frame()
        self.ui.rviz_frame.getManager().getRootDisplayGroup().getDisplayAt(1).setValue(True)
        self.ui.rviz_frame.setVisible(True)
        self.ui.rviz_frame.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        self.ui.page_layout.addWidget(self.ui.rviz_frame, 0, 0)

        self.ui.corner_preview_label_descriptor = QLabel('Preview of checkerboard vertices.')
        self.ui.corner_preview_visibility_label = QLabel('Visibility: ')
        self.ui.corner_preview_visibility_label = QLabel()
        self.ui.corner_preview_descriptor_layout = QGridLayout()
        self.ui.corner_preview_descriptor_layout.addWidget(self.ui.corner_preview_label_descriptor, 0, 0)
        self.ui.corner_preview_descriptor_layout.addWidget(self.ui.corner_preview_visibility_label, 1, 0)
        self.ui.page_layout.addLayout(self.ui.corner_preview_descriptor_layout, 1, 0)

        self.ui.corner_preview_label = ImageWidget(self)
        self.ui.corner_preview_label.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        self.ui.page_layout.addWidget(self.ui.corner_preview_label, 2, 0)

        # camera info thread
        self.wait_for_camera_info_thread.join()
        width = self.camera_info.width
        height = self.camera_info.height
        preview_image = QImage(width, height, QImage.Format_RGB32)
        preview_image.fill(Qt.gray)
        pixmap = QPixmap()
        pixmap.convertFromImage(preview_image)
        self.ui.corner_preview_label.setPixmap(pixmap)

        self.ui.limb_selection_label = QLabel('Select the final link for pose definition: ')
        self.ui.limb_combobox = QComboBox()
        self.ui.limb_combobox.setSizeAdjustPolicy(QComboBox.AdjustToContents)

        self.ui.pose_selection_label = QLabel('Available poses for final link: ')
        self.ui.available_poses_combobox = QComboBox()

        self.ui.save_pose_button = QPushButton('Save')
        self.ui.save_pose_button.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        self.ui.edit_pose_button = QPushButton('Edit')
        self.ui.edit_pose_button.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        self.ui.delete_pose_button = QPushButton('Delete')
        self.ui.delete_pose_button.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)

        self.ui.pose_selection_layout = QGridLayout()
        self.ui.pose_selection_layout.addWidget(self.ui.limb_selection_label, 0, 0, 1, 2)
        self.ui.pose_selection_layout.addWidget(self.ui.limb_combobox, 0, 2, 1, 3)
        self.ui.pose_selection_layout.addWidget(self.ui.pose_selection_label, 1, 0)
        self.ui.pose_selection_layout.addWidget(self.ui.available_poses_combobox, 1, 1)
        self.ui.pose_selection_layout.addWidget(self.ui.save_pose_button, 1, 2)
        self.ui.pose_selection_layout.addWidget(self.ui.edit_pose_button, 1, 3)
        self.ui.pose_selection_layout.addWidget(self.ui.delete_pose_button, 1, 4)
        self.ui.pose_selection_layout.addWidget(self.ui.joint_tab_widget, 2, 0, 1, 5)

        self.ui.page_layout.addLayout(self.ui.pose_selection_layout, 0, 1, 3, 1)

        self.ui.setLayout(self.ui.page_layout)

        if self.target_cs is not []:
            for i in range(0, len(self.target_cs)):
                self.ui.limb_combobox.addItem(self.target_cs[i])

        self.connect_to_ui_elements()

    def _create_rviz_frame(self):
        rviz_frame = rviz.VisualizationFrame()
        rviz_frame.setSplashPath('')
        rviz_frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        rp = rospkg.RosPack()
        reader.readFile(config,
                        os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'config', 'rviz_config.rviz'))
        rviz_frame.load(config)
        rviz_frame.setMenuBar(None)
        rviz_frame.setStatusBar(None)
        rviz_frame.setHideButtonVisibility(False)

        return rviz_frame

    def connect_to_ui_elements(self):
        self.ui.limb_combobox.currentIndexChanged.connect(self.handle_target_selection)
        self.ui.available_poses_combobox.currentIndexChanged.connect(self.handle_pose_selection)
        self.ui.edit_pose_button.clicked.connect(self.handle_edit_pose_button)
        self.ui.save_pose_button.clicked.connect(self.handle_save_pose_button)
        self.ui.delete_pose_button.clicked.connect(self.handle_delete_pose_button)

        for key in self.ui.tab_line_edits.keys():
            edit = self.ui.tab_line_edits[key]
            edit.editingFinished.connect(lambda x=str(key): self.handle_edits(x))

    def log_target_cs(self, msg):
        self.target_cs = []
        for i in range(0, len(msg)):
            entry = msg[i]
            name = str(entry[0])
            joint_list = entry[1]

            self.target_cs.append(name)
            if name not in self.involved_joints.keys():
                self.involved_joints[name] = joint_list

        if hasattr(self.ui, 'limb_combobox'):
            self.ui.limb_combobox.clear()
            for i in range(0, len(self.target_cs)):
                self.ui.limb_combobox.addItem(self.target_cs[i])

    def log_models(self, msg):
        for i in range(0, len(msg)):
            entry = msg[i]
            name = str(entry[0])
            x = entry[1][0]
            y = entry[1][1]
            link_to_camera_coordinates = entry[1][2]

            if name not in self.models.keys():
                self.models[name] = {}
                self.models[name]['x_function'] = x
                self.models[name]['y_function'] = y
                self.models[name]['link_to_camera_coordinates_function'] = link_to_camera_coordinates

    def handle_target_selection(self):
        link = str(self.ui.limb_combobox.currentText())

        if link == '':
            return

        if link in self.poses.keys():
            poses = self.poses[link]
            self.ui.available_poses_combobox.clear()
            self.ui.available_poses_combobox.addItem('New')
            for key in sorted(poses.keys()):
                self.ui.available_poses_combobox.addItem(str(key))
        else:
            self.poses[link] = {}
            self.ui.available_poses_combobox.clear()
            self.ui.available_poses_combobox.addItem('New')

        joint_list = self.involved_joints[link]
        self.show_involved_joints(joint_list)

    def match_labels_to_edits(self, edits):
        labels = {}

        for key in edits.keys():
            edit = edits[str(key)]
            layout = edit.parent().layout()

            label = None
            for i in range(0, layout.count()):
                child = layout.itemAt(i)

                if type(child) is QWidgetItem:
                    widget = child.widget()
                    if type(widget) is QLabel:
                        text = str(widget.text())[:-2]
                        if text == key:
                            label = widget
                            break
            if label is not None:
                labels[key] = label

        return labels

    def show_involved_joints(self, joint_list):
        for key in self.ui.tab_line_edits.keys():
            edit = self.ui.tab_line_edits[str(key)]
            if key not in joint_list:
                color = "color: darkgrey;"
            else:
                color = "color: limegreen;"
            edit.setStyleSheet(color)
            self.ui.tab_labels[key].setStyleSheet(color)

    def handle_edits(self, sender_key):
        edit = self.ui.tab_line_edits[sender_key]
        edit.clearFocus()

        text = edit.text()
        valid, value = self._string_to_float(text)
        if value != self.current_pose[sender_key]:
            if valid:
                upper_limit = self.joint_info[sender_key]['limits']['max']
                lower_limit = self.joint_info[sender_key]['limits']['min']
                if radians(value) > upper_limit:
                    value = round(degrees(upper_limit), 2)
                    print(str(sender_key) + ' has an upper limit of ' + str(value) + ' degrees!')
                elif radians(value) < lower_limit:
                    value = round(degrees(lower_limit), 2)
                    print(str(sender_key) + ' has a lower limit of ' + str(value) + ' degrees!')
                self.current_pose[sender_key] = value
            else:
                value = self.current_pose[sender_key]

            edit.setText(str(value))
        else:
            return
        self.preview_pose.emit(self.current_pose)
        current_limb = str(self.ui.limb_combobox.currentText())
        self.show_approximate_vertex_position(current_limb)

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

    def handle_pose_selection(self):
        pose_name = str(self.ui.available_poses_combobox.currentText())
        target_limb = str(self.ui.limb_combobox.currentText())
        if pose_name == '':
            return

        if pose_name != 'New':
            self.ui.edit_pose_button.setEnabled(True)
            self.ui.save_pose_button.setEnabled(False)
            self.ui.delete_pose_button.setEnabled(True)

            pose = copy.deepcopy(self.poses[target_limb][pose_name])
            self.enable_edits(False)
        else:
            self.ui.edit_pose_button.setEnabled(False)
            self.ui.save_pose_button.setEnabled(True)
            self.ui.delete_pose_button.setEnabled(False)

            pose = copy.deepcopy(self.zero_pose)
            self.enable_edits(True)

        self.current_pose = pose

        self.set_edits_to_pose_values(pose)
        self.preview_pose.emit(pose)
        self.show_approximate_vertex_position(target_limb)

    def enable_edits(self, enable):
        for key in self.ui.tab_line_edits.keys():
            edit = self.ui.tab_line_edits[str(key)]
            edit.setEnabled(enable)

    def set_edits_to_pose_values(self, pose):
        for key in pose.keys():
            edit = self.ui.tab_line_edits[str(key)]
            value = pose[str(key)]
            edit.setText(str(value))

    def handle_edit_pose_button(self):
        self.ui.edit_pose_button.setEnabled(False)
        self.ui.save_pose_button.setEnabled(True)

        self.enable_edits(True)

    def handle_save_pose_button(self):
        self.ui.edit_pose_button.setEnabled(True)
        self.ui.save_pose_button.setEnabled(False)
        self.enable_edits(False)

        current_target_link = str(self.ui.limb_combobox.currentText())
        current_pose = str(self.ui.available_poses_combobox.currentText())
        num_poses = len(self.poses[current_target_link].keys())
        insert_at = num_poses + 1

        if current_pose == 'New':
            pose_name = 'Pose ' + str(num_poses)
            if pose_name in self.poses[current_target_link].keys():
                for i in range(0, num_poses):
                    if 'Pose ' + str(i) not in self.poses[current_target_link].keys():
                        pose_name = 'Pose ' + str(i)
                        insert_at = i + 1
                        break
        else:
            pose_name = current_pose

        self.poses[current_target_link][pose_name] = self.current_pose

        if current_pose == 'New':
            self.ui.available_poses_combobox.insertItem(insert_at, pose_name)
            self.ui.available_poses_combobox.setCurrentIndex(insert_at)

        self.show_save_window = True

    def handle_delete_pose_button(self):
        current_target_link = str(self.ui.limb_combobox.currentText())
        current_entry = str(self.ui.available_poses_combobox.currentText())
        current_index = self.ui.available_poses_combobox.currentIndex()

        self.poses[current_target_link].pop(current_entry)

        self.ui.available_poses_combobox.setCurrentIndex(current_index - 1)
        self.ui.available_poses_combobox.removeItem(current_index)

        self.show_save_window = True

    def sort_pose(self, pose):
        a = []
        for joint in self.calibratable_joints:
            a.append(radians(pose[joint]))
        return a

    def calculate_position_of_world_points_in_image(self, target_limb):
        if str(target_limb) in self.world_points.keys():
            world_points = self.world_points[str(target_limb)]
        else:
            #print('debug: no measured world_points found for: ' + str(target_limb))
            return None

        pose = self.current_pose

        if str(target_limb) not in self.models.keys():
            return None

        a = self.sort_pose(pose)
        model = self.models[str(target_limb)]['link_to_camera_coordinates_function']

        image_points = []

        for world_point in world_points:
            camera_coordinate_vector = model(a, world_point)
            image_coordinates = self.project_camera_coordinate_vector(camera_coordinate_vector)

            image_points.append([image_coordinates[0], image_coordinates[1]])

        return image_points

    def project_camera_coordinate_vector(self, camera_coordinate_vector):
        image_coordinates = self.P * camera_coordinate_vector
        image_coordinates = image_coordinates / image_coordinates[2, 0]

        return image_coordinates

    def get_checkerboard_normal(self, world_points, rows, model, pose):
        origin_point = model(pose, world_points[0])
        x_dir_point = model(pose, world_points[1])
        y_dir_point = model(pose, world_points[rows])

        origin_point = origin_point[0:3] / origin_point[3]
        x_dir_point = x_dir_point[0:3] / x_dir_point[3]
        y_dir_point = y_dir_point[0:3] / y_dir_point[3]

        # x- & y-axis of world points in camera coordinates
        x_vector = x_dir_point - origin_point
        y_vector = y_dir_point - origin_point

        x_vector = self.normalize_vector(x_vector)
        y_vector = self.normalize_vector(y_vector)

        z_vec = self.cross_product_3d_inhomogenous(x_vector, y_vector)
        z_vec = self.normalize_vector(z_vec)

        return z_vec

    def check_visibility(self, target_limb):
        world_points = {}
        if str(target_limb) in self.world_points.keys():
            world_points = self.world_points[str(target_limb)]
        else:
            #print('debug: no measured world_points found for: ' + str(target_limb))
            return None

        if str(target_limb) not in self.models.keys():
            return None

        model = self.models[str(target_limb)]['link_to_camera_coordinates_function']
        pose = self.current_pose
        a = self.sort_pose(pose)
        rows = self.world_point_definition[target_limb]['checkerboard']['rows']
        cols = self.world_point_definition[target_limb]['checkerboard']['cols']

        checkerboard_normal = self.get_checkerboard_normal(world_points, rows, model, a)
        line_of_sight = sympy.Matrix([0, 0, 1])

        dot_product = self.dot_product(checkerboard_normal, line_of_sight)
        angle = np.degrees(float(sympy.acos(dot_product)))

        #print('debug: angle between checkerboard normal and line of sight: ' + str(angle))

        range_for_good_visibility = 80
        range_for_bad_visibility = 90
        angle_visibility = 'INVISIBLE'
        if 180 - range_for_good_visibility < angle < 180 + range_for_good_visibility: # +1 bc python range
            angle_visibility = 'GOOD'
        elif 180 - range_for_bad_visibility < angle < 180 + range_for_bad_visibility:
            angle_visibility = 'BAD'

        # 0, rows - 1, rows * (cols - 1), rows * cols - 1
        origin_corner = model(a, world_points[0])
        x_corner = model(a, world_points[rows - 1])
        y_corner = model(a, world_points[rows * (cols - 1)])
        x_y_corner = model(a, world_points[rows * cols - 1])

        origin_corner = self.project_camera_coordinate_vector(origin_corner)
        x_corner = self.project_camera_coordinate_vector(x_corner)
        y_corner = self.project_camera_coordinate_vector(y_corner)
        x_y_corner = self.project_camera_coordinate_vector(x_y_corner)

        corners = [origin_corner, x_corner, y_corner, x_y_corner]

        width = self.camera_info.width
        height = self.camera_info.height

        invisible_corners = 0

        for i in range(0, len(corners)):
            corner = corners[i]
            if not (0 < corner[0] <= width and 0 < corner[1] <= height):
                invisible_corners += 1

        if invisible_corners == 0:
            corner_visibility = 'FULL'
        elif invisible_corners == 4:
            corner_visibility = 'INVISIBLE'
        else:
            corner_visibility = 'PARTIAL'

        return [angle_visibility, corner_visibility]

    def normalize_vector(self, vector):
        vector_sum = 0
        for i in range(0, len(vector)):
            vector_sum += vector[i]**2

        vector_sum = vector_sum ** 0.5
        vector = vector / vector_sum

        return vector

    def dot_product(self, vector1, vector2):
        if len(vector1) != len(vector2):
            #print('debug: dot_produt: len(vector1) != len(vector2).')
            return None

        dot_product = 0
        for i in range(0, len(vector1)):
            dot_product += vector1[i] * vector2[i]

        return float(dot_product)

    def cross_product_3d_inhomogenous(self, vec1, vec2):
        e0 = vec1[1] * vec2[2] - vec1[2] * vec2[1]
        e1 = vec1[2] * vec2[0] - vec1[0] * vec2[2]
        e2 = vec1[0] * vec2[1] - vec1[1] * vec2[0]

        return sympy.Matrix([e0, e1, e2])

    def visibility_tags_to_text(self, visibility):
        angle = visibility[0]
        corners = visibility[1]
        text = 'Angle: ' + str(angle.lower()) + ' - Corners: ' + str(corners.lower())
        self.ui.corner_preview_visibility_label.setText(text)

    def show_approximate_vertex_position(self, target_limb):
        if not self.ui.isVisible():
            return

        width_orig = self.camera_info.width
        height_orig = self.camera_info.height
        width = self.ui.corner_preview_label.width()
        height = self.ui.corner_preview_label.height()

        image = np.zeros((height, width, 3), np.uint8) # black image

        visibility = self.check_visibility(target_limb)
        self.visibility_tags_to_text(visibility)

        if visibility[0] == 'INVISIBLE' or visibility is None:
            self.set_corner_preview_label_image(image)
            return

        image_coordinates = self.calculate_position_of_world_points_in_image(target_limb)

        if image_coordinates is None:
            self.set_corner_preview_label_image(image)
            return

        area = 3
        if visibility[0] == 'GOOD' and visibility[1] == 'FULL':
            color = [0, 255, 0]  # green
        elif visibility[0] == 'GOOD' and visibility[1] == 'PARTIAL':
            color = [230, 115, 0]  # orange
        else:
            color = [255, 0, 0]  # red #QColor(Qt.red).rgb()

        #rows = self.world_point_definition[target_limb]['checkerboard']['rows']
        #cols = self.world_point_definition[target_limb]['checkerboard']['cols']
        #corner_indices = [0, rows - 1, rows * (cols - 1), rows * cols - 1]
        #color_save = copy.deepcopy(color)

        for i in range(0, len(image_coordinates)):
            #color = color_save
            #if i in corner_indices:
            #    color = [0, 0, 255]

            point = image_coordinates[i]
            x = int(point[0] * width / width_orig)
            y = int(point[1] * height / height_orig)

            if not (0 < x <= width and 0 < y <= height):
                continue

            for j in range(- int(area / 2), int(area / 2) + 1):
                for k in range(- int(area / 2), int(area / 2) + 1):
                    a = x + j
                    b = y + k

                    if not (a < 0 or b < 0 or a >= width or b >= height):
                        image[b, a] = color #[0, 0, 255]
                        #image.setPixel(a, b, color)

            #if x + 10 <= width and y + 10 <= height:
            #    cv2.putText(image, str(i + 1), (x + 10, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        self.set_corner_preview_label_image(image)

    def set_corner_preview_label_image(self, image):
        img = QImage(image.data, image.shape[1], image.shape[0], 3 * image.shape[1],
                     QImage.Format_RGB888)  # .rgbSwapped()
        pixmap = QPixmap.fromImage(img)
        pixmap = pixmap.scaled(image.shape[1], image.shape[0])

        self.ui.corner_preview_label.setPixmap(pixmap)

    def closeEvent(self, event):

        if self.show_save_window:
            reply = QMessageBox.question(self, "Save", "Do you wish to save the poses?\nWarning: Will override file",
                                         QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

            if reply == QMessageBox.Yes:
                #print('debug: would like to save - poses')
                self.save_poses.emit([self.poses, 'automatic_calibration_poses.yaml'])
            #else:
                #print('debug: would like to discard')

        super(PoseDefinitionUi, self).closeEvent(event)
