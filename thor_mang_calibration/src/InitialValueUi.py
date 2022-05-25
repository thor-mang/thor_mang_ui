#! /usr/bin/env python

import copy

from PyQt5.QtWidgets import QWidget, QLineEdit, QCheckBox, QGridLayout, QLabel, QGroupBox, QSpacerItem, QSizePolicy,\
    QRadioButton, QHBoxLayout, QVBoxLayout, QFrame, QScrollArea
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QValidator, QDoubleValidator

#import dill
#dill.settings['recurse'] = True


class InitialValueUi(QWidget):
    initial_values = pyqtSignal(dict)
    calibrate_joint = pyqtSignal(dict)

    calibrate_target_offset = pyqtSignal(list)
    initial_values_target_offset = pyqtSignal(list)

    def __init__(self, calibratable_joints, target_offsets, joint_offset_mins, joint_offset_maxs, old_offsets,
                 parent=None):
        super(InitialValueUi, self).__init__(parent)
        self.parent = parent

        self.calibratable_joints = calibratable_joints
        self.joint_offset_mins = joint_offset_mins
        self.joint_offset_maxs = joint_offset_maxs
        self.zeros = {str(i): 0 for i in self.calibratable_joints}
        self.old_joint_offsets = old_offsets
        self.custom = copy.deepcopy(self.zeros)

        self.target_offsets = target_offsets
        self.target_offset_initial_values = {}

        self.ui = QWidget(self.parent)

        self.ui.checkboxes = {}
        self.ui.lineedits = {}
        self.ui.joint_labels = {}

        self.targets = None

        self.startUI()

        if parent is not None:
            self.parent.chosen_target_cs.connect(self.handle_joints_to_calibrate)

    def startUI(self):
        self.ui.layout = QVBoxLayout()

        # radio buttons
        label = QLabel('Initial offset values for optimization: ')
        self.ui.minimum_radio_button = QRadioButton('Minimum')
        self.ui.maximum_radio_button = QRadioButton('Maximum')
        self.ui.zero_radio_button = QRadioButton('Zero')
        self.ui.custom_radio_button = QRadioButton('Custom')
        self.ui.old_offset_radio_button = QRadioButton('Old Offsets')

        self.ui.radio_button_layout = QHBoxLayout()

        self.ui.radio_button_layout.addWidget(label)
        self.ui.radio_button_layout.addWidget(self.ui.minimum_radio_button)
        self.ui.radio_button_layout.addWidget(self.ui.zero_radio_button)
        self.ui.radio_button_layout.addWidget(self.ui.maximum_radio_button)
        self.ui.radio_button_layout.addWidget(self.ui.old_offset_radio_button)
        self.ui.radio_button_layout.addWidget(self.ui.custom_radio_button)

        self.ui.layout.addLayout(self.ui.radio_button_layout)

        # line
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)

        self.ui.layout.addWidget(line)

        # layout for columns
        self.ui.limb_layout = QGridLayout()
        target_label = QLabel('Target:')
        involved_joints_label = QLabel('Involved joints:')
        target_offset_label = QLabel('Include offsets \nfor checkerboard:')

        self.ui.limb_layout.addWidget(target_label, 0, 0)
        self.ui.limb_layout.addWidget(involved_joints_label, 1, 0)
        self.ui.limb_layout.addWidget(target_offset_label, 2, 0)

        self.ui.layout.addLayout(self.ui.limb_layout)

        # spacer
        self.ui.layout.addItem(QSpacerItem(40, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))

        self.ui.setLayout(self.ui.layout)

        self.create_lineedits()
        self.set_radio_button_connections()

    def create_lineedits(self):
        for joint in self.calibratable_joints:
            checkbox = QCheckBox(str(joint) + ":")
            checkbox.setToolTip('<b>Check</b> to include the joint in the optimization.\n '
                                '<b>Uncheck</b> to exclude.')
            checkbox.stateChanged.connect(lambda x, y=str(joint): self.handle_check_boxes(x, y))
            checkbox.setTristate(False)
            checkbox.setChecked(True)

            lineedit = QLineEdit()
            lineedit.setMaximumWidth(50)
            lineedit.editingFinished.connect(lambda x=str(joint): self.handle_line_edits(x))

            self.ui.checkboxes[joint] = checkbox
            self.ui.lineedits[joint] = lineedit

    def set_initial_selections(self):
        #print('debug: setting initial values for initialValueUi')
        self.ui.custom_radio_button.click()

    def clear_ui(self):
        page_layout = self.ui.limb_layout

        for i in range(0, page_layout.rowCount()):
            for j in range(1, page_layout.columnCount()):
                item = page_layout.itemAtPosition(i, j)
                if item is None:
                    continue

                widget = item.widget()
                layout = item.layout()

                if widget is not None:
                    widget.setVisible(False)
                    page_layout.removeWidget(widget)
                    continue

                if layout is not None:
                    while layout.count():
                        item2 = layout.takeAt(0)
                        if item2.widget() is not None:
                            item2.widget().setVisible(False)
                    page_layout.removeItem(item)
                    continue

                page_layout.removeItem(item)

        page_layout.update()

    def set_radio_button_connections(self):
        self.ui.minimum_radio_button.clicked.connect(self.handle_radio_buttons)
        self.ui.maximum_radio_button.clicked.connect(self.handle_radio_buttons)
        self.ui.zero_radio_button.clicked.connect(self.handle_radio_buttons)
        self.ui.custom_radio_button.clicked.connect(self.handle_radio_buttons)
        self.ui.old_offset_radio_button.clicked.connect(self.handle_radio_buttons)

    def find_shared_joints(self, target_joint_lists):
        shared_joints_among_targets = []
        all_joints = []
        for i in range(0, len(target_joint_lists)):
            all_joints += target_joint_lists[i][1]
        all_joints = sorted(all_joints)

        for i in range(1, len(all_joints)):
            if all_joints[i - 1] == all_joints[i]:
                if all_joints[i] not in shared_joints_among_targets:
                    shared_joints_among_targets += [all_joints[i]]

        return shared_joints_among_targets

    def create_scroll_area(self, content_layout):
        scroll_area = QScrollArea()
        group_box = QGroupBox()

        content_layout.setSpacing(0)

        group_box.setLayout(content_layout)
        group_box.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)

        scroll_area.setWidget(group_box)
        scroll_area.setWidgetResizable(True)
        scroll_area.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)

        return scroll_area

    def add_joints_to_page(self, joint_list, exclude_joints, add_target_offset_options):
        target = joint_list[0]
        joints_involved = joint_list[1]

        layout = self.ui.limb_layout
        num_cols = layout.columnCount()

        label = QLabel(str(target))
        layout.addWidget(label, 0, num_cols)

        group_box_layout = QGridLayout()

        count = 0
        for joint in joints_involved:
            if joint not in exclude_joints:
                if joint not in self.ui.checkboxes.keys():
                    checkbox = QCheckBox(str(joint) + ":")
                    checkbox.setToolTip('<b>Check</b> to include the joint in the optimization.\n '
                                        '<b>Uncheck</b> to exclude.')
                    checkbox.stateChanged.connect(lambda x, y=str(joint): self.handle_check_boxes(x, y))
                    checkbox.setTristate(False)
                    checkbox.setChecked(True)

                    lineedit = QLineEdit()
                    lineedit.setMaximumWidth(50)
                    lineedit.editingFinished.connect(lambda x=str(joint): self.handle_line_edits(x))

                    self.ui.checkboxes[joint] = checkbox
                    self.ui.lineedits[joint] = lineedit

                else:
                    checkbox = self.ui.checkboxes[joint]
                    lineedit = self.ui.lineedits[joint]

                group_box_layout.addWidget(checkbox, count, 0)
                group_box_layout.addWidget(lineedit, count, 1)
                count += 1

        scroll_area = self.create_scroll_area(group_box_layout)
        layout.addWidget(scroll_area, 1, num_cols)

        if add_target_offset_options:
            self.add_target_offset_options_to_page(target, num_cols)

    def add_target_offset_options_to_page(self, target, column):
        layout = self.ui.limb_layout
        offsets = self.target_offsets

        if target not in self.target_offset_initial_values.keys():
            self.target_offset_initial_values[target] = {}

        group_box_layout = QGridLayout()
        for i in range(0, len(offsets)):
            name = str(target) + '_' + str(offsets[i])

            if name not in self.ui.checkboxes.keys():
                checkbox = QCheckBox(str(offsets[i]) + ":")
                checkbox.setToolTip('<b>Check</b> to include the selected potential target offset in the optimization.\n '
                                    '<b>Uncheck</b> to exclude.')
                checkbox.stateChanged.connect(lambda x, y=str(offsets[i]), z=str(target): self.handle_check_boxes_for_target_offsets(x, y, z))
                checkbox.setTristate(False)
                checkbox.setChecked(True)

                lineedit = QLineEdit('0')
                lineedit.setMaximumWidth(50)
                lineedit.editingFinished.connect(lambda x=str(target), y=str(offsets[i]): self.handle_line_edits_of_target_offsets(x, y))

                self.ui.checkboxes[name] = checkbox
                self.ui.lineedits[name] = lineedit

                self.target_offset_initial_values[target][offsets[i]] = 0
                self.initial_values_target_offset.emit([target, offsets[i], 0])
            else:
                checkbox = self.ui.checkboxes[name]
                lineedit = self.ui.lineedits[name]

            group_box_layout.addWidget(checkbox, i, 0)
            group_box_layout.addWidget(lineedit, i, 1)

        scroll_area = self.create_scroll_area(group_box_layout)
        layout.addWidget(scroll_area, 2, column)

    def handle_joints_to_calibrate(self, data):
        targets = [x[0] for x in data]

        if targets == self.targets:
            return

        self.clear_ui()

        shared_among_lists = self.find_shared_joints(data)

        for joint_list in data:
            self.add_joints_to_page(joint_list, shared_among_lists, True)

        if shared_among_lists:
            self.add_joints_to_page(['Shared', shared_among_lists], [], False)

        layout = self.ui.limb_layout
        new_num_columns = layout.columnCount()
        spacer = QSpacerItem(10, 20, QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addItem(spacer, 0, new_num_columns)

        self.targets = targets

    def handle_check_boxes(self, state, sender_key):
        calibrate_joint = True if state == 2 else False
        self.calibrate_joint.emit({sender_key: calibrate_joint})

    def handle_check_boxes_for_target_offsets(self, state, axis, relevant_target):
        calibrate_target_offset = True if state == 2 else False
        self.calibrate_target_offset.emit([relevant_target, axis, calibrate_target_offset])

    def handle_line_edits(self, sender_key):
        edit = self.ui.lineedits[sender_key]
        value_string = edit.text()
        valid, value = self._string_to_float(value_string)

        source = {}
        if sender_key in self.custom.keys():
            source = self.custom
        elif sender_key in self.target_offset_initial_values.keys():
            source = self.target_offset_initial_values

        if valid:
            if value != source[sender_key]:
                source[sender_key] = value
                self.initial_values.emit({sender_key: value})

        else:
            edit.setText(str(source[sender_key]))

    def handle_line_edits_of_target_offsets(self, relevant_target, axis):
        name = relevant_target + '_' + axis
        edit = self.ui.lineedits[name]
        value_string = edit.text()
        valid, value = self._string_to_float(value_string)

        #source = {}
        #if name in self.custom.keys():
        #    source = self.custom
        #elif name in self.target_offset_initial_values.keys():
        source = self.target_offset_initial_values

        if valid:
            if value != source[relevant_target][axis]:
                source[relevant_target][axis] = value
                self.initial_values_target_offset.emit([relevant_target, axis, value])

        else:
            edit.setText(str(source[name]))

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

    def handle_radio_buttons(self):
        desired_values = []
        use_custom = False

        if self.sender() == self.ui.minimum_radio_button:
            desired_values = self.joint_offset_mins
        elif self.sender() == self.ui.maximum_radio_button:
            desired_values = self.joint_offset_maxs
        elif self.sender() == self.ui.zero_radio_button:
            desired_values = self.zeros
        elif self.sender() == self.ui.custom_radio_button:
            desired_values = self.custom
            use_custom = True
        elif self.sender() == self.ui.old_offset_radio_button:
            desired_values = self.old_joint_offsets

        for joint in self.ui.lineedits.keys():
            edit = self.ui.lineedits[joint]

            if joint in desired_values.keys():
                value = desired_values[joint]
            elif joint in self.target_offset_initial_values.keys():
                if not use_custom:
                    self.target_offset_initial_values[joint] = 0
                value = self.target_offset_initial_values[joint]

            else:
                #print('debug: initial value page found no suitable value.')
                continue

            edit.setText(str(value))
            edit.setEnabled(use_custom)

        self.initial_values.emit(desired_values)
        self.initial_values.emit(self.target_offset_initial_values)
