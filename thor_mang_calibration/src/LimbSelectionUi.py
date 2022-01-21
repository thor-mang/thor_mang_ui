#! /usr/bin/env python

import os
import rospkg

from python_qt_binding import loadUi
from PyQt5.QtWidgets import QApplication, QWidget, QWidgetItem, QMessageBox
from PyQt5.QtCore import pyqtSignal, Qt


class AutomaticOption:

    def __init__(self, name):
        self.name = name
        self.indent = 0
        self.list_position = None
        self.top_entry = None
        self.subentries = []
        self.panels = []

    def to_dict(self):
        option = {'name': str(self.name), 'indent': self.indent, 'list_position': self.list_position,
                  'panels': self.panels_to_dict()}
        if self.top_entry is not None:
            option['top_entry'] = str(self.top_entry.name)
        else:
            option['top_entry'] = None

        return option

    def panels_to_dict(self):
        panels = []

        for panel in self.panels:
            panels.append(panel.to_dict())

        return panels

    def create_string(self):
        string = '[name: ' + self.name + ', list_pos:' + str(self.list_position) + \
                 ', parent: '
        if self.top_entry is not None:
            string += str(self.top_entry.name)
        else:
            string += 'None'
        string += ', children: '

        if self.subentries:
            length = len(self.subentries)
            if length != 0:
                for i in range(0, length):
                    string += self.subentries[i].name
                    if i < len(self.subentries) - 1:
                        string += ','
            else:
                string += 'None'
        else:
            string += 'None'
        string += ']'

        return string

    def __str__(self):
        return self.create_string()

    def __repr__(self):
        return self.create_string()


class EditableOptionPanel(QWidget):

    def __init__(self, id_num, parent=None):
        super(EditableOptionPanel, self).__init__(parent)

        self.parent = parent
        self.id = id_num
        self.name = 'New Panel ' + str(id_num)
        self.chosen_cs = None

        rp = rospkg.RosPack()
        panel_ui_path = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'ui', 'option_edit_panel.ui')
        self.ui = QWidget(self.parent)
        loadUi(panel_ui_path, self.ui, {'QWidget': QWidget})

        self.ui.limb_overview_groupbox.setTitle('New Panel ' + str(id_num))

        self.connect_signals()

    def connect_signals(self):
        self.ui.available_cs_combobox.currentIndexChanged.connect(self.handle_joint_input)
        self.ui.panel_name_line_edit.editingFinished.connect(self.handle_name_input)
        self.ui.remove_panel_button.clicked.connect(self.handle_remove_panel)

    def handle_name_input(self):
        text = self.ui.panel_name_line_edit.text()
        self.name = text
        self.ui.limb_overview_groupbox.setTitle(text)

    def set_selectable_link_cs(self, links):
        for link in links:
            self.ui.available_cs_combobox.addItem(str(link))

    def handle_joint_input(self):
        cs = self.ui.available_cs_combobox.currentText()
        self.chosen_cs = str(cs)

    def set_sub_entry_options(self, new_entries):
        for entry in new_entries:
            self.ui.available_options_combobox.addItem(str(entry))

    def handle_remove_panel(self):
        self.ui.setDisabled(True)
        self.ui.setVisible(False)


class OverviewOptionPanel(QWidget):
    def __init__(self, name, target_cs, parent=None):
        super(OverviewOptionPanel, self).__init__(parent)

        self.name = name
        self.target_cs = target_cs
        self.parent = parent

        rp = rospkg.RosPack()
        panel_ui_path = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'ui', 'limb_overview_panel.ui')
        self.ui = QWidget(self.parent)
        loadUi(panel_ui_path, self.ui, {'QWidget': QWidget})

        self.ui.limb_overview_groupbox.setTitle(name)
        self.ui.target_cs_label.setText(target_cs)

    def to_dict(self):
        panel = {'name': str(self.name), 'target_cs': self.target_cs}
        return panel

    def set_model_status(self, status_text):
        self.ui.model_label.setText(status_text)

    def set_poses_status(self, status_text):
        self.ui.poses_label.setText(status_text)

    def __str__(self):
        return '(' + self.name + ', ' + str(self.target_cs) + ')'

    def __repr__(self):
        return '(' + self.name + ', ' + str(self.target_cs) + ')'


class LimbSelectionUi(QWidget):
    target_coordinate_systems = pyqtSignal(list)
    joints_to_calibrate = pyqtSignal(dict)
    save_options = pyqtSignal(list)
    save_models = pyqtSignal(bool)

    def __init__(self, robot_links, options, parent=None):
        super(LimbSelectionUi, self).__init__(parent)

        self.parent = parent
        self.show_save_window = False

        self.option_panels = {}

        self.robot_links = robot_links
        self.options = self.convert_to_option_list(options)

        self.ui = QWidget()

        self.limb_panels = {}
        self.active_editable_panels = []
        self.limb_panel_count = 0

        self.startUi()

        if parent is not None:
            self.connect_to_signals()

    def loadOptionEditPanel(self):
        rp = rospkg.RosPack()
        panel_ui_path = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'ui', 'option_edit_panel.ui')
        panel = QWidget(self.parent)
        loadUi(panel_ui_path, panel, {'QWidget': QWidget})

        return panel

    def startUi(self):
        rp = rospkg.RosPack()
        panel_ui_path = os.path.join(rp.get_path('thor_mang_calibration'), 'resource', 'ui', 'configure_automatic.ui')
        loadUi(panel_ui_path, self.ui, self.parent)

        if self.options is not None:
            self.add_options_to_list()

        self.connect_buttons()

        return self.ui

    def add_options_to_list(self):
        for option in self.options:
            name = option.name
            indent = option.indent

            entry = self.entry_from_name_and_indent(name, indent)

            self.ui.option_list.insertItem(option.list_position, entry)
            self.ui.available_options_combobox.addItem(name)

        self.ui.option_list.insertItem(self.ui.option_list.count(), 'New')

    def convert_to_option_list(self, options):
        option_list = []
        option_name_list = []
        if options:
            for option in options:

                name = str(option['name'])
                new = AutomaticOption(name)

                indent = option['indent']
                new.indent = indent

                new.list_position = option['list_position']
                top_entry = option['top_entry']

                if top_entry is not None and top_entry in option_name_list:
                    index = option_name_list.index(top_entry)
                    entry = option_list[index]
                    entry.subentries.append(new)
                    new.top_entry = entry

                if option['panels']:
                    for panel in option['panels']:
                        name = panel['name']
                        target_cs = panel['target_cs']
                        if str(name) in self.option_panels.keys():
                            panel_from_dict = self.option_panels[str(name)]
                            new.panels.append(panel_from_dict['panel'])
                        else:
                            new_panel = OverviewOptionPanel(panel['name'], panel['target_cs'], self)
                            new.panels.append(new_panel)
                            self.option_panels[str(name)] = {'target_cs': target_cs, 'panel': new_panel}

                option_list.append(new)
                option_name_list.append(name)

        return option_list

    def connect_buttons(self):
        self.ui.option_list.currentItemChanged.connect(self.on_option_list_item_changed)

        self.ui.remove_entry_button.clicked.connect(self.handle_remove_entry_button)
        self.ui.edit_entry_button.clicked.connect(self.handle_edit_entry_button)
        self.ui.save_entry_button.clicked.connect(self.handle_save_entry_button)

        self.ui.add_new_panel_button.clicked.connect(self.add_new_editable_panel)
        self.ui.option_name_edit.cursorPositionChanged.connect(self.handle_cursor_change)

    def connect_to_signals(self):
        self.parent.loading_status.connect(self.handle_loading_status)
        self.parent.poses_status.connect(self.handle_poses_status)
        self.parent.model_status.connect(self.handle_model_status)

    def set_initial_selections(self):
        self.ui.option_list.setCurrentRow(0)

    def handle_poses_status(self, poses_status):
        currentRow = self.ui.option_list.currentRow()
        if currentRow < len(self.options):
            option = self.options[currentRow]
            panels = option.panels

            for i in range(0, len(panels)):
                panel = panels[i]
                status = poses_status[i]

                msg = ''
                if status[1] == 0:
                    msg = 'None'
                else:
                    msg = str(status[1]) + ' poses found.'

                if panel.target_cs == status[0]:
                    panel.set_poses_status(msg)

    def handle_model_status(self, model_status):
        current_row = self.ui.option_list.currentRow()
        if current_row < len(self.options):
            option = self.options[current_row]
            panels = option.panels

            for i in range(0, len(model_status)):
                model = model_status[i]

                msg = ''
                if model[1] == 1 or type(model[1]) == list:
                    msg = 'Available.' # Involved joints: ' + str(model[1][0]) + ' to ' + str(model[1][-1])
                elif model[1] == 0:
                    msg = 'Unavailable.'
                elif model[1] == 2:
                    msg = 'Calculating...'

                for j in range(0, len(panels)):
                    panel = panels[j]
                    if panel.target_cs == model[0]:
                        panel.set_model_status(msg)

    def handle_cursor_change(self):
        res = self.ui.option_name_edit.styleSheet().find('background-color: rgb(255, 255, 255)')
        if res == -1:
            self.ui.option_name_edit.setStyleSheet('background-color: rgb(255, 255, 255);')

    def handle_loading_status(self, msg):
        for key in msg.keys():
            entries = msg[key]

            msg_type = entries['type']
            limb = key
            status = entries['status']

            if status:
                status_text = 'Successfully loaded.'
                style_sheet = "color: green;"
            else:
                status_text = 'Failed to load.'
                style_sheet = "color: red;"

            if limb in self.limb_panels.keys():
                panel = self.limb_panels[limb]

                if msg_type == 'model':
                    panel.model_label.setText(status_text)
                    panel.model_label.setStyleSheet(style_sheet)
                elif msg_type == 'poses':
                    panel.poses_label.setText(status_text)
                    panel.poses_label.setStyleSheet(style_sheet)
                elif msg_type == 'data':
                    panel.data_label.setText(status_text)
                    panel.data_label.setStyleSheet(style_sheet)
                else:
                    print('debug: Error msg_type not recognized in handle_loading_status!')

    def option_name_from_list_entry(self, entry):
        split = entry.split(' ')

        option_name = ''
        for i in range(0, len(split)):
            part = split[i]
            if part != '':
                option_name += part
                if i != len(split) - 1:
                    option_name += ' '

        return option_name

    def on_option_list_item_changed(self, new_item, old_item):
        if new_item == old_item or new_item is None:
            return

        open_page = self.ui.option_display_stacked_widget.currentIndex()
        list_position = self.ui.option_list.currentRow()
        item_key = self.option_name_from_list_entry(str(new_item.text()))

        active_target_cs = []

        if item_key.find('New') != -1:
            if old_item is not None and old_item.text() != 'New' and open_page == 1:
                self.remove_editable_panels()
            self.ui.option_display_stacked_widget.setCurrentIndex(1)

            self.ui.option_name_edit.setText('')
            self.ui.available_options_combobox.setCurrentIndex(0)

            if self.ui.new_option_page.layout().count() < 6:
                self.add_new_editable_panel()

            self.ui.remove_entry_button.setEnabled(False)
            self.ui.edit_entry_button.setEnabled(False)
            self.ui.save_entry_button.setEnabled(True)
        else:
            self.ui.option_display_stacked_widget.setCurrentIndex(0)

            active_target_cs = self.set_panels(list_position)

            self.ui.remove_entry_button.setEnabled(True)
            self.ui.edit_entry_button.setEnabled(True)
            self.ui.save_entry_button.setEnabled(False)

        QApplication.processEvents()

        self.target_coordinate_systems.emit(active_target_cs)

    def add_new_editable_panel(self):
        layout = self.ui.new_option_page.layout()
        position = layout.count() - 2

        panel = EditableOptionPanel(self.limb_panel_count, self.parent)
        self.limb_panel_count += 1

        layout.insertWidget(position, panel.ui)
        panel.set_selectable_link_cs(self.robot_links)

        self.active_editable_panels.append(panel)

    def adjust_available_subentry_options(self, option_key):
        combobox = self.ui.available_options_combobox
        hidden_options = []
        if option_key != 'New':
            for i in range(0, len(self.options)):
                if self.options[i].name == option_key:
                    option = self.options[i]

            hidden_options.append(option.name)
            children = []
            children += option.subentries
            while children:
                child = children.pop(0)
                hidden_options.append(child.name)
                children += child.subentries

        combobox.clear()
        count = 0
        if self.options is not None:
            for option in self.options:
                if option.name not in hidden_options:
                    combobox.insertItem(count, option.name)
                    count += 1
        combobox.insertItem(0, 'None')

    def handle_remove_entry_button(self):
        option_name = self.option_name_from_list_entry(self.ui.option_list.currentItem().text())
        option_list_position = self.ui.option_list.currentRow()

        if option_name == 'New':
            return

        option = self.options[option_list_position]

        remove_index = option_list_position
        count = 1
        children = []
        children += option.subentries

        while children:
            child = children.pop()
            children += child.subentries
            count += 1

        for i in range(0, count):
            self.ui.option_list.takeItem(remove_index)
            self.options.remove(self.options[remove_index])

        self.update_option_position()

        self.show_save_window = True

    def handle_edit_entry_button(self):
        self.ui.option_display_stacked_widget.setCurrentIndex(1)
        self.remove_editable_panels()

        self.ui.edit_entry_button.setDisabled(True)
        self.ui.save_entry_button.setEnabled(True)

        option = self.options[self.ui.option_list.currentRow()]
        layout = self.ui.new_option_page.layout()
        position = layout.count() - 2

        self.ui.option_name_edit.setText(option.name)
        top_entry = 'None'
        if option.top_entry is not None:
            top_entry = option.top_entry.name
        row = self.ui.available_options_combobox.findText(str(top_entry), Qt.MatchEndsWith)
        self.ui.available_options_combobox.setCurrentIndex(row)

        for i in range(0, len(option.panels)):
            panel = option.panels[i]
            name = panel.name
            target_cs = panel.target_cs

            edit_panel = EditableOptionPanel(self.limb_panel_count, self.parent)

            layout.insertWidget(position + i, edit_panel.ui)
            edit_panel.set_selectable_link_cs(self.robot_links)

            edit_panel.name = name
            edit_panel.ui.limb_overview_groupbox.setTitle(name)
            edit_panel.chosen_cs = target_cs
            row = edit_panel.ui.available_cs_combobox.findText(target_cs, Qt.MatchEndsWith)
            edit_panel.ui.available_cs_combobox.setCurrentIndex(row)
            edit_panel.ui.panel_name_line_edit.setText(name)

            self.active_editable_panels.append(edit_panel)

    def handle_save_entry_button(self):
        new_option_name = self.ui.option_name_edit.text()
        subentry_to = self.ui.available_options_combobox.currentText()

        is_edit, option = self.check_if_edit()
        valid, text = self.check_option_name(new_option_name, is_edit)

        if not valid:
            self.ui.option_name_edit.setStyleSheet('background-color: rgb(255, 0, 0);')
            #print('debug: ' + str(text))
            return

        self.ui.edit_entry_button.setEnabled(True)
        self.ui.save_entry_button.setDisabled(True)

        if not is_edit:
            option = self.create_new_option(new_option_name, subentry_to)
            list_entry_name = self.entry_from_name_and_indent(new_option_name, option.indent)
            self.ui.option_list.insertItem(option.list_position, list_entry_name)
            self.ui.available_options_combobox.addItem(new_option_name)

        else:
            name_changed, top_entry_changed, panels_changed = self.check_changes(option)
            self.handle_option_changes(option, name_changed, top_entry_changed, panels_changed)

        self.remove_editable_panels()

        if self.options is None:
            self.options = [option]
        elif not is_edit:
            self.options.insert(option.list_position, option)

        self.update_option_position()

        if is_edit:
            self.ui.option_list.setCurrentRow(self.ui.option_list.count())
        self.ui.option_list.setCurrentRow(option.list_position)

        self.show_save_window = True

    def create_new_option(self, option_name, top_entry):
        option = AutomaticOption(option_name)

        option.list_position = self.ui.option_list.count() - 1

        if top_entry != 'None':
            item = self.ui.option_list.findItems(top_entry, Qt.MatchEndsWith)[0]
            item_position = self.ui.option_list.row(item)

            top_entry = self.options[item_position]

            option.top_entry = top_entry
            option.indent = top_entry.indent + 1
            option.list_position = self.calculate_list_position(top_entry)

            top_entry.subentries.append(option)

        option.panels = self.generate_overview_panels(self.active_editable_panels)

        return option

    def entry_from_name_and_indent(self, name, indent):
        entry_name = ''
        for i in range(0, indent):
            entry_name += '  '
        entry_name += name

        return entry_name

    def remove_editable_panels(self):
        for edit_panel in self.active_editable_panels:
            self.ui.new_option_page.layout().removeWidget(edit_panel.ui)
            edit_panel.ui.hide()
        self.active_editable_panels = []

    def check_option_name(self, name, is_edit):
        valid = True
        text = ''

        if name == '':
            text = 'Option name invalid: Name empty.'
            valid = False
            return valid, text

        if name == 'New':
            text = 'Option name invalid: Name \'New\' is reserved.'
            valid = False
            return valid, text

        if self.options is None or is_edit:
            return valid, text

        for option in self.options:
            if option.name == name:
                valid = False
                text = 'Option name invalid: Name already in use.'

        return valid, text

    def check_if_edit(self):
        current_option = self.option_name_from_list_entry(self.ui.option_list.currentItem().text())
        if current_option == 'New':
            return False, None

        options = self.options

        edited_option = None
        is_edit = False

        for option in options:
            if option.name == current_option:
                is_edit = True
                edited_option = option

                return is_edit, edited_option

        return is_edit, edited_option

    def handle_option_changes(self, option, name_changed, top_entry_changed, panels_changed):
        if top_entry_changed:
            old_top_entry = option.top_entry
            if old_top_entry is not None:
                for i in range(0, len(old_top_entry.subentries)):
                    if old_top_entry.subentries[i].name == option.name:
                        old_top_entry.subentries.remove(old_top_entry.subentries[i])
                        break

            new_top_entry = self.ui.available_options_combobox.currentText()
            if new_top_entry != 'None':
                item = self.ui.option_list.findItems(new_top_entry, Qt.MatchEndsWith)[0]
                item_position = self.ui.option_list.row(item)

                top_entry = self.options[item_position]
                option.top_entry = top_entry
                option.indent = top_entry.indent + 1

                new_list_position = self.calculate_list_position(top_entry)

                top_entry.subentries.append(option)
            else:
                option.top_entry = None
                option.indent = 0
                new_list_position = self.ui.option_list.count() - 1
            self.move_option_and_children(option, new_list_position)

        if name_changed:
            old_name = option.name
            new_name = self.ui.option_name_edit.text()
            option.name = new_name
            entry = self.entry_from_name_and_indent(option.name, option.indent)

            list_item = self.ui.option_list.currentItem()
            list_item.setText(entry)

            row = self.ui.available_options_combobox.findText(old_name, Qt.MatchEndsWith)
            self.ui.available_options_combobox.setItemText(row, new_name)

        if panels_changed:
            option.panels = self.generate_overview_panels(self.active_editable_panels)

    def move_option_and_children(self, option, position):
        mod_static = True
        if position > option.list_position:
            mod_static = False
            position -= 1
        old_position_modifier = 0

        self.options.insert(position, self.options.pop(option.list_position))
        self.ui.option_list.insertItem(position,
                                       self.ui.option_list.takeItem(option.list_position))
        self.ui.option_list.item(position).setText(self.entry_from_name_and_indent(option.name, option.indent))
        option.list_position = position

        children = []
        children += option.subentries
        while children:
            if not mod_static:
                old_position_modifier -= 1
            else:
                position += 1

            child = children.pop(0)
            child.indent = child.top_entry.indent + 1
            subentries = child.subentries
            children = subentries + children

            self.options.insert(position, self.options.pop(child.list_position + old_position_modifier))
            child_item = self.ui.option_list.takeItem(child.list_position + old_position_modifier)
            child_item.setText(self.entry_from_name_and_indent(child.name, child.indent))
            self.ui.option_list.insertItem(position, child_item)

    def calculate_list_position(self, top_entry_option):
        position = top_entry_option.list_position + 1

        children = []
        children += top_entry_option.subentries

        while children:
            head = children.pop()
            position += 1
            children += head.subentries

        return position

    def update_option_position(self):
        for i in range(0, len(self.options)):
            self.options[i].list_position = i

    def generate_overview_panels(self, edit_panels):
        panels = []

        for editable in edit_panels:
            if editable.ui.isVisible():
                name = editable.name
                cs = editable.chosen_cs

                panel = OverviewOptionPanel(name, cs, self.parent)
                panels.append(panel)

        return panels

    def handle_new_option(self, new_option_name):
        option = AutomaticOption(new_option_name)

        subentry_to = self.ui.available_options_combobox.currentText()

        indent = 0
        new_list_position = self.ui.option_list.count() - 1
        entry = ''
        if subentry_to != 'None':
            item = self.ui.option_list.findItems(subentry_to, Qt.MatchEndsWith)[0]
            item_position = self.ui.option_list.row(item)

            top_entry = self.options[item_position]
            new_list_position = self.calculate_list_position(top_entry)

            option.top_entry = top_entry

            indent = top_entry.indent + 1
            top_entry.subentries.append(option)

        for i in range(0, indent):
            entry += '  '

        entry += new_option_name

        self.ui.option_list.insertItem(new_list_position, entry)
        self.ui.available_options_combobox.addItem(new_option_name)

        option.indent = indent
        option.list_position = new_list_position

        new_panels = self.generate_overview_panels(self.active_editable_panels)
        option.panels = new_panels

        for edit_panel in self.active_editable_panels:
            self.ui.new_option_page.layout().removeWidget(edit_panel.ui)
            edit_panel.ui.hide()
        self.active_editable_panels = []

        self.update_option_position()

        for panel in new_panels:
            self.limb_panels[panel.ui.limb_overview_groupbox.title()] = panel

        if self.options is None:
            self.options = [option]
        else:
            self.options.insert(new_list_position, option)

    def check_changes(self, option):
        new_name = self.ui.option_name_edit.text()
        subentry_to = self.ui.available_options_combobox.currentText()

        name_changed = False
        if new_name != option.name:
            name_changed = True

        top_entry_changed = False
        if option.top_entry is not None:
            if subentry_to != option.top_entry.name:
                top_entry_changed = True
        else:
            if subentry_to != 'None':
                top_entry_changed = True

        panels_changed = self.check_panels_for_difference(option.panels, self.active_editable_panels)

        return name_changed, top_entry_changed, panels_changed

    def check_panels_for_difference(self, option_panels, active_panels):
        difference = False

        if len(option_panels) != len(active_panels):
            return True

        for option in option_panels:
            option_name = option.name
            option_cs = option.target_cs

            match_found = False
            for active in active_panels:
                active_name = active.name
                active_cs = active.chosen_cs

                if active_name == option_name and active_cs == option_cs and active.ui.isVisible():
                    match_found = True
                    continue

            if not match_found:
                return True

        return difference

    def clear_layout(self, layout):
        for i in range(0, layout.count()):
            item = layout.itemAt(i)
            if type(item) is QWidgetItem:
                item.widget().hide()

    def set_panels(self, option_index):
        layout = self.ui.existing_option_page.layout()
        self.clear_layout(layout)

        option = self.options[option_index]
        panels = option.panels

        panel_count = 0
        chosen_cs = []
        for panel in panels:
            layout.insertWidget(panel_count, panel.ui)
            panel.ui.show()
            chosen_cs.append(panel.target_cs)
            panel_count += 1

        return chosen_cs

    def options_to_list(self):
        options = []

        for option in self.options:
            options.append(option.to_dict())

        return options

    def closeEvent(self, event):
        if self.show_save_window:
            reply = QMessageBox.question(self, "Save", "Do you wish to save the set options?\nWarning: Will override file",
                                         QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

            if reply == QMessageBox.Yes:
                self.save_options.emit([self.options_to_list(), 'automatic_options.yaml'])

        #reply = QMessageBox.question(self, "Save", "Do you wish to save the models?",
        #                             QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        #if reply == QMessageBox.Yes:
        self.save_models.emit(True)

        super(LimbSelectionUi, self).closeEvent(event)

