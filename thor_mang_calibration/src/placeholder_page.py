#!/usr/bin/env python

# This file contains four classes. They serve as an example on how to create a customized path, that is independent from
# any pre-defined pages, in the CalibrationWidget.
# One class, Placeholder, sets two numbers on two pages and displays their addition on a third. It is a simplified
# analogue to automatic_calibration.py, in that it manages information over multiple pages and creates a result from them.
# The other, PlaceholderPage is an interface between the more complex class Placeholder and the final displaying and
# page managing widget, calibration_widget.py. In that it is analogue to auto_page.py .
# The other two classes are the pages designed for the


from PyQt5.QtWidgets import QWidget, QLabel, QLineEdit, QGridLayout
from PyQt5.QtGui import QValidator, QDoubleValidator
from python_qt_binding.QtCore import pyqtSignal


class PlaceholderPage(QWidget):

    def __init__(self, page_id, config, parent=None):
        super(PlaceholderPage, self).__init__()

        self.parent = parent

        self.ui_names = []
        self.ui_widgets = []

        self.placeholder = Placeholder(self.parent)
        self.placeholder_pages = self.placeholder.pages

        for i in range(0, len(self.placeholder_pages)):
            self.ui_names.append(self.placeholder_pages[i][0])
            self.ui_widgets.append(self.placeholder_pages[i][1].ui)


class NumberPage(QWidget):
    number_signal = pyqtSignal(list)

    def __init__(self, initial_value, index, parent):
        super(NumberPage, self).__init__(parent)
        self._parent = parent

        self.index = index
        self.value = initial_value

        self.ui = QWidget()

        self.create_ui()
        self.connect_signals()

    def create_ui(self):
        self.ui.layout = QGridLayout()

        self.ui.page_title = QLabel('<b>Number Page ' + str(self.index + 1) + '</b>')

        self.ui.layout.addWidget(self.ui.page_title, 0, 0)

        self.ui.label = QLabel("Enter a number:")
        self.ui.lineedit = QLineEdit(str(self.value))

        self.ui.layout.addWidget(self.ui.label, 1, 0)
        self.ui.layout.addWidget(self.ui.lineedit, 1, 1)

        self.ui.setLayout(self.ui.layout)

    def connect_signals(self):
        self.ui.lineedit.editingFinished.connect(self.handle_lineedit)

    def handle_lineedit(self):
        self.ui.lineedit.clearFocus()

        number_string = self.ui.lineedit.text()

        valid, value = self._string_to_float(number_string)

        if valid:
            self.value = value
            self.number_signal.emit([self.index, self.value])
        else:
            self.ui.lineedit.setText(str(self.value))

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


class DisplayPage(QWidget):

    def __init__(self, initial_result, parent=None):
        super(DisplayPage, self).__init__(parent)

        self.parent = parent

        self.result = initial_result

        self.ui = QWidget()
        self.create_ui()

        if parent is not None:
            parent.result.connect(self.handle_result_signal)

    def create_ui(self):
        self.ui.layout = QGridLayout()

        self.ui.label = QLabel("Result: ")
        self.ui.result_label = QLabel(str(self.result))

        self.ui.layout.addWidget(self.ui.label, 0, 0)
        self.ui.layout.addWidget(self.ui.result_label, 0, 1)

        self.ui.setLayout(self.ui.layout)

    def handle_result_signal(self, result):
        self.result = result

        self.ui.result_label.setText(str(result))


class Placeholder(QWidget):
    result = pyqtSignal(float)

    def __init__(self, parent):
        super(Placeholder, self).__init__(parent)

        self.parent = parent

        self.pages = []
        self.numbers = []

        amount_of_number_pages = 2
        initial_value_for_number_page = 1

        for i in range(0, amount_of_number_pages):
            self.numbers.append(initial_value_for_number_page)

            page = NumberPage(initial_value_for_number_page, i, self)
            self.pages.append(['Number Page ' + str(i + 1), page])
            page.number_signal.connect(self.handle_incoming_numbers)

        page = DisplayPage(sum(self.numbers), self)
        self.pages.append(['Result Page', page])

    def handle_incoming_numbers(self, data):
        index = data[0]
        value = data[1]

        self.numbers[index] = value

        self.result.emit(sum(self.numbers))
