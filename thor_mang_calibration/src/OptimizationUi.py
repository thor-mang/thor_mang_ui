#! /usr/bin/env python

from PyQt5.QtWidgets import QWidget, QLineEdit, QGridLayout, QPushButton, QLabel, QSpacerItem, QSizePolicy
from PyQt5.QtCore import pyqtSignal


class OptimizationUi(QWidget):
    run_optimization = pyqtSignal(bool)
    option_set = pyqtSignal(list)

    def __init__(self, parent=None):
        super(OptimizationUi, self).__init__(parent)
        self.parent = parent

        self.ui = QWidget(self.parent)

        self.startUi()

        if parent is not None:
            self.parent.option_value.connect(self.handle_option_value_signal)
            self.parent.optimization_status.connect(self.handle_optimization_status_signal)
            self.parent.optimization_progress.connect(self.handle_progress_signal)

    def startUi(self):
        self.ui.page_layout = QGridLayout()

        self.ui.information_layout = QGridLayout()

        label1 = QLabel('Status: ')
        self.ui.status_label = QLabel('')
        label2 = QLabel('Progress: ')
        self.ui.progress_label = QLabel('\n-\n')

        space_horizontal = QSpacerItem(20, 40, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.ui.information_layout.addWidget(label1, 0, 0)
        self.ui.information_layout.addWidget(self.ui.status_label, 0, 1)
        self.ui.information_layout.addWidget(label2, 1, 0)
        self.ui.information_layout.addWidget(self.ui.progress_label, 1, 1)
        self.ui.information_layout.addItem(space_horizontal, 0, 2)

        self.ui.option_layout = QGridLayout()

        self.ui.option_label = QLabel("Optimization Options:")

        self.ui.tolerance_label = QLabel("tol:")
        self.ui.tolerance_label.setToolTip("Tolerance for termination. Float or None.")
        self.ui.tolerance_edit = QLineEdit()
        self.ui.tolerance_edit.editingFinished.connect(lambda x="tol": self.handle_option_edits(x))

        self.ui.max_correction_label = QLabel("maxcor:")
        self.ui.max_correction_label.setToolTip("The maximum number of variable metric corrections used to define the"
                                                 " limited memory matrix.")
        self.ui.max_correction_edit = QLineEdit()
        self.ui.max_correction_edit.editingFinished.connect(lambda x="maxcor": self.handle_option_edits(x))

        self.ui.max_linesearch_label = QLabel('maxls:')
        self.ui.max_linesearch_label.setToolTip("Maximum number of line search steps (per iteration).")
        self.ui.max_linesearch_edit = QLineEdit()
        self.ui.max_linesearch_edit.editingFinished.connect(lambda x="maxls": self.handle_option_edits(x))

        self.ui.function_tolerance_label = QLabel("ftol:")
        self.ui.function_tolerance_label.setToolTip("The iteration stops when (f^k - f^{k+1})/max{|f^k|,|f^{k+1}|,1} <="
                                                    " ftol")
        self.ui.function_tolerance_edit = QLineEdit()
        self.ui.function_tolerance_edit.editingFinished.connect(lambda x="ftol": self.handle_option_edits(x))

        self.ui.gradient_tolerance_label = QLabel("gtol:")
        self.ui.gradient_tolerance_label.setToolTip("The iteration will stop when max{|proj g_i | i = 1, ..., n} <= gtol "
                                                 "where pg_i is the i-th component of the projected gradient.")
        self.ui.gradient_tolerance_edit = QLineEdit()
        self.ui.gradient_tolerance_edit.editingFinished.connect(lambda x="gtol": self.handle_option_edits(x))

        self.ui.step_size_label = QLabel("eps:")
        self.ui.step_size_label.setToolTip("The absolute step size used for numerical approximation of the jacobian via"
                                        " forward differences.")
        self.ui.step_size_edit = QLineEdit()
        self.ui.step_size_edit.editingFinished.connect(lambda x="eps": self.handle_option_edits(x))

        self.ui.max_function_label = QLabel("maxfun")
        self.ui.max_function_label.setToolTip("Maximum number of function evaluations.")
        self.ui.max_function_edit = QLineEdit()
        self.ui.max_function_edit.editingFinished.connect(lambda x="maxfun": self.handle_option_edits(x))

        self.ui.max_iteration_label = QLabel("maxiter:")
        self.ui.max_iteration_label.setToolTip("Maximum number of iterations.")
        self.ui.max_iteration_edit = QLineEdit()
        self.ui.max_iteration_edit.editingFinished.connect(lambda x="maxiter": self.handle_option_edits(x))

        self.ui.option_layout.addWidget(self.ui.option_label, 0, 0)
        self.ui.option_layout.addWidget(self.ui.step_size_label, 1, 0)
        self.ui.option_layout.addWidget(self.ui.step_size_edit, 1, 1)
        #self.ui.option_layout.addWidget(self.ui.tolerance_label, 1, 2)
        #self.ui.option_layout.addWidget(self.ui.tolerance_edit, 1, 3)
        self.ui.option_layout.addWidget(self.ui.function_tolerance_label, 2, 0)
        self.ui.option_layout.addWidget(self.ui.function_tolerance_edit, 2, 1)
        self.ui.option_layout.addWidget(self.ui.gradient_tolerance_label, 2, 2)
        self.ui.option_layout.addWidget(self.ui.gradient_tolerance_edit, 2, 3)
        self.ui.option_layout.addWidget(self.ui.max_function_label, 3, 0)
        self.ui.option_layout.addWidget(self.ui.max_function_edit, 3, 1)
        self.ui.option_layout.addWidget(self.ui.max_iteration_label, 3, 2)
        self.ui.option_layout.addWidget(self.ui.max_iteration_edit, 3, 3)
        self.ui.option_layout.addWidget(self.ui.max_correction_label, 4, 0)
        self.ui.option_layout.addWidget(self.ui.max_correction_edit, 4, 1)
        self.ui.option_layout.addWidget(self.ui.max_linesearch_label, 4, 2)
        self.ui.option_layout.addWidget(self.ui.max_linesearch_edit, 4, 3)

        self.ui.button_layout = QGridLayout()

        #@TODO wieder einbauen
        self.ui.start_optimization_button = QPushButton('Start')
        #self.ui.start_optimization_button.setEnabled(False)
        self.ui.cancel_optimization_button = QPushButton('Cancel')
        self.ui.cancel_optimization_button.setToolTip('<b></b>Cancels the optimization.\n <b>Warning</b>:'
                                                      '\nAll progress will be lost.'
                                                      '\nMay cause application to freeze briefly.') #<b>Check</b>
        #self.ui.cancel_optimization_button.setEnabled(False)

        self.ui.button_layout.addWidget(self.ui.start_optimization_button, 0, 0)
        self.ui.button_layout.addWidget(self.ui.cancel_optimization_button, 0, 1)

        space_vertical = QSpacerItem(40, 20, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.ui.page_layout.addLayout(self.ui.information_layout, 0, 0)
        self.ui.page_layout.addItem(space_vertical, 1, 0)
        self.ui.page_layout.addLayout(self.ui.option_layout, 2, 0)
        self.ui.page_layout.addLayout(self.ui.button_layout, 3, 0)

        self.ui.setLayout(self.ui.page_layout)

        self.ui.start_optimization_button.clicked.connect(self.handle_start_optimization_button)
        self.ui.cancel_optimization_button.clicked.connect(self.handle_cancel_optimization_button)

    def handle_option_edits(self, option):
        value = None
        if option == "tol":
            value = self.ui.tolerance_edit.text()
        elif option == "eps":
            value = self.ui.step_size_edit.text()
        elif option == "maxcor":
            value = self.ui.max_correction_edit.text()
        elif option == "maxls":
            value = self.ui.max_linesearch_edit.text()
        elif option == "ftol":
            value = self.ui.function_tolerance_edit.text()
        elif option == "gtol":
            value = self.ui.gradient_tolerance_edit.text()
        elif option == "maxfun":
            value = self.ui.max_function_edit.text()
        elif option == "maxiter":
            value = self.ui.max_iteration_edit.text()

        self.option_set.emit([option, value])

    def handle_option_value_signal(self, data):
        option = data[0]
        value = data[1]

        lineedit = None
        if option == "tol":
            lineedit = self.ui.tolerance_edit
        elif option == "eps":
            lineedit = self.ui.step_size_edit
        elif option == "maxcor":
            lineedit = self.ui.max_correction_edit
        elif option == "maxls":
            lineedit = self.ui.max_linesearch_edit
        elif option == "ftol":
            lineedit = self.ui.function_tolerance_edit
        elif option == "gtol":
            lineedit = self.ui.gradient_tolerance_edit
        elif option == "maxfun":
            lineedit = self.ui.max_function_edit
        elif option == "maxiter":
            lineedit = self.ui.max_iteration_edit

        lineedit.setText(str(value))

    def handle_start_optimization_button(self):
        self.run_optimization.emit(True)

    def handle_cancel_optimization_button(self):
        self.run_optimization.emit(False)

    def handle_optimization_status_signal(self, start_allowed, status):
        self.ui.status_label.setText(str(status))

        #@TODO wieder einbauen
        #self.ui.start_optimization_button.setEnabled(start_allowed)
        #self.ui.cancel_optimization_button.setEnabled(not start_allowed)

    def handle_progress_signal(self, data):
        self.ui.progress_label.setText(str(data))
