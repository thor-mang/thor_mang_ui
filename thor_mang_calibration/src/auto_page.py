#!/usr/bin/env python

from PyQt5.QtWidgets import QWidget

from automatic_calibration import AutomaticCalibration


class AutomaticPage(QWidget):

    def __init__(self, page_id, config, parent=None):
        super(AutomaticPage, self).__init__()

        self.parent = parent

        self.calibration = AutomaticCalibration(parent)

        self.automatic_pages = self.calibration.pages
        self.ui_names = []
        self.ui_widgets = []

        for i in range(0, len(self.automatic_pages)):
            self.ui_names.append(self.automatic_pages[i][0])
            self.ui_widgets.append(self.automatic_pages[i][1].ui)

    def closeEvent(self, event):
        #print('debug: page caught close event')

        for page in self.automatic_pages:
            page[1].close()

        super(AutomaticPage, self).closeEvent(event)
