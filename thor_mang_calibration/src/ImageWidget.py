#! /usr/bin/env python

from PyQt5.QtWidgets import QLabel, QMainWindow
from PyQt5.QtCore import Qt

#import dill
#dill.settings['recurse'] = True


class ImageWidget(QLabel):

    def __init__(self, parent=None):
        super(ImageWidget, self).__init__(parent)
        self.setScaledContents(True)

    def hasHeightForWidth(self):
        return self.pixmap() is not None

    def heightForWidth(self, w):
        if self.pixmap():
            return int(w * self.pixmap().height() / self.pixmap().width())

    def widthForHeight(self, h):
        if self.pixmap():
            return int(h * self.pixmap().width() / self.pixmap().height())

    def resizeEvent(self, resizeEvent):
        width = int(self.parent().width() / 2 - 10)
        height = self.heightForWidth(width)

        if height > self.parent().height() / 2 - 10:
            height = self.parent().height() / 2 - 10
            width = self.widthForHeight(height)

        self.resize(width, height)
        self.setPixmap(self.pixmap().scaled(width, height, Qt.KeepAspectRatio))

        QMainWindow.resizeEvent(self, resizeEvent)
