#!/usr/bin/env python

from python_qt_binding.QtWidgets import QTabWidget, QTabBar, QStyleOptionTab, QStyle, QWidget, QStylePainter, QGridLayout
from python_qt_binding.QtGui import QPainter, QColor
#from python_qt_binding.QtCore import

def get_QTabBar_style():
    styleStr = str("""
        QTabBar[colorToggle=true]::tab {                   
            color: #ff0000;                         
        }                                                  
    """)

    return styleStr

class TabBar(QTabBar):
    def __init__(self):
        super(TabBar, self).__init__()
        self.__coloredTabs = []
        self.setProperty("colorToggle", False)

    def colorTab(self, index):
        if (index >= self.count()) or (index < 0) or (index in self.__coloredTabs):
            return
        self.__coloredTabs.append(index)
        self.update()

    def uncolorTab(self, index):
        if index in self.__coloredTabs:
            self.__coloredTabs.remove(index)
            self.update()

    def paintEvent(self, event):
        painter = QStylePainter(self)
        opt = QStyleOptionTab()
        painter.save()

        for i in range(self.count()):
            self.initStyleOption(opt, i)
            if i in self.__coloredTabs:
                self.setProperty("colorToggle", True)
                self.style().unpolish(self)
                self.style().polish(self)

                painter.drawControl(QStyle.CE_TabBarTabShape, opt)
                painter.drawControl(QStyle.CE_TabBarTabLabel, opt)
            else:
                self.setProperty("colorToggle", False)
                self.style().unpolish(self)
                self.style().polish(self)

                painter.drawControl(QStyle.CE_TabBarTabShape, opt)
                painter.drawControl(QStyle.CE_TabBarTabLabel, opt)

        painter.restore()

class TabWidget(QTabWidget):
    def __init__(self, tabs = None):
        super(TabWidget, self).__init__()
        self.myTabBar = TabBar()
        self.setTabBar(self.myTabBar)
        self.setTabsClosable(False)

        for tab_name in tabs:
            widget = QWidget()
            widget.setLayout(QGridLayout())
            self.addTab(widget, tab_name)

        self.tabBar().setStyleSheet(get_QTabBar_style())

    def colorTab(self, index):
        self.myTabBar.colorTab(index)

    def uncolorTab(self, index):
        self.myTabBar.uncolorTab(index)
