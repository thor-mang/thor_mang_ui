#!/usr/bin/env python

from python_qt_binding.QtWidgets import QTabWidget, QTabBar, QStyleOptionTab, QStyle, QWidget, QStylePainter, QGridLayout, QLabel, QLineEdit, QFrame, QSpacerItem, QSizePolicy
from python_qt_binding.QtGui import QPainter, QColor

from python_qt_binding.QtCore import Qt

from yaml import load

import os
import rospkg

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
        
        self.tabBar().setStyleSheet(get_QTabBar_style())

    def colorTab(self, index):
        self.myTabBar.colorTab(index)

    def uncolorTab(self, index):
        self.myTabBar.uncolorTab(index)
        
    def _sort_columns_by_order(self, columns, order):
        ordered_columns = []
        
        for part in order:
                for column in columns:
                    if str(column).find(part) == -1:
                        continue
                    ordered_columns = ordered_columns + [column]
                    
                    
        return ordered_columns
                    
        
    def setup_tab_widget(self, config, no_line_edits, show_arrow):
        
        line_edit_dicts = {}
        for i in range(0, no_line_edits):
            line_edit_dicts[i] = {}
    
        tabs = config['tabs']
        order = config['order']
        
        # including vertical line between sets of columns
        no_cols_per_entry = 1 + int(show_arrow) * 1 + no_line_edits + 1

        for tab in tabs:
            layout = QGridLayout()
            widget = QWidget()
            
            tab_columns = config[tab].keys()
            
            tab_columns = self._sort_columns_by_order(tab_columns, order)
            
            column_counter = 0
            
            for column in tab_columns:
                row_counter = 0
                column_layout = QGridLayout()
                
                lines = config[tab][column]
                                
                label = QLabel('<b>' + str(column) + ': </b>')

                zero_pos = column_counter * (no_cols_per_entry)

                layout.addWidget(label, row_counter, zero_pos)
                
                row_counter += 1
                
                for list in lines:
                    for item in list:
                        label = QLabel(str(item) + ': ')
                        layout.addWidget(label, row_counter, zero_pos)
                        
                        position = 1
                        for i in range(0, no_line_edits):
                            edit = QLineEdit()
                            edit.setMaximumWidth(80)
                            #edit.setReadOnly(True)
                            
                            line_edit_dicts[i][item] = edit
                            
                            layout.addWidget(edit, row_counter, zero_pos + position)
                            position += 1
                            
                            if show_arrow and i == 0:
                                arrow = QLabel('->')
                                arrow.setMaximumWidth(25)
                                layout.addWidget(arrow, row_counter, zero_pos + position)
                                position += 1
                            
                        row_counter += 1
                        
                    line = QFrame()
                    line.setFrameShape(QFrame.HLine)
                    line.setFrameShadow(QFrame.Sunken)
                    
                    layout.addWidget(line, row_counter, zero_pos, 1, no_line_edits + 1 + 1 * int(show_arrow))
                    row_counter += 1
                    
                spacer = QSpacerItem(40, 1, QSizePolicy.Minimum, QSizePolicy.Expanding)
                   
                column_counter += 1
                
            for i in range(no_cols_per_entry, layout.columnCount(), no_cols_per_entry):
                line = QFrame()
                line.setFrameShape(QFrame.VLine)
                line.setFrameShadow(QFrame.Sunken)
                layout.addWidget(line, 0, i - 1, layout.rowCount(), 1, Qt.AlignHCenter)
        
            layout.addItem(QSpacerItem(20, 40, QSizePolicy.Expanding, QSizePolicy.Minimum), 0, zero_pos + position)
            for i in range(0, column_counter, 2):
                layout.addItem(QSpacerItem(40, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))
                i += 1
            
            widget.setLayout(layout)
            self.addTab(widget, tab)
    
        return line_edit_dicts
    
