#!/usr/bin/env python

"""
Visualization of place on the table
"""

from PyQt4 import QtGui, QtCore
from item import Item

class PlaceItem(Item):
    
    def __init__(self,  scene,  rpm, caption,  x,  y,  outline_diameter=0.1,  selected = False):
        
        self.outline_diameter = outline_diameter
        self.caption = caption
        self.in_collision = False
        
        super(PlaceItem,  self).__init__(scene,  rpm,  x,  y)
        
        # TODO how to enable/disable this for all items?
        self.setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)
        
    def boundingRect(self):
        
        es = self.m2pix(self.outline_diameter)
        return QtCore.QRectF(-es/2, -es/2, es/2, es/2)
        
    def mouseMoveEvent(self, event):
        
        if len(self.collidingItems()) > 0: self.in_collision = True
        else: self.in_collision = False
        
        super(Item, self).mouseMoveEvent(event)
        
    def paint(self, painter, option, widget):
        
        es = self.m2pix(self.outline_diameter)
        
        if not self.in_collision:
            painter.setBrush(QtCore.Qt.cyan)
        else:
            painter.setBrush(QtCore.Qt.red)
        
        painter.drawEllipse(-es/2, -es/2, es/2, es/2)
        
        painter.setFont(QtGui.QFont('Arial', 12));
        
        painter.setPen(QtCore.Qt.gray)
        
        if self.hover:
            
            painter.setPen(QtCore.Qt.white)
            painter.drawText(0,  20, self.get_pos_str())
            
        painter.drawText(0,  0, self.caption);
