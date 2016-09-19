#!/usr/bin/env python

from PyQt4 import QtGui, QtCore

# spolecny predek vseho ve scene
class Item(QtGui.QGraphicsItem):
    
    def __init__(self,  scene,  rpm,  x,  y):
        
        super(Item, self).__init__(scene=scene)
        
        self.rpm = rpm
        self.set_pos(x,  y)
        self.set_enabled()
        self.hover = False
    
    def m2pix(self,  m):
        
        return m*self.rpm
        
    def pix2m(self,  p):
        
        return p/self.rpm
    
    # world coordinates to scene coords
    def set_pos(self,  x,  y):
        
        self.setPos(self.m2pix(x),  self.m2pix(y))
        
    def get_pos(self):
        
        return (self.pix2m(self.x()),  self.pix2m(self.y()))
        
    def get_pos_str(self):
        
        (x, y) = self.get_pos()
        # TODO fixed width format
        return "[X: " + str(round(x, 3)).ljust(5, '0') + ", Y: " + str(round(y, 3)).ljust(5, '0') + "]"
        
    def set_enabled(self):
        
        self.setVisible(True)
        self.setAcceptHoverEvents(True)        
        self.setEnabled(True)
        self.setActive(True)
        
    def hoverEnterEvent(self,  evt):
        
        self.hover = True
        self.update()
        
    def hoverLeaveEvent(self,  evt):
        
        self.hover = False
        self.update()
        
    def boundingRect(self):
        
        raise NotImplementedError("Please implement this method")
        
    def paint(self, painter, option, widget):
        
        raise NotImplementedError("Please implement this method")
