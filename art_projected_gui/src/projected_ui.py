#!/usr/bin/env python

"""
Module contains the main class of 2D projected UI (ProjectedUI). It produces 2D image of given size. This image is send to Project class(es) to be displayed.

"""

from PyQt4 import QtGui, QtCore
from object_item import ObjectItem
from place_item import PlaceItem

# z tohodle vyleze 2D obraz
# scene_res -> rozliseni v jakem se ma scena udrzovat
# world_coords -> souradnice rohu obrazu
class ProjectedUI(QtCore.QObject):
    
    def __init__(self,  x,  y,  width,  height):
        
        super(ProjectedUI, self).__init__()
        
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.rpm = 1280.0 # resolution per meter ;)

        # TODO make 'DPI' configurable
        w = self.width*self.rpm
        h = self.height/self.width*w

        self.scene = QtGui.QGraphicsScene(0, 0,  int(w),  int(h))
        self.scene.setBackgroundBrush(QtCore.Qt.black)
        #self.state_manager = interface_state_manager("PROJECTED UI",  cb=self.interface_state_cb)
        
        self.scene.changed.connect(self.scene_changed)
        
        self.projectors = []
        
        self.objects = {}
        self.places = []
        
    def interface_state_cb(self,  state):
        
        pass
        
    def add_projector(self,  proj):
        
        self.projectors.append(proj)
        
    def debug_view(self):
        
        self.view = QtGui.QGraphicsView(self.scene)
        self.view.setRenderHint(QtGui.QPainter.Antialiasing)
        self.view.setViewportUpdateMode(QtGui.QGraphicsView.FullViewportUpdate)
        self.view.setStyleSheet( "QGraphicsView { border-style: none; }" )
        
        self.view.show()
        
    def scene_changed(self,  rects):
        
        pix = QtGui.QPixmap(self.scene.width(), self.scene.height())
        painter = QtGui.QPainter(pix)
        self.scene.render(painter)
        painter.end()
        
        for proj in self.projectors:
            
            proj.set_img(pix)
            
    def add_object(self,  object_id,  object_type,  x,  y):
        
        self.objects[object_id] = ObjectItem(self.scene,  self.rpm, object_id,  object_type,  x,  y)
        
    def remove_object(self,  object_id):
        
        try:
            self.scene.remove(self.objects[object_id])
            del self.objects[object_id]
        except KeyError:
            return False
            
        return True
        
    def add_place(self,  caption,  x,  y):
        
        self.places.append(PlaceItem(self.scene,  self.rpm,  caption,  x,  y))
