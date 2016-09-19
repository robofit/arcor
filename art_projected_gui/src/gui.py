#!/usr/bin/env python

import sys
import signal
import rospy
from PyQt4 import QtGui, QtCore

def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QtGui.QApplication.quit()

# warpovani obrazu pro kazdy z projektoru
# podle vysky v pointcloudu se vymaskuji mista kde je neco vyssiho - aby se promitalo jen na plochu stolu
class Projector(QtGui.QWidget):
    
    def __init__(self, screen,  calibration):
        
        super(Projector, self).__init__()
        
        desktop = QtGui.QDesktopWidget()
        geometry = desktop.screenGeometry(screen)
        self.move(geometry.left(), geometry.top())
        self.resize(geometry.width(), geometry.height())
        self.showFullScreen()
        
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), QtCore.Qt.black)
        #self.setPalette(p)
        
        self.pix_label = QtGui.QLabel(self)
        self.pix_label.resize(self.size())
    
    def set_img(self,  img):
        
        # TODO warp image according to calibration
        print "set_img"
        self.pix_label.setPixmap(img)
    
# z tohodle vyleze 2D obraz
# scene_res -> rozliseni v jakem se ma scena udrzovat
# world_coords -> souradnice rohu obrazu
class ProjectedUI(object):
    
    def __init__(self,  x,  y,  width,  height):
        
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
        
    def interface_state_cb(self,  state):
        
        pass
        
    def add_projector(self,  proj):
        
        self.projectors.append(proj)
        
    def scene_changed(self,  rects):
        
        print "scene_changed"
        
        pix = QtGui.QPixmap(self.scene.width(), self.scene.height())
        painter = QtGui.QPainter(pix)
        self.scene.render(painter)
        painter.end()
        pix.save('/home/imaterna/pokus.png', "PNG")
        
        for proj in self.projectors:
            
            proj.set_img(pix)
            
    def add_object(self,  object_id,  object_type,  x,  y):
        
        self.objects[object_id] = ObjectItem(self.scene,  self.rpm, object_id,  object_type)
        self.objects[object_id].set_pos(x,  y)

# spolecny predek vseho ve scene
class Item(object):
    
    def __init__(self,  scene,  rpm):
        
        self.scene = scene
        self.rpm = rpm
        self.enabled = False # read-only
        
        self.viz = None #self.scene.createItemGroup()
    
    # world coordinates to scene coords
    def set_pos(self,  x,  y):
        
        self.viz.setPos(x*self.rpm,  y*self.rpm)
        
    def get_pos(self):
        
        return (self.viz.x()/self.rpm,  self.viz.y()/self.rpm)
        
# vizualizace detekovaneho objektu
class ObjectItem(Item):
    
    def __init__(self,  scene,  rpm,  object_id,  object_type,  selected = False):
        
        super(ObjectItem,  self).__init__(scene,  rpm)
        
        self.object_id = object_id
        self.object_type = object_type
        self.selected = False
        
        self.viz = self.scene.addEllipse(0, 0, 150, 150, QtCore.Qt.white, QtCore.Qt.white)
        
        self.label = self.scene.addText(object_id + '\n(' + self.object_type + ')', QtGui.QFont('Arial', 16))
        self.label.setDefaultTextColor(QtCore.Qt.white)
        self.label.setParentItem(self.viz)


def main(args):
    
    rospy.init_node('projected_gui', anonymous=True)
    
    signal.signal(signal.SIGINT, sigint_handler)
    
    app = QtGui.QApplication(sys.argv)

    ui = ProjectedUI(0, 0, 1.2,  0.75)
    
    ui.add_projector(Projector(0, None))
    ui.add_object("profile_1",  "profile",  0.5,  0.5)
    
    ui. scene_changed(None)

    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.
    
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
