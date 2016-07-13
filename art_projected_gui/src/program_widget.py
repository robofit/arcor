#!/usr/bin/env python

import sys
import signal
import rospy
from PyQt4 import QtGui, QtCore
from art_msgs.msg import Program,  ProgramItem
from art_msgs.srv import getProgram

class program_widget(QtGui.QWidget):

    def __init__(self,  parent):
       
        super(program_widget, self).__init__(parent)
        
        QtCore.QObject.connect(self, QtCore.SIGNAL('set_prog'), self.prog_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('set_current'), self.set_current_evt)
        
        self.labels = {}
        self.items_to_be_learned = []
        self.prog = None
        self.template = None
        
        self.items_req_learning = [ProgramItem.MANIP_PICK,  ProgramItem.MANIP_PLACE,  ProgramItem.MANIP_PICK_PLACE]
        
        self.scene=QtGui.QGraphicsScene(self)
        self.scene.setBackgroundBrush(QtCore.Qt.black)
        self.view = QtGui.QGraphicsView(self.scene, self)
        self.view.setRenderHint(QtGui.QPainter.Antialiasing)
        self.view.setViewportUpdateMode(QtGui.QGraphicsView.FullViewportUpdate)
        self.view.setStyleSheet( "QGraphicsView { border-style: none; }" )
        
        self.resizeEvent = self.on_resize
        
    def on_resize(self,  event):
        
        self.view.setFixedSize(self.width(), self.height())
        self.view.setSceneRect(QtCore.QRectF(0, 0, self.width(), self.height()))
        self.view.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff);
        self.view.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff);
        
    def get_item_by_id(self,  id):
        
        for it in self.prog.items:
            
            if it.id == id:
                
                return it
                
        return None
        
    def learned(self,  id):
        
        self.items_to_be_learned.remove(id)

        self.labels[id].setDefaultTextColor(QtCore.Qt.white)
        self.labels[id].setPlainText(self.get_text_for_item(self.get_item_by_id(id)))
        
    def get_item_to_learn(self):
        
        if len(self.items_to_be_learned) == 0: return None
        
        for it in self.prog.items:
            
            if it.id == self.items_to_be_learned[0]:
                
                self.set_current(self.prog.id,  it.id)
                return it
                
        return None # TODO rather throw exception?
        
    def set_prog(self,  prog,  template = False):
        
        self.prog = prog
        self.template = False
        
        if template:
            
            for it in self.prog.items:
                if it.type in self.items_req_learning: self.items_to_be_learned.append(it.id)
            
            rospy.loginfo("Loading program template: " + self.prog.name + ", " + str(len(self.prog.items)) + " steps, with " + str(len(self.items_to_be_learned)) + ' steps to be learned')
            
        else:
            
            rospy.loginfo("Loading program: " + self.prog.name + ", " + str(len(self.prog.items)) + " steps")
            self.items_to_be_learned = []
        
        self.emit(QtCore.SIGNAL('set_prog'))
    
    def set_current(self,  prog_id,  step_id):
        
        self.current_prog_id = prog_id
        self.current_step_id = step_id
        self.emit(QtCore.SIGNAL('set_current'))
    
    def set_bold(self,  label,  bold=True):
        
        myFont=QtGui.QFont('Arial', 14)
        myFont.setBold(bold)
        label.setFont(myFont)
        
    def set_current_evt(self):
        
        for id,  lab in self.labels.iteritems():
            if id >= 0:
                self.set_bold(lab,  False)
        
        self.set_bold(self.labels[self.current_step_id])
    
    def get_text_for_item(self,  it):
        
        if it.id in self.items_to_be_learned:
                
                pose_str = "x=??, y=??"
                obj = "??"
                
        else:
            
            pose_str = "x=" + str(round(it.place_pose.pose.position.x,  2)) + ", y=" + str(round(it.place_pose.pose.position.y,  2))
            obj = it.object
        
        if it.type == ProgramItem.GET_READY:
            return "[" + str(it.id) + "] get ready"
        elif it.type == ProgramItem.MANIP_PICK:
            
            if it.spec == ProgramItem.MANIP_ID:
                return "[" + str(it.id) + "] pick object ID="
            elif it.spec == ProgramItem.MANIP_TYPE:
                return "[" + str(it.id) + "] pick object type="
                
        elif it.type == ProgramItem.MANIP_PLACE:
            
            # TODO pose / polygon
            return "[" + str(it.id) + "] place object at "  + pose_str
            
        elif it.type == ProgramItem.MANIP_PICK_PLACE:
            
            if it.spec == ProgramItem.MANIP_ID:
                return "[" + str(it.id) + "] pick object ID=" + obj + ", place to " + pose_str
            elif it.spec == ProgramItem.MANIP_TYPE:
                return "[" + str(it.id) + "] pick object type=" + obj + ", place to " + pose_str
                
        elif it.type == ProgramItem.WAIT:
            
            if it.spec == ProgramItem.WAIT_FOR_USER:
                return "[" + str(it.id) + "] wait for user"
            elif it.spec == ProgramItem.WAIT_UNTIL_USER_FINISHES:
                return "[" + str(it.id) + "] wait until user finishes"

    def prog_evt(self):
        
        self.header = self.scene.addText(self.prog.name, QtGui.QFont('Arial', 14))
        self.header.setPos(self.width() - 10,  self.height() - 10)
        self.header.rotate(180)
        self.header.setDefaultTextColor(QtCore.Qt.white)
        self.set_bold(self.header)
        
        for it in self.prog.items:
            
            self.labels[it.id] = self.scene.addText(self.get_text_for_item(it), QtGui.QFont('Arial', 14))
            self.labels[it.id].setPos(self.width() - 10,  self.height() - (20+len(self.labels)*25))
            self.labels[it.id].rotate(180)
            
            if it.id in self.items_to_be_learned:
                self.labels[it.id].setDefaultTextColor(QtCore.Qt.red)
            else:
                self.labels[it.id].setDefaultTextColor(QtCore.Qt.white)

def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QtGui.QApplication.quit()

# just for testing purposes
def main(args):
    
    rospy.init_node('program_widget_node', anonymous=True)
    
    signal.signal(signal.SIGINT, sigint_handler)

    app = QtGui.QApplication(sys.argv)
    window = program_widget(None)
    
    window.resize(200, 200)
    window.show()
    
    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.
    
    rospy.wait_for_service('/art/db/program/get')
    
    try:
        prog_srv = rospy.ServiceProxy('/art/db/program/get', getProgram)
        resp = prog_srv(0)
        window.set_prog(resp.program)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
