#!/usr/bin/env python

import sys
import signal
import rospy
from PyQt4 import QtGui, QtCore
from art_msgs.msg import Program,  ProgramItem
from art_msgs.srv import getProgram

# TODO rotate label
# TODO highlight current step

class program_widget(QtGui.QWidget):

    def __init__(self,  parent):
       
        super(program_widget, self).__init__(parent)
        
        QtCore.QObject.connect(self, QtCore.SIGNAL('set_prog'), self.prog_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('set_current'), self.set_current_evt)
        
        self.labels = {}
        
    def set_prog(self,  prog):
        
        self.prog = prog
        rospy.loginfo("Loading program: " + self.prog.name + ", " + str(len(self.prog.items)) + " steps")
        self.emit(QtCore.SIGNAL('set_prog'))
        
    def set_current(self,  prog_id,  step_id):
        
        self.current_prog_id = prog_id
        self.current_step_id = step_id
        self.emit(QtCore.SIGNAL('set_current'))
        
    def set_current_evt(self):
        
        myFont=QtGui.QFont()
        myFont.setBold(False)
        
        for id,  lab in self.labels.iteritems():
            if id >= 0:
                lab.setFont(myFont)
        
        myFont.setBold(True)
        self.labels[self.current_step_id].setFont(myFont)
    
    def label(self,  text,  id=-100):
        
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.white)
        lab = QtGui.QLabel(text)
        lab.setPalette(palette)
        self.labels[id] = lab
        return lab

    def prog_evt(self):
        
        self.vbox = QtGui.QVBoxLayout()
        self.setLayout(self.vbox)

        self.vbox.addWidget(self.label(self.prog.name))
        
        for it in self.prog.items:
            
            pose_str = "x=" + str(round(it.place_pose.pose.position.x,  2)) + ", y=" + str(round(it.place_pose.pose.position.x,  2))
            
            if it.type == ProgramItem.GET_READY:
                self.vbox.addWidget(self.label("[" + str(it.id) + "] get ready",  it.id))
            elif it.type == ProgramItem.MANIP_PICK:
                
                if it.spec == ProgramItem.MANIP_ID:
                    self.vbox.addWidget(self.label("[" + str(it.id) + "] pick object ID=" + it.object, it.id))
                elif it.spec == ProgramItem.MANIP_TYPE:
                    self.vbox.addWidget(self.label("[" + str(it.id) + "] pick object type=" + it.object,  it.id))
                    
            elif it.type == ProgramItem.MANIP_PLACE:
                
                # TODO pose / polygon
                self.vbox.addWidget(self.label("[" + str(it.id) + "] place object at "  + pose_str,  it.id))
                
            elif it.type == ProgramItem.MANIP_PICK_PLACE:
                
                if it.spec == ProgramItem.MANIP_ID:
                    self.vbox.addWidget(self.label("[" + str(it.id) + "] pick object ID=" + it.object) + ", place to " + pose_str,  it.id)
                elif it.spec == ProgramItem.MANIP_TYPE:
                    self.vbox.addWidget(self.label("[" + str(it.id) + "] pick object type=" + it.object + ", place to " + pose_str,  it.id))
                    
            elif it.type == ProgramItem.WAIT:
                
                if it.spec == ProgramItem.WAIT_FOR_USER:
                    self.vbox.addWidget(self.label("[" + str(it.id) + "] wait for user",  it.id))
                elif it.spec == ProgramItem.WAIT_UNTIL_USER_FINISHES:
                    self.vbox.addWidget(self.label("[" + str(it.id) + "] wait until user finishes",  it.id))
                    
        

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
    
    rospy.wait_for_service('/art_db/program/get')
    
    try:
        prog_srv = rospy.ServiceProxy('/art_db/program/get', getProgram)
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
