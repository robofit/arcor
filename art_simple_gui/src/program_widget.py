#!/usr/bin/env python

import sys
import signal
import rospy
from PyQt4 import QtGui, QtCore
from art_msgs.msg import Program,  ProgramItem
from art_msgs.srv import getProgram

class program_widget(QtGui.QWidget):

    def __init__(self):
       
        super(program_widget, self).__init__()
        
        QtCore.QObject.connect(self, QtCore.SIGNAL('prog'), self.prog_evt)
        
    def prog(self,  prog):
        
        self.emit(QtCore.SIGNAL('prog'),  prog)
        
    def prog_evt(self,  prog):
        
        self.vbox = QtGui.QVBoxLayout()
        self.setLayout(self.vbox)
        
        self.vbox.addWidget(QtGui.QLabel(prog.name))
        
        for it in prog.items:
            
            pose_str = "x=" + str(round(it.place_pose.pose.position.x,  2)) + ", y=" + str(round(it.place_pose.pose.position.x,  2))
            
            if it.type == ProgramItem.GET_READY:
                self.vbox.addWidget(QtGui.QLabel("[" + str(it.id) + "] get ready"))
            elif it.type == ProgramItem.MANIP_PICK:
                
                if it.spec == ProgramItem.MANIP_ID:
                    self.vbox.addWidget(QtGui.QLabel("[" + str(it.id) + "] pick object ID=" + it.object))
                elif it.spec == ProgramItem.MANIP_TYPE:
                    self.vbox.addWidget(QtGui.QLabel("[" + str(it.id) + "] pick object type=" + it.object))
                    
            elif it.type == ProgramItem.MANIP_PLACE:
                
                # TODO pose / polygon
                self.vbox.addWidget(QtGui.QLabel("[" + str(it.id) + "] place object at "  + pose_str))
                
            elif it.type == ProgramItem.MANIP_PICK_PLACE:
                
                if it.spec == ProgramItem.MANIP_ID:
                    self.vbox.addWidget(QtGui.QLabel("[" + str(it.id) + "] pick object ID=" + it.object) + ", place to " + pose_str)
                elif it.spec == ProgramItem.MANIP_TYPE:
                    self.vbox.addWidget(QtGui.QLabel("[" + str(it.id) + "] pick object type=" + it.object + ", place to " + pose_str))
                    
            elif it.type == ProgramItem.WAIT:
                
                if it.spec == ProgramItem.WAIT_FOR_USER:
                    self.vbox.addWidget(QtGui.QLabel("[" + str(it.id) + "] wait for user"))
                elif it.spec == ProgramItem.WAIT_UNTIL_USER_FINISHES:
                    self.vbox.addWidget(QtGui.QLabel("[" + str(it.id) + "] wait until user finishes"))
                    
        

def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QtGui.QApplication.quit()

# just for testing purposes
def main(args):
    
    rospy.init_node('program_widget_node', anonymous=True)
    
    signal.signal(signal.SIGINT, sigint_handler)

    app = QtGui.QApplication(sys.argv)
    window = program_widget()
    
    window.resize(200, 200)
    window.show()
    
    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.
    
    rospy.wait_for_service('/art_program/get')
    
    try:
        prog_srv = rospy.ServiceProxy('/art_program/get', getProgram)
        resp = prog_srv(0)
        window.prog(resp.program)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
