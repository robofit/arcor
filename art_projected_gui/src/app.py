#!/usr/bin/env python

import sys
import signal
import rospy
from PyQt4 import QtGui, QtCore

from projector import Projector
from projected_ui_ros import ProjectedUIRos
from object_item import ObjectItem

def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QtGui.QApplication.quit()

def main(args):
    
    rospy.init_node('projected_gui', anonymous=True)
    
    signal.signal(signal.SIGINT, sigint_handler)
    
    app = QtGui.QApplication(sys.argv)

    ui = ProjectedUIRos(0, 0, 1.2,  0.75)
    
    ui.debug_view()
    
    #ui.add_projector(Projector(0, None))
    ui.add_object("profile_1",  "profile",  0.5,  0.5)
    ui.add_place("PLACE POSE",  0.3,  0.2)
    
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
