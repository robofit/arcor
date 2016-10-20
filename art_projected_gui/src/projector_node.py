#!/usr/bin/env python

import sys
import signal
import rospy
from PyQt4 import QtGui, QtCore
from gui import Projector


def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QtGui.QApplication.quit()

def main(args):

    rospy.init_node('projected_gui_projector', anonymous=True)

    signal.signal(signal.SIGINT, sigint_handler)

    app = QtGui.QApplication(sys.argv)

    projector_id = rospy.get_param('~projector_id', 'test')
    screen_number = rospy.get_param('~screen_number', 0)
    camera_image_topic = rospy.get_param('~camera_image_topic', '/kinect2/sd/image_color_rect')
    camera_info_topic = rospy.get_param('~camera_info_topic', '/kinect2/sd/camera_info')
    calibration_matrix = rospy.get_param('~calibration_matrix',  None)

    proj = Projector(projector_id,  screen_number, camera_image_topic,  camera_info_topic,  calibration_matrix)

    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.

    sys.exit(app.exec_())

if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
