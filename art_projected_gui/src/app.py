#!/usr/bin/env python

import sys
import signal
import rospy
from PyQt4 import QtGui, QtCore
import rospkg

from art_projected_gui.gui import UICoreRos


def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QtGui.QApplication.quit()


def main():

    rospy.init_node('projected_gui', anonymous=True, log_level=rospy.DEBUG)

    while not rospy.is_shutdown():
        try:
            instructions = rospy.get_param("/art/instructions")
            break
        except KeyError:
            rospy.loginfo("Waiting for /art/instructions param...")
            rospy.sleep(1.0)

    if rospy.is_shutdown():
        return

    signal.signal(signal.SIGINT, sigint_handler)

    rospack = rospkg.RosPack()

    packages = ['art_projected_gui']

    try:

        for k, v in instructions["instructions"].iteritems():

            packages.append(v["gui"]["package"])

    except KeyError as e:

        rospy.logerr("Param /art/instructions is invalid: " + str(e))
        return

    app = QtGui.QApplication(sys.argv)

    loc = rospy.get_param('~locale', 'cs_CZ')

    for package in packages:

        translator = QtCore.QTranslator()
        try:
            translator.load(loc + '.qm', rospack.get_path(package) + '/lang')
        except rospkg.ResourceNotFound:
            rospy.logerr("Could not find package: " + package)
            continue

        app.installTranslator(translator)

    ui = UICoreRos(instructions)

    dbg = rospy.get_param('~show_scene', False)
    if dbg:
        ui.debug_view()

    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.

    sys.exit(app.exec_())


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down")
