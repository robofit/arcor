#!/usr/bin/env python

import unittest
import rostest
import sys
from art_projected_gui.gui import UICoreRos
from art_projected_gui.items import ObjectItem
from PyQt4.QtGui import QApplication
import rospy
import rospkg
from PyQt4 import QtCore
from art_msgs.msg import UserStatus

app = QApplication(sys.argv)

rospack = rospkg.RosPack()

translator = QtCore.QTranslator()
translator.load(QtCore.QLocale.system().name() + '.qm',
                rospack.get_path('art_projected_gui') + '/lang')
app.installTranslator(translator)


class TestUICoreRos(unittest.TestCase):

    def test_ui(self):

        user_status_pub = rospy.Publisher(
            "/art/user/status", UserStatus, queue_size=10)

        ui = UICoreRos()
        ui.start()

        rospy.sleep(2.0)
        QtCore.QCoreApplication.processEvents(QtCore.QEventLoop.AllEvents, 50)

        objects = list(ui.get_scene_items_by_type(ObjectItem))
        self.assertEquals(len(objects), 3, 'test_detected_object_count')

        self.assertEquals(ui.fsm.state, 'waiting_for_user',
                          'test_state_waiting_for_user')

        user_status_pub.publish(UserStatus(
            user_state=UserStatus.USER_NOT_CALIBRATED))
        rospy.sleep(0.1)
        QtCore.QCoreApplication.processEvents(QtCore.QEventLoop.AllEvents, 50)

        rospy.loginfo(ui.fsm.state)
        self.assertEquals(ui.fsm.state, 'waiting_for_user_calibration',
                          'test_state_waiting_for_user_calibration')

        user_status_pub.publish(UserStatus(
            user_state=UserStatus.USER_CALIBRATED))
        rospy.sleep(0.1)
        QtCore.QCoreApplication.processEvents(QtCore.QEventLoop.AllEvents, 50)

        self.assertEquals(ui.fsm.state, 'program_selection',
                          'test_state_program_selection')
        self.assertEquals(ui.program_list.isVisible(),
                          True, "test_program_list_visible")
        self.assertEquals(ui.program_list.isEnabled(),
                          True, "test_program_list_enabled")
        self.assertEquals(ui.program_vis, None, "test_program_vis_none")

        ui.program_list.list.set_current_idx(0, select=True)
        ui.program_list.item_selected_cb()
        self.assertEquals(ui.program_list.run_btn.isEnabled(),
                          True, "test_program_selection")

        ui.program_list.run_btn.cursor_click()
        self.assertEquals(ui.fsm.state, 'running',
                          'test_state_program_running')

        self.assertEquals(ui.program_vis.isVisible(),
                          True, "test_program_vis_visible")
        self.assertEquals(ui.program_vis.isEnabled(),
                          True, "test_program_vis_enabled")
        self.assertEquals(ui.program_list, None, "test_program_list_none")

        # TODO test running program visualization


if __name__ == '__main__':

    rospy.init_node('test_node')
    rospy.sleep(2.0)  # ...let init_db script does its work...
    rostest.run('art_projected_gui', 'test_ui_core_ros',
                TestUICoreRos, sys.argv)
