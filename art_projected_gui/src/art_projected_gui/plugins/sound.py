from art_projected_gui.plugins import GuiPlugin
import rospy
from PyQt4 import QtCore
from std_srvs.srv import Empty

translate = QtCore.QCoreApplication.translate


class SoundPlugin(GuiPlugin):

    def __init__(self, ui, parameters):

        super(SoundPlugin, self).__init__(ui)

        self.sound_info_srv = rospy.ServiceProxy("/art/interface/sound/info", Empty)
        self.sound_warning_srv = rospy.ServiceProxy("/art/interface/sound/warning", Empty)

    def init(self):

        rospy.loginfo("Waiting for sound services...")
        self.sound_info_srv.wait_for_service()
        self.sound_warning_srv.wait_for_service()

    def notify_info(self):

        try:
            self.sound_info_srv.call()
        except rospy.ServiceException:
            rospy.logerr("Sound service call failed!")
            pass

    def notify_warn(self):

        try:
            self.sound_warning_srv.call()
        except rospy.ServiceException:
            rospy.logerr("Sound service call failed!")
            pass
