from art_projected_gui.plugins import GuiPlugin
import rospy
from std_srvs.srv import Trigger
from art_msgs.srv import ProgramIdTrigger
from art_msgs.msg import HololensState
from std_msgs.msg import Bool

from PyQt4 import QtCore

translate = QtCore.QCoreApplication.translate


class HololensGuiPlugin(GuiPlugin):

    def __init__(self, ui, parameters):

        super(HololensGuiPlugin, self).__init__(ui)

        # HoloLens visualization
        self.start_visualizing_srv = rospy.ServiceProxy(
            '/art/brain/visualize/start', ProgramIdTrigger)  # TODO wait for service? where?
        self.stop_visualizing_srv = rospy.ServiceProxy(
            '/art/brain/visualize/stop', Trigger)  # TODO wait for service? where?
        # for checking if HoloLens is connected
        self.hololens_active_sub = rospy.Subscriber(
            '/art/interface/hololens/active/', Bool, self.hololens_active_cb)
        self.hololens_connected = False
        self.hololens_state_pub = rospy.Publisher(
            '/art/interface/hololens/state', HololensState)
