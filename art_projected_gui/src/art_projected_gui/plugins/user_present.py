from PyQt4 import QtSvg
from art_projected_gui.plugins import GuiPlugin
import rospy
from std_msgs.msg import Bool
import rospkg

from PyQt4 import QtCore

translate = QtCore.QCoreApplication.translate


class UserPresentPlugin(GuiPlugin):

    def __init__(self, *args, **kwargs):

        super(UserPresentPlugin, self).__init__(*args, **kwargs)

        rospack = rospkg.RosPack()
        icons_path = rospack.get_path('art_projected_gui') + '/icons/'

        self.icon = QtSvg.QGraphicsSvgItem(icons_path + 'person.svg', self)
        self.icon.setVisible(False)

        # TODO make position/size configurable
        self.icon.setScale(self.m2pix(0.05)/self.icon.boundingRect().height())
        self.icon.setPos(self.m2pix(0.05), self.m2pix(0.08))

        self.texts = {True: translate("UserPresentPlugin", "User detected."),
                      False: translate("UserPresentPlugin", "User left.")}

        QtCore.QObject.connect(self, QtCore.SIGNAL('user_pres_evt'), self.user_pres_evt)
        self.user_pres = None
        self.user_pres_sub = rospy.Subscriber("/art/interface/user/present", Bool, self.user_pres_cb)

        rospy.loginfo("UserPresentPlugin ready.")

    def user_pres_cb(self, msg):

        self.emit(QtCore.SIGNAL('user_pres_evt'), msg)

    def user_pres_evt(self, msg):

        if self.user_pres is None or msg.data != self.user_pres:
            self.user_pres = msg.data
            self.icon.setVisible(self.user_pres)
            self.ui.notif(self.texts[self.user_pres])
