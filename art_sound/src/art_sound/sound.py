import rospy
from art_msgs.msg import InterfaceState
from art_utils import InterfaceStateManager
from playsound import playsound
import rospkg
import os
from std_srvs.srv import Empty, EmptyResponse
from threading import Thread
from Queue import Queue


class ArtSound(object):
    def __init__(self):

        pkg = rospkg.RosPack().get_path('art_sound')

        # TODO info/error
        self.sounds = {
            InterfaceState.INFO: os.path.join(pkg, 'sounds', 'info.mp3'),
            InterfaceState.WARNING: os.path.join(pkg, 'sounds', 'warning.mp3')
        }

        self.manager = InterfaceStateManager("art_sound", cb=self.state_cb)

        self.srv_info = rospy.Service("info", Empty, self.srv_info_cb)
        self.srv_warning = rospy.Service("warning", Empty, self.srv_warning_cb)

        self.q = Queue()

        self.th = Thread(target=self.thread_cb)
        self.th.daemon = True
        self.th.start()

    def thread_cb(self):

        while not rospy.is_shutdown():

            lvl = self.q.get()
            playsound(self.sounds[lvl])

    def srv_info_cb(self, req):

        self.q.put(InterfaceState.INFO)
        return EmptyResponse()

    def srv_warning_cb(self, req):

        self.q.put(InterfaceState.WARNING)
        return EmptyResponse()

    def state_cb(self, old, new, flags):

        if new.error_severity != old.error_severity:

            if new.error_severity in self.sounds:

                self.q.put(new.error_severity)
