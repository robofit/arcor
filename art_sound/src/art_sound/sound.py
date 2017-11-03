import rospy
from art_msgs.msg import InterfaceState
from art_utils import InterfaceStateManager
from playsound import playsound
import rospkg
import os

class ArtSound(object):
    def __init__(self):

        pkg = rospkg.RosPack().get_path('art_sound')

        # TODO info/error
        self.sounds = {InterfaceState.WARNING: os.path.join(pkg, 'sounds', 'warning.mp3')}

        self.manager = InterfaceStateManager("art_sound", cb=self.state_cb)

    def state_cb(self, old, new, flags):

        if new.error_severity != old.error_severity:

            if new.error_severity in self.sounds:

                playsound(self.sounds[new.error_severity])
