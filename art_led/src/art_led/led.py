import rospy
from art_msgs.msg import InterfaceState
from art_helpers import InterfaceStateManager
from std_srvs.srv import Empty, EmptyResponse
from art_msgs.msg import Color


class ArtLed(object):
    def __init__(self):

        self.manager = InterfaceStateManager("art_led", cb=self.state_cb)

        self.ns = "/art/interface/led"
        self.srv_info = rospy.Service(self.ns + "info", Empty, self.srv_info_cb)
        self.srv_warning = rospy.Service(self.ns + "warning", Empty, self.srv_warning_cb)
        self.srv_error = rospy.Service(self.ns + "error", Empty, self.srv_error_cb)

        self.color_publisher = rospy.Publisher("/art/interface/led/set_color", Color, queue_size=1)

    def publish_color(self, state):

        color = Color()

        if state == InterfaceState.ERROR:
            color.r = 0.7
            color.b = 0
            color.g = 0
            color.mode = color.SOLID
            self.color_publisher.publish(color)
        elif state == InterfaceState.WARNING:
            color.r = 0.7
            color.b = 0
            color.g = 0
            color.mode = color.BREATHING

            self.color_publisher.publish(color)
        elif state == InterfaceState.INFO:
            color.r = 0
            color.b = 0
            color.g = 0.7
            color.mode = color.BREATHING
            color.timeout = 1
            self.color_publisher.publish(color)
        elif state == InterfaceState.NONE:
            color.r = 0
            color.b = 0
            color.g = 0
            color.mode = color.SOLID
            color.timeout = 0
            self.color_publisher.publish(color)

    def srv_info_cb(self, req):

        self.publish_color(InterfaceState.INFO)
        return EmptyResponse()

    def srv_warning_cb(self, req):

        self.publish_color(InterfaceState.WARNING)
        return EmptyResponse()

    def srv_error_cb(self, req):

        self.publish_color(InterfaceState.ERROR)
        return EmptyResponse()

    def state_cb(self, old, new, flags):

        if new.error_severity != old.error_severity:

            if new.error_severity in [
                    InterfaceState.ERROR,
                    InterfaceState.WARNING,
                    InterfaceState.INFO,
                    InterfaceState.NONE]:
                print new.error_severity
                print old.error_severity
                self.publish_color(new.error_severity)

        elif new.system_state != old.system_state:

            self.publish_color(InterfaceState.INFO)
