from art_projected_gui.plugins import GuiPlugin
import rospy
from PyQt4 import QtCore
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
from art_projected_gui.helpers import ProjectorHelper

translate = QtCore.QCoreApplication.translate


class ProjectorsPlugin(GuiPlugin):

    def __init__(self, ui, parameters):

        super(ProjectorsPlugin, self).__init__(ui)

        self.projectors = []

        try:

            for proj in parameters["projectors"]:
                if proj:
                    self.projectors.append(ProjectorHelper(proj))

        except KeyError:
            pass

        if not self.projectors:
            rospy.loginfo("Starting with no projector.")

        self.projectors_calibrated_pub = rospy.Publisher(GuiPlugin.BASE_NS + "projectors_calibrated", Bool,
                                                         queue_size=1, latch=True)
        self.projectors_calibrated_pub.publish(False)
        self.projector_calib_srv = None

    def init(self):

        proj_calib = True

        if self.projectors:
            rospy.loginfo("Waiting for projector nodes...")
            for proj in self.projectors:
                proj.wait_until_available()
                if not proj.is_calibrated():
                    proj_calib = False

        if proj_calib:

            rospy.loginfo('Projectors already calibrated.')
            self.projectors_calibrated_pub.publish(True)

        else:

            rospy.loginfo('Projectors not calibrated yet - waiting for command...')

        self.projector_calib_srv = rospy.Service(GuiPlugin.BASE_NS + 'calibrate_projectors', Trigger,
                                                 self.calibrate_projectors_cb)

    def calibrate_projectors_cb(self, req):

        resp = TriggerResponse()
        resp.success = True

        # call to start_projector_calibration is blocking
        self.proj_calib_timer = rospy.Timer(rospy.Duration(0.001), self.start_projector_calibration, oneshot=True)

        return resp

    def calib_done_cb(self, proj):

        if proj.is_calibrated():

            self.calib_proj_cnt += 1

            while self.calib_proj_cnt < len(self.projectors):

                if self.projectors[self.calib_proj_cnt].is_calibrated():
                    self.calib_proj_cnt += 1
                    continue

                self.projectors[self.calib_proj_cnt].calibrate(
                    self.calib_done_cb)
                return

            rospy.loginfo('Projectors calibrated.')
            self.projectors_calibrated_pub.publish(True)

        else:

            # calibration failed - let's try again
            rospy.logerr('Calibration failed for projector: ' + proj.proj_id)
            proj.calibrate(self.calib_done_cb)

    def start_projector_calibration(self, evt):

        if not self.projectors:

            rospy.loginfo('No projectors to calibrate.')
            self.projectors_calibrated_pub.publish(True)

        else:

            self.projectors_calibrated_pub.publish(False)
            rospy.loginfo('Starting calibration of ' +
                          str(len(self.projectors)) + ' projector(s)')

            self.calib_proj_cnt = 0

            for proj in self.projectors:

                if proj.is_calibrated():

                    self.calib_proj_cnt += 1
                    continue

                else:

                    if not proj.calibrate(self.calib_done_cb):
                        # TODO what to do?
                        rospy.logerr("Failed to start projector calibration")

                    return

            rospy.loginfo('Projectors calibrated.')
            self.projectors_calibrated_pub.publish(True)
