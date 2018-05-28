from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore
import rospy
from art_msgs.msg import InstancesArray
from art_msgs.srv import NotifyUserRequest
from geometry_msgs.msg import PoseStamped
from art_projected_gui.items import DialogItem

translate = QtCore.QCoreApplication.translate


class PickFromFeeder(GuiInstruction):

    CONTEXT = "PickFromFeeder"

    def __init__(self, ui):

        super(PickFromFeeder, self).__init(ui)

    def cleanup(self):

        super(PickFromFeeder, self).cleanup()


class PickFromFeederLearn(PickFromFeeder):

    def __init__(self, ui, editable=False):

        super(PickFromFeederLearn, self).__init(ui)

        self.editable = editable

        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'objects_raw'), self.object_raw_cb_evt)

        self.grasp_dialog = None
        self.grasp_dialog_timer = QtCore.QTimer()
        self.grasp_dialog_timer.timeout.connect(self.grasp_dialog_timer_tick)
        self.grasp_dialog_timer.start(1000)

        self.obj_raw_sub = rospy.Subscriber(
            '/art/object_detector/object', InstancesArray, self.object_raw_cb, queue_size=1)

        self.objects_by_sensor = {}

        if self.ui.ph.is_object_set(block_id, item_id):

            self.select_object_type(self.ph.get_object(block_id, item_id)[0][0])

            if state.edit_enabled:
                self.create_grasp_dialog()

        else:

            if self.editable:
                self.notif(
                    translate("UICoreRos", "Select object type to be picked up by tapping on its outline."))

    def create_grasp_dialog(self):

        if not self.grasp_dialog:

            if not self.ui.ph.is_pose_set(self.ui.program_vis.block_id, self.ui.program_vis.get_current_item().id):

                self.ui.notif(
                    translate("UICoreRos", "Use robot's arm and dialog to teach pose enabling part detection."))

            else:

                self.ui.notif(
                    translate(
                        "UICoreRos",
                        "Learned pose for part detection may be updated or different object type could be chosen."))

            self.grasp_dialog = DialogItem(
                self.ui.scene, self.ui.width / 2, 0.1, translate(
                    "UICoreRos", "Save gripper pose"), [
                    translate(
                        "UICoreRos", "Right arm (%1)").arg(0), translate(
                        "UICoreRos", "Left arm (%1)").arg(0)], self.save_gripper_pose_cb)

            for it in self.grasp_dialog.items:
                it.set_enabled(False)

    def save_gripper_pose_cb(self, idx):

        topics = ['/art/robot/right_arm/gripper/pose',
                  '/art/robot/left_arm/gripper/pose']

        # wait for message, set pose
        try:
            ps = rospy.wait_for_message(topics[idx], PoseStamped, timeout=2)
        except(rospy.ROSException) as e:
            rospy.logerr(str(e))
            self.notif(
                translate("UICoreRos", "Failed to store gripper pose."), temp=True, message_type=NotifyUserRequest.WARN)
            return

        self.notif(translate("UICoreRos", "Gripper pose stored."), temp=True)
        self.snd_info()
        self.program_vis.set_pose(ps)

        self.grasp_dialog.items[idx].set_enabled(False)
        self.grasp_dialog.items[idx].set_caption(translate("UICoreRos", "Stored"))

    def object_raw_cb_evt(self, msg):

        cnt = 0

        for obj in msg.instances:

            if obj.object_type in self.ui.selected_object_types:

                # this mainly serves for detection of objects in feeder so let's count only objects not on table
                o = self.ui.get_object(obj.object_id)

                if o and o.on_table:
                    continue

                cnt += 1

        self.objects_by_sensor[msg.header.frame_id] = [cnt, msg.header.stamp]

    def grasp_dialog_timer_tick(self):

        now = rospy.Time.now()

        for k, v in self.objects_by_sensor.iteritems():

            if now - v[1] > rospy.Duration(1.0):
                v[0] = 0

        if self.grasp_dialog:

            if now - self.ui.program_vis.get_current_item().pose[0].header.stamp < rospy.Duration(3.0):
                return

            frames = ["/r_forearm_cam_optical_frame", "/l_forearm_cam_optical_frame"]
            names = [translate("UICoreRos", "Right arm (%1)"), translate("UICoreRos", "Left arm (%1)")]

            for i in range(len(frames)):

                if frames[i] in self.objects_by_sensor:

                    cnt = self.objects_by_sensor[frames[i]][0]

                else:

                    cnt = 0

                self.grasp_dialog.items[i].set_enabled(cnt == 1)
                self.grasp_dialog.items[i].set_caption(names[i].arg(cnt))

    def object_raw_cb(self, msg):

        self.emit(QtCore.SIGNAL('objects_raw'), msg)

    def cleanup(self):

        super(PickFromFeeder, self).cleanup()

        self.ui.scene.removeItem(self.grasp_dialog)
        self.grasp_dialog = None


class PickFromFeederRun(PickFromFeeder):

    def __init__(self, ui):

        super(PickFromFeederRun, self).__init(ui)

        ps = self.ui.ph.get_pose(state.block_id, state.program_current_item.id)[0][0]

        if ps.pose.position.x < 1.5 / 2.0:
            self.ui.notif(
                translate("UICoreRos", "Picking object from feeder on my right."))
        else:
            self.ui.notif(
                translate("UICoreRos", "Picking object from feeder on my left."))
