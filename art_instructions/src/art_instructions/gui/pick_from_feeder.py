from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore
import rospy
from art_msgs.msg import InstancesArray
from art_projected_gui.items import DialogItem

translate = QtCore.QCoreApplication.translate


class PickFromFeeder(GuiInstruction):

    NAME = translate("PickFromFeeder", "Pick from feeder")

    def __init__(self, *args, **kwargs):

        super(PickFromFeeder, self).__init__(*args, **kwargs)

    @staticmethod
    def get_text(ph, block_id, item_id):

        text = "\n"

        if ph.is_pose_set(block_id, item_id):
            text += translate("PickFromFeeder", "     Pose stored.")
        else:
            text += translate("PickFromFeeder", "     Pose has to be set.")

        return text


class PickFromFeederLearn(PickFromFeeder):

    def __init__(self, *args, **kwargs):

        super(PickFromFeederLearn, self).__init__(*args, **kwargs)

        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'objects_raw'), self.object_raw_cb_evt)

        self.grasp_dialog = None
        self.grasp_dialog_timer = QtCore.QTimer()
        self.grasp_dialog_timer.timeout.connect(self.grasp_dialog_timer_tick)
        self.grasp_dialog_timer.start(1000)

        # TODO delegate getting detections per frame to object helper
        self.obj_raw_sub = rospy.Subscriber(
            '/art/object_detector/object', InstancesArray, self.object_raw_cb, queue_size=1)

        self.objects_by_sensor = {}

        if self.ui.ph.is_object_set(*self.cid):

            self.ui.select_object_type(self.ui.ph.get_object(*self.cid)[0][0])

            if self.editable:
                self.create_grasp_dialog()

        else:

            if self.editable:
                self.ui.notif(
                    translate("PickFromFeeder", "Select object type to be picked up by tapping on its outline."))

    def create_grasp_dialog(self):

        if not self.grasp_dialog:

            if not self.ui.ph.is_pose_set(self.ui.program_vis.block_id, self.ui.program_vis.get_current_item().id):

                self.ui.notif(
                    translate("PickFromFeeder", "Use robot's arm and dialog to teach pose enabling part detection."))

            else:

                self.ui.notif(
                    translate(
                        "PickFromFeeder",
                        "Learned pose for part detection may be updated or different object type could be chosen."))

            self.grasp_dialog = DialogItem(
                self.ui.scene, self.ui.width / 2, 0.1, translate(
                    "PickFromFeeder", "Save gripper pose"),
                [arm.name(self.ui.loc) + " (0)" for arm in self.ui.rh.get_robot_arms()], self.save_gripper_pose_cb)

            for it in self.grasp_dialog.items:
                it.set_enabled(False)

    def save_gripper_pose_cb(self, idx):

        ps = self.ui.rh.get_robot_arms()[idx].get_pose()

        if ps:

            self.ui.notif(translate("PickFromFeeder", "Gripper pose stored."), temp=True)
            self.ui.snd_info()
            self.ui.program_vis.set_pose(ps)
            self.grasp_dialog.items[idx].set_caption(translate("PickFromFeeder", "Stored"))

        else:

            self.ui.notif(translate("PickFromFeeder", "Failed to get gripper pose."), temp=True)
            self.grasp_dialog.items[idx].set_caption(translate("PickFromFeeder", "Failed"))
            self.ui.snd_warn()

        self.grasp_dialog.items[idx].set_enabled(False)

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

    def object_selected(self, obj, selected, msg):

        # this type of object is already set
        if msg.object and obj.object_type.name == msg.object[0]:
            self.logdebug("object type " +
                          obj.object_type.name + " already selected")
            return
        else:
            # TODO remove previously inserted polygon, do not insert new
            # place
            self.logdebug("selecting new object type: " +
                          obj.object_type.name)

        if obj.object_type.name != self.ui.ph.get_object(self.ui.program_vis.block_id, msg.id)[0][0]:
            self.ui.program_vis.clear_poses()

        self.ui.program_vis.set_object(obj.object_type.name)
        self.ui.select_object_type(obj.object_type.name)
        self.create_grasp_dialog()

    def grasp_dialog_timer_tick(self):

        now = rospy.Time.now()

        for k, v in self.objects_by_sensor.iteritems():

            if now - v[1] > rospy.Duration(1.0):
                v[0] = 0

        if self.grasp_dialog:

            if now - self.ui.program_vis.get_current_item().pose[0].header.stamp < rospy.Duration(3.0):
                return

            for idx, arm in enumerate(self.ui.rh.get_robot_arms()):

                try:
                    cnt = self.objects_by_sensor[arm.camera_link][0]
                except KeyError:
                    cnt = 0

                self.grasp_dialog.items[idx].set_enabled(cnt == 1)
                self.grasp_dialog.items[idx].set_caption(arm.name(self.ui.loc) + " (" + str(cnt) + ")")

    def object_raw_cb(self, msg):

        self.emit(QtCore.SIGNAL('objects_raw'), msg)

    def cleanup(self):

        self.ui.scene.removeItem(self.grasp_dialog)
        self.grasp_dialog = None
        return ()

    def learning_done(self):

        if self.grasp_dialog:
            self.ui.scene.removeItem(self.grasp_dialog)
            self.grasp_dialog = None

    def detected_objects(self, msg):

        if self.grasp_dialog:

            sel_obj_type = self.ui.ph.get_object(*self.cid)[0][0]

            if self.grasp_dialog:

                for obj in msg.instances:

                    if obj.object_type == sel_obj_type:
                        self.grasp_dialog.set_enabled(True)
                        break
                else:
                    self.grasp_dialog.set_enabled(False)


class PickFromFeederRun(PickFromFeeder):

    def __init__(self, *args, **kwargs):

        super(PickFromFeederRun, self).__init__(*args, **kwargs)

        ps = self.ui.ph.get_pose(*self.cid)[0][0]

        if ps.pose.position.x < 1.5 / 2.0:  # TODO this is not nice solution!
            self.ui.notif(
                translate("PickFromFeeder", "Picking object from feeder on my right."))
        else:
            self.ui.notif(
                translate("PickFromFeeder", "Picking object from feeder on my left."))


class PickFromFeederVis(PickFromFeeder):

    def __init__(self, *args, **kwargs):

        super(PickFromFeederRun, self).__init__(*args, **kwargs)

        self.ui.select_object_type(self.ui.ph.get_object(*self.cid)[0][0])
