from art_instructions.gui import GuiInstruction
from art_projected_gui.items import ImageItem, DialogItem
import qimage2ndarray
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from PyQt4 import QtCore
from art_utils import array_from_param

translate = QtCore.QCoreApplication.translate


class VisualInspection(GuiInstruction):

    NAME = translate("VisualInspection", "Visual inspection")

    def __init__(self, *args, **kwargs):

        super(VisualInspection, self).__init__(*args, **kwargs)

        self.bridge = CvBridge()

        topic = ""
        try:
            topic = rospy.get_param("/art/visual_inspection/topic")
        except KeyError:
            self.logerr("Topic for visual inspection not set!")

        if not topic:
            self.logwarn("Using default topic!")
            topic = "image_color"

        self.img_sub = rospy.Subscriber(topic, Image, self.image_callback, queue_size=1)
        self.result_sub = rospy.Subscriber("/art/visual_inspection/result", Bool, self.result_callback, queue_size=10)

        try:
            img_origin = array_from_param("/art/visual_inspection/origin", float, 2)
            img_size = array_from_param("/art/visual_inspection/size", float, 2)
            fixed = True
        except KeyError:
            img_origin = (0.3, 0.3)
            img_size = (0.2, 0.1)
            fixed = False

        self.img_item = ImageItem(self.ui.scene, img_origin[0], img_origin[1], img_size[0], img_size[1], fixed)

        self.text_timer = QtCore.QTimer()
        self.text_timer.timeout.connect(self.text_timer_tick)
        self.text_timer.setSingleShot(True)

    @staticmethod
    def get_text(ph, block_id, item_id):

        text = "\n"

        if ph.is_pose_set(block_id, item_id):
            text += translate("VisualInspection", "     Pose stored.")
        else:
            text += translate("VisualInspection", "     Pose has to be set.")

        return text

    def result_callback(self, msg):

        if not self.img_item:
            return

        if msg.data:
            self.img_item.set_text("OK", QtCore.Qt.green)
        else:
            self.img_item.set_text("NOK", QtCore.Qt.red)

        self.text_timer.start(1000)

    def text_timer_tick(self):

        if not self.img_item:
            return

        self.img_item.set_text()

    def image_callback(self, msg):

        if not self.img_item:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as e:
            print(e)
            return

        self.img_item.set_image(qimage2ndarray.array2qimage(cv_image))

    def cleanup(self):

        self.img_sub.unregister()
        self.result_sub.unregister()
        self.ui.scene.removeItem(self.img_item)
        self.img_item = None


class VisualInspectionLearn(VisualInspection):

    def __init__(self, *args, **kwargs):

        super(VisualInspectionLearn, self).__init__(*args, **kwargs)

        self.dialog = None

        if self.editable:

            self.ui.notif(
                translate(
                    "VisualInspection",
                    "Now you may adjust pose for visual inspection."))

            self.dialog = DialogItem(
                self.ui.scene, self.ui.width / 2, 0.1, translate(
                    "VisualInspection", "Save visual inspection pose"),
                [arm.name(self.ui.loc) for arm in self.ui.rh.get_robot_arms()], self.save_pose_cb)

            self.dialog_timer = QtCore.QTimer()
            self.dialog_timer.timeout.connect(self.dialog_timer_tick)
            self.dialog_timer.setSingleShot(True)

    def save_pose_cb(self, idx):

        ps = self.ui.rh.get_robot_arms()[idx].get_pose()

        if ps:

            self.ui.notif(translate("VisualInspection", "Pose was stored."), temp=True)
            self.ui.snd_info()
            self.ui.program_vis.set_pose(ps)
            self.dialog.items[idx].set_caption(translate("VisualInspection", "Stored"))

        else:

            self.ui.notif(translate("VisualInspection", "Failed to get pose."), temp=True)
            self.ui.snd_warn()
            self.dialog.items[idx].set_caption(translate("VisualInspection", "Failed"))

        self.dialog.items[idx].set_enabled(False)
        self.dialog_timer.start(1000)

    def cleanup(self):

        super(VisualInspectionLearn, self).cleanup()

        if self.dialog:
            self.ui.scene.removeItem(self.dialog)
            self.dialog = None

    def dialog_timer_tick(self):

        for idx, arm in enumerate(self.ui.rh.get_robot_arms()):
            self.dialog.items[idx].set_caption(arm.name(self.ui.loc))

        for v in self.dialog.items:
            v.set_enabled(True)


class VisualInspectionRun(VisualInspection):

    def __init__(self, *args, **kwargs):

        super(VisualInspectionRun, self).__init__(*args, **kwargs)

        self.ui.notif(
            translate(
                "VisualInspection",
                "Visual inspection in progress..."))
