from art_instructions.gui import GuiInstruction
from art_projected_gui.items import ImageItem, DialogItem
import qimage2ndarray
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from PyQt4 import QtCore
from geometry_msgs.msg import PoseStamped

translate = QtCore.QCoreApplication.translate


class VisualInspection(GuiInstruction):

    CONTEXT = "VisualInspection"

    def __init__(self, *args, **kwargs):

        super(VisualInspection, self).__init__(*args, **kwargs)

        self.bridge = CvBridge()
        # TODO read topic from some (setup) param
        self.img_sub = rospy.Subscriber("/kinect2/sd/image_color_rect", Image, self.image_callback, queue_size=1)
        self.result_sub = rospy.Subscriber("/art/visual_inspection/result", Bool, self.result_callback, queue_size=10)

        # TODO take placement from setup params
        self.img_item = ImageItem(self.ui.scene, 0, 0, 0.2, 0.05)

    def result_callback(self, msg):

        # TODO temporal green/red border of image
        pass

    def image_callback(self, msg):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        self.img_item.set_image(qimage2ndarray.array2qimage(cv_image))

    def cleanup(self):

        super(VisualInspection, self).cleanup()

        self.ui.scene.removeItem(self.img_item)


class VisualInspectionLearn(VisualInspection):

    def __init__(self, *args, **kwargs):

        super(VisualInspectionLearn, self).__init__(*args, **kwargs)

        self.ui.notif(
            translate(
                self.CONTEXT,
                "Now you may adjust pose for visual inspection."))

        # TODO how to know number of arms?
        self.dialog = DialogItem(
            self.ui.scene, self.ui.width / 2, 0.1, translate(
                self.CONTEXT, "Save visual inspection pose"), [
                translate(
                    self.CONTEXT, "Right arm"), translate(
                    self.CONTEXT, "Left arm")], self.save_pose_cb)

        self.dialog_timer = QtCore.QTimer()
        self.dialog_timer.timeout.connect(self.dialog_timer_tick)
        self.dialog_timer.setSingleShot(True)

    def save_pose_cb(self, idx):

        # TODO where to get topics / poses?
        topics = ['/art/robot/right_arm/gripper/pose',
                  '/art/robot/left_arm/gripper/pose']

        # wait for message, set pose
        try:
            ps = rospy.wait_for_message(topics[idx], PoseStamped, timeout=2)
        except rospy.ROSException as e:
            rospy.logerr(str(e))
            # TODO what to do?
            return

        self.ui.notif(translate(self.CONTEXT, "Pose was stored."), temp=True)
        self.ui.snd_info()
        self.ui.program_vis.set_pose(ps)

        self.dialog.items[idx].set_enabled(False)
        self.dialog.items[idx].set_caption(translate(self.CONTEXT, "Stored"))
        self.dialog_timer.start(1000)

    def dialog_timer_tick(self):

        # TODO do it in a portable way
        self.dialog.items[0] = translate(self.CONTEXT, "Right arm")
        self.dialog.items[1] = translate(self.CONTEXT, "Left arm")

        for v in self.dialog.items:
            v.set_enabled(True)


class VisualInspectionRun(VisualInspection):

    def __init__(self, *args, **kwargs):

        super(VisualInspectionRun, self).__init__(*args, **kwargs)

        self.ui.notif(
            translate(
                self.CONTEXT,
                "Visual inspection in progress..."))
