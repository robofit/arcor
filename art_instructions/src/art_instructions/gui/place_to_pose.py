from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore
import rospy
from art_msgs.msg import ProgramItem as ProgIt

translate = QtCore.QCoreApplication.translate


class PlaceToPose(GuiInstruction):

    CONTEXT = "PlaceToPose"

    def __init__(self, *args, **kwargs):

        super(PlaceToPose, self).__init__(*args, **kwargs)


class PlaceToPoseLearn(PlaceToPose):

    def __init__(self, *args, **kwargs):

        super(PlaceToPoseLearn, self).__init__(*args, **kwargs)

        if not self.ui.ph.is_object_set(*self.cid):

            (obj_arr, ref_id) = self.ui.ph.get_object(*self.cid)

            self.ui.notif(translate(
                self.CONTEXT, "Select object to be picked up in instruction %1").arg(ref_id))
            notified = True

        else:

            for it_id in self.ph.get_items_ids(self.block_id):

                if self.ui.ph.get_item_msg(*self.cid).type != ProgIt.PLACE_TO_POSE:  # TODO get rid of PLACE_TO_POSE
                    continue

                object_type = None
                object_id = None

                if self.ph.is_object_set(*self.cid):
                    object_type = self.art.get_object_type(self.ph.get_object(*self.cid)[0][0])

                if it_id == self.item_id:

                    if self.editable:
                        self.ui.notif(
                            translate(
                                self.CONTEXT,
                                "Drag object outline to set place pose. Use blue point to set orientation."))

                    if self.ui.ph.is_pose_set(*self.cid):

                        if object_type is not None:
                            self.ui.select_object_type(object_type.name)
                            self.ui.add_place(
                                translate(
                                    self.CONTEXT,
                                    "PLACE POSE"),
                                self.ui.ph.get_pose(*self.cid)[0][0],
                                object_type,
                                object_id,
                                place_cb=self.ui.place_pose_changed,  # TODO place_cb should be set in add_place?
                                fixed=not self.editable)
                    else:

                        self.ui.add_place(translate(self.CONTEXT, "PLACE POSE"), self.ui.get_def_pose(
                        ), object_type, object_id, place_cb=self.ui.place_pose_changed, fixed=not self.editable)

                    continue

                if self.ph.is_pose_set(*self.cid):
                    self.ui.add_place(
                        unicode(
                            translate(
                                self.CONTEXT,
                                "PLACE POSE")) + " (" + str(it_id) + ")",
                        self.ui.ph.get_pose(
                            *self.cid)[0][0],
                        object_type,
                        object_id,
                        fixed=True,
                        dashed=True)

    def object_selected(self, obj, selected, msg):

        return


class PlaceToPoseRun(PlaceToPose):

    def __init__(self, *args, **kwargs):

        super(PlaceToPoseRun, self).__init__(*args, **kwargs)

        try:
            obj_id = self.flags["SELECTED_OBJECT_ID"]
        except KeyError:
            rospy.logerr(
                "PLACE_TO_POSE: SELECTED_OBJECT_ID flag not set")
            return

        obj = self.ui.get_object(obj_id)

        if obj is not None:

            place_pose = self.ui.ph.get_pose(*self.cid)[0][0]

            self.ui.add_place(translate(self.CONTEXT, "OBJECT PLACE POSE"),
                              place_pose, obj.object_type, obj_id, fixed=True)

            self.ui.notif(translate(self.CONTEXT, "Placing object to pose."))

        else:

            rospy.logerr("Selected object_id not found: " + obj_id)
