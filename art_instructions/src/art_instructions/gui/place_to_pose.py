from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore

translate = QtCore.QCoreApplication.translate


class PlaceToPose(GuiInstruction):

    NAME = translate("PlaceToPose", "Place to pose")

    def __init__(self, *args, **kwargs):

        super(PlaceToPose, self).__init__(*args, **kwargs)

    def get_name(self, block_id, item_id):

        name = self.ui.ph.get_name(block_id, item_id)

        if name:
            return name

        return translate("PlaceToPose", "PLACE POSE")


class PlaceToPoseLearn(PlaceToPose):

    def __init__(self, *args, **kwargs):

        super(PlaceToPoseLearn, self).__init__(*args, **kwargs)

        if not self.ui.ph.is_object_set(*self.cid):

            (obj_arr, ref_id) = self.ui.ph.get_object(*self.cid)

            self.ui.notif(translate(
                "PlaceToPose", "Select object to be picked up in instruction %1").arg(ref_id))
            self.notified = True

        else:

            for it_id in self.ui.ph.get_items_ids(self.block_id):

                if self.ui.ph.get_item_msg(self.block_id, it_id).type != "PlaceToPose":
                    continue

                object_type = None
                object_id = None

                if self.ui.ph.is_object_set(self.block_id, it_id):
                    object_type = self.ui.art.get_object_type(self.ui.ph.get_object(self.block_id, it_id)[0][0])

                if it_id == self.instruction_id:

                    if self.editable:
                        self.ui.notif(
                            translate(
                                "PlaceToPose",
                                "Drag object outline to set place pose. Use blue point to set orientation."))

                    if self.ui.ph.is_pose_set(self.block_id, it_id):

                        if object_type is not None:
                            self.ui.select_object_type(object_type.name)
                            self.ui.add_place(
                                self.get_name(self.block_id, it_id),
                                self.ui.ph.get_pose(self.block_id, it_id)[0][0],
                                object_type,
                                object_id,
                                place_cb=self.ui.place_pose_changed,  # TODO place_cb should be set in add_place?
                                fixed=not self.editable)
                    else:

                        self.ui.add_place(self.get_name(self.block_id, it_id), self.ui.get_def_pose(
                        ), object_type, object_id, place_cb=self.ui.place_pose_changed, fixed=not self.editable)

                    continue

                if self.ui.ph.is_pose_set(self.block_id, it_id):
                    self.ui.add_place(
                        unicode(
                            self.get_name(self.block_id, it_id)) + " (" + str(it_id) + ")",
                        self.ui.ph.get_pose(self.block_id, it_id)[0][0],
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
            self.logerr(
                "PLACE_TO_POSE: SELECTED_OBJECT_ID flag not set")
            return

        obj = self.ui.get_object(obj_id)

        if obj is not None:

            place_pose = self.ui.ph.get_pose(*self.cid)[0][0]
            self.ui.add_place(self.get_name(*self.cid), place_pose, obj.object_type, obj_id, fixed=True)
            self.ui.notif(translate("PlaceToPose", "Placing object to pose."))

        else:

            self.logerr("Selected object_id not found: " + obj_id)


class PlaceToPoseVis(PlaceToPose):

    def __init__(self, *args, **kwargs):

        super(PlaceToPoseVis, self).__init__(*args, **kwargs)

        object_type = None
        object_id = None

        self.ui.select_object_type(self.ui.ph.get_object(*self.cid)[0][0])

        if self.ui.ph.is_object_set(*self.cid):
            object_type = self.ui.art.get_object_type(self.ui.ph.get_object(*self.cid)[0][0])

        if object_type is not None:
            place_pose = self.ui.ph.get_pose(*self.cid)[0][0]

            self.ui.add_place(translate("PlaceToPose", "OBJECT PLACE POSE"),
                              place_pose, object_type, object_id, fixed=True)
