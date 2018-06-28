from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore
import rospy
from art_msgs.srv import NotifyUserRequest
import numpy as np
from art_projected_gui.items import ObjectItem, DialogItem, PolygonItem
from math import sqrt
import tf
import matplotlib.path as mplPath
from art_projected_gui.helpers import conversions


translate = QtCore.QCoreApplication.translate


class DrillPoints(GuiInstruction):

    NAME = translate("DrillPoints", "Glue application")

    def __init__(self, *args, **kwargs):

        super(DrillPoints, self).__init__(*args, **kwargs)

    @staticmethod
    def get_text(ph, block_id, item_id):

        ps, ref_id = ph.get_pose(block_id, item_id)
        ps_learned = 0.0

        for i in range(len(ps)):

            if ph.is_pose_set(block_id, item_id, i):
                ps_learned += 1

        return translate("DrillPoints", " learned poses: %1/%2").arg(ps_learned).arg(len(ps))


class DrillPointsLearn(DrillPoints):

    def __init__(self, *args, **kwargs):

        super(DrillPointsLearn, self).__init__(*args, **kwargs)

        self.drill_dialog = None
        self.drill_pose_idx = 0

        # TODO check if object is to be set here or somewhere else!
        if self.ui.ph.is_object_set(*self.cid):

            self.ui.select_object_type(self.ui.ph.get_object(*self.cid)[0][0])

            if self.editable:
                self.create_drill_dialog()

            if self.ui.ph.is_polygon_set(*self.cid):
                polygons = self.ui.ph.get_polygon(*self.cid)[0]

                self.ui.add_polygon(translate("DrillPoints", "OBJECTS TO BE DRILLED"),
                                    poly_points=conversions.get_pick_polygon_points(polygons),
                                    polygon_changed=self.ui.polygon_changed, fixed=not self.editable)

        else:

            # TODO pokud nema byt nastaveny v teto instrukci - rict kde je potreba ho nastavit
            # TODO pokud tam neni vybrany, ani nedovolit editaci - neni co editovat
            if self.editable:
                self.ui.notif(
                    translate("DrillPoints", "Select object type to be drilled"))

    def cleanup(self):

        self.ui.scene.removeItem(self.drill_dialog)
        self.drill_dialog = None
        return ()

    def object_selected(self, obj, selected, msg):

        # polygon is not from some other instruction (through ref_id)
        # and new object type was selected
        if obj.object_type.name != self.ui.ph.get_object(*self.cid)[0][0]:

            if msg.polygon:

                self.ui.remove_scene_items_by_type(PolygonItem)

                poly_points = []

                self.ui.program_vis.set_object(obj.object_type.name)
                self.ui.select_object_type(obj.object_type.name)

                # TODO avoid code duplication with PICK_FROM_POLYGON
                for ob in self.ui.get_scene_items_by_type(ObjectItem):
                    if ob.object_type.name != obj.object_type.name:
                        continue

                    # TODO refactor somehow (into ObjectItem?)
                    if not ob.on_table or ob.position[0] < 0 or ob.position[0] > self.ui.width or ob.position[1] < 0 \
                            or ob.position[1] > self.ui.height:
                        continue

                    sbr = ob.sceneBoundingRect()

                    w = ob.pix2m(sbr.width())
                    h = ob.pix2m(sbr.height())

                    # TODO limit to scene size?
                    poly_points.append((ob.position[0] + w / 2.0, ob.position[1] + h / 2.0))
                    poly_points.append((ob.position[0] - w / 2.0, ob.position[1] - h / 2.0))
                    poly_points.append((ob.position[0] + w / 2.0, ob.position[1] - h / 2.0))
                    poly_points.append((ob.position[0] - w / 2.0, ob.position[1] + h / 2.0))

                # TODO polygon_changed should be now set for add_polygon?
                self.ui.add_polygon(translate("DrillPoints", "OBJECTS TO BE DRILLED"),
                                    poly_points, polygon_changed=self.ui.polygon_changed)
                self.ui.notif(
                    translate(
                        "DrillPoints",
                        "Check and adjust area with objects to be drilled. Then use robot arm to set drill poses."))

            self.ui.program_vis.clear_poses()

            self.create_drill_dialog()

    def detected_objects(self, msg):

        if self.drill_dialog:

            sel_obj_type = self.ui.ph.get_object(*self.cid)[0][0]

            polygon = self.ui.ph.get_polygon(*self.cid)[0][0]
            pp = []

            for point in polygon.polygon.points:
                pp.append([point.x, point.y])
            pp.append([0, 0])
            pol = mplPath.Path(np.array(pp), closed=True)

            for obj in msg.instances:

                if obj.object_type == sel_obj_type and pol.contains_point(
                        [obj.pose.position.x, obj.pose.position.y]):
                    self.drill_dialog.set_enabled(True)
                    break

            else:
                self.drill_dialog.set_enabled(False)

    def learning_done(self):

        if self.drill_dialog:
            self.ui.scene.removeItem(self.drill_dialog)
            self.drill_dialog = None

    def get_drill_caption(self):

        return translate("DrillPoints", "Save gripper pose (%1/%2)").arg(self.drill_pose_idx +
                                                                         1).arg(self.ui.program_vis.get_poses_count())

    def create_drill_dialog(self):

        if not self.drill_dialog:

            arr = [arm.name(self.ui.loc) for arm in self.ui.rh.get_robot_arms()]
            arr.append(translate("DrillPoints", "Prev pose"))
            arr.append(translate("DrillPoints", "Next pose"))

            self.drill_pose_idx = 0
            self.drill_dialog = DialogItem(
                self.ui.scene, self.ui.width / 2, 0.1, self.get_drill_caption(), arr, self.save_gripper_pose_drill_cb)

    def save_gripper_pose_drill_cb(self, idx):

        # "Right arm", "Left arm", "Prev pose", "Next pose"

        if idx in [2, 3]:

            if idx == 3:
                self.drill_pose_idx += 1
                if self.drill_pose_idx >= self.ui.program_vis.get_poses_count():
                    self.drill_pose_idx = 0
            else:

                self.drill_pose_idx -= 1
                if self.drill_pose_idx < 0:
                    self.drill_pose_idx = self.ui.program_vis.get_poses_count() - 1

            self.ui.drill_dialog.set_caption(self.get_drill_caption())

            return

        ps = self.ui.rh.get_robot_arms()[idx].get_pose()

        if not ps:
            self.ui.notif(
                translate("DrillPoints", "Failed to get gripper pose."), temp=True, message_type=NotifyUserRequest.WARN)
            return

        assert ps.header.frame_id == "marker"

        obj_type = self.ui.ph.get_object(*self.cid)[0][0]
        polygon = self.ui.ph.get_polygon(*self.cid)[0][0]
        pp = []

        for point in polygon.polygon.points:
            pp.append([point.x, point.y])
        pp.append([0, 0])
        pol = mplPath.Path(np.array(pp), closed=True)

        dist = None
        c_obj = None
        for obj in self.ui.get_scene_items_by_type(ObjectItem):

            # skip objects of different type or outside of polygon
            if obj.object_type.name != obj_type or not pol.contains_point([obj.position[0], obj.position[1]]):
                continue

            d = sqrt((obj.position[0] - ps.pose.position.x)**2 +
                     (obj.position[1] - ps.pose.position.y)**2 +
                     (obj.position[2] - ps.pose.position.z)**2)

            if dist is None or d < dist:

                dist = d
                c_obj = obj

        if c_obj:

            self.logdebug("Closest object is: " + c_obj.object_id + " (dist: " + str(dist) + ")")

        else:

            self.logdebug("No object of type " + obj_type + " found.")

        if c_obj and dist < 0.4:

            frame_id = "object_id_" + c_obj.object_id

            try:
                self.ui.tfl.waitForTransform(frame_id, ps.header.frame_id, ps.header.stamp, rospy.Duration(2.0))
                ps = self.ui.tfl.transformPose(frame_id, ps)
            except tf.Exception:

                self.logerr(
                    "Failed to transform gripper pose (" +
                    ps.header.frame_id +
                    ") to object frame_id: " +
                    frame_id)
                return

            self.ui.notif(
                translate(
                    "DrillPoints",
                    "Gripper pose relative to object %1 stored").arg(
                    c_obj.object_id),
                temp=True)
            self.ui.snd_info()
            self.ui.program_vis.update_pose(ps, self.drill_pose_idx)

            self.drill_pose_idx += 1
            if self.drill_pose_idx >= self.ui.program_vis.get_poses_count():
                self.drill_pose_idx = 0

            self.drill_dialog.set_caption(self.get_drill_caption())

        else:

            self.ui.notif(
                translate(
                    "DrillPoints",
                    "Failed to find object near gripper."),
                temp=True,
                message_type=NotifyUserRequest.WARN)
            self.ui.snd_warn()


class DrillPointsRun(DrillPoints):

    def __init__(self, *args, **kwargs):

        super(DrillPointsRun, self).__init__(*args, **kwargs)

        polygons = self.ui.ph.get_polygon(*self.cid)[0]
        poses = self.ui.ph.get_pose(*self.cid)[0]

        try:
            self.ui.select_object(self.flags["SELECTED_OBJECT_ID"])
            self.ui.notif(
                translate(
                    "DrillPoints",
                    "Going to drill hole %1 out of %2 into object %3.").arg(
                    self.flags["DRILLED_HOLE_NUMBER"]).arg(
                    len(poses)).arg(
                    self.flags["SELECTED_OBJECT_ID"]))
        except KeyError as e:
            self.logerr(
                "flag not set: " + str(e))

        self.ui.add_polygon(translate("DrillPoints", "Objects to be drilled"),
                            poly_points=conversions.get_pick_polygon_points(polygons), fixed=True)


class DrillPointsVis(DrillPoints):

    def __init__(self, *args, **kwargs):

        super(DrillPointsVis, self).__init__(*args, **kwargs)

        self.ui.select_object_type(self.ph.get_object(*self.cid)[0][0])

        polygons = self.ui.ph.get_polygon(*self.cid)[0]

        self.ui.add_polygon(translate("DrillPoints", "Objects to be drilled"),
                            poly_points=conversions.get_pick_polygon_points(polygons), fixed=True)
