from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore
import rospy
from art_projected_gui.items import PolygonItem, ObjectItem
from art_projected_gui.helpers import conversions

translate = QtCore.QCoreApplication.translate


class PickFromPolygon(GuiInstruction):

    CONTEXT = "PickFromPolygon"

    def __init__(self, ui):

        super(PickFromPolygon, self).__init(ui)

    def cleanup(self):

        super(PickFromPolygon, self).cleanup()


class PickFromPolygonLearn(PickFromPolygon):

    def __init__(self, ui, editable=False):

        super(PickFromPolygonLearn, self).__init(ui)

        self.editable = editable

        if not self.ui.ph.is_object_set(*self.cid):

            if editable:
                self.notif(
                    translate(self.CONTEXT, "Select object type to be picked up by tapping on its outline."))

        else:

            object_type_name = self.ui.ph.get_object(*self.cid)[0][0]
            self.ui.select_object_type(object_type_name)

        if self.ui.ph.is_polygon_set(*self.cid):

            polygons = self.ui.ph.get_polygon(*self.cid)[0]

            self.ui.add_polygon(
                translate(
                    self.CONTEXT,
                    "PICK AREA"),
                poly_points=conversions.get_pick_polygon_points(polygons),
                polygon_changed=self.polygon_changed,
                fixed=not editable)

            if editable:
                self.notif(
                    translate(self.CONTEXT, "Adjust pick area or select another object type."))

    def object_selected(self, obj, selected, msg):

        if obj.object_type.name not in self.ui.selected_object_types:

            self.ui.remove_scene_items_by_type(PolygonItem)

            poly_points = []

            self.ui.program_vis.set_object(obj.object_type.name)
            self.ui.select_object_type(obj.object_type.name)

            for ob in self.ui.get_scene_items_by_type(ObjectItem):
                if ob.object_type.name != obj.object_type.name:
                    continue

                # TODO refactor somehow (into ObjectItem?)
                if not ob.on_table or ob.position[0] < 0 or ob.position[0] > self.width or ob.position[1] < 0 \
                        or ob.position[1] > self.height:
                    continue

                sbr = ob.sceneBoundingRect()

                w = ob.pix2m(sbr.width())
                h = ob.pix2m(sbr.height())

                # TODO limit to scene size?
                poly_points.append((ob.position[0] + w / 2.0, ob.position[1] + h / 2.0))
                poly_points.append((ob.position[0] - w / 2.0, ob.position[1] - h / 2.0))
                poly_points.append((ob.position[0] + w / 2.0, ob.position[1] - h / 2.0))
                poly_points.append((ob.position[0] - w / 2.0, ob.position[1] + h / 2.0))

            self.ui.add_polygon(translate(self.CONTEXT, "PICK POLYGON"),
                                poly_points, polygon_changed=self.ui.polygon_changed)
            self.ui.notif(
                translate(self.CONTEXT, "Check and adjust pick polygon. You may also change object type."))


class PickFromPolygonRun(PickFromPolygon):

    def __init__(self, ui, flags=None):

        super(PickFromPolygonRun, self).__init(ui)

        obj_id = None
        try:
            obj_id = flags["SELECTED_OBJECT_ID"]
        except KeyError:
            rospy.logerr(
                "PICK_FROM_POLYGON: SELECTED_OBJECT_ID flag not set")

        if obj_id is not None:
            self.ui.select_object(obj_id)

            obj = self.ui.get_object(obj_id)  # TODO notif - object type
            if obj is not None:
                self.ui.notif(
                    translate(
                        self.CONTEXT,
                        "Going to pick object ID ") +
                    obj_id +
                    translate(
                        self.CONTEXT,
                        " of type ") +
                    obj.object_type.name +
                    translate(
                        self.CONTEXT,
                        " from polygon."))

        self.ui.add_polygon(
            translate(
                self.CONTEXT,
                "PICK POLYGON"),
            poly_points=conversions.get_pick_polygon_points(
                self.ui.ph.get_polygon(
                    self.block_id,
                    self.instruction_id)[0]),
            fixed=True)
