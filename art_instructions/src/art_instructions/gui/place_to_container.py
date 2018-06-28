from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore
from art_projected_gui.helpers import conversions
from art_projected_gui.items import PolygonItem

translate = QtCore.QCoreApplication.translate


class PlaceToContainer(GuiInstruction):

    NAME = translate("PlaceToContainer", "Place to container")

    def __init__(self, *args, **kwargs):

        super(PlaceToContainer, self).__init__(*args, **kwargs)


class PlaceToContainerLearn(PlaceToContainer):
    """
    1) user sets pick instruction
    2) user selects container object
    3) ui selects type and add polygon around the selected object
    """

    def __init__(self, *args, **kwargs):

        super(PlaceToContainerLearn, self).__init__(*args, **kwargs)

        # TODO check if pick is set?

        if not self.ui.ph.is_object_set(*self.cid):

            if self.editable:
                self.ui.notif(
                    translate("PlaceToContainer", "Select container by tapping on its outline."))

            return

        object_type_name = self.ui.ph.get_object(*self.cid)[0][0]
        self.ui.select_object_type(object_type_name)

        if self.ui.ph.is_polygon_set(*self.cid):

            polygons = self.ui.ph.get_polygon(*self.cid)[0]

            self.ui.add_polygon(
                translate(
                    "PlaceToContainer",
                    "Containers"),
                poly_points=conversions.get_pick_polygon_points(polygons),
                polygon_changed=self.ui.polygon_changed,
                fixed=not self.editable)

            if self.editable:
                self.ui.notif(
                    translate("PlaceToContainer", "Adjust area with containers or select another object type."))

    def object_selected(self, obj, selected, msg):

        if not obj.object_type.container:
            self.ui.notif(
                translate("PlaceToContainer", "Please select container."), temp=True)
            return

        if obj.object_type.name not in self.ui.selected_object_types:

            self.ui.remove_scene_items_by_type(PolygonItem)

            poly_points = []

            self.ui.program_vis.set_object(obj.object_type.name)
            self.ui.select_object_type(obj.object_type.name)

            ob = self.ui.get_object(obj.object_id)

            sbr = ob.sceneBoundingRect()

            r = 2.0
            w = ob.pix2m(sbr.width()) * r
            h = ob.pix2m(sbr.height()) * r

            poly_points.append((ob.position[0] + w / 2.0, ob.position[1] + h / 2.0))
            poly_points.append((ob.position[0] - w / 2.0, ob.position[1] - h / 2.0))
            poly_points.append((ob.position[0] + w / 2.0, ob.position[1] - h / 2.0))
            poly_points.append((ob.position[0] - w / 2.0, ob.position[1] + h / 2.0))

            self.ui.add_polygon(translate("PlaceToContainer", "Containers"),
                                poly_points, polygon_changed=self.ui.polygon_changed)
            self.ui.notif(
                translate("PlaceToContainer", "Check and adjust container selection. You may also change object type."))


class PlaceToContainerRun(PlaceToContainer):

    def __init__(self, *args, **kwargs):

        super(PlaceToContainerRun, self).__init__(*args, **kwargs)

        obj_id = None
        try:
            obj_id = self.flags["SELECTED_OBJECT_ID"]
        except KeyError:
            self.logerr("SELECTED_OBJECT_ID flag not set")

        cont_id = None
        try:
            cont_id = self.flags["SELECTED_CONTAINER_ID"]
        except KeyError:
            self.logerr("SELECTED_CONTAINER_ID flag not set")

        if cont_id:
            self.ui.select_object(cont_id)

            if obj_id:
                self.ui.notif(translate("PlaceToContainer", "Placing %1 to %2...").arg(obj_id).arg(cont_id))

        self.ui.add_polygon(
            translate(
                "PlaceToContainer",
                "Containers"),
            poly_points=conversions.get_pick_polygon_points(
                self.ui.ph.get_polygon(*self.cid)[0]),
            fixed=True)
