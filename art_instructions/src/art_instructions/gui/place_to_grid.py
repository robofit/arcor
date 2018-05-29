from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore
from art_projected_gui.helpers import conversions

translate = QtCore.QCoreApplication.translate


class PlaceToGrid(GuiInstruction):

    CONTEXT = "PlaceToGrid"

    def __init__(self, *args, **kwargs):

        super(PlaceToGrid, self).__init__(*args, **kwargs)


class PlaceToGridLearn(PlaceToGrid):

    def __init__(self, *args, **kwargs):

        super(PlaceToGridLearn, self).__init__(*args, **kwargs)

        object_type_name = self.uiph.get_object(*self.cid)[0][0]
        poses = self.ui.ph.get_pose(*self.cid)[0]
        polygons = self.ph.get_polygon(*self.cid)[0]

        object_type = self.ui.art.get_object_type(object_type_name)

        self.ui.notif(translate(self.CONTEXT, "Place grid"))
        self.ui.add_square(
            translate(
                self.CONTEXT,
                "PLACE SQUARE GRID"),
            self.ui.width / 2,
            self.ui.height / 2,
            0.1,
            0.075,
            object_type,
            poses,
            grid_points=conversions.get_pick_polygon_points(polygons),
            square_changed=self.ui.square_changed,
            fixed=not self.editable)

    def object_selected(self, obj, selected, msg):

        return


class PlaceToGridRun(PlaceToGrid):

    def __init__(self, ui, flags):

        super(PlaceToGridRun, self).__init__(ui)

        polygons = self.ui.ph.get_polygon(*self.cid)[0]
        poses = self.ui.ph.get_pose(*self.cid)[0]
        object_type_name = self.ui.ph.get_object(*self.cid)[0][0]

        object_type = self.ui.art.get_object_type(object_type_name)

        self.ui.notif(translate(self.CONTEXT, "Going to place objects into grid"))
        self.ui.add_square(translate(self.CONTEXT, "PLACE SQUARE GRID"),
                           self.ui.width / 2, self.ui.height / 2, 0.1, 0.075,
                           object_type, poses, grid_points=conversions.get_pick_polygon_points(polygons),
                           square_changed=self.ui.square_changed, fixed=True)
