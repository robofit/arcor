import rospy
from PyQt4 import QtCore
from std_msgs.msg import Empty

from art_projected_gui.plugins import GuiPlugin
from art_projected_gui.items import TouchPointsItem, TouchTableItem, LabelItem
from art_msgs.srv import TouchCalibrationPoints, TouchCalibrationPointsResponse


translate = QtCore.QCoreApplication.translate

# TODO most or all of the functionality of TouchTableItem should be refactored here


class TouchTablePlugin(GuiPlugin):

    def __init__(self, *args, **kwargs):

        super(TouchTablePlugin, self).__init__(*args, **kwargs)

        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'touch_calibration_points_evt'), self.touch_calibration_points_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'touch_detected_evt'), self.touch_detected_evt)

        self.touch_table = TouchTableItem(
            self.ui.scene,
            '/art/interface/touchtable/touch',
            [],
            show_touch_points=False)  # TODO get it from some param

        self.touched_sub = None
        self.touch_points = None
        self.touch_calib_srv = rospy.Service('/art/interface/projected_gui/touch_calibration', TouchCalibrationPoints,
                                             self.touch_calibration_points_cb)

    def touch_calibration_points_evt(self, pts):

        for it in self.ui.scene.items():

            if isinstance(it, LabelItem):
                continue

            it.setVisible(False)  # TODO come up with more intelligent way how to hide scene content (overlay?)

        self.ui.notif(translate(
            "TouchTablePlugin", "Touch table calibration started. Please press the white point."))
        self.touch_points = TouchPointsItem(self.ui.scene, pts)

    def touch_calibration_points_cb(self, req):

        resp = TouchCalibrationPointsResponse()

        pts = []

        for pt in req.points:

            pts.append((pt.point.x, pt.point.y))

        self.emit(QtCore.SIGNAL('touch_calibration_points_evt'), pts)
        self.touched_sub = rospy.Subscriber(
            '/art/interface/touchtable/touch_detected', Empty, self.touch_detected_cb, queue_size=10)
        resp.success = True
        return resp

    def touch_detected_evt(self, msg):

        if self.touch_points is None:
            return

        if not self.touch_points.next():

            for it in self.ui.scene.items():

                if isinstance(it, LabelItem):
                    continue

                # TODO fix this - in makes visible even items that are invisible by purpose
                it.setVisible(True)

            self.ui.notif(
                translate("TouchTablePlugin", "Touch table calibration finished."), temp=False)
            self.ui.scene.removeItem(self.touch_points)
            self.touch_points = None
            self.touched_sub.unregister()

    def touch_detected_cb(self, msg):

        self.emit(QtCore.SIGNAL('touch_detected_evt'), msg)