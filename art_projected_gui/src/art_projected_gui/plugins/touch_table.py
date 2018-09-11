import rospy
from PyQt4 import QtCore
from std_msgs.msg import Empty

from art_projected_gui.plugins import GuiPlugin
from art_projected_gui.items import TouchPointsItem, TouchTableItem, LabelItem
from art_msgs.srv import TouchCalibrationPoints, TouchCalibrationPointsResponse


translate = QtCore.QCoreApplication.translate

# TODO most or all of the functionality of TouchTableItem should be refactored here


class TouchTablePlugin(GuiPlugin):

    def __init__(self, ui, parameters):

        super(TouchTablePlugin, self).__init__(ui)

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
        self.items_state = {}

    def touch_calibration_points_evt(self, pts):

        self.items_state = {}

        for it in self.ui.scene.items():

            if isinstance(it, LabelItem):  # ...to keep notification area displayed
                continue

            self.items_state[it] = it.isVisible()
            it.setVisible(False)

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

            self.ui.scene.removeItem(self.touch_points)
            self.touch_points = None
            self.touched_sub.unregister()

            for it in self.ui.scene.items():

                if isinstance(it, LabelItem):
                    continue

                it.setVisible(self.items_state[it])

            self.items_state = {}

            self.ui.notif(
                translate("TouchTablePlugin", "Touch table calibration finished."), temp=False)

    def touch_detected_cb(self, msg):

        self.emit(QtCore.SIGNAL('touch_detected_evt'), msg)
