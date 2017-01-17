#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from point_item import PointItem


class PolygonItem(Item):

    def __init__(self, scene, rpm, caption, obj_coords=[], poly_points=[], polygon_changed=None, fixed=False):

        self.caption = caption
        self.polygon_changed = polygon_changed

        self.poly = QtGui.QPolygon()

        super(PolygonItem, self).__init__(scene, rpm, 0.5, 0.5)  # TODO what should be here?
        self.fixed = fixed

        self.pts = []

        if len(obj_coords) > 0:

            min = [obj_coords[0][0], obj_coords[0][1]]
            max = [obj_coords[0][0], obj_coords[0][1]]

            for pt in obj_coords:

                pt = [pt[0], pt[1]]

                if pt[0] < min[0]:
                    min[0] = pt[0]
                if pt[1] < min[1]:
                    min[1] = pt[1]
                if pt[0] > max[0]:
                    max[0] = pt[0]
                if pt[1] > max[1]:
                    max[1] = pt[1]

            pad = 0.1

            min[0] -= pad
            min[1] -= pad
            max[0] += pad
            max[1] += pad

            self.pts.append(PointItem(scene, rpm, min[0], min[1], self,  self.point_changed))
            self.pts.append(PointItem(scene, rpm, max[0], min[1], self, self.point_changed))
            self.pts.append(PointItem(scene, rpm, max[0], max[1], self, self.point_changed))
            self.pts.append(PointItem(scene, rpm, min[0], max[1], self, self.point_changed))

            if self.polygon_changed is not None:
                self.polygon_changed(self.get_poly_points())

        elif len(poly_points) > 0:

            for pt in poly_points:

                self.pts.append(PointItem(scene, rpm, pt[0], pt[1], self, self.point_changed, fixed))

        else:

            pass  # TODO chyba

        for pt in self.pts:
                self.poly.append(pt.pos().toPoint())

        self.update()

    def point_changed(self, pt, finished):

        self.poly.setPoint(self.pts.index(pt),  pt.pos().toPoint())

        self.update()

        if finished and self.polygon_changed is not None:
            self.polygon_changed(self.get_poly_points())

    def get_poly_points(self):

        pts = []

        for pt in self.pts:

            pts.append(pt.get_pos())

        return pts
    # TODO impl.
    # def shape(self):

    def boundingRect(self):

        return QtCore.QRectF(self.poly.boundingRect())

    def paint(self, painter, option, widget):
        # TODO detekovat ze je polygon "divny" (prekrouceny) a zcervenat
        # TODO vypsat kolik obsahuje objektu

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        # painter.setPen(QtCore.Qt.white)
        # painter.drawRect(self.boundingRect())

        pen = QtGui.QPen()
        pen.setStyle(QtCore.Qt.DotLine)
        pen.setWidth(5)
        pen.setBrush(QtCore.Qt.white)
        pen.setCapStyle(QtCore.Qt.RoundCap)
        pen.setJoinStyle(QtCore.Qt.RoundJoin)

        painter.setPen(pen)

        painter.drawPolygon(self.poly)

        # TODO nazev polygonu -> kam kreslit? Polopruhledne / sede pres cely polygon?
        # painter.setFont(QtGui.QFont('Arial', 12));
        # painter.setPen(QtCore.Qt.white)
        # painter.drawText(QtCore.QPoint(self.pts[0].x(), self.pts[0].y()), self.caption)
