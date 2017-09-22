#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from point_item import PointItem
from desc_item import DescItem
from math import fabs, atan2


class PolygonItem(Item):

    def __init__(self, scene, caption, obj_coords=[], poly_points=[], polygon_changed=None, fixed=False):

        self.caption = caption
        self.polygon_changed = polygon_changed

        self.poly = QtGui.QPolygon()
        self.desc = None

        super(PolygonItem, self).__init__(scene, 0.5, 0.5)  # TODO what should be here?
        self.fixed = fixed
        self.convex = True

        if not self.fixed:
            self.setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)
            self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)

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

            pad = 0.02

            min[0] -= pad
            min[1] -= pad
            max[0] += pad
            max[1] += pad

            self.pts.append(PointItem(scene, min[0], min[1], self, self.point_changed))
            self.pts.append(PointItem(scene, max[0], min[1], self, self.point_changed))
            self.pts.append(PointItem(scene, max[0], max[1], self, self.point_changed))
            self.pts.append(PointItem(scene, min[0], max[1], self, self.point_changed))

            if self.polygon_changed is not None:
                self.polygon_changed(self.get_poly_points())

        elif len(poly_points) > 0:

            for pt in poly_points:

                self.pts.append(PointItem(scene, pt[0], pt[1], self, self.point_changed, fixed))

        else:

            pass  # TODO chyba

        for pt in self.pts:
            self.poly.append(pt.pos().toPoint())

        self.desc = DescItem(scene, 0, 0, self)
        self.desc.setFlag(QtGui.QGraphicsItem.ItemIgnoresTransformations)
        self.__update_desc_pos()
        self.__update_text()

        self.update()

    def __update_desc_pos(self):

        if self.desc is not None:

            p = self.poly.boundingRect().bottomLeft() + QtCore.QPointF(0, self.m2pix(0.02))
            self.desc.setPos(p)

    def __update_text(self):

        if self.desc is None:
            return

        desc = self.caption
        # TODO number of objects in polygon?
        self.desc.set_content(desc)

    def point_changed(self, pt, finished):

        self.poly.setPoint(self.pts.index(pt), pt.pos().toPoint())

        ps = self.poly.size()

        # TODO what to do with shuffled points? should be fixed...
        for i in range(2, ps + 2):

            line1 = QtCore.QLineF(self.poly.point(i - 2), self.poly.point((i - 1) % ps))
            line2 = QtCore.QLineF(self.poly.point((i - 1) % ps), self.poly.point(i % ps))

            a1 = line1.angle()
            a2 = line2.angle()

            d = a2 - a1

            if d < 0:
                d = 360 + d

            if d < 315 and d > 180:

                self.convex = False
                break
        else:
            self.convex = True

        self.__update_desc_pos()
        self.update()

        if finished and self.convex and self.polygon_changed is not None:
            self.polygon_changed(self.get_poly_points())

    def mouseReleaseEvent(self, event):

        self.cursor_release()
        super(Item, self).mouseReleaseEvent(event)

    def cursor_release(self):

        # TODO call base class method

        if self.convex and self.polygon_changed is not None:
            self.polygon_changed(self.get_poly_points())

    def get_poly_points(self):

        pts = []

        try:
            for pt in self.pts:

                pts.append(pt.get_pos())
        except AttributeError:
            return None

        return pts

    def shape(self):

        path = QtGui.QPainterPath()
        path.addPolygon(QtGui.QPolygonF(self.poly))
        return path

    def boundingRect(self):

        return QtCore.QRectF(self.poly.boundingRect())

    def paint(self, painter, option, widget):
        # TODO detekovat ze je polygon "divny" (prekrouceny) a zcervenat
        # TODO vypsat kolik obsahuje objektu

        if not self.scene():
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        # painter.setPen(QtCore.Qt.white)
        # painter.drawRect(self.boundingRect())

        pen = QtGui.QPen()
        pen.setStyle(QtCore.Qt.DotLine)
        pen.setWidth(5)

        if self.convex:
            pen.setBrush(QtCore.Qt.white)
        else:
            pen.setBrush(QtCore.Qt.red)

        pen.setCapStyle(QtCore.Qt.RoundCap)
        pen.setJoinStyle(QtCore.Qt.RoundJoin)
        painter.setPen(pen)

        painter.drawPolygon(self.poly)
