#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item

class PolygonPointItem(Item):

    def __init__(self,  scene,  rpm, x,  y,  parent,  fixed = False):

        self.outline_diameter = 0.025

        super(PolygonPointItem,  self).__init__(scene,  rpm,  x,  y,  parent)
        self.fixed = fixed

    def boundingRect(self):

        es = self.m2pix(self.outline_diameter*1.8)

        return QtCore.QRectF(-es/2, -es/2, es, es)

    def shape(self):

        path = QtGui.QPainterPath()
        es = self.m2pix(self.outline_diameter)
        path.addEllipse(QtCore.QPoint(0,  0),  es/2,  es/2)
        return path

    def item_moved(self):

        self.parentItem().point_changed()

    def cursor_release(self):

        self.parentItem().point_changed(True)

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        es = self.m2pix(self.outline_diameter)

        if self.hover:
            # TODO coordinates
            painter.setBrush(QtCore.Qt.gray)
            painter.setPen(QtCore.Qt.gray)
            painter.drawEllipse(QtCore.QPoint(0,  0), es/2*1.8, es/2*1.8)

        painter.setBrush(QtCore.Qt.cyan)
        painter.setPen(QtCore.Qt.cyan)

        painter.drawEllipse(QtCore.QPoint(0,  0), es/2, es/2)

class PolygonItem(Item):

    def __init__(self,  scene,  rpm, caption, obj_coords = [],  poly_points=[],  polygon_changed=None,  fixed=False):

        self.caption = caption
        self.polygon_changed = polygon_changed

        super(PolygonItem,  self).__init__(scene,  rpm,  0,  0)
        self.fixed = fixed

        if len(obj_coords) > 0:

            self.min = [obj_coords[0][0], obj_coords[0][1]]
            self.max = [obj_coords[0][0], obj_coords[0][1]]

            for pt in obj_coords:

                pt = [pt[0],  pt[1]]

                if pt[0] < self.min[0]: self.min[0] = pt[0]
                if pt[1] < self.min[1]: self.min[1] = pt[1]
                if pt[0] > self.max[0]: self.max[0] = pt[0]
                if pt[1] > self.max[1]: self.max[1] = pt[1]

            pad = 0.1

            self.min[0] -= pad
            self.min[1] -= pad
            self.max[0] += pad
            self.max[1] += pad

            self.pts = []

            self.pts.append(PolygonPointItem(scene,  rpm,  self.min[0],  self.min[1],  self))
            self.pts.append(PolygonPointItem(scene,  rpm,  self.max[0], self.min[1],  self))
            self.pts.append(PolygonPointItem(scene,  rpm,  self.max[0],  self.max[1],  self))
            self.pts.append(PolygonPointItem(scene,  rpm,  self.min[0], self.max[1],  self))

            if self.polygon_changed is not None:
                self.polygon_changed(self.get_poly_points())

        elif len(poly_points) > 0:

            self.min = [poly_points[0][0], poly_points[0][1]]
            self.max = [poly_points[0][0], poly_points[0][1]]

            for pt in poly_points:

                pt = [pt[0],  pt[1]]

                if pt[0] < self.min[0]: self.min[0] = pt[0]
                if pt[1] < self.min[1]: self.min[1] = pt[1]
                if pt[0] > self.max[0]: self.max[0] = pt[0]
                if pt[1] > self.max[1]: self.max[1] = pt[1]

            self.pts = []

            for pt in poly_points:

                self.pts.append(PolygonPointItem(scene,  rpm,  pt[0],  pt[1],  self,  fixed))

        else:

            pass # TODO chyba

        self.update()

    def point_changed(self,  finished = False):

        self.prepareGeometryChange()

        # update of bounding rect
        self.min[0] = self.pix2m(self.pts[0].x())
        self.min[1] = self.pix2m(self.pts[0].y())
        self.max[0] = self.pix2m(self.pts[0].x())
        self.max[1] = self.pix2m(self.pts[0].y())

        for pt in self.pts:

            p = (self.pix2m(pt.x()),  self.pix2m(pt.y()))

            if p[0] < self.min[0]: self.min[0] = p[0]
            if p[1] < self.min[1]: self.min[1] = p[1]

            if p[0] > self.max[0]: self.max[0] = p[0]
            if p[1] > self.max[1]: self.max[1] = p[1]

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

        return QtCore.QRectF(self.m2pix(self.min[0])-2.5,  self.m2pix(self.min[1])-2.5, self.m2pix(self.max[0]-self.min[0])+5, self.m2pix(self.max[1]-self.min[1])+5)

    def paint(self, painter, option, widget):

        # TODO detekovat ze je polygon "divny" (prekrouceny) a zcervenat
        # TODO vypsat kolik obsahuje objektu

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        pen = QtGui.QPen()
        pen.setStyle(QtCore.Qt.DotLine)
        pen.setWidth(5)
        pen.setBrush(QtCore.Qt.white)
        pen.setCapStyle(QtCore.Qt.RoundCap)
        pen.setJoinStyle(QtCore.Qt.RoundJoin)

        painter.setPen(pen)

        poly = QtGui.QPolygon()

        for i in range(0,  len(self.pts)):

            poly.append(self.pts[i].pos().toPoint())

        painter.drawPolygon(poly)

        # TODO nazev polygonu -> kam kreslit? Polopruhledne / sede pres cely polygon?
        #painter.setFont(QtGui.QFont('Arial', 12));
        #painter.setPen(QtCore.Qt.white)
        #painter.drawText(QtCore.QPoint(self.pts[0].x(), self.pts[0].y()), self.caption)
