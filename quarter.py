from matplotlib.axes import Axes
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import geopandas as gpd
import re
import math
import time
import json
import logging
from decimal import Decimal

from qgis.core import *

LOCAL_DIR = "/home/vu/.local/share/QGIS/QGIS3/profiles/default/python/plugins/quadtree-qgis-plugin"

class BoundingBox:
    PARTIAL_OVERLAP = 0
    INSIDE = 1
    OUTSIDE = 2

    def __init__(self, center: QgsPointXY, width: Decimal, height: Decimal):
        self.center: QgsPointXY = center
        self.width =  width
        self.height = height
        # logging.info(f"Current coordinate: {self.center.x()},{self.center.y()}")
        self.west = Decimal(str("{:.17f}".format(self.center.x()))) - width / 2
        self.east = Decimal(str("{:.17f}".format(self.center.x()))) + width / 2
        self.north = Decimal(str("{:.17f}".format(self.center.y()))) + height / 2
        self.south = Decimal(str("{:.17f}".format(self.center.y()))) - height / 2
        self.BoundBox = QgsGeometry.fromPolygonXY([[QgsPointXY(self.west,self.south),QgsPointXY(self.west,self.north),QgsPointXY(self.east,self.north),QgsPointXY(self.east,self.south)]])

    def positionVsPolygon(self, polygon: QgsGeometry) -> int:
        """
        Returns the relative position of the bounding box with respect to polygon
        """
        # bounding box intersects partially with the polygon
        if polygon == None or self.BoundBox.within(polygon):
            return self.INSIDE,None
        else:
            intersect = self.BoundBox.intersection(polygon)
            if intersect.isEmpty():
                return self.OUTSIDE,None
            else:
                return self.PARTIAL_OVERLAP,intersect


class BoxData:
    def __init__(self) -> None: # init dictionary to write file
        self.id = 0
        self.boxes = {"type":"FeatureCollection","name":"quadtree","crs":{"type":"name","properties":{"name":"urn:ogc:def:crs:EPSG::3857"}},"features":[]}

    # append square into dictionary
    def insertBox(self,west,north,east,south):
        insertNode = {"type":"Feature","properties": {"id": self.id,"left": west,"top": north,"right": east,"bottom": south},"geometry": {"type": "Polygon","coordinates":[[[west,north],[east,north], [east, south], [west,south], [west, north]]]}}
        self.id += 1
        self.boxes["features"].append(insertNode)

    def exportJSON(self,save_path = 'Q:\\test\\testcase\\grids\\quadtree.geojson'):
        writeDest = open(save_path, 'w')
        json.dump(self.boxes,writeDest,separators=(',', ':'))
        writeDest.close()

class QTNode:
    def __init__(self,polygon:QgsGeometry, div_unit:float, boundBox:BoundingBox=None) -> None:
        self.boundBox: BoundingBox = boundBox
        self.children: list["QTNode"] = None
        self.polygon = polygon
        self.min_size = div_unit


    def initPolygon(self):
        extent = self.polygon.boundingBox()
        xmin = Decimal(str("{:.17f}".format(extent.xMinimum())))
        ymin = Decimal(str("{:.17f}".format(extent.yMinimum())))
        xmax = Decimal(str("{:.17f}".format(extent.xMaximum())))
        ymax = Decimal(str("{:.17f}".format(extent.yMaximum())))
        treeLv = math.ceil(math.log2(max(ymax-ymin, xmax-xmin) / self.min_size))
        # size = Decimal(str(self.DIVISION_UNIT * math.pow(2,treeLv))) if treeLv >= 0 else Decimal(str(self.DIVISION_UNIT))
        size = Decimal(str(self.min_size * math.pow(2,treeLv)))
        self.boundBox = BoundingBox(
                    QgsPointXY(
                        xmin + size/2,
                        ymin + size/2
                    ),
                    size,
                    size,
                )
        self.insertPolygon()
        return size

    def insertPolygon(self) -> None:
        """
        Insert new polygon into the root node of self
        """
        position,intersect = self.boundBox.positionVsPolygon(self.polygon)
        if position != self.boundBox.OUTSIDE:
            #if position == self.boundBox.INSIDE or self.boundBox.width == Quarter.DIVISION_UNIT:
            #    Quarter.boxes.insertBox(float(self.boundBox.west), float(self.boundBox.north), float(self.boundBox.east), float(self.boundBox.south))
            if self.boundBox.width <= self.min_size:
                Quarter.boxes.insertBox(float(self.boundBox.west),float(self.boundBox.north),float(self.boundBox.east),float(self.boundBox.south))
            else:
                # overdrawn
                self.__divide(intersect)


    def __divide(self, newPolygon:QgsGeometry) -> None:
        """ """
        newWidth = self.boundBox.width / 2
        newHeight = self.boundBox.height / 2
        self.children = []

        for i in range(4):
            child = QTNode(
                polygon=newPolygon,
                div_unit=self.min_size,
                boundBox=BoundingBox(
                    QgsPointXY(
                        Decimal(str("{:.17f}".format(self.boundBox.center.x()))) + pow(-1, i & 1) * newWidth/2,
                        Decimal(str("{:.17f}".format(self.boundBox.center.y()))) + pow(-1, i >> 1) * newHeight/2,
                    ),
                    newWidth,
                    newHeight,
                )
            )
            child.insertPolygon()
            self.children.append(child)

class Quarter:
    boxes = BoxData()

    def __init__(self, DIV_UNIT=1000) -> None:
        # Init values
        self.quadtree:QTNode = None
        self.div_unit:float = DIV_UNIT

    def setup(self, inplay='Q:\\test\\testcase\\AnGiang.geojson'):
        layer1 = QgsVectorLayer(inplay, 'Layer 1', 'ogr')
        size = 0
        for feat in layer1.getFeatures():
            geometry = feat.geometry()
            self.quadtree = QTNode(polygon=geometry, div_unit=self.div_unit)
            size = self.quadtree.initPolygon()
        return size
