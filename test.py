# -*- coding: utf-8 -*-
from matplotlib.axes import Axes
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import geopandas as gpd
import re
import math
import time
import json
import multiprocessing




class Point:
    def __init__(self, lon: float, lat: float) -> None:
        self.lon: float = lon
        self.lat: float = lat


class Polygon:
    COLLINEAR = 0
    CLOCKWISE = 1
    COUNTER_CLOCKWISE = 2

    def __init__(self,inp,inputType = "file") -> None:
        """
        A polygon is instantiated using:
        points: list of points
        """
        self.points: list[Point] = []
        self.maxLon = float("-inf")
        self.maxLat = float("-inf")
        self.minLon = float("inf")
        self.minLat = float("inf")

        if inputType == "file":
            region = gpd.read_file(inp)
            reg_shape = region['geometry'][0]
            for pol in reg_shape.geoms:
                for poi in pol.exterior.coords:
                    p = re.findall (r'([^(,)]+)(?!.*\()', str(poi))
                    self.points.append(Point(float(p[0]),float(p[1])))

        else:
            self.points = inp

        for point in self.points:
            if self.maxLon < point.lon:
                self.maxLon = point.lon
            if self.maxLat < point.lat:
                self.maxLat = point.lat
            if self.minLon > point.lon:
                self.minLon = point.lon
            if self.minLat > point.lat:
                self.minLat = point.lat   

    def __onSegment(self, p: Point, q: Point, r: Point) -> bool:
        if (
            (q.lon <= max(p.lon, r.lon))
            and (q.lon >= min(p.lon, r.lon))
            and (q.lat <= max(p.lat, r.lat))
            and (q.lat >= min(p.lat, r.lat))
        ):
            return True
        return False

    def __orientation(self, p: Point, q: Point, r: Point) -> int:
        """
        Find the orientation of an ordered triplet (p,q,r)
        function returns the following values:
        0 : Collinear points
        1 : Clockwise points
        2 : Counterclockwise

        See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/
        for details of below formula.
        """
        val = (float(q.lat - p.lat) * (r.lon - q.lon)) - (
            float(q.lon - p.lon) * (r.lat - q.lat)
        )
        if val > 0:
            return self.CLOCKWISE
        elif val < 0:
            return self.COUNTER_CLOCKWISE
        else:
            return self.COLLINEAR


    def __doIntersect(self, p1: Point, q1: Point, p2: Point, q2: Point) -> bool:
        """
        The function that returns true if
        the line segment 'p1q1' and 'p2q2' intersect.
        """
        # Find the 4 orientations required for
        # the general and special cases
        o1 = self.__orientation(p1, q1, p2)
        o2 = self.__orientation(p1, q1, q2)
        o3 = self.__orientation(p2, q2, p1)
        o4 = self.__orientation(p2, q2, q1)
        # General case
        # remove any cases of overlap and touch of 
        if (o1 == 0 or o2 == 0 or o3 == 0 or o4 == 0):
            return False

        if (o1 != o2) and (o3 != o4):
            return True

        # If none of the cases
        return False

    def __doTouch(self, p1: Point, q1: Point, p2: Point, q2: Point) -> int:
        """
        The function that returns true if
        the line segment 'p1q1' and 'p2q2' intersect.
        """
        o2 = self.__orientation(p1, q1, q2)
        # p1 , q1 and q2 are collinear and q2 lies on segment p1q1
        if (o2 == 0) and self.__onSegment(p1, q2, q1):
            return True
        # If none of the cases
        return False

    def intersectLine(self, p0: Point, p1: Point) -> bool:
        """
        Returns true if the line segment intersects with
        the edges of self
        p0: starting point of line segment
        p1: ending point of line segment
        """
        # need parallel
        for idx in range(len(self.points)):
            e0 = self.points[idx]
            e1 = self.points[(idx + 1) % len(self.points)]
            if self.__doIntersect(p0, p1, e0, e1):
                return True
        return False

    def touchIntersect(self, p0: Point, p1: Point) -> bool:
        """
        Check if there exist any intersection between polygon and a line segment
        p0: starting point of line segment
        p1: ending point of line segment
        """
        # need parallel
        for i in range(len(self.points)):
            e0 = self.points[i]
            e1 = self.points[(i + 1) % len(self.points)]
            # if self.__onSegment(p0,e0,p1) and self.__onSegment(p0,e1,p1):
            #     return True
            if self.__doTouch(p0, p1, e0, e1) or self.__doIntersect(p0, p1, e0, e1):
                return True
        return False

    def containsPoint(self, p: Point) -> bool:
        # if polygon contains the input point
        right = self.touchIntersect(p, Point(self.maxLon, p.lat))
        left = self.touchIntersect(Point(-1, p.lat),p)
        up = self.touchIntersect(p, Point(p.lon, self.maxLat))
        down = self.touchIntersect(p, Point(p.lon, -1))
        return right and left and up and down

    


class BoundingBox:
    PARTIAL_OVERLAP = 0
    INSIDE = 1
    OUTSIDE = 2

    def __init__(self, center: Point, width: float, height: float):
        self.center: Point = center
        self.width: float = width
        self.height: float = height
        self.west: float = self.center.lon - width / 2
        self.east: float = self.center.lon + width / 2
        self.north: float = self.center.lat + height / 2
        self.south: float = self.center.lat - height / 2

    def containsPoint(self, point: Point) -> bool:
        """
        Returns true if the bounding box contains point
        """
        return (point is not None) and (
            self.west < point.lon < self.east and self.south < point.lat < self.north
        )

    def positionVsPolygon(self, polygon: Polygon) -> int:
        """
        Returns the relative position of the bounding box with respect to polygon
        """
        # bounding box intersects partially with the polygon
        if (
            any([self.containsPoint(point) for point in polygon.points])
            or polygon.intersectLine(
                Point(self.east, self.north), Point(self.west, self.north)
            )
            or polygon.intersectLine(
                Point(self.west, self.north), Point(self.west, self.south)
            )
            or polygon.intersectLine(
                Point(self.west, self.south), Point(self.east, self.south)
            )
            or polygon.intersectLine(
                Point(self.east, self.south), Point(self.east, self.north)
            )
        ):
            return self.PARTIAL_OVERLAP
        else:
            if polygon.containsPoint(self.center):
                return self.INSIDE
            else:
                return self.OUTSIDE

    def draw(self, ax: Axes) -> None:
        box = patches.Rectangle(
            (self.west, self.south),
            self.width,
            self.height,
            linewidth=0.1,
            edgecolor="r",
            facecolor="r",
        )
        ax.add_patch(box)

class BoxData:
    def __init__(self) -> None: # init dictionary to write file
        self.id = 0
        self.boxes = {
            "type" : "FeatureCollection",
            "name" : "quadtree",
            "crs" : {"type" : "name", "properties" : {"name" : "urn:ogc:def:crs:EPSG::3857"}},
            "features" : []
        }

# append square into dictionary
    def insertBox(self,west,north,east,south):
        insertNode = { 
                        "type": "Feature", 
                        "properties": 
                        { 
                            "id": self.id, 
                            "left": west, 
                            "top": north, 
                            "right": east, 
                            "bottom": south 
                        }, 
                        "geometry": 
                        { 
                            "type": "Polygon", 
                            "coordinates": [ [ [ west, north ], [ east, north ], [ east, south  ], [ west,south  ], [ west, north ] ] ] 
                        } 
                    }
        self.id += 1
        self.boxes["features"].append(insertNode)

    def exportJSON(self):
        writeDest = open('quadtree.geojson', 'w')
        json.dump(self.boxes,writeDest,indent = 4)
        writeDest.close()



boxes = BoxData()
class PolygonQuadTree:
    DIVISION_UNIT : float = 500  # smallest width of a node

    def __init__(self, boundBox: BoundingBox = None) -> None:
        self.boundBox: BoundingBox = boundBox
        self.isColored: bool = False
        self.children: list["PolygonQuadTree"] = None
        
        
    def initPolygon(self,polygon:Polygon):
        treeLv = math.ceil(math.log2(max(polygon.maxLat - polygon.minLat,polygon.maxLon - polygon.minLon)/ self.DIVISION_UNIT))
        size = self.DIVISION_UNIT * math.pow(2,treeLv)
        self.boundBox = BoundingBox(
                    Point(
                        polygon.minLon + size/2,
                        polygon.minLat + size/2,
                    ),
                    size,
                    size,
                )
        self.insertPolygon(polygon)
        return size

    def insertPolygon(self, polygon: Polygon) -> None:
        """
        Insert new polygon into the root node of self
        """
        
        position = self.boundBox.positionVsPolygon(polygon)
        if position == self.boundBox.PARTIAL_OVERLAP:
            if self.boundBox.width > self.DIVISION_UNIT:
                self.__divide(polygon)
            else:
                # overdrawn
                self.isColored = True
        else:
            self.isColored = position == self.boundBox.INSIDE

    def __divide(self, polygon: Polygon):
        """ """
        newWidth = self.boundBox.width / 2
        newHeight = self.boundBox.height / 2
        self.children = []

        for i in range(4):
            child = PolygonQuadTree(
                BoundingBox(
                    Point(
                        self.boundBox.center.lon + pow(-1, i & 1) * newWidth/2,
                        self.boundBox.center.lat + pow(-1, i >> 1) * newHeight/2,
                    ),
                    newWidth,
                    newHeight,
                )
            )
            child.insertPolygon(polygon)
            self.children.append(child)
            # print(child.boundBox.center.lon, child.boundBox.center.lat,"\n")

    def draw(self, ax: Axes) -> None:
        if self.isColored:
            self.boundBox.draw(ax)
            boxes.insertBox(self.boundBox.west, self.boundBox.north, self.boundBox.east, self.boundBox.south)
        if self.children is not None:
            for child in self.children:
                child.draw(ax)

DPI = 72  # dots (pixels) per inch


#test01

# A=Point(20,40)
# B=Point(39,96)
# C=Point(71,68)
# D=Point(99,115)
# E=Point(120,60)
# F=Point(75,53)
# G=Point(115,24)
# H=Point(7,4)
# I=Point(40,20)
# J=Point(3,4)
# K=Point(7,32)


#test02

# A=Point(11,20)
# B=Point(23,47)
# C=Point(7,38)
# D=Point(40,111)
# E=Point(33,79)
# F=Point(80,120)
# G=Point(120,80)
# H=Point(40,60)
# I=Point(62,53)
# J=Point(119,52)
# K=Point(120,20)
# L=Point(79,8)
# M=Point(37,49)





# listP = [A,B,C,D,E,F,G,H,I,J,K]




begin = time.time()
testPoly = Polygon("/home/vu/Documents/quarter/VNM.geojson","file")
# testPoly = Polygon(listP,"list")
testQT = PolygonQuadTree()
# parallel starts here
size = testQT.initPolygon(testPoly)
# parallel ends here
end = time.time()
print(end - begin)

# draw rectangles
fig = plt.figure(
    figsize=(700 / DPI, 500 / DPI), dpi=DPI
)  # each figure has to have w=700px and h=500px
ax = plt.subplot()
ax.set_xlim(testPoly.minLon, testPoly.minLon + size)  # The right limit of x axis is 360
ax.set_ylim(testPoly.minLat, testPoly.minLat + size)  # The upper limit of y axis is 180
testQT.draw(ax)



plt.tight_layout()
plt.savefig("/home/vu/Documents/quarter/search-quadtree.png", dpi=72)
plt.show()

boxes.exportJSON()






