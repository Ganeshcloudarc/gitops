#!/usr/bin/env python3
import rospy
import math
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

import numpy as np
from scipy.spatial import Delaunay
import sys
sys.path.insert(0, "..")

# https://github.com/MaxMagazin/ma_rrt_path_plan/blob/master/src/MaRRTPathPlanNode.py
from autonav_route import SubscriberHandler

def ccw( A, B, C):
    # if three points are listed in a counterclockwise order.
    # return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

def getLineIntersection(a1, a2, b1, b2):
      """
      Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
      a1: [x, y] a point on the first line
      a2: [x, y] another point on the first line
      b1: [x, y] a point on the second line
      b2: [x, y] another point on the second line
      https://stackoverflow.com/questions/3252194/numpy-and-line-intersections
      """
      s = np.vstack([a1,a2,b1,b2])        # s for stacked
      h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
      l1 = np.cross(h[0], h[1])           # get first line
      l2 = np.cross(h[2], h[3])           # get second line
      x, y, z = np.cross(l1, l2)          # point of intersection
      if z == 0:                          # lines are parallel
          return (float('inf'), float('inf'))
      return (x/z, y/z)


def getLineSegmentIntersection( a1, a2, b1, b2):
    # https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
    # Return true if line segments a1a2 and b1b2 intersect
    # return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
    return ccw(a1, b1, b2) != ccw(a2, b1, b2) and ccw(a1, a2, b1) != ccw(a1, a2, b2)

class Cone:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f"x :{self.x} y : {self.y}"


class Edge():
   def __init__(self, x1, y1, x2, y2):
    self.x1 = x1
    self.y1 = y1
    self.x2 = x2
    self.y2 = y2
    self.intersection = None

   def getMiddlePoint(self):
    return (self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2

   def length(self):
    return math.sqrt((self.x1 - self.x2) ** 2 + (self.y1 - self.y2) ** 2)

   def getPartsLengthRatio(self):
    import math

    part1Length = math.sqrt((self.x1 - self.intersection[0]) ** 2 + (self.y1 - self.intersection[1]) ** 2)
    part2Length = math.sqrt((self.intersection[0] - self.x2) ** 2 + (self.intersection[1] - self.y2) ** 2)

    return max(part1Length, part2Length) / min(part1Length, part2Length)

   def __eq__(self, other):
    return (self.x1 == other.x1 and self.y1 == other.y1 and self.x2 == other.x2 and self.y2 == other.y2
            or self.x1 == other.x2 and self.y1 == other.y2 and self.x2 == other.x1 and self.y2 == other.y1)

   def __str__(self):
    return "(" + str(round(self.x1, 2)) + "," + str(round(self.y1, 2)) + "),(" + str(round(self.x2, 2)) + "," + str(
     round(self.y2, 2)) + ")"

   def __repr__(self):
    return str(self)

def getDelaunayEdges(frontCones):
    if len(frontCones) < 4:  # no sense to calculate delaunay
        return

    conePoints = np.zeros((len(frontCones), 2))

    for i in range(len(frontCones)):
        cone = frontCones[i]
        conePoints[i] = ([cone.x, cone.y])

    # print conePoints
    tri = Delaunay(conePoints)
    # print "len(tri.simplices):", len(tri.simplices)

    delaunayEdges = []
    print(dir(tri))
    for simp in tri.simplices:
        # print simp

        for i in range(3):
            j = i + 1
            if j == 3:
                j = 0
            edge = Edge(conePoints[simp[i]][0], conePoints[simp[i]][1], conePoints[simp[j]][0], conePoints[simp[j]][1])

            if edge not in delaunayEdges:
                delaunayEdges.append(edge)

    return delaunayEdges


def publishDelaunayEdgesVisual( edges):
    if not edges:
     return

    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    # marker.lifetime = rospy.Duration(1)
    marker.ns = "publishDelaunayLinesVisual"

    marker.type = marker.LINE_LIST
    marker.action = marker.ADD
    marker.scale.x = 0.05

    marker.pose.orientation.w = 1

    marker.color.a = 0.5
    marker.color.r = 1.0
    marker.color.b = 1.0

    for edge in edges:
       # print edge

       p1 = Point(edge.x1, edge.y1, 0)
       p2 = Point(edge.x2, edge.y2, 0)

       marker.points.append(p1)
       marker.points.append(p2)
    return marker
    # delaunayLinesVisualPub.publish(marker)
def points_to_marker(points):
   marker = Marker()
   marker.header.frame_id = "base_link"
   marker.header.stamp = rospy.Time.now()
   # marker.lifetime = rospy.Duration(1)
   marker.ns = "raman"

   marker.type = marker.LINE_STRIP
   marker.action = marker.ADD
   marker.scale.x = 0.05

   marker.pose.orientation.w = 1

   marker.color.a = 0.5
   marker.color.r = 1.0
   marker.color.b = 1.0

   for point in points:
     print("point", point)
     p1 = Point(point[0], point[1], 0)
     marker.points.append(p1)
   return marker


if __name__ == "__main__":
    rospy.init_node("traingeualtiom")
    delaunayLinesVisualPub = rospy.Publisher("/visual/delaunay_lines", Marker, queue_size=1, latch=True)
    delaunayLinesVisualPub_filtered = rospy.Publisher("/visual/delaunayLinesVisualPub_filtered", Marker, queue_size=1, latch=True)
    mid_points_pub = rospy.Publisher("/visual/mid_points", Marker, queue_size=1, latch=True)


    sub_handler = SubscriberHandler()
    rospy.Rate(2).sleep()

    rate= rospy.Rate(10)
    while not rospy.is_shutdown():
     # bboxes = sub_handler.get_local_bboxes()
     bboxes = sub_handler.get_bboxes("base_link")
     if not bboxes:
      rate.sleep()
      continue
     frontCones = []
     for box in bboxes.boxes:
        x,y = box.pose.position.x, box.pose.position.y
        cone = Cone(x, y)
        frontCones.append(cone)

     # frontCones = []
     # for i in range(1, 10, 2):
     #     x = i
     #     y = i
     #     cone = Cone(x, y)
     #     frontCones.append(cone)
     #     x = i
     #     y = -i
     #     cone = Cone(x, y)
     #     frontCones.append(cone)
     # # cone = Cone(0, 10)
     # # frontCones.append(cone)

     for cone in frontCones:
         print(cone)

     delaunayEdges = getDelaunayEdges(frontCones)

     # print "delaunay time: {0} ms".format((time.time() - delaunayStartTime) * 1000);
     delaunayLinesVisualPub.publish(publishDelaunayEdgesVisual(delaunayEdges))


     a1,a2= np.array([-10,0]), np.array([10,0])
     # b1 = np.array([edge.x1, edge.y1])
     # b2 = np.array([edge.x2, edge.y2])
     valid_edges = []
     maxAcceptedEdgeLength = 10
     for edge in delaunayEdges:
        b1 = np.array([edge.x1, edge.y1])
        b2 = np.array([edge.x2, edge.y2])
        print("intersect ",getLineSegmentIntersection(a1,a2,b1,b2))
        if getLineSegmentIntersection(a1,a2,b1,b2):
         # print(edge.length())
         if edge.length() < maxAcceptedEdgeLength:
          valid_edges.append(edge)
     print(valid_edges)
     mid_points = []
     for egde in valid_edges:
      mid_points.append(egde.getMiddlePoint())

     delaunayLinesVisualPub_filtered.publish(publishDelaunayEdgesVisual(valid_edges))
     print("mid_points", mid_points)
     mid_points_pub.publish(points_to_marker(mid_points))
     rospy.loginfo("publisged")
     rate.sleep()
    rospy.spin()
