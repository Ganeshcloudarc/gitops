#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Polygon, PolygonStamped, Point,PoseStamped
from jsk_recognition_msgs.msg import BoundingBoxArray, PolygonArray

class PolygonCheck:
    def __init__(self, polygon_arr):

        self._poly_arr = polygon_arr

    def is_point_inside(self, point):
        collision_list = []
        for poly in self._poly_arr.polygons:
            _poly = poly.polygon.points
            poly_size = len(_poly)
            i, j = 0, 0
            res = False
            i = poly_size - 1
            for j in range(0, poly_size):
                if (point[1] <= _poly[i].y) == (point[1] > _poly[j].y):
                    x_inter = _poly[i].x + \
                              (point[1] - _poly[i].y) * (_poly[j].x - _poly[i].x) / (
                                      _poly[j].y - _poly[i].y)
                    if x_inter > point[0]:
                        res = not res
                i = j
            collision_list.append(res)

        if any(collision_list):
            return True
        else:
            return False


def callback(data):
    # print("data", data)
    print("pl", pl_check.is_point_inside([data.pose.position.x, data.pose.position.y]))

if __name__ == "__main__":
    rospy.init_node("test_pilgon")
    pub_poly = rospy.Publisher("PolygonCheckArr", PolygonArray, queue_size=1, latch=True )
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)
    polygon_arr = PolygonArray()
    polygon_arr.header.frame_id = "map"
    for i in range(1, 5):
        pl = Polygon()
        # for i in range(4):
        point = Point()
        point.x = 0
        point.y = 0
        pl.points.append(point)

        point = Point()
        point.x = 0
        point.y = i
        pl.points.append(point)
        point = Point()
        point.x = i
        point.y = i
        pl.points.append(point)
        point = Point()
        point.x = i
        point.y = 0
        pl.points.append(point)
        point = Point()
        point.x = 0
        point.y = 0
        pl_st = PolygonStamped()
        pl_st.header.frame_id = "map"
        pl_st.polygon = pl
        polygon_arr.polygons.append(pl_st)
    pub_poly.publish(polygon_arr)
    print("polygon_arr", polygon_arr)
    global pl_check
    pl_check = PolygonCheck(polygon_arr)
    rospy.loginfo("published")
    rospy.spin()


    # ##
    # pl = Polygon()
    # # for i in range(4):
    # point = Point()
    # point.x = 5
    # point.y = 0
    # pl.points.append(point)
    #
    # point = Point()
    # point.x = -5
    # point.y = 0
    # pl.points.append(point)
    # point = Point()
    # point.x = -0
    # point.y = -5
    # pl.points.append(point)
    # point = Point()
    # point.x = 0
    # point.y = -0
    # pl.points.append(point)
    # point = Point()
    # point.x = 5
    # point.y = 0
    # pl.points.append(point)
    # l.append(pl)

    # pl_st = PolygonStamped()
    # pl_st.header.frame_id = "map"
    # pl_st.polygon = l


    # polygon_arr = PolygonArray()
    # polygon_arr.header.frame_id = "map"
    # polygon_arr.polygons.append(pl_st)

