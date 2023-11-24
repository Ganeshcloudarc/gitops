#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Polygon, PolygonStamped, Point, PoseStamped

from collision_monitor_core import PolygonCheck


def callback(data):
    # print("data", data)
    print("pl", pl_check.is_point_inside([data.pose.position.x, data.pose.position.y]))



if __name__ == "__main__":
    rospy.init_node("test_pilgon")
    pub_poly = rospy.Publisher("PolygonCheck", PolygonStamped, queue_size=1, latch=True )
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)
    global pl
    pl = Polygon()
    # for i in range(4):
    point = Point()
    point.x = 1
    point.y = 1
    pl.points.append(point)

    point = Point()
    point.x = -1
    point.y = 1
    pl.points.append(point)
    point = Point()
    point.x = -1
    point.y = -1
    pl.points.append(point)
    point = Point()
    point.x = 1
    point.y = -1
    pl.points.append(point)
    point = Point()
    point.x = 10
    point.y = -1
    pl.points.append(point)

    pl_st = PolygonStamped()
    pl_st.header.frame_id = "map"
    pl_st.polygon = pl
    pub_poly.publish(pl_st)
    global pl_check
    pl_check = PolygonCheck(pl)
    rospy.loginfo("published")
    rospy.spin()





