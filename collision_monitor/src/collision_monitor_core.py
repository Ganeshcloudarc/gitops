#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
from geometry_msgs.msg import Polygon, PolygonStamped, Point
import logging
from laser_geometry import LaserProjection
import tf2_geometry_msgs
import sensor_msgs.point_cloud2 as pcd2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from autopilot_utils.tf_helper import transform_cloud
import ros_numpy
import numpy as np
from diagnostic_updater._diagnostic_status_wrapper import DiagnosticStatusWrapper
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue

OK = DiagnosticStatus.OK
ERROR = DiagnosticStatus.ERROR
WARN = DiagnosticStatus.WARN
STALE = DiagnosticStatus.STALE


class CommonCallaback:
    def __init__(self, topic_name, topic_type, min_z=None, max_z=None):
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.min_z = min_z
        self.max_z = max_z
        self.data = None
        if isinstance(self.topic_type(), LaserScan):
            rospy.logerr("inside inistam")
            self.laser_proj = LaserProjection()

    def callback(self, data):
        self.data = data

    def get_data(self, target_frame="base_link"):
        """
        Should return Numpy array of xy in base_link
        """
        if self.data is None:
            return None
        if isinstance(self.topic_type(), LaserScan):
            points = self.laser_proj.projectLaser(self.data)
            if self.data.header.frame_id == target_frame:
                points_numpy = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(points, remove_nans=False)
            else:
                tf_points = transform_cloud(points, self.data.header.frame_id, target_frame)
                if tf_points:
                    points_numpy = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(tf_points, remove_nans=False)
                else:
                    return None

            return points_numpy
        if isinstance(self.topic_type(), PointCloud2):
            points_numpy = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.data, remove_nans=False)
            points_numpy_reshape = np.reshape(points_numpy, (-1, 3), order='C')
            return points_numpy_reshape


class PolygonCheck:
    def __init__(self, polygon):

        self._poly = polygon.points

    def is_point_inside(self, point):

        poly_size = len(self._poly)
        i, j = 0, 0
        res = False
        i = poly_size - 1
        for j in range(0, poly_size):
            if (point[1] <= self._poly[i].y) == (point[1] > self._poly[j].y):
                x_inter = self._poly[i].x + \
                          (point[1] - self._poly[i].y) * (self._poly[j].x - self._poly[i].x) / (
                                  self._poly[j].y - self._poly[i].y)
                if x_inter > point[0]:
                    res = not res

            i = j

        return res


class CollisionMonitor:
    def __init__(self):
        self.observation_details = []
        self.load_params()

        pub_poly = rospy.Publisher("collision_monitor_polygon", PolygonStamped, queue_size=1, latch=True)
        self.diagnostics_pub = rospy.Publisher("/collision_monitor_diagnostics", DiagnosticArray, queue_size=1)
        self.diagnostics_arr = DiagnosticArray()
        self.diagnostics_arr.header.frame_id = self.robot_base_frame
        self.diagnose = DiagnosticStatusWrapper()
        self.diagnose.name = rospy.get_name()
        print(self.observation_details)
        collision_polygon = self.vehicle_foot_print()
        polygon_st = PolygonStamped()
        polygon_st.header.frame_id = self.robot_base_frame
        polygon_st.polygon = collision_polygon
        pub_poly.publish(polygon_st)
        rospy.loginfo("Polygon published")
        self.polygon_check = PolygonCheck(collision_polygon)
        for source in self.observation_details:
            print(source.topic_name, source.topic_type, source.callback)
            rospy.Subscriber(source.topic_name, source.topic_type, source.callback)

    def run(self):
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            collision_points_count = 0
            self.diagnose.clearSummary()
            self.diagnose.values = []
            for source in self.observation_details:
                print()
                points = source.get_data()
                if points is not None:
                    for point in points:
                        collision_in = self.polygon_check.is_point_inside(point)
                        if collision_in:
                            collision_points_count += 1
            rospy.loginfo(f"collision_points_count : {collision_points_count}")
            if collision_points_count > self.min_points:
                self.diagnose.summary(ERROR, "Collision Found near the vehicle")
                self.diagnose.add("collision_points_count", collision_points_count)
                rospy.logwarn("Collision Found")
            else:
                self.diagnose.summary(OK, "No collsion points Found")
                self.diagnose.add("collision_points_count", collision_points_count)
                rospy.logwarn("No collision")

            self.diagnostics_arr.status = []
            self.diagnostics_arr.status.append(self.diagnose)
            self.diagnostics_pub.publish(self.diagnostics_arr)
            rate.sleep()


    def load_params(self):
        self.robot_base_frame = rospy.get_param("~collision_monitor/base_frame_id", 1.0)
        self.frequency = rospy.get_param("~collision_monitor/frequency", 10)
        self.souce_timeout = rospy.get_param("~collision_monitor/source_timeout", 5)
        self.min_points = rospy.get_param("~collision_monitor/min_points", 4)
        self.polygon_points = rospy.get_param("~collision_monitor/points")
        observation_sources = rospy.get_param("~collision_monitor/observation_sources")
        print("observation_souces", observation_sources)
        for observation_source in observation_sources:
            topic_name = rospy.get_param("~collision_monitor/" + observation_source + "/topic", "scan")
            type = rospy.get_param("~collision_monitor/" + observation_source + "/type", "scan")

            if type == "scan":
                obj = CommonCallaback(topic_name, LaserScan)
            if type == "pointcloud":
                obj = CommonCallaback(topic_name, PointCloud2)
            self.observation_details.append(obj)

    def vehicle_foot_print(self):
        pl = Polygon()
        for i in range(0, len(self.polygon_points), 2):
            point = Point()
            point.x = self.polygon_points[i]
            point.y = self.polygon_points[i+1]
            pl.points.append(point)
        return pl


if __name__ == "__main__":
    rospy.init_node("collsion_monitor_node")
    collsion_monitor = CollisionMonitor()
    collsion_monitor.run()
    rospy.spin()
