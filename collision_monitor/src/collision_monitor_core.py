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
import numpy as np
np.float = np.float64
import ros_numpy
from diagnostic_updater._diagnostic_status_wrapper import DiagnosticStatusWrapper
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
from pilot.msg import vehicle_stop_command

OK = DiagnosticStatus.OK
ERROR = DiagnosticStatus.ERROR
WARN = DiagnosticStatus.WARN
STALE = DiagnosticStatus.STALE


class CommonCallback:
    def __init__(self, topic_name, topic_type, min_z=None, max_z=None):
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.min_z = min_z
        self.max_z = max_z
        self.data = None
        self.data_received = False
        if isinstance(self.topic_type(), LaserScan):
            self.laser_proj = LaserProjection()

    def callback(self, data):
        self.data = data
        if not self.data_received:
            self.data_received = True

    def last_update_time(self):
        if self.data_received:
            return self.data.header.stamp.secs
        else:
            return 0

    def check_data_received(self):
        return self.data_received

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
                    points_numpy = None

            return points_numpy
        if isinstance(self.topic_type(), PointCloud2):
            if self.data.header.frame_id == target_frame:
                points_numpy = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.data, remove_nans=False)
                points_numpy_reshape = np.reshape(points_numpy, (-1, 3), order='C')

            else:
                tf_points = transform_cloud(self.data, self.data.header.frame_id, target_frame)
                if tf_points:
                    points_numpy = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(tf_points, remove_nans=False)
                    points_numpy_reshape = np.reshape(points_numpy, (-1, 3), order='C')
                    return points_numpy_reshape
                else:
                    points_numpy_reshape = None
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

        self.pub_poly = rospy.Publisher("collision_monitor_polygon", PolygonStamped, queue_size=1, latch=True)
        self.diagnostics_pub = rospy.Publisher("/collision_monitor_diagnostics", DiagnosticArray, queue_size=1)
        self.pilot_stop_command_pub = rospy.Publisher("/vehicle/break_command", vehicle_stop_command, queue_size=1)
        self.diagnostics_arr = DiagnosticArray()
        self.diagnostics_arr.header.frame_id = self.robot_base_frame
        self.diagnose = DiagnosticStatusWrapper()
        self.diagnose.name = rospy.get_name()
        collision_polygon = self.vehicle_foot_print()
        self.polygon_st = PolygonStamped()
        self.polygon_st.header.frame_id = self.robot_base_frame
        self.polygon_st.polygon = collision_polygon
        self.pub_poly.publish(self.polygon_st)
        rospy.loginfo("Polygon published")
        self.polygon_check = PolygonCheck(collision_polygon)
        for source in self.observation_details:
            print(source.topic_name, source.topic_type, source.callback)
            rospy.Subscriber(source.topic_name, source.topic_type, source.callback)

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.diagnose.clearSummary()
            self.diagnose.values = []
            status_list = []
            for source in self.observation_details:
                status = source.check_data_received()
                self.diagnose.add(source.topic_name, status)
                status_list.append(status)
            if all(status_list):
                self.diagnose.summary(OK, "Data received on all the sensors")
                rospy.loginfo("Data received on all the sensors")
                self.diagnostics_arr.status.append(self.diagnose)
                self.diagnostics_pub.publish(self.diagnostics_arr)
                break
            else:
                self.diagnose.summary(WARN, "No received on all the sensors")
                rospy.logwarn("No received on all the sensors")
                self.diagnostics_arr.status.append(self.diagnose)
                self.diagnostics_pub.publish(self.diagnostics_arr)
                rate.sleep()
                continue
        rate = rospy.Rate(self.frequency)

        vehicle_stop_msg = vehicle_stop_command()
        vehicle_stop_msg.node = rospy.get_name()
        while not rospy.is_shutdown():
           
            collision_points_count = 0
            self.diagnose.clearSummary()
            self.diagnose.values = []
            vehicle_stop_msg.message = ""
            vehicle_stop_msg.status = None
            time_out_status = False
            for source in self.observation_details:
                if rospy.Time.now().secs - source.last_update_time() > self.souce_timeout:
                    time_out_status = True
                    rospy.logwarn(f"No update on sensor source : {source.topic_name}")
                    self.diagnose.add(f"time out from {source.topic_name}", rospy.Time.now().secs - source.last_update_time())
                points = source.get_data()
                count = 0
                if points is not None:
                    for point in points:
                        collision_in = self.polygon_check.is_point_inside(point)
                        if collision_in:
                            count += 1
                collision_points_count += count
                self.diagnose.add(source.topic_name+":  collision_points_count ", count)
            rospy.loginfo(f"total : collision_points_count : {collision_points_count}")
            if collision_points_count > self.min_points or time_out_status:
                self.diagnose.summary(ERROR, "Collision Found near the vehicle")
                vehicle_stop_msg.status = True
                vehicle_stop_msg.message =  "Collision Found near the vehicle"
                self.diagnose.add("total collision_points_count", collision_points_count)
                rospy.logwarn("Collision Found")
            else:
                self.diagnose.summary(OK, "No collision points Found")
                vehicle_stop_msg.status = False
                vehicle_stop_msg.message =  "Collision Found near the vehicle"
                self.diagnose.add("total collision_points_count", collision_points_count)
                rospy.logwarn("No collision")

            self.diagnostics_arr.status = []
            self.diagnostics_arr.status.append(self.diagnose)
            self.diagnostics_pub.publish(self.diagnostics_arr)
            self.pilot_stop_command_pub.publish(vehicle_stop_msg)
            self.pub_poly.publish(self.polygon_st)
            rate.sleep()


    def load_params(self):
        self.robot_base_frame = rospy.get_param("~collision_monitor/base_frame_id", "base_link")
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
                obj = CommonCallback(topic_name, LaserScan)
            if type == "pointcloud":
                obj = CommonCallback(topic_name, PointCloud2)
            self.observation_details.append(obj)

    def vehicle_foot_print(self):
        pl = Polygon()
        for i in range(0, len(self.polygon_points)):
            point = Point()
            point.x = self.polygon_points[i][0]
            point.y = self.polygon_points[i][1]
            pl.points.append(point)
        return pl


if __name__ == "__main__":
    rospy.init_node("collsion_monitor_node")
    collsion_monitor = CollisionMonitor()
    collsion_monitor.run()
    rospy.spin()
