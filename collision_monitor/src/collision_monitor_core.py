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
from pilot_msgs.msg import vehicle_stop_command
from ackermann_msgs.msg import AckermannDrive
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
        self.debug = rospy.get_param("/patrol/debug",False)
        self.pub_poly = rospy.Publisher("collision_monitor_polygon", PolygonStamped, queue_size=1, latch=True)
        self.diagnostics_pub = rospy.Publisher("/collision_monitor_diagnostics", DiagnosticArray, queue_size=1)
        self.pilot_stop_command_pub = rospy.Publisher("/vehicle/break_command", vehicle_stop_command, queue_size=1) 
        rospy.Subscriber('/vehicle/cmd_drive_safe',AckermannDrive, self.vehicle_cmd_drive_cb)
        self.steering_angle = None
        self.polygon_points_rotated = None
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
            steering_angle_based_collision_check = rospy.get_param("~collision_monitor/steering_angle_based_collision_check", False)
            if steering_angle_based_collision_check:
                collision_polygon = self.vehicle_foot_print(rotated_polygon=steering_angle_based_collision_check)
                self.polygon_st = PolygonStamped()
                self.polygon_st.header.frame_id = self.robot_base_frame
                self.polygon_st.polygon = collision_polygon
                self.pub_poly.publish(self.polygon_st)
                rospy.loginfo("Polygon published")
                self.polygon_check = PolygonCheck(collision_polygon)

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
                    self.diagnose.summary(ERROR, "No data from front lidar") 
                    self.diagnose.add("Time out from sensor_source", source.topic_name)
                    self.diagnose.add("Timeout" , rospy.Time.now().secs - source.last_update_time())
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
                # stopping vehicle_stop_command when bypass dist is non zero
                self.by_pass_dist = rospy.get_param("/obstacle_stop_planner/by_pass_dist", 0) 
                if self.by_pass_dist != 0: 
                    vehicle_stop_msg.status = False
                    vehicle_stop_msg.message =  "Bypassing Collision Found"  
                    if self.debug: 
                        self.diagnose.summary(ERROR, "Collision Found near the vehicle, Bypassing")  
                    else: 
                        self.diagnose.summary(ERROR, "BYPASSING COLLISION")
                else: 
                    vehicle_stop_msg.status = True
                    vehicle_stop_msg.message =  "Collision Found near the vehicle"
                    if self.debug:
                        self.diagnose.summary(ERROR, "Collision Found near the vehicle, Stopping")
                    else:
                        self.diagnose.summary(ERROR, "COLLISION FOUND")
                self.diagnose.add("message","Collision Found near the vehicle")                
                self.diagnose.add("total collision_points_count", collision_points_count)
                rospy.logwarn("Collision Found")
            else:
                self.diagnose.summary(OK, "No collision points Found")
                vehicle_stop_msg.status = False
                vehicle_stop_msg.message =  "No collision points Found"
                self.diagnose.add("total collision_points_count", collision_points_count)
                rospy.loginfo("No collision")
                
                # #501 Fix Autobypass when obs stop planner is false. 
                # Set bypass to false when obs stop planner is not running and if no obstacle
                # If obs planner is runnig, it will take care of bypass reset. 
                is_obs_planner = rospy.get_param("/patrol/is_obs_stop_planner",False)
                if not is_obs_planner:
                    rospy.set_param("/obstacle_stop_planner/by_pass_dist",0)

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

    def vehicle_foot_print(self, rotated_polygon=False):
        pl = Polygon()
        if not rotated_polygon:
            for i in range(0, len(self.polygon_points)):
                point = Point()
                point.x = self.polygon_points[i][0]
                point.y = self.polygon_points[i][1]
                pl.points.append(point)
        else:
            try:
                pl = Polygon()
                self.polygon_points_rotated = self.rotate_polygon(self.polygon_points, self.steering_angle)
                for i in range(0, len(self.polygon_points_rotated)):
                    point = Point()
                    point.x = self.polygon_points_rotated[i][0]
                    point.y = self.polygon_points_rotated[i][1]
                    pl.points.append(point)
            except Exception as e:
                rospy.logdebug(e)
        return pl

    def rotate_point(self,x, y, theta):
        theta_rad = np.deg2rad(theta)
        cos_theta = np.cos(theta_rad)
        sin_theta = np.sin(theta_rad)
        return x * cos_theta - y * sin_theta, x * sin_theta + y * cos_theta

    # Function to rotate a polygon by an angle theta
    def rotate_polygon(self, polygon, theta):
        return [self.rotate_point(x, y, -theta) for x, y in polygon]

    def vehicle_cmd_drive_cb(self, data):
        self.steering_angle = data.steering_angle


if __name__ == "__main__":
    rospy.init_node("collsion_monitor_node")
    collsion_monitor = CollisionMonitor()
    collsion_monitor.run()
    rospy.spin()
