import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray, PolygonArray
from sensor_msgs.msg import PointCloud2, LaserScan
from laser_geometry import LaserProjection
import numpy as np
from nav_msgs.msg import Odometry
np.float = np.float64
import ros_numpy
from autopilot_utils.tf_helper import transform_cloud, transform_lidar_objects


class SubscriberHandler:
    def __init__(self):
        self.local_bboxes_data = None
        self.bboxes_data = None
        self.laser_scan_data = None
        self.local_laser_scan_data = None
        self.odom_data = None
        rospy.Subscriber("vehicle/odom", Odometry, self.odom_callback)
        rospy.Subscriber("local_cloud_laser_scan", LaserScan, self.local_laser_scan_callback)
        rospy.Subscriber("/laser_scan", LaserScan, self.laser_scan_callback)
        rospy.Subscriber("/local_bboxes", BoundingBoxArray, self.local_bboxes_callback)
        rospy.Subscriber("/obstacle_detector/jsk_bboxes", BoundingBoxArray, self.bboxes_callback)
        self.laser_projection = LaserProjection()

    def __str__(self):
        return "SubscriberHandler class"

    def data_received(self):
        if bool(self.local_bboxes_data) and bool(self.bboxes_data) and bool(self.laser_scan_data) and bool(
                self.odom_data):
            return True, "all data received"
        else:
            return False, f"local_bboxes_data: {bool(self.local_bboxes_data)} bboxes_data : {bool(self.bboxes_data)}  laser_scan_data : {bool(self.laser_scan_data)}  odom_data : {bool(self.odom_data)}"

    def odom_callback(self, data):
        self.odom_data = data

    def get_odom(self):
        return self.odom_data

    def local_laser_scan_callback(self, data):
        self.local_laser_scan_data = data

    def get_local_laser_scan(self, target_frame=""):
        if self.laser_scan_data:
            point_cloud_points = self.laser_projection.projectLaser(self.local_laser_scan_data)
            if target_frame == "":
                scan_points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(point_cloud_points, remove_nans=False)[:,
                              :-1]
                return scan_points
            else:
                transformed_cloud = transform_cloud(point_cloud_points, point_cloud_points.header.frame_id,
                                                    target_frame)
                if transformed_cloud:
                    scan_points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(transformed_cloud, remove_nans=False)[
                                  :, :-1]
                    return scan_points
                else:
                    return None
        else:
            return None

    def laser_scan_callback(self, data):
        self.laser_scan_data = data

    def get_raw_scan(self):
        return self.laser_scan_data

    def get_scan(self, target_frame=""):
        if self.laser_scan_data:
            point_cloud_points = self.laser_projection.projectLaser(self.laser_scan_data)
            if target_frame == "":
                scan_points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(point_cloud_points, remove_nans=False)[:,
                              :-1]
                return scan_points
            else:
                transformed_cloud = transform_cloud(point_cloud_points, point_cloud_points.header.frame_id,
                                                    target_frame)
                if transformed_cloud:
                    scan_points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(transformed_cloud, remove_nans=False)[
                                  :,
                                  :-1]
                    return scan_points
                else:
                    return None
        else:
            return None

    def local_bboxes_callback(self, data):
        self.local_bboxes_data = data

    def get_local_bboxes(self, target_frame=""):
        if self.local_bboxes_data:
            if target_frame == "":
                return self.local_bboxes_data
            else:
                data = transform_lidar_objects(self.local_bboxes_data, target_frame)
                if data:
                    return data
                else:
                    return None
        else:
            return None

    def bboxes_callback(self, data):
        self.bboxes_data = data

    def get_bboxes(self, target_frame=""):
        if self.bboxes_data:
            if target_frame == "":
                return self.bboxes_data
            else:
                return transform_lidar_objects(self.bboxes_data, target_frame)
        else:
            return None
