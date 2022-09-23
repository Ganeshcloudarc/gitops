#!/usr/bin/env python3
from logging import exception
import roslib
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pcd2
from std_msgs.msg import Header, Bool
import numpy as np
import math
import open3d as o3d
from sklearn.neighbors import KDTree
from laser_geometry import LaserProjection
from autopilot_utils.tf_helper import current_robot_pose, convert_point, transform_cloud
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import ros_numpy
"""
angle_min: -1.5707999467849731
angle_max: 1.5707999467849731
angle_increment: 0.008700000122189522
time_increment: 0.0
scan_time: 0.33329999446868896
range_min: 0.44999998807907104
range_max: 10.0
"""

tfBuffer = None
listener = None


def _init_tf():
    # Create buffer and listener
    # Something has changed in tf that means this must happen after init_node
    global tfBuffer, listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)


def transform_cloud1(cloud, from_frame, to_frame):
    global tfBuffer, listener
    if tfBuffer is None or listener is None:
        _init_tf()
    try:
        trans = tfBuffer.lookup_transform(to_frame, from_frame, rospy.Time.now(), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr('FAILED TO GET TRANSFORM FROM %s to %s' % (to_frame, from_frame))
        return None
    cloud_out = do_transform_cloud(cloud, trans)
    # converts_to_numpy(cloud_out)
    return cloud_out


class LaserObstacle:
    def __init__(self):

        data = None
        self.min_group_distance = rospy.get_param("min_group_distance", 0.1)
        self.distance_proportion = rospy.get_param("distance_proportion", 0.00628)
        self.min_stop_distance_x = rospy.get_param("min_stop_distance_x", 3)
        self.min_stop_distance_y = rospy.get_param("min_stop_distance_y", 1.5)

        scan_in = rospy.get_param("scan_in", "zed/laser_scan")
        self.laser_geo_obj = LaserProjection()

        while not rospy.is_shutdown():
            try:
                data = rospy.wait_for_message(scan_in, LaserScan, timeout=2)
            except:
                pass
            if data != None:
                self.frame_id = data.header.frame_id
                self.angle_min = data.angle_min
                self.angle_max = data.angle_max
                self.angle_increment = data.angle_increment

                rospy.Subscriber(scan_in, LaserScan, self.laser_callback)
                self.laser_obj_pub = rospy.Publisher("laser_object_status", Bool, queue_size=1)
                self.point_cloud_pub = rospy.Publisher("point_cloud", PointCloud2, queue_size=10)
                break
            else:
                rospy.loginfo(f"Waiting for {scan_in} ")

    def laser_callback(self, data):
        print("------------------------------")
        # self.polar_to_carteesian(data.ranges)

        ## to trasform to map frame
        # self.transfrom_laser_to_map(data)

        ## to convert to points cloud
        points = self.laser_geo_obj.projectLaser(data)
        # print(points)
        tf_points = transform_cloud(points, "ego_vehicle", "map" )

        # self.point_cloud_pub.publish(tf_points)

        # points_numpy = converts_to_numpy(tf_points)
        # print("points_numpy",points_numpy.size())
        pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(tf_points, remove_nans=True)
        print("points_numpy", pc_np.shape)
        # print("points_numpy", pc_np.shape)
        tree = KDTree(pc_np, leaf_size=2)
        ind_list = []
        for i in range(0, 250):
            # tree = KDTree(pc_np, leaf_size=2)
            # print("i",i)
            ind = tree.query_radius(np.array([pc_np[i]]), r=0.3)
            print("ind",ind[0])
            ind_list.extend(list(ind[0]))

        axis = 0
        print("ind_list",len(ind_list))
        print(ind_list)
        a = np.take(pc_np, ind_list, axis)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        print("--")
        # print(a[0])
        scaled_polygon_pcl = pcd2.create_cloud_xyz32(header, a)
        rospy.loginfo("happily publishing sample pointcloud.. !")
        self.point_cloud_pub.publish(scaled_polygon_pcl)




    def polar_to_carteesian(self, ranges):
        cloud_points = []
        for i, rang in enumerate(ranges):
            if rang == np.inf:
                continue
            angle = self.angle_min + (i * self.angle_increment)
            x, y = pol2cart(rang, angle)
            cloud_points.append([x, y, 0])
        cloud_points = np.array(cloud_points)
        tree = KDTree(cloud_points, leaf_size=2)
        ind = tree.query_radius(np.array([[3, 1, 0]]), r=0.2)
        print(ind)  # indices of neighbors within distance 0.3
        axis = 0
        a = np.take(cloud_points, list(ind), axis)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        print("--")
        print(a[0])
        scaled_polygon_pcl = pcd2.create_cloud_xyz32(header, a[0])
        rospy.loginfo("happily publishing sample pointcloud.. !")
        self.point_cloud_pub.publish(scaled_polygon_pcl)

    def transfrom_laser_to_map(self, data):
        cloud_points = []
        for i, rang in enumerate(data.ranges):
            if rang == np.inf:
                continue
            angle = data.angle_min + (i * data.angle_increment)
            x, y = pol2cart(rang, angle)

            map_frame_point = convert_point([x, y, 0], data.header.frame_id, "map")
            # map_frame_point = [x,y,0]
            print("x "+ str(x)+" -> "+ str(map_frame_point[0]))
            print("y "+ str(y)+" -> "+ str(map_frame_point[1]))

            # print(map_frame_point)
            cloud_points.append(map_frame_point)
        cloud_points = np.array(cloud_points)
        # pritnt
        # self.tree = KDTree(cloud_points, leaf_size=2)

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        # print("--")
        # print()
        scaled_polygon_pcl = pcd2.create_cloud_xyz32(header, cloud_points)
        rospy.loginfo("happily publishing sample pointcloud.. !")
        self.point_cloud_pub.publish(scaled_polygon_pcl)

    # @staticmethod


def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


# @staticmethod
def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return (x, y)

    # angle = angle_min + (i * angle_increment)
    # x = r × cos( θ )
    # y = r × sin( θ )


if __name__ == "__main__":
    rospy.init_node("laser_obstalces")
    print("a")
    a = LaserObstacle()
    print("b")
    rospy.spin()
