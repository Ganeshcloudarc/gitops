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
"""
angle_min: -1.5707999467849731
angle_max: 1.5707999467849731
angle_increment: 0.008700000122189522
time_increment: 0.0
scan_time: 0.33329999446868896
range_min: 0.44999998807907104
range_max: 10.0
"""


class LaserObstacle:
    def __init__(self):
        
        data = None
        self.min_group_distance = rospy.get_param("min_group_distance", 0.1)
        self.distance_proportion = rospy.get_param("distance_proportion", 0.00628)
        self.min_stop_distance_x = rospy.get_param("min_stop_distance_x", 3)
        self.min_stop_distance_y = rospy.get_param("min_stop_distance_y", 1.5)

        scan_in = rospy.get_param("scan_in","zed/laser_scan")
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
        self.polar_to_carteesian(data.ranges)
        # points = self.laser_geo_obj.projectLaser(data)
        # print(points)
        # self.point_cloud_pub.publish(points)

    def polar_to_carteesian(self, ranges):
        cloud_points = []
        for i, rang in enumerate(ranges):
            if rang ==np.inf:
                continue
            angle = self.angle_min + (i * self.angle_increment)
            x, y = pol2cart(rang, angle)
            cloud_points.append([x, y,0])
        cloud_points = np.array(cloud_points)
        tree = KDTree(cloud_points, leaf_size=2) 
        ind = tree.query_radius(np.array([[2,1,0]]), r=0.3)  
        print(ind)  # indices of neighbors within distance 0.3
        axis = 0
        a=np.take(cloud_points, list(ind), axis)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        print("--")
        print(a[0])
        scaled_polygon_pcl = pcd2.create_cloud_xyz32(header, a[0])
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