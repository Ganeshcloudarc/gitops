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

        scan_in = rospy.get_param("scan_in","zed2i/laser_scan")

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

    def polar_to_carteesian(self, ranges):
        cloud_points = []
        for i, rang in enumerate(ranges):
            angle = self.angle_min + (i * self.angle_increment)
            x, y = pol2cart(rang, angle)
            cloud_points.append([x, y, 0])

            
        point_set = {0: 1}
        point_set_final = []
        j = 0
        for i in range(int(len(cloud_points) - 1)):

            if (
                cloud_points[i][0] < self.min_stop_distance_x
                and abs(cloud_points[i][1]) < self.min_stop_distance_y
            ):
                dis = distance(cloud_points[i], cloud_points[i + 1])
                if dis < self.min_group_distance + ranges[i] * self.distance_proportion:
                    point_set[j] = point_set[j] + 1
                else:
                    j += 1
                    point_set[j] = 0
        print(point_set)
        if len(point_set) > 1:
            self.laser_obj_pub.publish(True)
        else:
            if point_set[0] > 1:
                self.laser_obj_pub.publish(True)
                
            else:
                self.laser_obj_pub.publish(False)
        
        # a= np.array(cloud_points)
        # a= a.reshape(3,-1)
        # print(a.shape)
        # header = Header()
        # header.stamp = rospy.Time.now()
        # header.frame_id = self.frame_id
        # scaled_polygon_pcl = pcd2.create_cloud_xyz32(header, a)
        # rospy.loginfo("happily publishing sample pointcloud.. !")
        # self.point_cloud_pub.publish(scaled_polygon_pcl)

      

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