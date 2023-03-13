#!/usr/bin/env python3

import numpy as np
from numpy import random
import numpy
import matplotlib.pyplot as plt
import rospy
# from RDP import rdp
import rospy
from std_msgs.msg import String, Bool
from nav_msgs.msg import Path
# from scipy import spatial
# import scipy.spatial
# from scipy.spatial import distance
from nav_msgs.msg import Odometry
from matplotlib.animation import FuncAnimation

line = []
# https://stackoverflow.com/questions/14631776/calculate-turning-points-pivot-points-in-trajectory-path

"""
The Ramer-Douglas-Peucker algorithm roughly ported from the pseudo-code provided
by http://en.wikipedia.org/wiki/Ramer-Douglas-Peucker_algorithm
"""
from math import sqrt

class curve_detector:
    def __init__(self):
        self.min_angle = np.pi*0.05
        self.curve_points=[]
        self.dist_from_curve_point_th = 5 # +/- 5m meters from the curve point
        self.rdp_tolerance = 2.0 # tolerance to detect or ignore curve
        self.is_curve_publisher = rospy.Publisher("/global_gps_path/is_curve", Bool , queue_size=1)
        rospy.Subscriber("/global_gps_path", Path, self.gps_path_cb)
        rospy.Subscriber("/vehicle/odom", Odometry, self.odom_cb)
        
    def distance(self,a, b):
        return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def point_line_distance(self,point, start, end):
        if (start == end):
            return distance(point, start)
        else:
            n = abs(
                (end[0] - start[0]) * (start[1] - point[1]) -
                (start[0] - point[0]) * (end[1] - start[1])
            )
            d = sqrt(
                (end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2
            )
            return n / d


    def rdp(self,points, epsilon):
        """Reduces a series of points to a simplified version that loses detail, but
        maintains the general shape of the series.
        """
        dmax = 0.0
        index = 0
        for i in range(1, len(points) - 1):
            d = self.point_line_distance(points[i], points[0], points[-1])
            if d > dmax:
                index = i
                dmax = d

        if dmax >= epsilon:
            results = self.rdp(points[:index+1], epsilon)[:-1] + self.rdp(points[index:], epsilon)
        else:
            results = [points[0], points[-1]]

        return results


    def angle(self,dir):
        """
        Returns the angles between vectors.

        Parameters:
        dir is a 2D-array of shape (N,M) representing N vectors in M-dimensional space.

        The return value is a 1D-array of values of shape (N-1,), with each value
        between 0 and pi.

        0 implies the vectors point in the same direction
        pi/2 implies the vectors are orthogonal
        pi implies the vectors point in opposite directions
        """
        dir2 = dir[1:]
        dir1 = dir[:-1]
        return np.arccos((dir1*dir2).sum(axis=1)/(
            np.sqrt((dir1**2).sum(axis=1)*(dir2**2).sum(axis=1))))
        
    def rdp_calculate(self,line):
        rdp_line = np.array(self.rdp(line, self.rdp_tolerance))
        simplified = rdp_line
        sx, sy = simplified.T
        directions = np.diff(simplified, axis=0)
        theta = self.angle(directions)
        # Select the index of the points with the greatest theta
        # Large theta is associated with greatest change in direction.
        idx = np.where(theta>self.min_angle)[0]+1
        for xx, yy in zip(sx[idx], sy[idx]):
            self.curve_points.append((xx,yy))
        # return sx,sy,idx
        # print(self.curve_points)
        rospy.loginfo(self.curve_points)
        
    def gps_path_cb(self,data):

        for pose in data.poses:
                # _, _, heading = euler_from_quaternion(
                #     [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
                # heading = math.degrees(heading)
                line.append((pose.pose.position.x, pose.pose.position.y))
        self.rdp_calculate(line)
        return line

    def odom_cb(self,data):
        vehicle_x = data.pose.pose.position.x
        vehicle_y = data.pose.pose.position.y
        import numpy as np
        A = self.curve_points
        A = np.array(A)
        try:
            vehicle_loc = np.array((vehicle_x,vehicle_y))
            distances = np.linalg.norm(A-vehicle_loc, axis=1)
            min_index = np.argmin(distances)
            # print(f"the closest point is {A[min_index]}, at a distance of {distances[min_index]}")
            if distances[min_index] <= self.dist_from_curve_point_th:
                rospy.loginfo_once("curve")
                self.is_curve_publisher.publish(True)
            else:
                rospy.loginfo_once("Straight line")
                self.is_curve_publisher.publish(False)
        except Exception as e:
            rospy.logdebug(e)

if __name__ == '__main__':
    rospy.init_node('curve_detector')
    cd = curve_detector()
    rospy.spin()
