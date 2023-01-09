#!/usr/bin/env python3
import numpy as np
from numpy import random
import numpy
import matplotlib.pyplot as plt
import rospy
# from RDP import rdp
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from scipy import spatial
import scipy.spatial
from scipy.spatial import distance
from nav_msgs.msg import Odometry

line = []
# https://stackoverflow.com/questions/14631776/calculate-turning-points-pivot-points-in-trajectory-path

"""
The Ramer-Douglas-Peucker algorithm roughly ported from the pseudo-code provided
by http://en.wikipedia.org/wiki/Ramer-Douglas-Peucker_algorithm
"""
from math import sqrt


def distance(a, b):
    return  sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def point_line_distance(point, start, end):
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


def rdp(points, epsilon):
    """Reduces a series of points to a simplified version that loses detail, but
    maintains the general shape of the series.
    """
    dmax = 0.0
    index = 0
    for i in range(1, len(points) - 1):
        d = point_line_distance(points[i], points[0], points[-1])
        if d > dmax:
            index = i
            dmax = d

    if dmax >= epsilon:
        results = rdp(points[:index+1], epsilon)[:-1] + rdp(points[index:], epsilon)
    else:
        results = [points[0], points[-1]]

    return results


def angle(dir):
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
def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global line
    position_ = data.poses
    
    for pose in data.poses:
            # _, _, heading = euler_from_quaternion(
            #     [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            # heading = math.degrees(heading)
            line.append((pose.pose.position.x, pose.pose.position.y))
    rdp_calculate(line)
    return line

def odom_cb(data):
    vehicle_x = data.pose.pose.position.x
    vehicle_y = data.pose.pose.position.y
    import numpy as np
    A = curve_points
    A = np.array(A)
    vehicle_loc = np.array((vehicle_x,vehicle_y))
    distances = np.linalg.norm(A-vehicle_loc, axis=1)
    min_index = np.argmin(distances)
    # print(f"the closest point is {A[min_index]}, at a distance of {distances[min_index]}")
    if distances[min_index] <= 5:
        print("curve")
    else:
        print("Straight line")
#min_angle = np.pi*0.22
min_angle = np.pi*0.05

curve_points=[]

def rdp_calculate(line):
    rdp_line = np.array(rdp(line, 2.0))
    simplified = rdp_line
    sx, sy = simplified.T
    directions = np.diff(simplified, axis=0)
    theta = angle(directions)
    # Select the index of the points with the greatest theta
    # Large theta is associated with greatest change in direction.
    idx = np.where(theta>min_angle)[0]+1
    some_pt = (-0.926711858715862, -31.08365059318021)
    for xx, yy in zip(sx[idx], sy[idx]):
        curve_points.append((xx,yy))
    # return sx,sy,idx
    print(curve_points)

def draw_line(line):
    
    x, y = zip(*line)
    # plt.plot(line)
    fig = plt.figure()
    plt.plot(*zip(*line),color='g')

    line2 = np.array(rdp(line, 2.0))
    x2, y2 = zip(*line2)
    plt.plot(*zip(*line2),color='r')
    simplified = line2
    sx, sy = simplified.T
    directions = np.diff(simplified, axis=0)
    theta = angle(directions)
    # Select the index of the points with the greatest theta
    # Large theta is associated with greatest change in direction.
    idx = np.where(theta>min_angle)[0]+1
    plt.plot(sx[idx], sy[idx], 'ro', markersize = 10, label='turning points')
    sxy = np.vstack((sx[idx], sy[idx])).T #https://stackoverflow.com/a/49322083
    some_pt = (-0.926711858715862, -31.08365059318021)
    ax = fig.gca()
    ax.axis("equal")
    for xx, yy in zip(sx[idx], sy[idx]):
        cir = plt.Circle((xx,yy), 3.0, color='r')
        ax.add_patch(cir)
        
    plt.plot(some_pt[0],some_pt[1], 'o', markersize = 10, label='turning points',color='b')
    plt.show()
#[(0, 0), (2, 0), (2, 2), (0, 2), (0, 0)]

#!/usr/bin/env python

print(len(line))
def listener():
    global line
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    is_path = rospy.wait_for_message("global_gps_path", Path, timeout=5)
    if is_path:
        rospy.Subscriber("global_gps_path", Path, callback)
        rospy.Subscriber("/vehicle/odom", Odometry, odom_cb)
    # draw_line(line)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    draw_line(line)
    