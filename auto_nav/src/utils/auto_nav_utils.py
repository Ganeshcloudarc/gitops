import random
import numpy as np
import sensor_msgs.point_cloud2 as pcd2
from autopilot_msgs.msg import Trajectory, TrajectoryPoint
from nav_msgs.msg import Path, Odometry
import math
from autopilot_utils.pose_helper import yaw_to_quaternion, distance_btw_poses
from std_msgs.msg import Header
import rospy
from geometry_msgs.msg import Point, PoseArray, Pose, TransformStamped, PoseStamped
from math import cos, sin


class Pose2D:
    def __init__(self, x, y, yaw=0, curv=np.inf, turn_info=0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.curv = curv
        self.turn_info = turn_info

    def __str__(self):
        return f"x: {self.x} y: {self.y} z:{self.yaw} curv:{self.curv} turn_info:{self.turn_info}"

    def __add__(self, other):
        x = self.x + other.x
        y = self.y + other.y
        return Pose2D(x, y)

    def update_curv(self, curv):
        self.curv = curv

    def update_turn_info(self, turn_info):
        # +1 => right turn
        # -1 negative turn
        # 0 No turn
        self.turn_info = turn_info

    def to_numpy(self):
        return np.array([self.x, self.y])


class Line:
    """Ref : https: // www.askpython.com / python - modules / matplotlib / intersection - point - of - two - lines"""

    def __init__(self, slope, const):
        self.m = slope
        self.c = const

    def __str__(self):
        return f"y= {self.m}*x + {self.c}"

    def intersect(self, l2):
        """
        finds the intersection point of lines

        x = (L1.c - L2.c) / (L2.m - L1.m)
        y = L1.m * x + L1.c
        """
        if self.m == l2.m:
            return np.inf, np.inf
        x = (self.c - l2.c) / (l2.m - self.m)
        y = self.m * x + self.c
        return x, y

    def intersct_point_to_line(self, pose2d):
        """
        finds coordinates on point on the path, which is closest to point,
        """
        perp_slope = -1 / self.m
        perp_c = pose2d.y - perp_slope * pose2d.x
        x = (self.c - perp_c) / (perp_slope - self.m)
        y = self.m * x + self.c
        return x, y

    def distance_to_point(self, point):
        if isinstance(point, list) or isinstance(point, tuple):
            dino = pow(self.m * self.m + (-1 * -1), 0.5)
            perp_dis = (self.m * (point[0]) - point[1] + self.c) / dino
            return perp_dis
        elif isinstance(point, Pose2D):
            dino = pow(self.m * self.m + (-1 * -1), 0.5)
            perp_dis = (self.m * (point.x) - point.y + self.c) / dino
            return perp_dis
        else:
            raise TypeError("Only lists,tuples and Pose2D are allowed")

    def heading(self):
        # range from -pi to pi
        # would like to another point at x = 1
        # origin point
        p1 = [0, self.m * 0 + self.c]
        p2 = [1, self.m * 1 + self.c]
        y_diff = p2[1] - p1[1]
        x_diff = p2[0] - p1[0]
        return math.atan2(y_diff, x_diff)

    def inlier(self, points, offset):
        # Find the number of iliner, find the front pointd
        dino = dino = pow(self.m * self.m + (-1 * -1), 0.5)
        numerator = (self.m * (points[:, 0]) - points[:, 1] + self.c)
        dir_perpendicular_dis = numerator / dino
        left_points_ind = np.where((abs(dir_perpendicular_dis) <= offset))[0]
        return left_points_ind


def get_tie(yaw):
    # returns unit vector pointing to yaw
    return np.array([math.cos(yaw), math.sin(yaw)])


def get_2d_rotation_matrix(theta):
    return np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])


def dubins_path_to_ros_path(dubins_path, frame_id="map"):
    path_msg = Path()
    path_msg.header.frame_id = frame_id
    for pt in dubins_path:
        # print("pt", pt)
        pose_st = PoseStamped()
        pose_st.header.frame_id = frame_id
        pose_st.pose.position.x = pt[0]
        pose_st.pose.position.y = pt[1]
        pose_st.pose.orientation = yaw_to_quaternion(pt[2])
        path_msg.poses.append(pose_st)
    return path_msg


def pose_array_to_pose_stamped(pose_arr, frame_id="map"):
    if len(pose_arr) == 3:
        pst = PoseStamped()
        pst.header.frame_id = frame_id
        pose = Pose()
        pst.pose.position.x = pose_arr[0]
        pst.pose.position.y = pose_arr[1]
        pst.pose.orientation = yaw_to_quaternion(pose_arr[2])
        return pst
    elif len(pose_arr) == 2:
        pst = PoseStamped()
        pst.header.frame_id = frame_id
        pose = Pose()
        pst.pose.position.x = pose_arr[0]
        pst.pose.position.y = pose_arr[1]
        pst.pose.orientation.w = 1
        return pst
    else:
        IndexError(f"Input should be of len 2 or 3, given has {len(b)}")


def ganerate_random_pairs(len_of_points):
    ind1 = random.randint(0, len_of_points - 1)
    ind2 = random.randint(0, len_of_points - 1)
    while ind1 == ind2:
        ind2 = random.randint(0, len_of_points - 1)
    return ind1, ind2


def xy_to_pointcloud2(points, frame_id):
    z = np.zeros((points.shape[0], 1))
    points = np.concatenate((points, z), axis=1)
    # try:
    header = Header()
    # header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    scaled_polygon_pcl = pcd2.create_cloud_xyz32(header, points)
    return scaled_polygon_pcl
    # except Exception as error:
    #     rospy.logwarn("not able publish collision points %s", str(error))


def two_points_to_path(point1, point2, frame_id):
    path_msg = Path()
    path_msg.header.frame_id = frame_id
    pose_st = PoseStamped()
    pose_st.header.frame_id = frame_id
    pose_st.pose.position.x = point1[0]
    pose_st.pose.position.y = point1[1]
    pose_st.pose.orientation.z = 1
    path_msg.poses.append(pose_st)

    pose_st = PoseStamped()
    pose_st.header.frame_id = frame_id
    pose_st.pose.position.x = point2[0]
    pose_st.pose.position.y = point2[1]
    pose_st.pose.orientation.z = 1
    path_msg.poses.append(pose_st)

    return path_msg


def getLine(x1, y1, x2, y2, res=0.2):
    rev = False
    if x1 == x2:  # Perfectly horizontal line, can be solved easily
        # return [(x1, i) for i in np.arange(y1, y2, abs(y2 - y1) / (y2 - y1))]
        return [(x1, i) for i in np.arange(y1, y2, res)]

    else:  # More of a problem, ratios can be used instead
        if x1 > x2:  # If the line goes "backwards", flip the positions, to go "forwards" down it.
            rev = True
            x = x1
            x1 = x2
            x2 = x
            y = y1
            y1 = y2
            y2 = y
        slope = (y2 - y1) / (x2 - x1)  # Calculate the slope of the line
        line = []
        i = 0
        while x1 + i < x2:  # Keep iterating until the end of the line is reached
            i += res
            line.append((x1 + i, y1 + slope * i))  # Add the next point on the line
        if rev:
            line.reverse()

        return line  # Finally, return the line!


def path_to_traj(path, speed, resolution=0.2):
    traj_msg = Trajectory()
    traj_msg.header = path.header
    acc_dis = 0
    pose1 = path.poses[0].pose
    pose2 = path.poses[1].pose

    dis = distance_btw_poses(pose1, pose2)
    # print("dis ", dis)
    if dis > resolution:
        # print("resolution", resolution)
        yaw = math.atan2(pose2.position.y - pose1.position.y, pose2.position.x - pose1.position.x)
        quaternion = yaw_to_quaternion(yaw)
        n_points = dis // resolution
        # rospy.logdebug(f"n_points : {n_points}")
        for i in np.arange(0, dis, resolution):
            # print(point)
            traj_point = TrajectoryPoint()
            traj_point.pose.position.x = pose1.position.x + i * cos(yaw)
            traj_point.pose.position.y = pose1.position.y + i * sin(yaw)
            traj_point.pose.orientation = quaternion
            traj_point.longitudinal_velocity_mps = speed
            if i == 0:
                acc_dis = 0.0
            else:
                acc_dis = math.hypot(traj_msg.points[-1].pose.position.x - pose1.position.x,
                                     traj_msg.points[-1].pose.position.y - pose1.position.y)
                # print("acc_dis", acc_dis)
            traj_point.accumulated_distance_m = acc_dis
            traj_msg.points.append(traj_point)
        return traj_msg
    else:
        yaw = math.atan2(pose2.position.y - pose1.position.y, pose2.position.x - pose1.position.x)
        quaternion = yaw_to_quaternion(yaw)

        for i, pose in enumerate(path.poses):
            print(pose)
            traj_point = TrajectoryPoint()
            traj_point.pose.position.x = pose.pose.position.x
            traj_point.pose.position.y = pose.pose.position.y
            traj_point.pose.orientation = quaternion
            traj_point.longitudinal_velocity_mps = speed

            traj_point.accumulated_distance_m = dis
            traj_msg.points.append(traj_point)
        return traj_msg


def GetAngle(v1, v2):
    return math.atan2(np.cross(v1, v2), np.dot(v1, v2))


def min_distance_to_object(pose, corners):
    dist_list = []
    for corner in corners:
        dis = math.hypot(pose.position.x - corner[0], pose.position.y - corner[1])
        dis.append(dist_list)
    return min(dist_list)


def circumradius(xvals, yvals):
    '''
    Calculates the circumradius for three 2D points
    '''
    x1, x2, x3, y1, y2, y3 = xvals[0], xvals[1], xvals[2], yvals[0], yvals[1], yvals[2]
    den = 2 * ((x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2))
    num = ((((x2 - x1) ** 2) + ((y2 - y1) ** 2)) * (((x3 - x2) ** 2) + ((y3 - y2) ** 2)) * (
            ((x1 - x3) ** 2) + ((y1 - y3) ** 2))) ** (0.5)
    if (den == 0):
        print('Failed: points are either collinear or not distinct')
        return 0
    R = num / den
    return R


def curvature(x_data, y_data):
    '''
    Calculates curvature for all interior points
    on a curve whose coordinates are provided
    Input:
        - x_data: list of n x-coordinates
        - y_data: list of n y-coordinates
    Output:
        - curvature: list of n-2 curvature values
    '''
    curvature = []
    for i in range(1, len(x_data) - 1):
        R = circumradius(x_data[i - 1:i + 2], y_data[i - 1:i + 2])
        if (R == 0):
            print('Failed: points are either collinear or not distinct')
            return 0
        curvature.append(1 / R)
    return curvature


if __name__ == "__main__":
    # lane = Line(-10, 0)
    # point = {2, 3}
    # print(lane.distance_to_point(point))
    #
    # print(math.degrees(lane.heading()))
    # (-0.7336240731572223, 16.773232044004445)
    # (1.2663759268427774, 44.453772849218105)]
    line_points = getLine(0, 16, 0, 44, res=0.2)
    print("line points", line_points)
