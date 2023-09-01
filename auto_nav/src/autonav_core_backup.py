import math
import sys

import rospy
import numpy as np
from math import cos, sin
import math
# import ros_numpy as rnp
import time
from jsk_recognition_msgs.msg import BoundingBoxArray
from autopilot_msgs.msg import Trajectory, TrajectoryPoint
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, PoseArray, Pose, TransformStamped, PoseStamped
from std_msgs.msg import Float32MultiArray, Header

from visualization_msgs.msg import Marker, MarkerArray

from autopilot_utils.tf_helper import transform_lidar_objects
from autopilot_utils.pose_helper import get_yaw, distance_btw_poses, angle_btw_poses, normalize_angle, yaw_to_quaternion
from autopilot_utils.trajectory_common import TrajectoryManager
from sklearn.linear_model import LinearRegression

# local files
from utils.bboxes_utils import *
from utils.vector2 import Vector2D


def GetAngle(v1, v2):
    return math.atan2(np.cross(v1, v2), np.dot(v1, v2));


class AutoNav:
    def __init__(self):
        self.angle_line = None
        self.row_start_point = None
        self.row_center_vector = None
        self.robot_yaw = None
        self.path_yaw = None
        self.path = None
        self.robot_tie = None
        self._close_idx = None
        self.robot_pose = None
        self.global_traj = None
        self.bboxes = None
        self._traj_manager = TrajectoryManager()
        self._input_traj_manager = TrajectoryManager()

        self.row_lock = False

        self.row_length = rospy.get_param("autonav/farm_details/row_length", 9)
        self.row_width = rospy.get_param("autonav/farm_details/row_width", 7)

        self.path_res = rospy.get_param("autonav/farm_details/path_resolution", 0.1)
        self.TURN_ANGLE_TH = 20  #
        self.LOCK_ANGLE_TH = 25

        # Subscribers
        rospy.Subscriber("/global_gps_trajectory", Trajectory, self.global_traj_callback)
        rospy.Subscriber("/vehicle/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/obstacle_detector/jsk_bboxes", BoundingBoxArray, self.bboxes_callback)

        # Publisher
        self.close_pose_pub = rospy.Publisher("autonav/close_point", PoseStamped, queue_size=1)
        self.turn_detected_pose_pub = rospy.Publisher("autonav/turn_pose", PoseStamped, queue_size=1)

        self.front_pose_pub = rospy.Publisher("autonav/front_point", PoseStamped, queue_size=1)
        self.filter_by_path_bbox_pub = rospy.Publisher("autonav/filter_by_path_bbox", BoundingBoxArray, queue_size=1)
        self.filter_by_path_left_bbox_pub = rospy.Publisher("autonav/filter_by_path_left_bbox", BoundingBoxArray,
                                                            queue_size=1)
        self.filter_by_path_right_bbox_pub = rospy.Publisher("autonav/filter_by_path_right_bbox", BoundingBoxArray,
                                                             queue_size=1)
        self.four_bboxes = rospy.Publisher("autonav/four_coords", BoundingBoxArray,
                                           queue_size=1)

        self.center_line_pub = rospy.Publisher("autonav/center_path", Path, queue_size=1)
        self.center_traj_pub = rospy.Publisher("local_gps_trajectory", Trajectory, queue_size=1)
        self.center_traj_path_pub = rospy.Publisher("autonav/center_trajectory", Path, queue_size=1)

        self.right_lane_pub = rospy.Publisher("autonav/right_lane_sqrt", Path, queue_size=1)
        self.left_lane_pub = rospy.Publisher("autonav/left_lane_sqrt", Path, queue_size=1)
        self.center_lane_pub = rospy.Publisher("autonav/center_lane_sqrt", Path, queue_size=1)

        time.sleep(1)
        self.main_loop()

    def global_traj_callback(self, data):
        self.global_traj = data
        self._traj_manager.update(data)
        self.path = []
        for point in data.points:
            yaw = get_yaw(point.pose.orientation)
            self.path.append([point.pose.position.x, point.pose.position.y, yaw])

    def odom_callback(self, data):
        self.robot_pose = data.pose.pose
        self.robot_yaw = get_yaw(self.robot_pose.orientation)
        self.robot_tie = np.array([cos(self.robot_yaw), sin(self.robot_yaw)])

    def bboxes_callback(self, data):
        if data.header.frame_id == "map":
            bboxes = data
        else:
            # transform bounding boxes to map frame.
            bboxes = transform_lidar_objects(data, "map")
        self.bboxes = bboxes
        # filtered_bboxes = filter_by_area(bboxes)
        # self.bboxes = filtered_bboxes
        # bboxes_list = []
        # for box in filtered_bboxes.boxes:
        #     bboxes_list.append([box.pose.position.x, box.pose.position.y])

    def main_loop(self):

        rate = rospy.Rate(1)

        # wait until all the callbacks are called.
        # rate.sleep()
        while not rospy.is_shutdown():
            if self.bboxes and self._traj_manager.get_len() > 0 and self.robot_pose:
                rospy.loginfo("scan bboxes, global path and robot_pose  are received")
                break
            else:
                rospy.logwarn(
                    f"waiting for data  bboxes :{bool(self.bboxes)}, "
                    f"global traj: {bool(self._traj_manager.get_len() > 0)}, odom: {bool(self.robot_pose)}")
                rate.sleep()

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rospy.loginfo("----------------------------------------------------")
            loop_start_time = time.time()
            if len(self.bboxes.boxes) < 0:
                rospy.logwarn("bounding boxes are not available")
                rate.sleep()
                continue
            # check for the close index on the trajectory
            if self._close_idx is None:
                angle_th = 90
                found, index = self._traj_manager.find_closest_idx_with_dist_ang_thr(self.robot_pose,
                                                                                     self.row_length, angle_th)
                if found:
                    self._close_idx = self._traj_manager.find_close_pose_after_index(self.robot_pose, index,
                                                                                     10)
                else:
                    rospy.logwarn(f"No close point found dist_thr: {self.row_length}, angle_thr: {angle_th}")
                    rate.sleep()
                    continue
            else:
                self._close_idx = self._traj_manager.find_close_pose_after_index(self.robot_pose, self._close_idx, 10)
            print("close ind", self._close_idx)
            self.close_pose_pub.publish(
                PoseStamped(header=Header(frame_id="map"),
                            pose=self._traj_manager.get_traj_point(self._close_idx).pose))

            # # finding the turn detection
            # try:
            #     turn_detect_id = self._traj_manager.next_point_within_dist(self._close_idx, self.row_length)
            # except:
            #     rospy.logwarn("could not found the turn_detect_id point")
            #     rospy.sleep()
            #     continue

            # increse more value

            # find the next turn points

            v1 = np.array([self.path[self._close_idx + 10][0] - self.path[self._close_idx][0],
                           self.path[self._close_idx + 10][1] - self.path[self._close_idx][1]])
            count = 0
            turn_detect_id = None
            rospy.loginfo(f"len of path :{len(self.path)}")
            for i in range(self._close_idx + 10, len(self.path), 10):
                v2 = np.array([self.path[i][0] - self.path[i - 10][0],
                               self.path[i][1] - self.path[i - 10][1]])
                angle = math.degrees(GetAngle(v1, v2))
                if abs(angle) > self.TURN_ANGLE_TH:
                    count = count + 1
                if count > 1:
                    turn_detect_id = i
                    break
            if turn_detect_id:
                self.turn_detected_pose_pub.publish(
                    PoseStamped(header=Header(frame_id="map"),
                                pose=self._traj_manager.get_traj_point(turn_detect_id).pose))
                rospy.loginfo(f"T - turn point detected: {turn_detect_id}")
            else:
                rospy.logwarn(f"F - turn point not detected: {turn_detect_id}")
                continue

            vehicle_dis_to_turn = math.hypot(self.robot_pose.position.x - self.path[turn_detect_id][0],
                                             self.robot_pose.position.y - self.path[turn_detect_id][1])
            rospy.loginfo(f"vehicle_dis_to_turn : {vehicle_dis_to_turn}")
            if vehicle_dis_to_turn <= self.row_length:
                self.row_lock = False
                rospy.logwarn("turn detected, forwarding global path")
                traj_msg = Trajectory()

                traj_msg.header.frame_id = "map"
                # trajectory_message.header.stamp = rospy.get_rostime()
                traj_msg.header.stamp = rospy.Time.now()
                # dis = 0.0
                traj_msg.points = self.global_traj.points[self._close_idx: turn_detect_id]
                print("traj_msg.points", len(traj_msg.points))

                self._input_traj_manager.update(traj_msg)
                path = self._input_traj_manager.to_path()
                self.center_traj_path_pub.publish(path)

                self.center_traj_pub.publish(traj_msg)

                rate.sleep()
                continue

            if self._close_idx > 100:
                path = self.path[self._close_idx - 100:turn_detect_id]
            else:
                path = self.path[self._close_idx - self._close_idx - 1:turn_detect_id]

            print("len of global path", len(path))

            filtered_bboxes, left_bboxes, right_bboxes = filter_bboxes_by_path(path, self.bboxes.boxes,
                                                                               self.row_length)
            bbox_msg = BoundingBoxArray()
            bbox_msg.header.frame_id = "map"
            bbox_msg.boxes = filtered_bboxes
            self.filter_by_path_bbox_pub.publish(bbox_msg)
            # left pub
            bbox_msg.boxes = None
            bbox_msg.boxes = left_bboxes
            self.filter_by_path_left_bbox_pub.publish(bbox_msg)
            # right pub
            bbox_msg.boxes = None
            bbox_msg.boxes = right_bboxes
            self.filter_by_path_right_bbox_pub.publish(bbox_msg)

            # Check for distance
            dis_list = []
            pose = self._traj_manager.get_traj_point(self._close_idx).pose
            for box in filtered_bboxes:
                dis = distance_btw_poses(pose, box.pose)
                dis_list.append(dis)
            print("distance list of filtered_bboxes", dis_list)
            dis_list = []
            for box in left_bboxes:
                dis = distance_btw_poses(pose, box.pose)
                dis_list.append(dis)
            print("distance list of left_bboxes", dis_list)

            dis_list = []
            for box in right_bboxes:
                dis = distance_btw_poses(pose, box.pose)
                dis_list.append(dis)
            print("distance list of right_bboxes", dis_list)

            # checking for robot pose
            left_bboxes_filtered_by_robot = []
            close_left_bbox_in_the_front = None
            close_left_bbox_in_the_back = None
            dis_front, dis_back = 1000.0, 1000.0
            for i in range(len(left_bboxes)):
                bx, by = left_bboxes[i].pose.position.x, left_bboxes[i].pose.position.y
                bbox_vect = [bx - self.robot_pose.position.x, by - self.robot_pose.position.y]
                distance = math.hypot(bx - self.robot_pose.position.x, by - self.robot_pose.position.y)
                if distance >= (self.row_length * self.row_width) / 2:
                    pass
                else:
                    area = np.cross(self.robot_tie, bbox_vect)
                    if area < 0:
                        rospy.loginfo("Bounding box is not on left side")
                    else:
                        left_bboxes_filtered_by_robot.append(left_bboxes[i])

                        if distance < dis_front:
                            proj = np.dot(self.robot_tie, bbox_vect)
                            if proj > 0:
                                close_left_bbox_in_the_front = left_bboxes[i]
                                dis_front = distance

                        if distance < dis_back:
                            proj = np.dot(self.robot_tie, bbox_vect)
                            if proj < 0:
                                close_left_bbox_in_the_back = left_bboxes[i]
                                dis_back = distance

            right_bboxes_filtered_by_robot = []
            close_right_bbox_in_the_front = None
            close_right_bbox_in_the_back = None
            dis_front, dis_back = 1000.0, 1000.0
            for i in range(len(right_bboxes)):
                bx, by = right_bboxes[i].pose.position.x, right_bboxes[i].pose.position.y
                bbox_vect = [bx - self.robot_pose.position.x, by - self.robot_pose.position.y]
                distance = math.hypot(bx - self.robot_pose.position.x, by - self.robot_pose.position.y)
                if distance >= (self.row_length * self.row_width) / 2:
                    pass
                else:
                    area = np.cross(self.robot_tie, bbox_vect)
                    if area > 0:
                        rospy.loginfo("Bounding box is not on right side")
                    else:
                        right_bboxes_filtered_by_robot.append(right_bboxes[i])
                        if distance < dis_front:
                            proj = np.dot(self.robot_tie, bbox_vect)
                            if proj > 0:
                                close_right_bbox_in_the_front = right_bboxes[i]
                                dis_front = distance

                        if distance < dis_back:
                            proj = np.dot(self.robot_tie, bbox_vect)
                            if proj < 0:
                                close_right_bbox_in_the_back = right_bboxes[i]
                                dis_back = distance
            bbox_msg = BoundingBoxArray()
            bbox_msg.header.frame_id = "map"
            if close_left_bbox_in_the_front and close_left_bbox_in_the_back and close_right_bbox_in_the_front \
                    and close_right_bbox_in_the_back:
                rospy.loginfo("four trees are detected")
                bbox_msg.boxes.append(close_left_bbox_in_the_front)
                bbox_msg.boxes.append(close_left_bbox_in_the_back)
                bbox_msg.boxes.append(close_right_bbox_in_the_front)
                bbox_msg.boxes.append(close_right_bbox_in_the_back)
                self.four_bboxes.publish(bbox_msg)

                left_yaw = math.atan2(
                    close_left_bbox_in_the_front.pose.position.y - close_left_bbox_in_the_back.pose.position.y,
                    close_left_bbox_in_the_front.pose.position.x - close_left_bbox_in_the_back.pose.position.x)
                right_yaw = math.atan2(
                    close_right_bbox_in_the_front.pose.position.y - close_right_bbox_in_the_back.pose.position.y,
                    close_right_bbox_in_the_front.pose.position.x - close_right_bbox_in_the_back.pose.position.x)

                rospy.loginfo(f"left_yaw : {left_yaw}, right_yaw: {right_yaw}")
                rospy.loginfo(f"robot: {math.degrees(self.robot_yaw)}")
                print(math.degrees(self.robot_yaw))
                diff_left_yaw = normalize_angle(left_yaw - self.robot_yaw)
                diff_right_yaw = normalize_angle(right_yaw - self.robot_yaw)
                rospy.loginfo(
                    f"diff_left_yaw: {math.degrees(diff_left_yaw)}, diff_right_yaw: {math.degrees(diff_right_yaw)}")
                if abs(math.degrees(diff_left_yaw)) < self.LOCK_ANGLE_TH and \
                        abs(math.degrees(diff_right_yaw)) < self.LOCK_ANGLE_TH:
                    rospy.loginfo("left and right bboxes are in the same directions of vehicle")

                    dis_front = math.hypot(
                        close_left_bbox_in_the_front.pose.position.y - close_right_bbox_in_the_front.pose.position.y,
                        close_left_bbox_in_the_front.pose.position.x - close_right_bbox_in_the_front.pose.position.x)
                    dis_back = math.hypot(
                        close_left_bbox_in_the_back.pose.position.y - close_right_bbox_in_the_back.pose.position.y,
                        close_left_bbox_in_the_back.pose.position.x - close_right_bbox_in_the_back.pose.position.x)
                    rospy.loginfo(f"dis_front : {dis_front}, dis_back : {dis_back}")

                    if dis_front < self.row_length and dis_back < self.row_length:
                        rospy.loginfo("T - Four tress are properly distanced")
                        self.row_start_point = \
                            [
                                (
                                            close_left_bbox_in_the_back.pose.position.x + close_right_bbox_in_the_back.pose.position.x) / 2,
                                (
                                            close_left_bbox_in_the_back.pose.position.y + close_right_bbox_in_the_back.pose.position.y) / 2
                            ]
                        row_end_point = \
                            [
                                (
                                            close_left_bbox_in_the_front.pose.position.x + close_right_bbox_in_the_front.pose.position.x) / 2,
                                (
                                            close_left_bbox_in_the_front.pose.position.y + close_right_bbox_in_the_front.pose.position.y) / 2
                            ]

                        self.angle_line = math.atan2(row_end_point[1] - self.row_start_point[1] , row_end_point[0] - self.row_start_point[0])
                        # dist_line = math.sqrt(row_end_point[1] - self.row_start_point[1] + row_end_point[0] - self.row_start_point[0])

                        # Trajectory publish
                        number_of_pts_line = int(30 / self.path_res)
                        print("number of points", number_of_pts_line)
                        intrepolated_points = [[ self.row_start_point[0],  self.row_start_point[1]]]
                        self.row_lock = True

                        traj_msg = Trajectory()
                        traj_msg.header.frame_id = "map"
                        # trajectory_message.header.stamp = rospy.get_rostime()
                        traj_msg.header.stamp = rospy.Time.now()
                        dis = 0.0

                        for i in range(int(number_of_pts_line)):
                            px = intrepolated_points[-1][0]
                            py = intrepolated_points[-1][1]
                            px_upd = px + self.path_res * math.cos(self.angle_line)
                            py_upd = py + self.path_res * math.sin(self.angle_line)
                            intrepolated_points.append([px_upd, py_upd])

                        for i in range(len(intrepolated_points)):
                            trajectory_point = TrajectoryPoint()
                            trajectory_point.pose.position.x = intrepolated_points[i][0]
                            trajectory_point.pose.position.y = intrepolated_points[i][1]
                            trajectory_point.pose.position.z = 0.3

                            if i + 1 != len(intrepolated_points):
                                yaw = math.atan2(intrepolated_points[i][1] - intrepolated_points[i + 1][1],
                                                 intrepolated_points[i][0] - intrepolated_points[i + 1][0])
                                trajectory_point.pose.orientation = yaw_to_quaternion(yaw)

                            if i == 0:
                                dis = 0.0
                            else:
                                dis += math.hypot(intrepolated_points[i][1] - intrepolated_points[i - 1][1],
                                                  intrepolated_points[i][0] - intrepolated_points[i - 1][0])
                            trajectory_point.accumulated_distance_m = dis
                            trajectory_point.longitudinal_velocity_mps = \
                            self._traj_manager.get_traj_point(self._close_idx).longitudinal_velocity_mps
                            traj_msg.points.append(trajectory_point)
                        self._input_traj_manager.update(traj_msg)
                        path = self._input_traj_manager.to_path()
                        self.center_traj_path_pub.publish(path)

                        self.center_traj_pub.publish(traj_msg)

                    else:
                        rospy.logwarn("F - Four tress are not properly distanced")
                        rospy.loginfo("moving with previous saved points")
                        if self.row_lock:
                            # Trajectory publish
                            number_of_pts_line = int(30 / self.path_res)
                            print("number of points", number_of_pts_line)
                            intrepolated_points = [[self.row_start_point[0], self.row_start_point[1]]]

                            traj_msg = Trajectory()
                            traj_msg.header.frame_id = "map"
                            # trajectory_message.header.stamp = rospy.get_rostime()
                            traj_msg.header.stamp = rospy.Time.now()
                            dis = 0.0

                            for i in range(int(number_of_pts_line)):
                                px = intrepolated_points[-1][0]
                                py = intrepolated_points[-1][1]
                                px_upd = px + self.path_res * math.cos(self.angle_line)
                                py_upd = py + self.path_res * math.sin(self.angle_line)
                                intrepolated_points.append([px_upd, py_upd])

                            for i in range(len(intrepolated_points)):
                                trajectory_point = TrajectoryPoint()
                                trajectory_point.pose.position.x = intrepolated_points[i][0]
                                trajectory_point.pose.position.y = intrepolated_points[i][1]
                                trajectory_point.pose.position.z = 0.3

                                if i + 1 != len(intrepolated_points):
                                    yaw = math.atan2(intrepolated_points[i][1] - intrepolated_points[i + 1][1],
                                                     intrepolated_points[i][0] - intrepolated_points[i + 1][0])
                                    trajectory_point.pose.orientation = yaw_to_quaternion(yaw)

                                if i == 0:
                                    dis = 0.0
                                else:
                                    dis += math.hypot(intrepolated_points[i][1] - intrepolated_points[i - 1][1],
                                                      intrepolated_points[i][0] - intrepolated_points[i - 1][0])
                                trajectory_point.accumulated_distance_m = dis
                                trajectory_point.longitudinal_velocity_mps = \
                                    self._traj_manager.get_traj_point(self._close_idx).longitudinal_velocity_mps
                                traj_msg.points.append(trajectory_point)
                            self._input_traj_manager.update(traj_msg)
                            path = self._input_traj_manager.to_path()
                            self.center_traj_path_pub.publish(path)
                            self.center_traj_pub.publish(traj_msg)

                else:
                    rospy.logwarn("left and right bboxes are in the same not in the directions of vehicle")
                    continue


            rate.sleep()

        # TODO
        """
         1> find the closest point on the gps path.
         2> filter bounding boxes with gps path by tree spacing distance(width).
         3> find the closest left and right bboxes, then detect tree pairs
         4> if tree pair detected:
                center line
            elif only one side:
                offset line.
            else:
                follow the gps path.
         5> menuaver detections -
                -> left
                -> right
         6> Detection of pair of trees
        """
