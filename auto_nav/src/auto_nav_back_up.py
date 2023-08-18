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
        self.path_yaw = None
        self.path = None
        self.robot_tie = None
        self._close_idx = None
        self.robot_pose = None
        self.global_traj = None
        self.bboxes = None
        self._traj_manager = TrajectoryManager()
        self._input_traj_manager = TrajectoryManager()

        self.row_length = rospy.get_param("autonav/farm_details/row_length", 9)
        self.row_width = rospy.get_param("autonav/farm_details/row_width", 7)

        self.path_res = rospy.get_param("autonav/farm_details/path_resolution", 0.1)
        self.TURN_ANGLE_TH = 20  #

        # Subscribers
        rospy.Subscriber("/global_gps_trajectory", Trajectory, self.global_traj_callback)
        rospy.Subscriber("/vehicle/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/obstacle_detector/jsk_bboxes", BoundingBoxArray, self.bboxes_callback)

        # Publisher
        self.close_pose_pub = rospy.Publisher("autonav/close_point", PoseStamped, queue_size=1)
        self.front_pose_pub = rospy.Publisher("autonav/front_point", PoseStamped, queue_size=1)
        self.filter_by_path_bbox_pub = rospy.Publisher("autonav/filter_by_path_bbox", BoundingBoxArray, queue_size=1)
        self.filter_by_path_left_bbox_pub = rospy.Publisher("autonav/filter_by_path_left_bbox", BoundingBoxArray,
                                                            queue_size=1)
        self.filter_by_path_right_bbox_pub = rospy.Publisher("autonav/filter_by_path_right_bbox", BoundingBoxArray,
                                                             queue_size=1)

        self.center_line_pub = rospy.Publisher("autonav/center_path", Path, queue_size=1)
        self.center_traj_pub = rospy.Publisher("local_gps_trajectory", Trajectory, queue_size=1)
        self.center_traj_path_pub = rospy.Publisher("autonav/center_path_complete", Path, queue_size=1)

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
        yaw = get_yaw(self.robot_pose.orientation)
        x, y = self.robot_pose.position.x, self.robot_pose.position.y
        x, y = x * cos(yaw), y * sin(yaw)
        self.robot_tie = np.array([x, y])

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

        rate = rospy.Rate(10)

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

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            loop_start_time = time.time()
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

            # finding the turn detection
            try:
                turn_detect_id = self._traj_manager.next_point_within_dist(self._close_idx, self.row_length)
            except:
                rospy.logwarn("could not found the turn_detect_id point")
                rospy.sleep()
                continue

            # increse more value
            if self._close_idx > 10:
                v1 = np.array([self.path[self._close_idx][0] - self.path[self._close_idx - 9][0],
                               self.path[self._close_idx][1] - self.path[self._close_idx - 9][1]])
                v2 = np.array([self.path[turn_detect_id][0] - self.path[turn_detect_id - 9][0],
                               self.path[turn_detect_id][1] - self.path[turn_detect_id - 9][1]])
            else:
                v1 = np.array([self.path[self._close_idx][0] - self.path[self._close_idx - 1][0],
                               self.path[self._close_idx][1] - self.path[self._close_idx - 1][1]])
                v2 = np.array([self.path[turn_detect_id][0] - self.path[turn_detect_id - 1][0],
                               self.path[turn_detect_id][1] - self.path[turn_detect_id - 1][1]])

            angle = math.degrees(GetAngle(v1, v2))
            rospy.logerr(f"angle is {angle}")
            if abs(angle) > self.TURN_ANGLE_TH:
                rospy.logwarn("turn detected, stopping the vehicle")
                traj_msg = Trajectory()
                traj_msg.header.frame_id = "map"
                # trajectory_message.header.stamp = rospy.get_rostime()
                traj_msg.header.stamp = rospy.Time.now()
                # dis = 0.0
                traj_msg.points = self.global_traj.points[self._close_idx: turn_detect_id]
                print("traj_msg.points", len(traj_msg.points))

                self._input_traj_manager.update(traj_msg)
                path = self._input_traj_manager.to_path()
                self.center_lane_pub.publish(path)

                self.center_traj_pub.publish(traj_msg)

                rate.sleep()
                continue

            # close bboxes detection
            try:
                far_point_id = self._traj_manager.next_point_within_dist(self._close_idx, self.row_length)
            except:
                rospy.logwarn("could not found the far_point_id point")
                rate.sleep()
                continue

            if self._close_idx > 10:
                path = self.path[self._close_idx - 9:far_point_id]
            else:
                path = self.path[self._close_idx:far_point_id]

            print("len of global path", len(path))

            if len(self.bboxes.boxes) < 0:
                rospy.logwarn("bounding boxes are not available")
                rate.sleep()
                continue

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

            # distances are looking good.

            # Filter the bouding boxes by robot_radius
            """ 
                Check for distance
                # dis_list = []
                # for self in filtered_bboxes:
                #     # dis = distance_btw_poses(self.robot_pose, box.pose)
                #     if dis > 2*self.row_length:
                #         pass`
            """

            # TODO MENUVER DETECTION
            """
            1> will find the directional angle differnce between  close point and another point which 2* row length
            2> it should tell right or left menuver, 
            3> depending upon the left or right a quarter circle(1/4) will be generated from the bounding box center. 
            4> starting from bounding center to another virtual point in the direction of turn, which is exacly at 1/2 
                of the distance between the tree pairs.
            5> Publish it until the vehicles follows the, the virtual point, 
            6> after that, depending upon the next turn point, switch between offset lane following and then row enter 
                method.



            #TODO TWO MAIN COMPONENTS FOR TURNING
            1> row exit 
            2> row enter
            3> row center
                Use if and else conditions to toggle between them and do the appropriate planning.

            """

            # Now Tree Pair Detection

            '''
            find the close orietation of trees.

            1>  find the orientation of first tree pair
            2>  check for the other tree pairs if the distance and angle are properly. 
            3>  

            '''

            # checking for the tree pairs
            # check if the distance between the first tree and vehicle are less than the 2 * farm width
            # if the
            # c

            # pair detection

            if len(left_bboxes) >= 2 and len(right_bboxes) >= 2:
                first_middle_point = [(left_bboxes[0].pose.position.x + right_bboxes[0].pose.position.x) / 2,
                                      (left_bboxes[0].pose.position.y + right_bboxes[0].pose.position.y) / 2]

                second_middle_point = [(left_bboxes[1].pose.position.x + right_bboxes[1].pose.position.x) / 2,
                                       (left_bboxes[1].pose.position.y + right_bboxes[1].pose.position.y) / 2]
            else:
                rospy.logwarn("Two tree pairs not detected")
                traj_msg.header.frame_id = "map"
                # trajectory_message.header.stamp = rospy.get_rostime()
                traj_msg.header.stamp = rospy.Time.now()
                # dis = 0.0
                traj_msg.points = self.global_traj.points[self._close_idx: turn_detect_id]
                print("traj_msg.points", len(traj_msg.points))

                self._input_traj_manager.update(traj_msg)
                path = self._input_traj_manager.to_path()
                self.center_lane_pub.publish(path)

                self.center_traj_pub.publish(traj_msg)

                rate.sleep()
                continue
                # rate.sleep()
                # continue

            first_pair_dis = distance_btw_poses(left_bboxes[0].pose, right_bboxes[0].pose)
            first_pair_angle = angle_btw_poses(left_bboxes[0].pose, right_bboxes[0].pose)
            print("first_pair_dis", first_pair_dis)
            print("first_pair_angle", math.degrees(normalize_angle(first_pair_angle)))

            # third_middle_point = [(left_bboxes[2].pose.position.x + right_bboxes[2].pose.position.x) / 2,
            #                       (left_bboxes[2].pose.position.y + right_bboxes[2].pose.position.y) / 2]

            path_msg = Path()
            path_msg.header.frame_id = "map"
            #
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = first_middle_point[0]
            pose_stamped.pose.position.y = first_middle_point[1]
            path_msg.poses.append(pose_stamped)
            # pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = second_middle_point[0]
            pose_stamped.pose.position.y = second_middle_point[1]
            path_msg.poses.append(pose_stamped)
            #
            # pose_stamped = PoseStamped()
            # pose_stamped.header.frame_id = "map"
            # pose_stamped.pose.position.x = third_middle_point[0]
            # pose_stamped.pose.position.y = third_middle_point[1]
            # path_msg.poses.append(pose_stamped)
            self.center_line_pub.publish(path_msg)

            first_to_sec_vect = np.array(
                [second_middle_point[0] - first_middle_point[0], second_middle_point[1] - first_middle_point[1]])

            dis = math.hypot(self.robot_pose.position.x - first_middle_point[0],
                             self.robot_pose.position.y - first_middle_point[1])
            print("distance from first center to robot: ", dis)

            # first_to_sec_vect_final = first_to_sec_vect * - 5
            first_to_sec_vect_final = first_to_sec_vect * (1 / np.linalg.norm(first_to_sec_vect)) * -dis
            print("first_to_sec_vect_final", first_to_sec_vect_final)

            close_mid_point = first_middle_point + first_to_sec_vect_final
            print("close_mid_point", close_mid_point)
            path_msg = Path()
            path_msg.header.frame_id = "map"
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = close_mid_point[0]
            pose_stamped.pose.position.y = close_mid_point[1]
            path_msg.poses.append(pose_stamped)

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = first_middle_point[0]
            pose_stamped.pose.position.y = first_middle_point[1]
            path_msg.poses.append(pose_stamped)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = second_middle_point[0]
            pose_stamped.pose.position.y = second_middle_point[1]
            path_msg.poses.append(pose_stamped)

            # pose_stamped = PoseStamped()
            # pose_stamped.header.frame_id = "map"
            # pose_stamped.pose.position.x = third_middle_point[0]
            # pose_stamped.pose.position.y = third_middle_point[1]
            # # path_msg.poses.append(pose_stamped)
            # self.center_line_pub.publish(path_msg)

            # Linear interpolation

            # from  close_mid_close_mid_point to second_middle_point with res of 0.1
            change_x, change_y = second_middle_point[0] - close_mid_point[0], second_middle_point[1] - close_mid_point[
                1]
            angle_line = math.atan2(change_y, change_x)
            dist_line = math.sqrt(change_y ** 2 + change_x ** 2)

            number_of_pts_line = int(dist_line / self.path_res)
            print("number of points", number_of_pts_line)
            intrepolated_points = [[close_mid_point[0], close_mid_point[1]]]

            print("intrepolated_points", intrepolated_points)

            for i in range(int(number_of_pts_line)):
                px = intrepolated_points[-1][0]
                py = intrepolated_points[-1][1]
                px_upd = px + self.path_res * math.cos(angle_line)
                py_upd = py + self.path_res * math.sin(angle_line)
                intrepolated_points.append([px_upd, py_upd])

            traj_msg = Trajectory()
            traj_msg.header.frame_id = "map"
            # trajectory_message.header.stamp = rospy.get_rostime()
            traj_msg.header.stamp = rospy.Time.now()
            dis = 0.0

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
                trajectory_point.longitudinal_velocity_mps = 1  # replace with traj
                traj_msg.points.append(trajectory_point)
            # self._input_traj_manager.update(traj_msg)
            # path = self._input_traj_manager.to_path()
            # self.center_traj_path_pub.publish(path)

            # self.center_traj_pub.publish(traj_msg)

            rospy.loginfo("filtered by path boxes publisher")

            # Least square error method

            if len(left_bboxes) >= 2 and len(right_bboxes) >= 2:

                # for left
                x, y = [], []
                for box in left_bboxes:
                    x.append(box.pose.position.x)
                    y.append(box.pose.position.y)
                A = np.vstack([x, np.ones(len(x))]).T
                m, c = np.linalg.lstsq(A, y, rcond=None)[0]
                # plt.plot(x, m * x + c, 'r', label='Fitted line')
                start_left = [x[0], m * x[0] + c]
                end_left = [x[-1], m * x[-1] + c]

                # For right
                x, y = [], []
                for box in right_bboxes:
                    x.append(box.pose.position.x)
                    y.append(box.pose.position.y)
                A = np.vstack([x, np.ones(len(x))]).T
                m, c = np.linalg.lstsq(A, y, rcond=None)[0]
                # plt.plot(x, m * x + c, 'r', label='Fitted line')
                start_right = [x[0], m * x[0] + c]
                end_right = [x[-1], m * x[-1] + c]

                path_msg = Path()
                path_msg.header.frame_id = "map"
                #
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose.position.x = start_left[0]
                pose_stamped.pose.position.y = start_left[1]
                path_msg.poses.append(pose_stamped)
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose.position.x = end_left[0]
                pose_stamped.pose.position.y = end_left[1]
                path_msg.poses.append(pose_stamped)

                self.left_lane_pub.publish(path_msg)
                rospy.loginfo("left lane published")

                path_msg = Path()
                path_msg.header.frame_id = "map"
                #
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose.position.x = start_right[0]
                pose_stamped.pose.position.y = start_right[1]
                path_msg.poses.append(pose_stamped)
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose.position.x = end_right[0]
                pose_stamped.pose.position.y = end_right[1]
                path_msg.poses.append(pose_stamped)

                self.right_lane_pub.publish(path_msg)
                rospy.loginfo("right lane published")

                # centering line ganeration
                start_center = [(start_right[0] + start_left[0]) / 2, (start_right[1] + start_left[1]) / 2]
                end_center = [(end_right[0] + end_left[0]) / 2, (end_right[1] + end_left[1]) / 2]

                path_msg = Path()
                path_msg.header.frame_id = "map"
                #
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose.position.x = start_center[0]
                pose_stamped.pose.position.y = start_center[1]
                path_msg.poses.append(pose_stamped)
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose.position.x = end_center[0]
                pose_stamped.pose.position.y = end_center[1]
                path_msg.poses.append(pose_stamped)

                self.center_lane_pub.publish(path_msg)
                rospy.loginfo("center lane published")

                change_x, change_y = end_center[0] - start_center[0], end_center[1] - start_center[1]
                angle_line = math.atan2(change_y, change_x)
                dist_line = math.sqrt(change_y ** 2 + change_x ** 2)

                number_of_pts_line = int(dist_line / self.path_res)
                print("number of points", number_of_pts_line)
                intrepolated_points = [[start_center[0], start_center[1]]]

                print("intrepolated_points", intrepolated_points)

                for i in range(int(number_of_pts_line)):
                    px = intrepolated_points[-1][0]
                    py = intrepolated_points[-1][1]
                    px_upd = px + self.path_res * math.cos(angle_line)
                    py_upd = py + self.path_res * math.sin(angle_line)
                    intrepolated_points.append([px_upd, py_upd])

                traj_msg = Trajectory()
                traj_msg.header.frame_id = "map"
                # trajectory_message.header.stamp = rospy.get_rostime()
                traj_msg.header.stamp = rospy.Time.now()
                dis = 0.0

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
                    trajectory_point.longitudinal_velocity_mps = 1  # replace with traj
                    traj_msg.points.append(trajectory_point)
                self._input_traj_manager.update(traj_msg)
                path = self._input_traj_manager.to_path()
                self.center_traj_path_pub.publish(path)

                self.center_traj_pub.publish(traj_msg)

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
