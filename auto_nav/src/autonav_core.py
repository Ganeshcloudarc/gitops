#!/usr/bin/env python3

try:
    # python3 general packages
    import math
    from math import cos, sin
    import numpy as np

    import copy

    np.float = np.float64
    import ros_numpy
    import rospy
    import tf2_ros
    import time
    import sys
    import random

    # ros messages
    from nav_msgs.msg import Path, Odometry
    from jsk_recognition_msgs.msg import BoundingBoxArray, PolygonArray
    from zed_interfaces.msg import ObjectsStamped, Object
    from geometry_msgs.msg import Point, PoseArray, Pose, TransformStamped, PoseStamped, PolygonStamped
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import Float32MultiArray, Header, Float32
    from sensor_msgs.msg import PointCloud2, LaserScan
    from std_msgs.msg import Float32
    from diagnostic_updater._diagnostic_status_wrapper import DiagnosticStatusWrapper
    from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
    from pilot.msg import vehicle_stop_command as VehicleStopCommand

    # utils
    from laser_geometry import LaserProjection
    import tf2_geometry_msgs
    import sensor_msgs.point_cloud2 as pcd2
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
    from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

    # autopilot related imports
    from autopilot_utils.tf_helper import current_robot_pose, convert_point, transform_cloud, convert_path, \
        convert_pose, yaw_to_quaternion, convert_path, transform_lidar_objects, bbox_to_corners
    from autopilot_utils.pose_helper import distance_btw_poses, get_yaw, normalize_angle
    from autopilot_utils.trajectory_smoother import TrajectorySmoother
    from vehicle_common.vehicle_config import vehicle_data
    from autopilot_utils.trajectory_common import TrajectoryManager
    from autopilot_msgs.msg import Trajectory, TrajectoryPoint
    # from velocity_planner import VelocityPlanner
    from utils.auto_nav_utils import Pose2D, ganerate_random_pairs, xy_to_pointcloud2, two_points_to_path, path_to_traj, \
        GetAngle, min_distance_to_object, circumradius, Line, get_tie, pose_array_to_pose_stamped, \
        dubins_path_to_ros_path
    from autopilot_utils.occ_grid_helper import OccupancyGridManager
    # python packages
    from sklearn.linear_model import LinearRegression
    import dubins
    from utils.dwa_path_candidate_generator import DwaPathGenerator
    from utils.bboxes_utils import inside_radius
    from utils.polygon_collision_check import PolygonCheck



except Exception as e:
    import rospy

    rospy.logerr("Module error %s", str(e))
    exit()

OK = DiagnosticStatus.OK
ERROR = DiagnosticStatus.ERROR
WARN = DiagnosticStatus.WARN
STALE = DiagnosticStatus.STALE

"""
# formulas

A=y1−y2
B=x2−x1
C=x1y2−x2y1

C2 = C + self.row_spacing * math.sqrt(A * A + B * B) # to compute right line.

C2 = C - self.row_spacing * math.sqrt(A * A + B * B) # to compute left line.       

y-intercept = (c / b) 
slope = -(a / b)             
"""
def points_to_path(points, frame_id="map"):
    path_msg = Path()
    path_msg.header.frame_id = frame_id
    for point in points:
        pose_st = PoseStamped()
        pose_st.header.frame_id = frame_id
        pose_st.pose.position.x = point[0]
        pose_st.pose.position.y = point[1]
        pose_st.pose.orientation = yaw_to_quaternion(point[2])
        path_msg.poses.append(pose_st)
    return path_msg


class AutoNav:
    def __init__(self):
        # laser related
        self.bboxes = None
        self.laser_data = None
        self.final_center_line = None
        self.center_line_heading = None
        self.center_line = None
        self.center_line_final = None
        self.robot_tie = None
        self.robot_yaw = None
        self.row_lock = False
        self.all_points = None
        self._cos_sin_map = None
        self.max_inlier_count_right = None
        self.max_inlier_count_left = None
        self.scan_in_frame_id = None
        self._cos_sin_map_right = None
        self._cos_sin_map_left = None
        self.right_points = None
        self.left_points = None
        self.path = []
        self._close_idx = None
        self._traj_in = None
        self._traj_end_index = None

        self.laser_np_3d = None
        self.robot_speed = None
        self.laser_np_2d = None
        self.traj_end_index = None
        # self.index_old = None

        self.laser_data_in_time = None
        self.scan_data_received = None
        self.pc_np = None
        self.tree = None
        self.robot_pose = None
        self.robot_head_pose = Pose()
        self.odom_data_in_time = None
        self._traj_manager = TrajectoryManager()
        self._input_traj_manager = TrajectoryManager()
        self.laser_geo_obj = LaserProjection()
        # self._a_max, self._slow_speed, self._stop_line_buffer = 1, 0.5, 3.5
        self.line_coefficient = []
        self.line_coefficient_list = []
        # ros parameters for Obstacle stop planner
        # TODO accept form patrol application if available else take from patrol params.
        self.row_spacing = rospy.get_param("auto_nav/farm_details/row_spacing", 8)

        self.ransac_tolerance = rospy.get_param("auto_nav/ransac/tolerance", 2)
        self.ransac_max_iterations = rospy.get_param("auto_nav/ransac/max_iterations", 200)
        self.moving_avg_filter_enable = rospy.get_param("auto_nav/moving_avarage_filter/enable", True)
        self.window_size = rospy.get_param("auto_nav/moving_avarage_filter/window_size", 10)
        self.use_previous_line = rospy.get_param("auto_nav/use_previous_line", False)

        self.latch_average_filter_enable = rospy.get_param("auto_nav/latch_avarage_filter/enable", False)
        self.latch_average_filter_window_size = rospy.get_param("auto_nav/latch_avarage_filter/window_size", 10)

        self.forward_collision_dis = rospy.get_param("auto_nav/forward_collision_dis", 10)
        self.obs_stop_dis_on_center_line = rospy.get_param("auto_nav/obstacle_dist_th", 7)

        # Thresoulds
        self.heading_diff_th = rospy.get_param("auto_nav/heading_diff_th", 5)  # in degrees
        self.heading_diff_th = self.heading_diff_th * np.pi / 180
        self.center_line_robot_offset_th = rospy.get_param("auto_nav/center_line_robot_offset_th", 0.5)  # in meters
        self._time_to_wait_at_ends = 2
        self.regulate_speed_enabled = rospy.get_param("/auto_nav/regulate_speed_enable", True)
        self.collision_detect_enabled = rospy.get_param("/auto_nav/collision_detection_enable", True)

        self.mission_continue = rospy.get_param("/auto_nav/mission_continue", True)

        self._TIME_OUT_FROM_LASER = 2  # in secs
        self._TIME_OUT_FROM_ODOM = 2
        self.TURN_ANGLE_TH = 20  # for turn detection

        # ros publishers
        local_traj_in_topic = rospy.get_param("auto_nav/traj_out", "local_gps_trajectory")
        self.diagnostics_pub = rospy.Publisher("/autonav_diagnostics", DiagnosticArray, queue_size=1)
        self.pilot_stop_command_pub = rospy.Publisher("/vehicle/stop_command", VehicleStopCommand, queue_size=1)
        self.center_traj_pub = rospy.Publisher(local_traj_in_topic, Trajectory, queue_size=10)
        self.center_traj_path_pub = rospy.Publisher("local_gps_path", Path, queue_size=10)
        self.close_pose_pub = rospy.Publisher("/auto_nav/close_point", PoseStamped, queue_size=1)
        self.front_pose_pub = rospy.Publisher("/auto_nav/front_point", PoseStamped, queue_size=1)
        self.turn_start_pose_pub = rospy.Publisher("/auto_nav/turn_start_pose", PoseStamped, queue_size=1)
        self.turn_end_pose_pub = rospy.Publisher("/auto_nav/turn_end_pose", PoseStamped, queue_size=1)
        self.mission_count_pub = rospy.Publisher('/mission_count', Float32, queue_size=2, latch=True)
        self.path_percent_publisher = rospy.Publisher("/osp_path_percentage",Float32, queue_size=1 ,latch=True)
        self.count_mission_repeat = 0
        self.vibration_path_msg = Path()

        self.pub_debug_topics = rospy.get_param("/auto_nav/pub_debug_topics", True)
        if self.pub_debug_topics:
            self.all_points_pub = rospy.Publisher('/auto_nav/all_points', PointCloud2,
                                                  queue_size=10)
            self.map_frame_all_points_pub = rospy.Publisher('/auto_nav/map_frame_all_points', PointCloud2,
                                                            queue_size=10)
            self.left_points_pub = rospy.Publisher('/auto_nav/left_points', PointCloud2,
                                                   queue_size=10)
            self.right_points_pub = rospy.Publisher('/auto_nav/right_points', PointCloud2,
                                                    queue_size=10)
            self.center_line_inliers_pub = rospy.Publisher('/auto_nav/center_line_inliers', PointCloud2,
                                                           queue_size=10)
            self.center_line_obstacle_points = rospy.Publisher('/auto_nav/center_line_obstacle_points', PointCloud2,
                                                               queue_size=10)
            self.left_points_inlier_pub = rospy.Publisher('/auto_nav/left_points_inliers', PointCloud2,
                                                          queue_size=10)
            self.right_points_inlier_pub = rospy.Publisher('/auto_nav/right_points_inliers', PointCloud2,
                                                           queue_size=10)
            self.left_random_path = rospy.Publisher('/auto_nav/left_random_path', Path,
                                                    queue_size=10)
            self.right_random_path = rospy.Publisher('/auto_nav/right_random_path', Path,
                                                     queue_size=10)

            self.turn_detected_pose_pub = rospy.Publisher("auto_nav/turn_pose", PoseStamped, queue_size=1)

            self.center_points_vibration_test_pub = rospy.Publisher('/auto_nav/vibration_test', Path,
                                                                    queue_size=10)
            self.global_path_linear = rospy.Publisher('/auto_nav/global_path_linear', Path,
                                                      queue_size=10, latch=True)

            self.before_filter = rospy.Publisher('/auto_nav/before_filter', Path,
                                                 queue_size=10, latch=True)
            self.after_filter = rospy.Publisher('/auto_nav/after_filter', Path,
                                                queue_size=10, latch=True)
            self.slope_pub = rospy.Publisher("/auto_nav/slope_pub", Float32, queue_size=10)
            self.left_intercept_pub = rospy.Publisher("/auto_nav/left_intercept_pub", Float32, queue_size=10)
            self.right_intercept_pub = rospy.Publisher("/auto_nav/right_intercept_pub", Float32, queue_size=10)
            self.dwa_marker_pub = rospy.Publisher("/auto_nav/dwa_all_paths", MarkerArray, queue_size=10)
            self.dwa_collision_free_paths_pub = rospy.Publisher("/auto_nav/dwa_collision_free", MarkerArray, queue_size=10)
            self.dwa_collision_free_and_close_to_line_pub = rospy.Publisher("/auto_nav/dwa_collision_free_and_close_to_line", Path, queue_size=10)
            self.collision_point_center_line_pub = rospy.Publisher("auto_nav/collision_point_center_line", PoseStamped, queue_size=1)
            self.enlarged_bboxes_pub = rospy.Publisher("enlarged_bboxes", PolygonArray, queue_size=1, latch=True)

        # Turnings related

        self.increment_for_curv = rospy.get_param("auto_nav/turnings/incement_index_for_curvature", 20)
        print("self.increment_for_curv", self.increment_for_curv)
        self.min_turning_radius = rospy.get_param("auto_nav/turnings/min_tunings_radius", 7)
        self.min_turn_radius_to_check_turn = rospy.get_param("auto_nav/turnings/radius_to_check_turn", 15)
        self.dedug_turn = rospy.get_param("auto_nav/turnings/debug", True)

        if self.dedug_turn:
            self.turn_path_left_pub = rospy.Publisher('/auto_nav/turn_path_left_pub', Path,
                                                      queue_size=10, latch=True)
            self.turn_path_right_pub = rospy.Publisher('/auto_nav/turn_path_right_pub', Path,
                                                       queue_size=10, latch=True)
            self.automatic_turns_left_pub = rospy.Publisher('/auto_nav/automatic_turns_left', Path,
                                                            queue_size=10, latch=True)
            self.automatic_turns_right_pub = rospy.Publisher('/auto_nav/automatic_turns_right', Path,
                                                             queue_size=10, latch=True)
            self.curv_pub = rospy.Publisher("/auto_nav/global_curv", MarkerArray, queue_size=1, latch=True)

        # ros subscribers
        global_traj_topic = rospy.get_param("auto_nav/traj_in", "global_gps_trajectory")
        scan_topic = rospy.get_param("auto_nav/scan_topic", "laser_scan")
        local_map_scan_topic = rospy.get_param("auto_nav/local_cloud_scan", "local_cloud_laser_scan")
        odom_topic = rospy.get_param("patrol/odom_topic", "vehicle/odom")
        rospy.Subscriber(global_traj_topic, Trajectory, self.global_traj_callback)
        rospy.Subscriber(local_map_scan_topic, LaserScan, self.local_map_scan_callback, queue_size=1)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber("/obstacle_detector/jsk_bboxes", BoundingBoxArray, self.bboxes_callback)

        self.offset_to_add_center_line_width = rospy.get_param("offset_to_add_center_line_width", 0.1)
        # initialise Dwa planner
        wheel_base = vehicle_data.dimensions.wheel_base
        steering_angle_max = vehicle_data.motion_limits.max_steering_angle
        steering_inc = 1
        max_path_length = 5
        path_resolution = 0.1
        self.dwa_path_gen = DwaPathGenerator(wheel_base, steering_angle_max, steering_inc, max_path_length, path_resolution)
        self.cost_map = OccupancyGridManager("/costmap_node/costmap/costmap", True)
        self.diagnostics_arr = DiagnosticArray()
        self.diagnostics_arr.header.frame_id = "base_link"
        self.diagnose = DiagnosticStatusWrapper()
        self.diagnose.name = rospy.get_name()

        self.main_loop()
        rospy.signal_shutdown("auto_nav node killed")
        # self.main_loop_ransac()
        # TODO
        # keep signal shutdowm

    def daignose_publish(self):
        self.diagnostics_arr.status = []
        self.diagnostics_arr.status.append(self.diagnose)
        self.diagnostics_pub.publish(self.diagnostics_arr)

    def main_loop(self):

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.diagnose.clearSummary()
            self.diagnose.values = []

            if self.scan_data_received and self._traj_manager.get_len() > 0 and self.robot_pose:
                rospy.loginfo("scan data, global path and robot_pose  are received")
                self.diagnose.summary(OK, "Data received on all the sensors")
                self.daignose_publish()
                break
            else:
                rospy.logwarn(
                    f"waiting for data  scan :{self.scan_data_received}, global traj: {self._traj_manager.get_len() > 0}, odom: {bool(self.robot_pose)}")
                self.diagnose.summary(WARN, f"waiting for data  scan :{self.scan_data_received}, global traj: {self._traj_manager.get_len() > 0}, odom: {bool(self.robot_pose)}")
                rospy.logwarn("No received on all the sensors")
                self.daignose_publish()
                rate.sleep()

        rate = rospy.Rate(10)
        count = 0
        m_list = []
        c_list = []
        vehicle_stop_msg = VehicleStopCommand()
        vehicle_stop_msg.node = rospy.get_name()
        while not rospy.is_shutdown():
            count += 1
            self.diagnose.clearSummary()
            self.diagnose.values = []
            self.diagnostics_arr.status = []


            # new method to publish line
            # new method to publish line
            robot_loc = Pose2D(self.robot_pose.position.x, self.robot_pose.position.y, self.robot_yaw)
            robot_loc_np = robot_loc.to_numpy()

            rospy.logdebug(f"yaw : {self.robot_yaw} , degress :{math.degrees(self.robot_yaw)} ")
            rospy.logdebug(f"robot_loc_np:{robot_loc}")
            robot_loc_backward = Pose2D(self.robot_pose.position.x - np.cos(self.robot_yaw) * 30,
                                        self.robot_pose.position.y - np.sin(self.robot_yaw) * 30)
            rospy.logdebug(f"robot_loc_backward:{robot_loc_backward}")

            robot_loc_backward_base_link = Pose2D(- np.cos(self.robot_yaw) * 30, -np.sin(self.robot_yaw) * 30)
            robot_loc_backward_base_link = Pose2D(-30, 0)
            rospy.logdebug(f"robot_loc_backward_base_link:{robot_loc_backward_base_link}")
            robot_loc_forward = Pose2D(self.robot_pose.position.x + np.cos(self.robot_yaw) * 30,
                                       self.robot_pose.position.y + np.sin(self.robot_yaw) * 30)
            rospy.logdebug(f"robot_loc_backward:{robot_loc_forward}")
            robot_loc_forward_base_link = Pose2D(np.cos(self.robot_yaw) * 30, np.sin(self.robot_yaw) * 30)
            robot_loc_forward_base_link = Pose2D(30, 0)
            rospy.logdebug(f"robot_loc_forward_base_link:{robot_loc_forward_base_link}")

            loop_start_time = time.time()
            # checks whether data from sensors are updated.
            if loop_start_time - self.odom_data_in_time > self._TIME_OUT_FROM_ODOM:
                rospy.logwarn(
                    f"No update on odom (robot position) from last {loop_start_time - self.laser_data_in_time} seconds")
                self.diagnose.summary(ERROR, f"No update on odom (robot position) from last {loop_start_time - self.laser_data_in_time} seconds")
                self.daignose_publish()

                rate.sleep()
                continue
            if loop_start_time - self.laser_data_in_time > self._TIME_OUT_FROM_LASER:
                rospy.logwarn(f"No update on laser data from last {loop_start_time - self.laser_data_in_time} seconds")
                self.diagnose.summary(ERROR,
                                      f"No update on laser data from last {loop_start_time - self.laser_data_in_time} seconds")
                self.daignose_publish()
                rate.sleep()
                continue

            # check for the close index on the trajectory
            if self._close_idx is None:
                angle_th = 90
                found, index = self._traj_manager.find_closest_idx_with_dist_ang_thr(self.robot_pose,
                                                                                     self.row_spacing / 2, angle_th)
                if found:
                    self._close_idx = index
                else:
                    rospy.logwarn(f"No close point found dist_thr: {self.row_spacing / 2}, angle_thr: {angle_th}")
                    self.diagnose.summary(ERROR,
                                          f"No close point found dist_thr: {self.row_spacing / 2}, angle_thr: {angle_th}")
                    self.daignose_publish()
                    rate.sleep()
                    continue
            else:
                self._close_idx = self._traj_manager.find_close_pose_after_index(self.robot_pose, self._close_idx, 10)
            # print(self._traj_manager.get_traj_point(self._close_idx))
            # self.close_pose_pub.publish(
            #     PoseStamped(header=Header(frame_id="map"),
            #                 pose=self._traj_manager.get_traj_point(self._close_idx).pose))
            # rate.sleep()

            # check for mission complete
            path_percent = (self._traj_in.points[self._close_idx].accumulated_distance_m /
                            self._traj_in.points[-1].accumulated_distance_m) * 100
            self.path_percent_publisher.publish(path_percent)
            if path_percent > 95.0 and distance_btw_poses(self.robot_pose,
                                                          self._traj_in.points[-1].pose) <= self.row_spacing / 2:
                self.count_mission_repeat += 1
                rospy.loginfo(' Mission count %s ', self.count_mission_repeat)
                self.mission_count_pub.publish(self.count_mission_repeat)
                if self.mission_continue:
                    self._close_idx = 1
                    time.sleep(self._time_to_wait_at_ends)
                    rate.sleep()
                    continue
                else:
                    rate.sleep()
                    rospy.logwarn("mission completed")
                    self.diagnose.summary(WARN, "mission completed")
                    self.daignose_publish()
                    break

            # turn detection
            forward_path_idx = self._traj_manager.next_point_within_dist(self._close_idx, 1 * self.row_spacing)
            self.front_pose_pub.publish(
                PoseStamped(header=Header(frame_id="map"),
                            pose=self._traj_manager.get_traj_point(forward_path_idx).pose))

            # TODO new method to detect turns
            # with bag file working.
            # count = 0.0
            turn_detect_id = None
            for i in range(self._close_idx, forward_path_idx):
                if self.path[i].turn_info != 0:
                    dist = math.hypot(self.robot_pose.position.y - self.path[i].y,
                                      self.robot_pose.position.x - self.path[i].x)
                    if dist <= 3:  # to detect turn at vehicle front.
                        turn_detect_id = i

            if turn_detect_id:
                rospy.logwarn(
                    f"T Turn detected at from close point : {(self._traj_in.points[turn_detect_id].accumulated_distance_m - self._traj_in.points[self._close_idx].accumulated_distance_m)} meters")
                self.diagnose.summary(WARN, f"T Turn detected at from close point : {(self._traj_in.points[turn_detect_id].accumulated_distance_m - self._traj_in.points[self._close_idx].accumulated_distance_m)} meters")
                self.daignose_publish()
                self.turn_detected_pose_pub.publish(
                    PoseStamped(header=Header(frame_id="map"),
                                pose=self._traj_manager.get_traj_point(turn_detect_id).pose))

                vehicle_dis_to_turn = math.hypot(self.robot_pose.position.x - self.path[turn_detect_id].x,
                                                 self.robot_pose.position.y - self.path[turn_detect_id].y)
                rospy.loginfo(f"vehicle_dis_to_turn : {vehicle_dis_to_turn}")
                # if vehicle_dis_to_turn <= self.row_spacing:
                rospy.loginfo(f"T - turn point detected: {turn_detect_id}")
                rospy.logwarn("turn detected, forwarding global path")
                traj_msg = Trajectory()

                traj_msg.header.frame_id = "map"
                # trajectory_message.header.stamp = rospy.get_rostime()
                traj_msg.header.stamp = rospy.Time.now()
                # dis = 0.0
                traj_msg.points = self._traj_in.points[self._close_idx: forward_path_idx]
                # print("traj_msg.points", len(traj_msg.points))
                self._input_traj_manager.update(traj_msg)
                path = self._input_traj_manager.to_path()
                self.center_traj_path_pub.publish(path)

                self.center_traj_pub.publish(traj_msg)
                self.line_coefficient_list = []
                m_list = []
                c_list = []
                self.row_lock = False
                self.final_center_line = None
                rate.sleep()
                count = 0

                continue
                # else:
                #     rospy.logwarn("turn detected ")

            # TEST CODE FOR TURNNGS
            # automatic turns based on dubins curves
            # decide on left turn or right turn
            # Find the starting and end point of turn
            # decide on minimum turning radius
            # use the dubins curves to connect start and point.
            # convert into traj and path and publish
            """ Uncomment after full completion
            min_turn_radius = 3.5
            if self.center_line:
                rospy.logdebug("At tunings testing part")
                start_point = self.center_line.intersct_point_to_line(self.path[forward_path_idx])
                start_point_yaw = self.center_line_heading

                # if self.path[turn_detect_id].turn_info == 1:
                self.turn_start_pose_pub.publish(
                    pose_array_to_pose_stamped([start_point[0], start_point[1], start_point_yaw]))
                rospy.logdebug(" left turn start pose published")
                theta = np.pi / 2
                rot = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
                vect = get_tie(start_point_yaw) * self.row_spacing
                dot_pro = np.dot(rot, vect)
                turn_end_point = [start_point[0] + dot_pro[0], start_point[1] + dot_pro[1]]
                turn_end_point_yaw = normalize_angle(start_point_yaw + np.pi)
                self.turn_end_pose_pub.publish(pose_array_to_pose_stamped([turn_end_point[0], turn_end_point[1],
                                                                           turn_end_point_yaw]))

                rospy.logdebug("left turn end pose published")
                dubins_path = dubins.path([start_point[0], start_point[1], start_point_yaw],
                                          [turn_end_point[0], turn_end_point[1], turn_end_point_yaw], min_turn_radius,
                                          0)
                step_size = 0.2
                dubins_path = dubins_path.sample_many(step_size)[0]

                path_msg = dubins_path_to_ros_path(dubins_path)
                self.automatic_turns_left_pub.publish(path_msg)
                rospy.logdebug("left automartic turn is published")

                # Turn point another
                theta = -np.pi / 2
                rot = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
                vect = get_tie(start_point_yaw) * self.row_spacing
                dot_pro = np.dot(rot, vect)
                turn_end_point = [start_point[0] + dot_pro[0], start_point[1] + dot_pro[1]]
                turn_end_point_yaw = normalize_angle(start_point_yaw + np.pi)
                self.turn_end_pose_pub.publish(pose_array_to_pose_stamped([turn_end_point[0], turn_end_point[1],
                                                                           turn_end_point_yaw]))
                rospy.logdebug("right turn end pose published")
                dubins_path = dubins.path([start_point[0], start_point[1], start_point_yaw],
                                          [turn_end_point[0], turn_end_point[1], turn_end_point_yaw], min_turn_radius,
                                          3)
                step_size = 0.2
                dubins_path = dubins_path.sample_many(step_size)[0]

                path_msg = dubins_path_to_ros_path(dubins_path)
                self.automatic_turns_right_pub.publish(path_msg)
                rospy.logdebug("automartic turn is published")
            """

            # decide on left and right points
            close_pose = self._traj_manager.get_traj_point(self._close_idx).pose
            forward_pose = self._traj_manager.get_traj_point(forward_path_idx).pose

            self.close_pose_pub.publish(
                PoseStamped(header=Header(frame_id="map"),
                            pose=close_pose))
            self.front_pose_pub.publish(
                PoseStamped(header=Header(frame_id="map"),
                            pose=forward_pose))
            # ref https://www.includehelp.com/python/distance-between-point-and-a-line-from-two-points-in-numpy.aspx

            # p1 to be close point to the robot
            # p2 to be far point to robot

            p1 = np.array([close_pose.position.x, close_pose.position.y])
            p2 = np.array([forward_pose.position.x, forward_pose.position.y])
            # replace with center line
            # if self.center_line and self.use_prev_center_line:
            #     rospy.logdebug("Taking center line points to clasify left and right")
            #     p1 = np.array(self.center_line.intersct_point_to_line(robot_loc))
            #     p2 = np.array(self.center_line.intersct_point_to_line(robot_loc_forward))

            if self.laser_np_2d is None:
                rospy.logwarn("No update on laser scan")
                self.diagnose.summary(ERROR, "No update on laser scan")
                self.daignose_publish()
                rate.sleep()
                continue
            p3 = self.laser_np_2d
            # print("laser scan",self.laser_np_2d)
            dir_perpendicular_dis = np.cross(p2 - p1, p1 - p3) / np.linalg.norm(p2 - p1)
            left_points_ind = np.where(np.logical_and(dir_perpendicular_dis > 0,
                                                      dir_perpendicular_dis < self.row_spacing))  # np.logical_and(res >0, res> 0.5)
            # print("left_points_ind", left_points_ind)
            right_points_ind = np.where(
                np.logical_and(dir_perpendicular_dis < 0, dir_perpendicular_dis > -self.row_spacing))
            # print("right_points_ind", right_points_ind)
            laser_np_2d_path_close_points = np.where(abs(dir_perpendicular_dis) < self.row_spacing)
            try:
                self.laser_np_2d_path_close_points = np.take(self.laser_np_2d, laser_np_2d_path_close_points, axis=0)[0]
                self.left_points = np.take(self.laser_np_2d, left_points_ind, axis=0)[0]
                self.right_points = np.take(self.laser_np_2d, right_points_ind, axis=0)[0]
                # print("self.left_points ", self.left_points)
            except IndexError:
                rospy.logerr("Index error ")
                print("left_points_ind", left_points_ind)
                print("right_points_ind", right_points_ind)
                print("laser scan", self.laser_np_2d)
                continue

            self.all_points_pub.publish(xy_to_pointcloud2(self.laser_np_2d_path_close_points, "map"))
            rospy.logdebug(f"all points are published")
            self.left_points_pub.publish(xy_to_pointcloud2(self.left_points, "map"))
            self.right_points_pub.publish(xy_to_pointcloud2(self.right_points, "map"))
            # continue
            rospy.loginfo("left and right points are published")
            # rate.sleep()
            # continue

            # RANSAC METHOD STARTS
            maximum_inliers_count_l = 0
            maximum_inliers_count_r = 0

            # inlier_threshould =

            # if self.prev_line_fit

            left_points_ind_final = []
            right_points_ind_final = []
            random_final_left_points = []
            random_final_right_points = []

            print(self.left_points.shape)
            print(self.right_points.shape)
            if self.left_points.shape[0] == 0 or self.right_points.shape[0] == 0:
                rospy.logwarn("No Left points and right points")
                self.diagnose.summary(ERROR, "No Left points and right points")
                self.daignose_publish()
                continue

            for i in range(0, self.ransac_max_iterations):
                # rospy.logdebug("on left side")
                ind1, ind2 = ganerate_random_pairs(self.left_points.shape[0])
                p1_left = np.array([self.left_points[ind1][0], self.left_points[ind1][1]])
                p2_left = np.array([self.left_points[ind2][0], self.left_points[ind2][1]])
                p3_left = self.left_points  # self.laser_np_2d_path_close_points
                # p3 = self.left_points #self.laser_np_2d_path_close_points

                dir_perpendicular_dis = np.cross(p2_left - p1_left, p1_left - p3_left) / np.linalg.norm(
                    p2_left - p1_left)
                left_points_ind = np.where((abs(dir_perpendicular_dis) <= self.ransac_tolerance))[0]
                theta = np.pi / 2
                rot = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
                v1 = p2_left - p1_left
                vect = v1 * (1 / np.linalg.norm(v1)) * self.row_spacing
                dot_pro = np.dot(rot, vect)
                p1_right = p1_left + dot_pro
                p2_right = p2_left + dot_pro
                # rospy.loginfo(f"maximum_inliers_count_l : {maximum_inliers_count_l},len(left_points_ind) : {len(left_points_ind)}, maximum_inliers_count_r : {maximum_inliers_count_r},len(right_points_ind) : {len(right_points_ind)}")

                p3_right = self.right_points  # self.laser_np_2d_path_close_points  # finding inliers on all points

                dir_perpendicular_dis = np.cross(p2_right - p1_right, p1_right - p3_right) / np.linalg.norm(
                    p2_right - p1_right)
                right_points_ind = np.where((abs(dir_perpendicular_dis) <= self.ransac_tolerance))[0]

                # print("right_points_ind", right_points_ind, type(right_points_ind))
                # print("left_points_ind",left_points_ind, type(left_points_ind) )
                # print("left_points_ind", type(left_points_ind))
                # print(left_points_ind[0])
                # print("len(left_points_ind)", len(left_points_ind))
                # rospy.loginfo(f"maximum_inliers_count_l : {maximum_inliers_count_l},len(left_points_ind) : {len(left_points_ind)}, maximum_inliers_count_r : {maximum_inliers_count_r},len(right_points_ind) : {len(right_points_ind)}")
                if len(left_points_ind) > maximum_inliers_count_l and \
                        len(right_points_ind) > maximum_inliers_count_r:
                    maximum_inliers_count_l = len(left_points_ind)
                    maximum_inliers_count_r = len(right_points_ind)
                    # point1 = Pose2D(p1[0], p1[1])
                    # point2 = Pose2D(p2[0], p2[1])

                    # p1_right_ = p1_right
                    # p2_right_ = p2_right
                    random_final_left_points = [p1_left, p2_left]
                    random_final_right_points = [p1_right, p2_right]

                    if p2_left[0] - p1_left[0] != 0:
                        m = (p2_left[1] - p1_left[1]) / (p2_left[0] - p1_left[0])
                        # line_heading = math.atan2((point2.y - point1.y), (point2.x - point1.x))
                    else:
                        m = 1000000
                        # line_heading = math.atan2((point2.y - point1.y), (point2.x - point1.x))

                    c_l = p1_left[1] - m * p1_left[0]  # y - mx
                    c_r = p1_right[1] - m * p1_right[0]
                    coefficents = [m, c_l, c_r]

                    # A = point1.y - point2.y
                    # B = point2.x - point1.x
                    # C = point1.x * point2.y - point2.x * point1.y
                    #
                    # C2 = C - self.row_spacing * math.sqrt(A * A + B * B)
                    # coefficents = [A, B, C, C2]
                    # coefficents_m = [-A / B, (C / B + C2 / B) / 2]
                    # coefficents_m = [-A / B, C / B]
                    left_points_ind_final = left_points_ind
                    right_points_ind_final = right_points_ind

                # on right side
                # rospy.logdebug("on right side")
                ind1, ind2 = ganerate_random_pairs(self.right_points.shape[0])
                p1_right = np.array([self.right_points[ind1][0], self.right_points[ind1][1]])
                p2_right = np.array([self.right_points[ind2][0], self.right_points[ind2][1]])
                p3_right = self.right_points  # self.laser_np_2d_path_close_points
                dir_perpendicular_dis = np.cross(p2_right - p1_right, p1_right - p3_right) / np.linalg.norm(
                    p2_right - p1_right)
                right_points_ind = np.where((abs(dir_perpendicular_dis) <= self.ransac_tolerance))[0]
                theta = -np.pi / 2
                rot = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
                v1 = p2_right - p1_right
                vect = v1 * (1 / np.linalg.norm(v1)) * self.row_spacing
                dot_pro = np.dot(rot, vect)
                p1_left = p1_right + dot_pro
                p2_left = p2_right + dot_pro
                p3_left = self.left_points  # self.laser_np_2d_path_close_points
                dir_perpendicular_dis = np.cross(p2_left - p1_left, p1_left - p3_left) / np.linalg.norm(
                    p2_left - p1_left)
                left_points_ind = np.where((abs(dir_perpendicular_dis) <= self.ransac_tolerance))[0]
                # rospy.loginfo(f"maximum_inliers_count_l : {maximum_inliers_count_l},len(left_points_ind) : {len(left_points_ind)}, maximum_inliers_count_r : {maximum_inliers_count_r},len(right_points_ind) : {len(right_points_ind)}")

                if len(left_points_ind) > maximum_inliers_count_l and \
                        len(right_points_ind) > maximum_inliers_count_r:
                    maximum_inliers_count_l = len(left_points_ind)
                    maximum_inliers_count_r = len(right_points_ind)
                    random_final_left_points = [p1_left, p2_left]
                    random_final_right_points = [p1_right, p2_right]

                    if p2_left[0] - p1_left[0] != 0:
                        m = (p2_left[1] - p1_left[1]) / (p2_left[0] - p1_left[0])
                        # line_heading = math.atan2((point2.y - point1.y), (point2.x - point1.x))
                    else:
                        m = 1000000
                        # line_heading = math.atan2((point2.y - point1.y), (point2.x - point1.x))

                    c_r = p1_right[1] - m * p1_right[0]  # y - mx
                    c_l = p1_left[1] - m * p1_left[0]
                    coefficents = [m, c_l, c_r]
                    # random_final_left_points.append()

                    # A = point1.y - point2.y
                    # B = point2.x - point1.x
                    # C = point1.x * point2.y - point2.x * point1.y
                    #
                    # C2 = C - self.row_spacing * math.sqrt(A * A + B * B)
                    # coefficents = [A, B, C, C2]
                    # coefficents_m = [-A / B, (C / B + C2 / B) / 2]
                    # coefficents_m = [-A / B, C / B]
                    left_points_ind_final = left_points_ind
                    right_points_ind_final = right_points_ind

                ## publish inliers
            try:
                # left_inlier_points = np.take(self.laser_np_2d_path_close_points, left_points_ind_final, axis=0)
                left_inlier_points = np.take(self.left_points, left_points_ind_final, axis=0)

                self.left_points_inlier_pub.publish(xy_to_pointcloud2(left_inlier_points, "map"))
                # right_inlier_points = np.take(self.laser_np_2d_path_close_points, right_points_ind_final, axis=0)
                right_inlier_points = np.take(self.right_points, right_points_ind_final, axis=0)

                self.right_points_inlier_pub.publish(xy_to_pointcloud2(right_inlier_points, "map"))
                rospy.loginfo("left and right inliers  are published")
            except IndexError:
                rospy.logerr("Index error")
                continue
            # continue
            print("random_final_left_points", random_final_left_points)

            print("random_final_right_points", random_final_right_points)
            # path = two_points_to_path(random_final_left_points[0], random_final_left_points[1], "map")
            # self.left_random_path.publish(path)
            # path = two_points_to_path(random_final_right_points[0], random_final_right_points[1], "map")
            # self.right_random_path.publish(path)
            # rospy.loginfo("left and right RANSAC lines are published")
            # # continue

            m, c_l, c_r = coefficents[0], coefficents[1], coefficents[2]
            # applying linear regreesion on left
            # converting left and right points to base_link frame
            left_liners_pc2_map = xy_to_pointcloud2(left_inlier_points, "map")
            left_liners_pc2_base_link = transform_cloud(left_liners_pc2_map, left_liners_pc2_map.header.frame_id,
                                                        "base_link")
            left_liners_pc2_base_link_numpy = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(
                left_liners_pc2_base_link, remove_nans=False)[:, :-1]
            right_liners_pc2_map = xy_to_pointcloud2(right_inlier_points, "map")
            right_liners_pc2_base_link = transform_cloud(right_liners_pc2_map, right_liners_pc2_map.header.frame_id,
                                                         "base_link")
            right_liners_pc2_base_link_numpy = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(
                right_liners_pc2_base_link, remove_nans=False)[:, :-1]

            model = LinearRegression()
            model.fit(left_liners_pc2_base_link_numpy[:, 0, np.newaxis],
                      left_liners_pc2_base_link_numpy[:, 1, np.newaxis])
            left_line_slope_base_link, left_y_intercept_base_link = model.coef_[0][0], model.intercept_[0]
            model = LinearRegression()
            model.fit(right_liners_pc2_base_link_numpy[:, 0, np.newaxis],
                      right_liners_pc2_base_link_numpy[:, 1, np.newaxis])
            right_line_slope_base_link, right_y_intercept_base_link = model.coef_[0][0], model.intercept_[0]
            center_line_slope_base_link = (left_line_slope_base_link + right_line_slope_base_link) / 2
            center_line_y_intercept_base_link = (left_y_intercept_base_link + right_y_intercept_base_link) / 2

            # m_list.append(m)
            # cl_list.append(c_l)
            # cr_list.append(c_r)
            # if len(m_list) > self.window_size:
            #     m_list.pop(0)
            # if len(cl_list) > self.window_size:
            #     cl_list.pop(0)
            # if len(cr_list) > self.window_size:
            #     cr_list.pop(0)
            #
            # avg = sum(m_list) / len(m_list)
            # m = avg
            # c_l_avg = sum(cl_list) / len(cl_list)
            # c_r_avg = sum(cr_list) / len(cr_list)
            # m = (m_l + m_2)/2

            # m = (m_l + m_r) / 2
            # c_l_avg = c_l
            # c_r_avg = c_r

            # self.slope_pub.publish(m)
            # self.left_intercept_pub.publish(c_l_avg)
            # self.right_intercept_pub.publish(c_r_avg)

            left_line = Line(center_line_slope_base_link, left_y_intercept_base_link)
            right_line = Line(center_line_slope_base_link, right_y_intercept_base_link)  # robot_loc_forward_base_link
            rospy.loginfo(f"left line : {left_line} , right line : {right_line}")
            left_starting_point = left_line.intersct_point_to_line(robot_loc_backward_base_link)
            left_end_point = left_line.intersct_point_to_line(robot_loc_forward_base_link)
            rospy.logdebug(f"robot_loc_backward_base_link: {robot_loc_backward_base_link}")
            rospy.logdebug(f"left_starting_point : {left_starting_point},left_end_point : {left_end_point} ")

            right_staring_point = right_line.intersct_point_to_line(robot_loc_backward_base_link)
            right_end_point = right_line.intersct_point_to_line(robot_loc_forward_base_link)
            path = two_points_to_path(left_starting_point, left_end_point, "base_link")
            left_path_map_frame = convert_path(path, "map")
            self.left_random_path.publish(left_path_map_frame)

            path = two_points_to_path(right_staring_point, right_end_point, "base_link")
            right_path_map_frame = convert_path(path, "map")
            self.right_random_path.publish(right_path_map_frame)

            center_line = Line(center_line_slope_base_link, center_line_y_intercept_base_link)
            center_starting_point = center_line.intersct_point_to_line(robot_loc_backward_base_link)
            center_end_point = center_line.intersct_point_to_line(robot_loc_forward_base_link)
            center_path_base_frame = two_points_to_path(center_starting_point, center_end_point, "base_link")
            center_path_map_frame = convert_path(center_path_base_frame, "map")
            self.before_filter.publish(center_path_map_frame)
            # rospy.loginfo(f"center_path_map_frame : {center_path_map_frame}")
            # Finding moving average filter
            center_line_slope_map_frame = (center_path_map_frame.poses[1].pose.position.y - center_path_map_frame.poses[
                0].pose.position.y) / (center_path_map_frame.poses[1].pose.position.x -
                                       center_path_map_frame.poses[0].pose.position.x)
            # y = mx+c => c = y - mx

            center_line_y_intercept_map_frame = center_path_map_frame.poses[0].pose.position.y - \
                                                center_line_slope_map_frame * center_path_map_frame.poses[
                                                    0].pose.position.x

            m_list.append(center_line_slope_map_frame)
            c_list.append(center_line_y_intercept_map_frame)
            if len(m_list) > self.window_size:
                m_list.pop(0)
            if len(c_list) > self.window_size:
                c_list.pop(0)

            filtered_slope_center_line_map_frame = sum(m_list) / len(m_list)
            filtered_y_intercept_center_line_map_frame = sum(c_list) / len(c_list)
            filtered_center_line = Line(filtered_slope_center_line_map_frame,
                                        filtered_y_intercept_center_line_map_frame)
            center_starting_point = filtered_center_line.intersct_point_to_line(robot_loc)
            center_end_point = filtered_center_line.intersct_point_to_line(robot_loc_forward)
            center_path_map_frame = two_points_to_path(center_starting_point, center_end_point, "map")
            # center_path_map_frame = convert_path(path, "map")
            # self.center_points_vibration_test_pub.publish(center_path_map_frame)
            # self.after_filter.publish(center_path_map_frame)
            # rate.sleep()
            # continue

            # Find the number of inliers of final_center line if available
            # current_line = filtered_center_line
            speed = 1.0

            final_center_inliers_exist = False
            current_line = None
            rospy.logdebug("------------------------")
            # self.final_center_line = None
            if self.final_center_line is not None and self.use_previous_line:
                # finding number of inliers final_center_line
                final_center_inliers_inds = self.final_center_line.inlier(self.laser_data,
                                                                          vehicle_data.dimensions.track_width / 2 +
                                                                          self.offset_to_add_center_line_width)
                if len(final_center_inliers_inds) > 0:
                    rospy.logwarn(f"Inlers found on final_center_line, N:{len(final_center_inliers_inds)}")
                    final_center_inliers_exist = True
                else:
                    final_center_inliers_exist = False
                    current_line = self.final_center_line
                    rospy.loginfo("Using final_center_line  as current line")
            else:
                ransac_center_inliers_idx = filtered_center_line.inlier(self.laser_data,
                                                                        vehicle_data.dimensions.track_width / 2 +
                                                                        self.offset_to_add_center_line_width)
                if len(ransac_center_inliers_idx) == 0 and len(m_list) >= self.window_size:
                    self.final_center_line = filtered_center_line
                    current_line = self.final_center_line
                    rospy.loginfo("Using ransac_center_line  as current line, and assigned to final_center_line")
                else:
                    # need to check for collsion below
                    current_line = filtered_center_line
                    rospy.loginfo("Using ransac_center_line  as current line")

            if final_center_inliers_exist:
                ransac_center_inliers_idx = filtered_center_line.inlier(self.laser_data,
                                                                        vehicle_data.dimensions.track_width / 2 +
                                                                        self.offset_to_add_center_line_width)
                rospy.loginfo(f"checking for inliers on ransac line and found n: {len(ransac_center_inliers_idx)}")
                if len(ransac_center_inliers_idx) > 0:
                    rospy.logerr("inliers on the center line, check for collision")
                    # current_line = self.final_center_line
                    current_line = filtered_center_line
                else:
                    rospy.loginfo("No inliers on ransac line, updating final_center_line with ransac line.")

                    self.final_center_line = filtered_center_line
                    current_line = self.final_center_line

            rospy.loginfo(f"current line equation : {current_line}")
            rospy.logdebug("------------------------")

            current_line_inliers_inds = current_line.inlier(self.laser_data,
                                                            vehicle_data.dimensions.track_width / 2 +
                                                            self.offset_to_add_center_line_width)

            current_line_inliers = np.array([])
            try:
                current_line_inliers = np.take(self.laser_data, current_line_inliers_inds, axis=0)
                # comment to check below forward inlier
                self.center_line_inliers_pub.publish(xy_to_pointcloud2(current_line_inliers, "map"))
            except IndexError:
                pass

            # Decide inliers on which side, for stopping consider only forward inliers,
            # forward and by some distance to the robot, and heading
            # 1st the heading check, then the distance check.
            # rospy.loginfo(f"center_line_inliers: {current_line_inliers}")
            vehicle_stop_command = False

            print("center_line_inliers", current_line_inliers.shape)
            if current_line_inliers.shape[0] > 0:
                # check for dis and dir th
                v1 = self.robot_tie
                v2 = np.array([current_line_inliers[:, 0] - self.robot_pose.position.x,
                               current_line_inliers[:, 1] - self.robot_pose.position.y]).T

                # rospy.logdebug(f"self.robot_tie: {self.robot_tie}, v2:{v2} ")
                # rospy.logdebug(f"x _diff : {center_line_inliers[0][0] - self.robot_pose.position.x}"
                #                f"y_diff : {center_line_inliers[0][1] - self.robot_pose.position.y} ")
                # dot pro
                dot_pro = np.dot(v2, v1)
                # rospy.logdebug(f"dot_pro : {dot_pro}")
                forward_inlier_inx = np.where((dot_pro >= 0))[0]

                # rospy.logdebug(f"forward_inlier_inx:{forward_inlier_inx}")
                forward_center_line_inliers = np.take(current_line_inliers, forward_inlier_inx, axis=0)

                # distance th
                # print("v2", v2)
                squared_v2 = np.array([forward_center_line_inliers[:, 0] - self.robot_pose.position.x,
                                       forward_center_line_inliers[:, 1] - self.robot_pose.position.y]).T
                # squared_v2 = v2 * v2
                squared_v2 = squared_v2 * squared_v2
                # print("squared_v2", squared_v2)
                dis = np.sqrt(np.sum(squared_v2, axis=1))
                # rospy.logdebug(f"distances: {dis}")
                close_dis_idx = np.where((dis <= self.forward_collision_dis))[0]
                # rospy.logdebug(f"close_dis_idx :{close_dis_idx}")
                close_dis_center_line_inliers = np.take(forward_center_line_inliers, close_dis_idx, axis=0)

                self.center_line_inliers_pub.publish(xy_to_pointcloud2(close_dis_center_line_inliers, "map"))
                rospy.logdebug("close dis center line inliers are published")
                rospy.logdebug("center line forward inliers are published")
                collision_points_ind = np.where((dis <= self.obs_stop_dis_on_center_line))[0]
                collision_points_center_line = np.take(forward_center_line_inliers, collision_points_ind, axis=0)
                rospy.logdebug(f"collision_points_center_line :{collision_points_center_line.shape}")
                if collision_points_center_line.shape[0] > 0:
                    rospy.logwarn("Collision detected in center line")
                    self.center_line_obstacle_points.publish(xy_to_pointcloud2(collision_points_center_line, "map"))
                    rospy.logdebug("collision points are published")
                    vehicle_stop_command = True
                else:
                    rospy.logwarn("No collision detected in center line")
                    rospy.loginfo("moving the vehicle")
                    vehicle_stop_command = False
            else:
                rospy.loginfo("No Inliers on the current line")
                vehicle_stop_command = False

            if vehicle_stop_command and self.collision_detect_enabled:
                speed = 0.0
            elif self.regulate_speed_enabled:
                center_starting_point = current_line.intersct_point_to_line(robot_loc)
                center_end_point = current_line.intersct_point_to_line(robot_loc_forward)
                # P1 -> reduce speed  based on heading diff and offset
                line_heading = math.atan2(center_end_point[1] - center_starting_point[1],
                                          center_end_point[0] - center_starting_point[0])
                rospy.loginfo(
                    f"robot heading: {math.degrees(self.robot_yaw)}, Line heading: {math.degrees(line_heading)}")
                alpha = line_heading - self.robot_yaw
                rospy.loginfo(f"heading diff : {math.degrees(alpha)}")
                lateral_off_set = current_line.distance_to_point(robot_loc)
                rospy.loginfo(f"Heading diff alpha : {alpha}, lateral_off_set : {lateral_off_set}")
                rospy.logdebug(f"regulate_speed : {self.regulate_speed_enabled}")
                # IF MORE THAN SOME MAX TH, STOP THE VEHICLE.
                if (abs(alpha) > self.heading_diff_th or abs(
                        lateral_off_set) > self.center_line_robot_offset_th):
                    rospy.loginfo("Low speed mode")

                    speed = self._traj_manager.get_traj_point(self._close_idx).longitudinal_velocity_mps / 2
                    rospy.loginfo(
                        f"alpha condition :  {alpha > self.heading_diff_th}, lateral_offset condition: "
                        f"{lateral_off_set > self.center_line_robot_offset_th}, redcued speed : {speed} ")

                else:
                    rospy.loginfo("Max speed mode")
                    speed = self._traj_manager.get_traj_point(self._close_idx).longitudinal_velocity_mps

            else:
                speed = self._traj_manager.get_traj_point(self._close_idx).longitudinal_velocity_mps / 2

            center_starting_point = current_line.intersct_point_to_line(robot_loc)
            center_end_point = current_line.intersct_point_to_line(robot_loc_forward)
            center_path = two_points_to_path(center_starting_point, center_end_point, "map")
            self.center_line_heading = math.atan2(center_end_point[1] - center_starting_point[1],
                                                  center_end_point[0] - center_starting_point[0])
            self.center_line = current_line
            # self.after_filter.publish(center_path)
            traj_msg = path_to_traj(center_path, speed)
            self._input_traj_manager.update(traj_msg)
            path = self._input_traj_manager.to_path()
            self.center_traj_path_pub.publish(path)
            # self.center_traj_pub.publish(traj_msg)
            rospy.loginfo("Center traj and path are published")
            # Publising vibration path.
            posest = PoseStamped()
            self.vibration_path_msg.header.frame_id = "map"
            posest.header.frame_id = "map"
            posest.pose.position.x = center_starting_point[0]
            posest.pose.position.y = center_starting_point[1]
            posest.pose.orientation.z = 1
            self.vibration_path_msg.poses.append(posest)
            self.center_points_vibration_test_pub.publish(self.vibration_path_msg)
            rospy.loginfo("Vibration path published")
            loop_end_time = time.time()
            rospy.loginfo(f"time taken {loop_end_time - loop_start_time}")
            # start
            dwa_paths = self.dwa_path_gen.generate_paths(robot_loc.to_list())
            self.dwa_marker_pub.publish(self.dwa_path_gen.get_dwa_paths_marker_array(dwa_paths, "map"))

            print("COSTMAP ORIFIN", self.cost_map.origin)
            print("cost val at robot pose ", self.cost_map.get_cost_from_world_x_y(robot_loc.x , robot_loc.y))
            collision_free_paths = []
            for path in dwa_paths:
                in_collision_flag = False
                # print("===")
                for point in path:
                    try:
                        cost = self.cost_map.get_cost_from_world_x_y(point[0], point[1])
                        # print("cost", cost)
                        if cost != 0:
                            in_collision_flag = True
                            break
                        else:
                            pass
                    except IndexError:
                        pass
                if in_collision_flag:
                    continue
                else:
                    collision_free_paths.append(path)
            print("len of collision free paths ", len(collision_free_paths))
            if len(collision_free_paths) == 0:
                rospy.logerr("could not found collision free dwa paths")
                self.diagnose.summary(ERROR, "could not found collision free dwa paths, Stopping the vehicle")
                vehicle_stop_msg.status = True
                vehicle_stop_msg.message = "could not found collision free dwa paths, Stopping the vehicle"
                self.pilot_stop_command_pub.publish(vehicle_stop_msg)
                self.daignose_publish()
                rate.sleep()
                continue

            self.dwa_collision_free_paths_pub.publish(
                self.dwa_path_gen.get_dwa_paths_marker_array(collision_free_paths, "map"))


            line_cost, obs_xy = self.cost_map.get_line_cost_world(center_starting_point[0], center_starting_point[1],
                                                                  center_end_point[0], center_end_point[1])
            rospy.logerr(f"Center line cost :{line_cost}")
            if line_cost != 0:
                rospy.logerr("collision detected on center")
                self.diagnose.summary(ERROR, "collision detected on center")
                self.daignose_publish()
                vehicle_stop_msg.status = True
                vehicle_stop_msg.message = "collision detected on center"
                self.pilot_stop_command_pub.publish(vehicle_stop_msg)
                pst = PoseStamped()
                pst.header.frame_id = "map"
                pst.pose.position.x = obs_xy[0]
                pst.pose.position.y = obs_xy[1]
                pst.pose.orientation.w = 1
                self.collision_point_center_line_pub.publish(pst)
                rate.sleep()
                continue
            else:
                rospy.loginfo(" no collision detected on center")
            dis_list = []
            angle_diff_list = []
            if self.center_line:
                for i in range(len(collision_free_paths)):
                    path = collision_free_paths[i]
                    # print("path[-1]", path[-1])
                    # print(type(path[-1].tolist()))
                    dis = self.center_line.distance_to_point(path[-1].tolist())
                    angle_diff = self.center_line_heading - path[-1].tolist()[-1]
                    dis_list.append(abs(dis))
                    angle_diff_list.append(abs(angle_diff))
                    # print("dis_list", dis_list)
                dis_list_norm = np.array([float(dis)/sum(dis_list) for dis in dis_list])
                angle_diff_norm = np.array([float(angle)/sum(angle_diff_list) for angle in angle_diff_list])
                total_norm = dis_list_norm + angle_diff_norm
                min_dis_idx = np.argmin(total_norm)
                print("min_dis_idx", min_dis_idx)
                print("angle diff list", angle_diff_list)
                print("min_dist angle", angle_diff_list[min_dis_idx] )
                close_path_to_first_row = collision_free_paths[min_dis_idx]
                dwa_output = points_to_path(close_path_to_first_row, "map")
                # self.pub_handler.dwa_collision_free_and_close_to_line_pub.publish(dwa_output)
                # dwa_output_map = convert_path(dwa_output, "map")
                self.dwa_collision_free_and_close_to_line_pub.publish(dwa_output)
                traj_msg = Trajectory()
                traj_msg.header.frame_id = "map"
                speed = 1
                for i in range(len(dwa_output.poses)):
                    traj_point = TrajectoryPoint()
                    traj_point.pose = dwa_output.poses[i].pose
                    traj_point.index = i
                    traj_point.longitudinal_velocity_mps = speed

                    if i == 0:
                        traj_point.accumulated_distance_m = 0
                    else:
                        acc_dis = math.hypot(traj_msg.points[-1].pose.position.x - dwa_output.poses[i].pose.position.x,
                                             traj_msg.points[-1].pose.position.y - dwa_output.poses[i].pose.position.y)
                        traj_point.accumulated_distance_m = traj_msg.points[-1].accumulated_distance_m + acc_dis
                    traj_msg.points.append(traj_point)
                self.center_traj_pub.publish(traj_msg)
                self.diagnose.summary(OK, "Published Dwa path")
                self.daignose_publish()
                vehicle_stop_msg.status = False
                vehicle_stop_msg.message = "Looks okay"
                self.pilot_stop_command_pub.publish(vehicle_stop_msg)
                # Finding the center line inliers on costmap

            rate.sleep()

    def find_number_inlier_on_center_line(self, A, B, C):
        dmr = pow(A * A + B * B, 0.5)
        inliers_count = 0
        inliers_points = []
        for i in range(0, len(self.all_points)):
            dis = abs((A * self.all_points[i][0] + B * self.all_points[i][1] + C) / dmr)
            if dis <= self.ransac_tolerance / 2:
                inliers_count += 1
                inliers_points.append(self.all_points[i])
        return inliers_count, inliers_points

    def bboxes_callback(self, data):
        self.bboxes = data

    def odom_callback(self, data):
        self.robot_pose = data.pose.pose
        self.robot_speed = math.hypot(data.twist.twist.linear.x ** 2, data.twist.twist.linear.y ** 2)
        self.odom_data_in_time = time.time()
        self.robot_yaw = get_yaw(data.pose.pose.orientation)
        self.robot_tie = np.array([cos(self.robot_yaw), sin(self.robot_yaw)])

    def local_map_scan_callback(self, scan_in):
        self.scan_in_frame_id = scan_in.header.frame_id
        self.laser_data_in_time = time.time()
        self.scan_data_received = True
        points = self.laser_geo_obj.projectLaser(scan_in)
        # self.all_points_pub.publish(points)
        all_points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(points, remove_nans=False)
        self.all_points = np.delete(all_points, -1, axis=1)
        # self.all_points_pub.publish(xy_to_pointcloud2(self.all_points[0:100], scan_in.header.frame_id))

        try:
            tf_points = transform_cloud(points, scan_in.header.frame_id, "map")
            # self.map_frame_all_points_pub.publish(tf_points)
            rospy.logdebug("transformed pointcloud is published")
            if tf_points:
                laser_np_3d = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(tf_points, remove_nans=True)
                self.laser_np_2d = np.delete(laser_np_3d, -1, axis=1)

                # self.map_frame_all_points_pub.publish(xy_to_pointcloud2(self.laser_np_2d[0:100], "map"))

        except Exception as error:
            rospy.logerr(f"Error in transform_cloud, Error : {error}")

    def scan_callback(self, scan_in):
        points = self.laser_geo_obj.projectLaser(scan_in)
        tf_points = transform_cloud(points, scan_in.header.frame_id, "map")
        if tf_points:
            laser_data = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(tf_points, remove_nans=True)
            self.laser_data = np.delete(laser_data, -1, axis=1)
        else:
            rospy.logerr("Error in transform_cloud")

    def global_traj_callback(self, data):
        self._traj_in = data
        # self.global_traj = data
        self._traj_manager.update(data)
        self._traj_end_index = self._traj_manager.get_len()
        for point in data.points:
            yaw = get_yaw(point.pose.orientation)
            self.path.append(Pose2D(point.pose.position.x, point.pose.position.y, yaw))

        # finding the curvature
        incement_i = self.increment_for_curv  # assuming points are saved at 10 cm resolution.
        for i in range(0, incement_i):
            self.path[i].update_circum_radius(1000)
        for i in range(incement_i, self._traj_end_index - incement_i):
            x_vals = [self.path[i - incement_i].x, self.path[i].x, self.path[i + incement_i].x]
            y_vals = [self.path[i - incement_i].y, self.path[i].y, self.path[i + incement_i].y]

            circum_radius = circumradius(x_vals, y_vals)
            self.path[i].update_circum_radius(circum_radius)

        for i in range(self._traj_end_index - incement_i, self._traj_end_index):
            self.path[i].update_circum_radius(1000)

        rospy.logdebug("applied circun radius")

        if self.dedug_turn:
            marker_arr_msg = MarkerArray()
            marker = Marker()
            marker.header.frame_id = self._traj_in.header.frame_id
            marker.action = marker.DELETEALL
            marker_arr_msg.markers.append(marker)

            i = 0
            once = True
            while i < len(self.path)-1:
                # run this loop untill turn end point
                # Turn end point condition, turn end point should be heading oppisite direction of turn start point.and circumradius should be less than th.

                if abs(self.path[i].circum_radius) < self.min_turn_radius_to_check_turn:
                    check = [abs(self.path[k].circum_radius) < self.min_turn_radius_to_check_turn for k in range(i, i+20)]
                    if all(check):
                        print("valid turn start point")
                    else:
                        print("not valid turn start point")
                        i = i + 1
                        continue
                    turn_start_point = self.path[i]
                    turn_start_point_actual = self.path[i]
                    # turn_start_point.update_turn_info(1)
                    self.path[i].update_turn_info(1)
                    i = i+1
                    rospy.logerr(f"start index : {i}")
                    while i < len(self.path)-1:
                        if turn_start_point.dir_check(self.path[i+1]) or turn_start_point.heading_check(self.path[i]) or abs(int(self.path[i].circum_radius)) <= self.min_turn_radius_to_check_turn:
                            self.path[i].update_turn_info(1) # or
                            i = i + 1
                        else:
                            break
                    rospy.logerr(f"end index : {i}")
                else:
                    i = i + 1
            i = 0
            for i in range(len(self.path)):
                # Filling of marker msg
                marker = Marker()
                marker.header.frame_id = self._traj_in.header.frame_id
                marker.type = marker.TEXT_VIEW_FACING
                marker.text = str(int(self.path[i].circum_radius))
                marker.id = i
                marker.action = marker.ADD
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                if self.path[i].turn_info != 0:
                    marker.color.r = 1.0
                else:
                    marker.color.g = 1.0
                marker.color.b = 0.0

                marker.pose = self._traj_in.points[i].pose
                marker_arr_msg.markers.append(marker)
            self.curv_pub.publish(marker_arr_msg)
            rospy.logdebug("global curvature marker are published")
            # print("self.path", self.path)

    def find_close_point(self, robot_pose, old_close_index):
        close_dis = distance_btw_poses(robot_pose, self.traj_in.points[old_close_index].pose)
        for ind in range(old_close_index + 1, self.traj_end_index):
            dis = distance_btw_poses(robot_pose, self.traj_in.points[ind].pose)
            if close_dis >= dis:
                close_dis = dis
            else:
                robot_heading = get_yaw(robot_pose.orientation)
                path_heading = get_yaw(self.traj_in.points[ind].pose.orientation)
                # print("robot_heading", robot_heading)
                # print("path_heading", path_heading)
                # print("heading err:", abs(abs(robot_heading) - abs(path_heading)))
                # print("math.radians(90)", math.radians(90))
                if abs(abs(robot_heading) - abs(path_heading)) > math.radians(90):
                    rospy.logwarn("Headings are %s apart ", str(abs(robot_heading - path_heading)))
                    heading_ok = False
                else:
                    heading_ok = True
                return ind - 1, close_dis, heading_ok
        return self.traj_end_index, 0, True

    def calc_nearest_ind(self, robot_pose):

        distance_list = [distance_btw_poses(robot_pose, self.traj_in.points[ind].pose) for ind in
                         range(len(self.traj_in.points))]
        ind = np.argmin(distance_list)
        robot_heading = get_yaw(robot_pose.orientation)
        path_heading = get_yaw(self.traj_in.points[ind].pose.orientation)
        if abs(robot_heading - path_heading) > math.radians(90):
            rospy.logwarn("Headings are %s apart ", str(abs(robot_heading - path_heading)))
            heading_ok = False
        else:
            heading_ok = True
        dis = distance_list[ind]
        return ind, dis, heading_ok
