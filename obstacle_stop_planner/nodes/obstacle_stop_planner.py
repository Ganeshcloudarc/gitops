#!/usr/bin/env python3
"""
Collision checking on path with laser scanner Approach is inspired from
https://tier4.github.io/obstacle_stop_planner_refine/pr-check/pr-30/site/obstacle_stop_planner_refine/ 
"""

try:
    # python3 general packages
    import math
    import numpy as np
    from sklearn.neighbors import KDTree
    import copy

    np.float = np.float64
    import ros_numpy
    import rospy
    import tf2_ros
    import time
    import sys

    # ros messages
    from nav_msgs.msg import Path, Odometry
    from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
    from zed_interfaces.msg import ObjectsStamped, Object
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import Float32MultiArray, Header
    from sensor_msgs.msg import PointCloud2, LaserScan
    from std_msgs.msg import Float32
    from std_msgs.msg import Bool

    from geometry_msgs.msg import Point, PoseArray, Pose, TransformStamped, PoseStamped, Polygon, PolygonStamped
    import logging

    # utils
    from laser_geometry import LaserProjection
    import tf2_geometry_msgs
    import sensor_msgs.point_cloud2 as pcd2
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
    from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

    # autopilot related imports
    from autopilot_utils.tf_helper import current_robot_pose, convert_point, transform_cloud, bbox_to_corners, \
        transform_lidar_objects, transform_zed_objects
    from autopilot_utils.pose_helper import distance_btw_poses, get_yaw
    from autopilot_utils.trajectory_smoother import TrajectorySmoother
    from vehicle_common.vehicle_config import vehicle_data
    from autopilot_utils.trajectory_common import TrajectoryManager
    from autopilot_msgs.msg import Trajectory, TrajectoryPoint, FloatKeyValue
    from velocity_planner import VelocityPlanner
    from autopilot_utils.footprint_transform import transform_footprint_circles
    from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
    from diagnostic_updater._diagnostic_status_wrapper import DiagnosticStatusWrapper 


except Exception as e: 
    import rospy

    rospy.logerr("Module error %s", str(e))
    exit()


def set_rospy_log_lvl(log_level):
    logger = logging.getLogger('rosout')
    logger.setLevel(rospy.impl.rosout._rospy_to_logging_levels[log_level])


def min_distance_to_object(pose, corners):
    dist_list = []
    for corner in corners:
        dis = math.hypot(pose.position.x - corner[0], pose.position.y - corner[1])
        dis.append(dist_list)
    return min(dist_list)

OK = DiagnosticStatus.OK
ERROR = DiagnosticStatus.ERROR
WARN = DiagnosticStatus.WARN
STALE = DiagnosticStatus.STALE




class ObstacleStopPlanner: 

    def __init__(self):
       
        self.osp_publisher = rospy.Publisher("/obstacle_stop_planner_diagnostics", DiagnosticArray, queue_size=1,latch= True) 
        self.diagnostics_publisher = DiagnosticStatusWrapper() 
        self.diagnostics_publisher.name = rospy.get_name()
        self.diagnostics_publisher.hardware_id = 'zekrom_v1'
        self.objects_number = None
        self.zed_objects = None
        self._close_idx = None
        self._traj_in = None
        self._traj_end_index = None

        self.laser_np_3d = None
        self.robot_speed = None
        self.laser_np_2d = None
        self.traj_end_index = None
        # self.index_old = None

        self.laser_data_in_time = None
        self.zed_data_in_time = None 
        self.obstacle_found_from_ZED  = None 
        self.scan_data_received = None
        self.bboxes = None
        self.pc_np = None
        self.tree = None
        self.robot_pose = None
        self.robot_head_pose = Pose()
        self.odom_data_in_time = None
        self.prev_by_pass_dist = None
        self.laser_geo_obj = LaserProjection()
        # self._a_max, self._slow_speed, self._stop_line_buffer = 1, 0.5, 3.5
        sigma = rospy.get_param("gaussian_velocity_filter/sigma", 1)
        kernal_size = rospy.get_param("gaussian_velocity_filter/kernal_size", 11) 
        self.robot_max_speed_th = rospy.get_param("patrol/max_forward_speed", 1.0)
        self.robot_min_speed_th = rospy.get_param("patrol/min_forward_speed", 0.8)
        self._smoother = TrajectorySmoother(sigma, kernal_size, self.robot_min_speed_th)
        self._traj_manager = TrajectoryManager()
        self._stop_line_buffer = rospy.get_param("obstacle_stop_planner/stop_line_buffer", 3.0) 
        self.slow_line_buffer = rospy.get_param("obstacle_stop_planner/slow_line_buffer",2.0)
        self.velocity_speed_profile_enable = rospy.get_param("obstacle_stop_planner/velocity_speed_profile_enable",True)
        # ros parameters for Obstacle stop planner
        # TODO accept form patrol application if available else take from patrol params.
        self.debug = rospy.get_param("/patrol/debug",False)
        radial_off_set_to_vehicle_width = rospy.get_param("obstacle_stop_planner/radial_off_set_to_vehicle_width", 0.5)
        self._trajectory_resolution = rospy.get_param("obstacle_stop_planner/trajectory_resolution", 0.5)
        self._lookup_collision_distance = rospy.get_param("obstacle_stop_planner/lookup_collision_distance", 20)
        self._vis_collision_points = rospy.get_param("/obstacle_stop_planner/vis_collision_points", True)
        self._vis_trajectory_rviz = rospy.get_param("/obstacle_stop_planner/vis_trajectory_rviz", True)
        self._robot_base_frame = rospy.get_param("robot_base_frame", "base_link")
        self.mission_continue = rospy.get_param("/mission_continue", True)
        self._time_to_wait_at_ends = rospy.get_param("patrol/wait_time_on_mission_complete", 20)
        self._min_look_ahead_dis = rospy.get_param("/pure_pursuit/min_look_ahead_dis", 3)
        self._max_look_ahead_dis = rospy.get_param("/pure_pursuit/max_look_ahead_dis", 6)
        self._TIME_OUT_FROM_LASER = 2  # in secs
        self._TIME_OUT_FROM_ODOM = 2                                                            
        self._TIME_OUT_FROM_ZED = 2  
        self.vehicle_stop_init_time_for_obs = None  
        self.stop_threshold_time_for_obs = rospy.get_param("/obstacle_stop_planner/stop_threshold_time_for_obs", 5) 
        # TODO consider vehicle diagonal to check for collision detection radius
        # distance within below value to laser point would make collision.
        self._radius_to_search = vehicle_data.dimensions.overall_width / 2 + radial_off_set_to_vehicle_width

        self._base_to_front = vehicle_data.dimensions.wheel_base + vehicle_data.dimensions.front_overhang
        self.use_zed_detections = rospy.get_param("obstacle_stop_planner/use_zed_object_detection", True)
        self.zed_objects_topic = rospy.get_param("obstacle_stop_planner/zed_object_topic",
                                                 "/zed2i/zed_node/obj_det/objects")
        # ros subscribers
        global_traj_topic = rospy.get_param("obstacle_stop_planner/traj_in", "global_gps_trajectory")
        scan_topic = rospy.get_param("obstacle_stop_planner/scan_in", "laser_scan")
        odom_topic = rospy.get_param("patrol/odom_topic", "vehicle/odom", )
        rospy.Subscriber(global_traj_topic, Trajectory, self.global_traj_callback)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.use_pcl_boxes = rospy.get_param("use_pcl_boxes", False)
        self.use_obs_v1 = rospy.get_param("/patrol/enable_obs_v1", True)
        if self.use_pcl_boxes:
            bbox_topic = rospy.get_param("/pcl_bbox_topic", "/obstacle_detector/jsk_bboxes")
            rospy.Subscriber(bbox_topic, BoundingBoxArray, self.pcl_bboxes_callback)
            self.bbox_pub = rospy.Publisher("collision_bbox", BoundingBox, queue_size=1)
            # self.box_corner_pub = rospy.Publisher("corner_boxes", PolygonStamped, queue_size=1)
            self.collision_points_polygon = rospy.Publisher("collision_points_polygon", PolygonStamped, queue_size=1) 
        if self.use_zed_detections: 
            rospy.loginfo("Running zed Obs") 
            rospy.Subscriber(self.zed_objects_topic, ObjectsStamped, self.zed_objects_callback, queue_size=1)
            self.transformed_zed_objects_publisher = rospy.Publisher('/obstacle_stop_planner/transformed_zed_obj',
                                                                     ObjectsStamped, queue_size=1)
        # ros publishers
        local_traj_in_topic = rospy.get_param("obstacle_stop_planner/traj_out", "local_gps_trajectory")
        self.local_traj_publisher = rospy.Publisher(local_traj_in_topic, Trajectory, queue_size=10)
        self.collision_points_publisher = rospy.Publisher('obstacle_stop_planner/collision_points', PointCloud2,
                                                          queue_size=10)
        self.velocity_marker_publisher = rospy.Publisher('obstacle_stop_planner/collision_velocity_marker', MarkerArray,
                                                         queue_size=10)
        self.slow_start_pub = rospy.Publisher("obstacle_stop_planner/slow_start",PoseStamped, queue_size=1)
        self.slow_pose_pub = rospy.Publisher("obstacle_stop_planner/slow_point",PoseStamped, queue_size=1)
        self.close_pose_pub = rospy.Publisher("obstacle_stop_planner/close_point", PoseStamped, queue_size=1)
        self.front_pose_pub = rospy.Publisher("obstacle_stop_planner/front_point", PoseStamped, queue_size=1)
        self.mission_count_pub = rospy.Publisher('/mission_count', Float32, queue_size=2, latch=True)
        self.robot_level_collision_check_enable = rospy.get_param("obstacle_stop_planner"
                                                                  "/robot_level_collision_check_enable", True)
        self.width_offset = rospy.get_param("obstacle_stop_planner/footprint/offset_to_width", 0.3)
        self.length_offset = rospy.get_param("obstacle_stop_planner/footprint/offset_to_forward_length", 1)

        self.circles_polygon_pub = rospy.Publisher('/obstacle_stop_planner/collision_checking_circles', PolygonStamped,
                                                   queue_size=2, latch=True)
        self.path_percent_publisher = rospy.Publisher("/osp_path_percentage",Float32, queue_size=1 ,latch=True)
        self.horn_publisher = rospy.Publisher("/horn_at_obs_found",Bool,queue_size=1,latch=False)
        self.count_mission_repeat = 0
        self.main_loop()

    
    def publish(self,diagnostics_publisher): 
        diagnostics = DiagnosticArray() 
        diagnostics.status.append(diagnostics_publisher) 
        diagnostics.header.stamp = rospy.Time.now() 
        self.osp_publisher.publish(diagnostics)
        
    def do_initial_sensor_check(self):

        is_lidar_sensor_healthy = False
        is_zed_sensor_healthy = False
        is_global_path_ok = False 
        sensor_msg = "No data from"
        if self._traj_manager.get_len() > 0 and self.robot_pose:
            rospy.loginfo("global path and robot_pose are received") 
            is_global_path_ok = True
            if self.use_obs_v1 or self.use_pcl_boxes:
                if self.scan_data_received:# and self._traj_manager.get_len() > 0 and self.robot_pose:
                    rospy.loginfo("scan data is received")
                    is_lidar_sensor_healthy = True
                    if self.use_pcl_boxes:
                        if self.bboxes:
                            rospy.loginfo("bounding boxes are received")
                            is_lidar_sensor_healthy = True
                        else: 
                            rospy.logwarn("waiting for bounding boxes")
                            is_lidar_sensor_healthy = False
                            if self.debug: 
                                sensor_msg+=" LIDAR data" 
                            else:
                                sensor_msg+=" lidar"
                    else:
                        is_lidar_sensor_healthy = True
                else:
                    rospy.logwarn( f"waiting for data  scan :{self.scan_data_received}") 
                    is_lidar_sensor_healthy = False 
                    if self.debug: 
                        sensor_msg+=" LIDAR data" 
                    else:
                        sensor_msg+=" lidar"
            else:
                is_lidar_sensor_healthy = True
            if self.use_zed_detections:
                if self.zed_objects:
                    rospy.loginfo("data from zed received") 
                    is_zed_sensor_healthy = True
                else:
                    rospy.logwarn("Waiting for zed data")
                    is_zed_sensor_healthy = False 
                    if self.debug:
                        sensor_msg+=" ZED data" 
                    else:
                        sensor_msg+=" camera"
            else:
                is_zed_sensor_healthy = True
        else:
            rospy.logwarn(
                f" waiting for global traj: {self._traj_manager.get_len() > 0}, odom: {self.robot_pose}")
            is_global_path_ok = False 
            if self.debug: 
                sensor_msg+=" global trajectory" 
            else:
                sensor_msg+=" Path"
        sensor_status = is_lidar_sensor_healthy and is_zed_sensor_healthy and is_global_path_ok
        return sensor_status,sensor_msg 

    def main_loop(self):
        # TODO: publish stop, slow_down margin's circle

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.diagnostics_publisher.clearSummary() 
            self.diagnostics_publisher.values = [] 
            # robot_pose = current_robot_pose("map", self.robot_base_frame)
            sensor_status,sensor_msg = self.do_initial_sensor_check() 
            if sensor_status: 
                self.diagnostics_publisher.summary(OK,"Received data") 
                self.diagnostics_publisher.add("Sensor_status ",sensor_status)
                self.publish(self.diagnostics_publisher) 
                break
            else : 
                self.diagnostics_publisher.summary(ERROR,f"{sensor_msg}") 
                self.diagnostics_publisher.add("Sensor_status ",sensor_status)
                self.publish(self.diagnostics_publisher)  
            rate.sleep()

        rate = rospy.Rate(100)
        start_by_pass_index = None
        end_by_pass_index = None
        prev_by_pass_dist = None
        while not rospy.is_shutdown(): 
            self.diagnostics_publisher.clearSummary() 
            self.diagnostics_publisher.values = [] 

            loop_start_time = time.time()
            # checks whether data from lidar sensor is updated.

            if loop_start_time - self.odom_data_in_time > self._TIME_OUT_FROM_ODOM: 
                rospy.logwarn("No update on odom (robot position)")
                if self.debug: 
                    self.diagnostics_publisher.summary(ERROR,"No update on odom (robot position)") 
                else:
                    self.diagnostics_publisher.summary(ERROR,"No update on gps")  
                self.diagnostics_publisher.add("message","No update on odom (robot position)")
                self.diagnostics_publisher.add("Last Odom time",loop_start_time - self.odom_data_in_time) 
                self.publish(self.diagnostics_publisher)
                rate.sleep()  
                continue
            if self.use_obs_v1 or self.use_pcl_boxes:
                if loop_start_time - self.laser_data_in_time > self._TIME_OUT_FROM_LASER: 
                    rospy.logwarn(f"No update on laser data from last {loop_start_time - self.laser_data_in_time}")
                    if self.debug: 
                        self.diagnostics_publisher.summary(ERROR,"No update on laser data")
                    else:
                        self.diagnostics_publisher.summary(ERROR,"No update on lidar") 
                    self.diagnostics_publisher.add("Last laser time",loop_start_time - self.laser_data_in_time)  
                    self.publish(self.diagnostics_publisher) 
                    rate.sleep()  
                    continue
            if self.use_zed_detections:
                if (loop_start_time - self.zed_data_in_time > self._TIME_OUT_FROM_ZED):
                    rospy.logwarn(f"No update on ZED data from last {loop_start_time - self.zed_data_in_time}") 
                    if self.debug: 
                        self.diagnostics_publisher.summary(ERROR,"No update on ZED data") 
                    else:
                        self.diagnostics_publisher.summary(ERROR,"No update on camera") 
                    self.diagnostics_publisher.add("Last ZED time",loop_start_time - self.zed_data_in_time)  
                    self.publish(self.diagnostics_publisher)
                    rate.sleep()
                    continue

            # check for the close index on the trajectory
            if self._close_idx is None:
                angle_th = 90
                found, index = self._traj_manager.find_first_closest_idx_with_dist_ang_thr(self.robot_pose,
                                                                                     self._max_look_ahead_dis, angle_th)
                if found:
                    self._close_idx = index

                else:
                    rospy.logwarn(f"No close point found dist_thr: {self._max_look_ahead_dis}, angle_thr: {angle_th}")
                    self.diagnostics_publisher.summary(ERROR,"No close point found") 
                    self.diagnostics_publisher.add("Distance Threshold",self._max_look_ahead_dis) 
                    self.diagnostics_publisher.add("Angle Threshold ",angle_th)   
                    self.publish(self.diagnostics_publisher)
                    rate.sleep()
                    continue
            else:
                self._close_idx = self._traj_manager.find_close_pose_after_index(self.robot_pose, self._close_idx, 10) 

            
            # print(self._traj_manager.get_traj_point(self._close_idx))
            
           
            self.close_pose_pub.publish(
                PoseStamped(header=Header(frame_id="map"),
                            pose=self._traj_manager.get_traj_point(self._close_idx).pose))
            # rate.sleep()

            front_tip_idx = self._traj_manager.next_point_within_dist(self._close_idx, self._base_to_front)
            self.front_pose_pub.publish(
                PoseStamped(header=Header(frame_id="map"), 
                            pose=self._traj_manager.get_traj_point(front_tip_idx).pose))

            # filling Kd true

            # check for mission complete
            path_percent = (self._traj_in.points[self._close_idx].accumulated_distance_m /
                                self._traj_in.points[-1].accumulated_distance_m) * 100
            self.path_percent_publisher.publish(path_percent)
           

            if path_percent > 95.0 and distance_btw_poses(self.robot_pose,
                                                          self._traj_in.points[-1].pose) <= self._min_look_ahead_dis:
                self.count_mission_repeat += 1
                rospy.loginfo(' Mission count %s ', self.count_mission_repeat)
                self.mission_count_pub.publish(self.count_mission_repeat)
                self.mission_continue = rospy.get_param("/mission_continue", False)
                if self.mission_continue:
                    self._close_idx = 1 
                    rospy.set_param("/obstacle_stop_planner/by_pass_dist", 0)
                    self.diagnostics_publisher.summary(OK,"Mission complete ") 
                    self.diagnostics_publisher.add("Mission Repeat ",self.count_mission_repeat) 
                    self.publish(self.diagnostics_publisher)
                    time.sleep(self._time_to_wait_at_ends)
                    continue
                else:
                    self.diagnostics_publisher.summary(ERROR,"Mission Completed") 
                    self.diagnostics_publisher.add("Mission Repeat ",self.count_mission_repeat) 
                    self.publish(self.diagnostics_publisher)
                    # self.send_ack_msg(0, 0, 0)
                    rate.sleep()
                    break 
            
            # bypass_mode
            prev_processed_ind = self._close_idx
            obstacle_found_zed = False 
            obstacle_found_v1 = False 
            obstacle_found_v2 = False 
            trajectory_msg = Trajectory()
            trajectory_msg.header.frame_id = "map"
            trajectory_msg.home_position = self._traj_in.home_position

            self.by_pass_dist = rospy.get_param("/obstacle_stop_planner/by_pass_dist", 0)
            rospy.logwarn_throttle(1,f"self.by_pass_dist: {self.by_pass_dist}")
            # bypass mode is true
            if self.by_pass_dist != 0: 
                
                if not prev_by_pass_dist:
                     prev_by_pass_dist = self.by_pass_dist 
                if (end_by_pass_index is None or prev_by_pass_dist != self.by_pass_dist):
                    rospy.logwarn("vehicle is bypassing") 
                    start_by_pass_index = self._close_idx
                    for ind in range(self._close_idx, self._traj_end_index):
                        path_acc_distance = self._traj_in.points[ind].accumulated_distance_m - \
                                            self._traj_in.points[self._close_idx].accumulated_distance_m

                        if path_acc_distance > self.by_pass_dist+self._base_to_front:
                            break
                        trajectory_msg.points.append(copy.deepcopy(self._traj_in.points[ind]))
                    end_by_pass_index = ind
                    # fixes the end bypass index and publishes till the bypass dist 
                else:
                    # fill traj msg till end_bypass_index
                    for ind in range(start_by_pass_index, end_by_pass_index +1):
                        # trajectory_msg.points.append(copy.deepcopy(self._traj_in.points[ind]))
                        traj_point = copy.deepcopy(self._traj_in.points[ind]) 
                        traj_point.longitudinal_velocity_mps = self.robot_min_speed_th
                        trajectory_msg.points.append(traj_point)
                        start_by_pass_index = self._close_idx 
                        
                    for ind in range(end_by_pass_index ,self._traj_end_index-1): 
                        path_acc_distance = self._traj_in.points[ind].accumulated_distance_m - \
                                            self._traj_in.points[self._close_idx].accumulated_distance_m

                        if path_acc_distance > self._lookup_collision_distance:
                            break
                        # trajectory_msg.points.append(copy.deepcopy(self._traj_in.points[ind]))
                        traj_point = copy.deepcopy(self._traj_in.points[ind]) 
                        traj_point.longitudinal_velocity_mps = 0.0
                        trajectory_msg.points.append(traj_point)
                        # as caution vehicle will bypass with minimum speed 
                rospy.loginfo(f"start_by_pass_index : {start_by_pass_index},end_by_pass_index : {end_by_pass_index} ")
                self.local_traj_publisher.publish(trajectory_msg)
                self.publish_velocity_marker(trajectory_msg) 
                self.diagnostics_publisher.summary(ERROR,"BYPASS_MODE") 
                self.diagnostics_publisher.add(f"Bypassing the obstacle {self.by_pass_dist} m",self.by_pass_dist) 
                self.publish(self.diagnostics_publisher)
            
                # reseting the bypass parameters 
                if self._close_idx > end_by_pass_index:
                    rospy.set_param("/obstacle_stop_planner/by_pass_dist", 0)
                    end_by_pass_index = None
                    start_by_pass_index = None 
                    prev_by_pass_dist = None 
                prev_by_pass_dist = self.by_pass_dist
                rate.sleep()
                continue
            else:
                start_by_pass_index = None
                end_by_pass_index = None
                prev_by_pass_dist = None



            if self.use_obs_v1: 
                try: 
                    kd_tree = KDTree(self.laser_np_2d, leaf_size=2)
                except Exception as e: 
                    rospy.logerr_throttle(10,"Could not fill KDtree, {}".format(e))
                    rate.sleep()
                    continue 
            
            # TODO
            """
            1. Ganerate few circles along the foot print of the vehicle.
            2. check for collisions using all the sensor inputs
            """
            # print("self._traj_in.points", self._traj_in.points[self._close_idx].longitudinal_velocity_mps) 

            if self.robot_level_collision_check_enable:
                # debug
                obs_found = False
                radius_to_search_near_robot = (vehicle_data.dimensions.overall_width + self.width_offset) / 2
                circles_array, circles_polygon = transform_footprint_circles(self.robot_pose, vehicle_data,
                                                                             width_offset=self.width_offset,
                                                                             length_offset=self.length_offset,
                                                                             to_polygon=True
                                                                             )
                self.circles_polygon_pub.publish(circles_polygon)
                if self.use_obs_v1:
                    collision_points = kd_tree.query_radius(circles_array, r=radius_to_search_near_robot)
                    collision_points_arr = []
                    for points in collision_points:
                        if points.any():
                            obs_found = True
                            rospy.logwarn_throttle(1, f'Obstacle found on Laser scan (V1)')
                            collision_points_arr = points
                            self.publish_points(collision_points_arr)
                            break 

                if self.use_pcl_boxes:
                    rospy.logdebug_once("RUNNING OBS V2")
                    if len(self.bboxes.boxes) > 0:
                        for center in circles_array:
                            try:
                                close_bbx_id, close_dis = self.find_close_object(self.bboxes,
                                                                                 [center[0],
                                                                                  center[1]])
                                # self.publish_bbox(self.bboxes.boxes[close_bbx_id]) 
                                if close_dis < radius_to_search_near_robot:
                                    rospy.logwarn_throttle(1, f'Obstacle found on BBoxes')
                                    self.bbox_pub.publish(self.bboxes.boxes[close_bbx_id]) 
                                    obs_found = True
                                    break
                                else:
                                    pass
                            except Exception as error:
                                rospy.logerr(f"Error in find_close_object: {error}")
                if self.use_zed_detections:
                    rospy.logdebug_once("RUNNING ZED")
                    for center in circles_array:
                        # check to neglect the nan position detected via zed camera
                        close_obj_zed_idx, close_obj_from_zed = self.find_close_object_zed(self.zed_objects, center)
                        # rospy.logerr(f'------------ZED OBS DIST -----------------{close_obj_from_zed}') 
                        if close_obj_from_zed < radius_to_search_near_robot:
                            rospy.logwarn_throttle(1, 'Obstacle found on Camera')  
                            self.obstacle_found_from_ZED = self.zed_objects.objects[close_obj_zed_idx].label
                            obs_found = True
                            break 

                        else:
                            pass 

                if obs_found:
                    if self._close_idx + 100 < self._traj_end_index:
                        end_idx = self._close_idx + 100
                    else:
                        end_idx = self._traj_end_index
                    trajectory_msg.points = copy.deepcopy(
                        self._traj_in.points[self._close_idx:end_idx])
                    for point in trajectory_msg.points:
                        point.longitudinal_velocity_mps = 0.0 
                    horn = 1
                    self.horn_publisher.publish(horn)
                    self.local_traj_publisher.publish(trajectory_msg)
                    self.publish_velocity_marker(trajectory_msg)
                    rospy.logwarn("Obstacle found near Vehicle, Stopping the vehicle")
                    rospy.loginfo("Collision points are published") 
                    if self.debug:
                        self.diagnostics_publisher.summary(ERROR,"ROBOT LEVEL COLLISION CHECK") 
                    else: 
                        self.diagnostics_publisher.summary(ERROR,"OBSTACLE FOUND") 
                    self.diagnostics_publisher.add("message","ROBOT LEVEL COLLISION CHECK")
                    self.diagnostics_publisher.add(f"OBSTACLE FOUND WITHIN {self.length_offset} m",self.length_offset)   
                    if self.use_zed_detections and self.obstacle_found_from_ZED:
                        self.diagnostics_publisher.add("ZED Collision Object",self.obstacle_found_from_ZED) 
                    self.publish(self.diagnostics_publisher)
                    rate.sleep() 
                    continue
                else: 
                    rospy.loginfo("No Obstacle found near Vehicle")
            
            for ind in range(self._close_idx, self._traj_end_index):
                path_acc_distance = self._traj_in.points[ind].accumulated_distance_m - \
                                    self._traj_in.points[self._close_idx].accumulated_distance_m

                if path_acc_distance > self._lookup_collision_distance:
                    break
                # TODO check for zed object detections (obstacle_found -> True and break)
                # TODO check for lidar object detections (obstacle_found -> True and break)
                collision_points = [np.array([])]
                if self._traj_in.points[ind].accumulated_distance_m - \
                        self._traj_in.points[prev_processed_ind].accumulated_distance_m > self._radius_to_search / 2:

                    path_pose = self._traj_in.points[ind].pose
                    if self.use_zed_detections: 
                        rospy.logdebug_once("RUNNING ZED") 
                        
                        if self.zed_objects is not None:
                            # to neglect the obstacle found with posiiion nan
                            close_obj_zed_idx, close_obj_from_zed = self.find_close_object_zed(self.zed_objects, [path_pose.position.x,
                                                                                            path_pose.position.y])                                 
                            # rospy.logerr(f'------------ZED OBS DIST -----------------{close_obj_from_zed}')
                            try:
                                if close_obj_from_zed < self._radius_to_search:
                                    rospy.logerr_throttle(10,
                                                        f'------------Obstacle found on Camera -----------------{close_obj_from_zed}')  
                                    obstacle_found_zed = True 
                                    self.diagnostics_publisher.add("Zed_detection",self.use_zed_detections)
                                    self.diagnostics_publisher.add("ZED-obstacle_width_distance_from_path",close_obj_from_zed) 
                                    self.diagnostics_publisher.add("Object_found_ZED",self.zed_objects.objects[close_obj_zed_idx].label)
                                    break
                                else:
                                    pass
                            except TypeError as t:
                                rospy.logerr_throttle(10, "empty data from zed") 
                        else:
                            rospy.logerr_throttle(2, "empty data from zed") 
                          
                    if self.use_obs_v1:
    
                        rospy.logdebug_once("RUNNING OBS V1")
                        
                        pose_xy = np.array([[path_pose.position.x, path_pose.position.y]])  # , path_pose.position.z]])
                        try:
                            collision_points = kd_tree.query_radius(pose_xy, r=self._radius_to_search)
                            prev_processed_ind = ind  
                            
                        except Exception as error:
                            rospy.logwarn(f"could not query KD tree,{error}") 

                        if len(list(collision_points[0])) > 0:
                            obstacle_found_v1 = True   
                            collision_point_index = list(collision_points[0])[0]
                            if 0 <= collision_point_index < len(self.laser_np_3d):
                                collision_points_x_y = self.laser_np_3d[collision_point_index][0], self.laser_np_3d[collision_point_index][1]
                                self.diagnostics_publisher.add("V1_detection",self.use_obs_v1)
                                self.diagnostics_publisher.add("V1-obstacle_width_distance_from_path",math.dist(pose_xy[0],collision_points_x_y))
                                break  

                        
                    if self.use_pcl_boxes:
                        rospy.logdebug_once("RUNNING OBS V2")
                        if len(self.bboxes.boxes) > 0:

                            try:
                                close_bbx_id, close_dis = self.find_close_object(self.bboxes,
                                                                                 [path_pose.position.x,
                                                                                  path_pose.position.y]) 
                                # self.publish_bbox(self.bboxes.boxes[close_bbx_id]) 
                                if close_dis < self._radius_to_search:
                                    obstacle_found_v2 = True 
                                    self.bbox_pub.publish(self.bboxes.boxes[close_bbx_id]) 
                                    self.diagnostics_publisher.add("V2_detection",self.use_pcl_boxes)
                                    self.diagnostics_publisher.add("V2-obstacle_width_distance_from_path",close_dis)
                                    break
                                else:
                                    pass 
                            except Exception as e:
                                rospy.logerr(f"Error in find_close_object: {e}")

            collision_index = ind
            collision_points = list(collision_points[0])
            # print("collision index",collision_index)
            print("self.index_old after loop", self._close_idx) 
                    
            obstacle_found = [obstacle_found_zed, obstacle_found_v1, obstacle_found_v2]
            if any(obstacle_found): 
                
                dis_to_obstacle = abs(self._traj_in.points[collision_index].accumulated_distance_m -
                                      self._traj_in.points[self._close_idx].accumulated_distance_m)
                # print(dis_to_obstacle)
                dis_from_front_to_obs = abs(self._traj_in.points[collision_index].accumulated_distance_m -
                                            self._traj_in.points[front_tip_idx].accumulated_distance_m)
                rospy.logwarn(f"obstacle found at {dis_to_obstacle} < hard stop {self._stop_line_buffer + self._base_to_front}")
                
                # if obstacle distance is less than _stop_line_buffer -> hard stop 
                if dis_to_obstacle < self._stop_line_buffer + self._base_to_front: 

                    # TODO apply break directly to pilot
                    horn = 1 # to publish the obstacle found details to collision.py for horn via autopilot auxillary command 
                    rospy.logwarn("obstacle is very close, applying hard breaking")
                    if self.debug:
                        self.diagnostics_publisher.summary(ERROR,"OBSTACLE IS VERY CLOSE ")   
                    else: 
                        self.diagnostics_publisher.summary(ERROR,"OBSTACLE FOUND")
                    self.diagnostics_publisher.add("message","OBSTACLE IS VERY CLOSE - hard brake")
                    self.diagnostics_publisher.add("Applying Brake",any(obstacle_found)) 
                    self.diagnostics_publisher.add("Obstacle Found At",dis_to_obstacle)                      
                    self.diagnostics_publisher.add("Radius search distance Threshold",self._radius_to_search)
                    self.publish(self.diagnostics_publisher)
                    self.vehicle_stop_init_time_for_obs = time.time()
                    for i in range(self._close_idx, collision_index + 1):
                        traj_point = copy.deepcopy(self._traj_in.points[i])
                        traj_point.longitudinal_velocity_mps = 0.0
                        trajectory_msg.points.append(traj_point) 
                    # traj_out = trajectory_msg 
                     

                else:
                    horn = 0
                    rospy.loginfo("obstacle dis is more than the stop_distance")
                    # find the stop index
                    stop_index = collision_index
                    temp_dist = 0.0

                    # Compute the index at which we should stop.

                    while temp_dist < self._stop_line_buffer + self._base_to_front and stop_index > self._close_idx:
                        temp_dist = abs(self._traj_in.points[collision_index].accumulated_distance_m -
                                        self._traj_in.points[stop_index].accumulated_distance_m)
                        stop_index -= 1
                    # Compute the index at which we should slow_Down. 

                    if self.velocity_speed_profile_enable: # speed profile enabled
                        slow_stop_index = stop_index 
                        tmp_dist = 0.0  
                        while tmp_dist < self.slow_line_buffer and slow_stop_index > self._close_idx: 
                            tmp_dist = abs(self._traj_in.points[stop_index].accumulated_distance_m -
                                            self._traj_in.points[slow_stop_index].accumulated_distance_m)
                            slow_stop_index -= 1 
                        # collison to stop - speed is 0
                        for i in range(collision_index,stop_index,-1):
                            traj_point = copy.deepcopy(self._traj_in.points[i])
                            traj_point.longitudinal_velocity_mps = 0.0
                            # traj_out.points.append(traj_point)
                            trajectory_msg.points.append(traj_point) 
                        # stp to slow - speed is gradual slow
                        for i in range(stop_index+1, slow_stop_index,-1): 
                            tmp_dist = abs(self._traj_in.points[stop_index].accumulated_distance_m -
                                            self._traj_in.points[i].accumulated_distance_m)
                            updt_spd = np.interp(tmp_dist,[0,self.slow_line_buffer],[self.robot_min_speed_th,self.robot_max_speed_th]) 
                            traj_point = copy.deepcopy(self._traj_in.points[i])   
                            traj_point.longitudinal_velocity_mps = min(updt_spd, self._traj_in.points[i].longitudinal_velocity_mps )
                            trajectory_msg.points.append(traj_point)
                        
                            rospy.logwarn_throttle(1,"found an obstacle, vehicle will slow down ")
                        # slow to clo - speed is max spd
                        for i in range(slow_stop_index, self._close_idx,-1): 
                            traj_point = copy.deepcopy(self._traj_in.points[i])   
                            trajectory_msg.points.append(traj_point) 
                    
                        self.slow_pose_pub.publish(PoseStamped(header=Header(frame_id="map"),
                                pose=self._traj_manager.get_traj_point(slow_stop_index).pose))
                        trajectory_msg.points.reverse()
                            
                        self.diagnostics_publisher.summary(WARN,"VEHICLE WILL SLOW DOWN")
                        self.diagnostics_publisher.add("OBSTACLE FOUND AT ",dis_to_obstacle)  
                        
                        self.publish(self.diagnostics_publisher)
                      
                    else: 
                         #speed profile disabled
                        for i in range(self._close_idx, stop_index + 1):
                            trajectory_msg.points.append(copy.deepcopy(self._traj_in.points[i]))

                        # trajectory_msg.points[-1].longitudinal_velocity_mps = 0.0
                        # # if self.robot_speed < self.robot_min_speed_th:
                        # #     trajectory_msg.points[0].longitudinal_velocity_mps = self.robot_min_speed_th
                        # # else:
                        # #     trajectory_msg.points[0].longitudinal_velocity_mps = self.robot_speed
                        # #
                        # # traj_out = self._smoother.filter(trajectory_msg)
                        for i in range(stop_index, collision_index):
                            traj_point = copy.deepcopy(self._traj_in.points[i])
                            traj_point.longitudinal_velocity_mps = 0.0
                            # traj_out.points.append(traj_point)
                            trajectory_msg.points.append(traj_point) 
                        
            else:
                horn = 0
                if self.vehicle_stop_init_time_for_obs is not None:
                    if time.time() - self.vehicle_stop_init_time_for_obs > self.stop_threshold_time_for_obs:
                        rospy.loginfo("No obstacle found")
                        self.diagnostics_publisher.summary(OK,"NO OBSTACLE IS FOUND")
                        self.diagnostics_publisher.add("VEHICLE IS MOVING :)",True)  
                        self.publish(self.diagnostics_publisher) 
                        for i in range(self._close_idx, collision_index):
                            trajectory_msg.points.append(copy.deepcopy(self._traj_in.points[i]))
                    else:
                        rospy.logwarn(
                            f'Obs Time limit not crossed. Remaining time: {self.stop_threshold_time_for_obs - (time.time() - self.vehicle_stop_init_time_for_obs)}')
                        if self.debug:
                            self.diagnostics_publisher.summary(ERROR,"OBSTACLE WAIT TIME") 
                        else:
                            self.diagnostics_publisher.summary(ERROR,"OBSTACLE FOUND") 
                        self.diagnostics_publisher.add("message","OBSTACLE WAIT TIME")
                        self.diagnostics_publisher.add("VEHICLE WILL MOVE IN ",(self.stop_threshold_time_for_obs - (time.time() - self.vehicle_stop_init_time_for_obs)))
                        self.publish(self.diagnostics_publisher)
                        for i in range(self._close_idx, collision_index):
                            traj_point = copy.deepcopy(self._traj_in.points[i])
                            traj_point.longitudinal_velocity_mps = 0.0
                            # traj_out.points.append(traj_point)
                            trajectory_msg.points.append(traj_point)
                else:
                    self.diagnostics_publisher.summary(OK,"NO OBSTACLE IS FOUND") 
                    self.diagnostics_publisher.add("VEHICLE IS MOVING :)",True)   
                    self.publish(self.diagnostics_publisher)  
                    for i in range(self._close_idx, collision_index): 
                        trajectory_msg.points.append(copy.deepcopy(self._traj_in.points[i])) 

            # obstacle found status publishing for horn to pure_pursuit_collision.py
            self.horn_publisher.publish(horn)
            self.local_traj_publisher.publish(trajectory_msg)
            self.publish_points(collision_points)
            print("robot_speed", self.robot_speed)
            print(f"time taken for a loop is: {time.time() - loop_start_time} ")
            # print("len of local traj", len(traj_out.points))
            print("collision_index", collision_index) 

            self.publish_velocity_marker(trajectory_msg) 
            rate.sleep() 
  
    def odom_callback(self, data):
        self.robot_pose = data.pose.pose
        self.robot_speed = math.sqrt(data.twist.twist.linear.x ** 2 + data.twist.twist.linear.y ** 2)
        # print("self.robot_speed", self.robot_speed)
        self.odom_data_in_time = time.time()
        pose_heading = get_yaw(data.pose.pose.orientation)
        self.robot_head_pose.position.x = data.pose.pose.position.x + self._base_to_front * np.cos(pose_heading)
        self.robot_head_pose.position.y = data.pose.pose.position.y + self._base_to_front * np.sin(pose_heading)
        self.robot_head_pose.position.z = data.pose.pose.position.z
        self.robot_head_pose.orientation = data.pose.pose.orientation 
    
    def remove_zed_obs_nan_position(self,data):   
        updated_zed_data = ObjectsStamped() 
        updated_zed_data.header = data.header
        if data is not None:
            if len(data.objects)!=0:
                for val in data.objects:  
                    if any(not math.isnan(x) for x in val.position):
                        updated_zed_data.objects.append(val)
            return updated_zed_data 
        else: 
            return None

    def zed_objects_callback(self, data):
        start = time.time()
        # self.zed_data_in_time = time.time()
        data_in_map_frame = None 
        data = self.remove_zed_obs_nan_position(data)
        if data.header.frame_id == "map":
            data_in_map_frame = data
        else: 
            data_in_map_frame = transform_zed_objects(data, "map")
   
        # transform_zed_objects() fn returns None on TF Error
        if data_in_map_frame is not None:
            self.zed_data_in_time = time.time() # start time only when no tf error
            self.transformed_zed_objects_publisher.publish(data_in_map_frame)
            self.zed_objects = data_in_map_frame  
        rospy.logdebug(f"time taken for zed objects callback: {time.time() - start} ")


    def find_close_object_zed(self, objects, point):
        zed_obs_dis_data = []
        '''
        the axes are defined according to the ROS standard: X Forward, Y LEFT, Z UP, so the distance of an object from the camera is on the X axis.
        The position of the object is the centroid of the positions off all the 3D points that compose the object itself.
        In case of partial object detection the centroid is calculated according to the visible data, if the tracking is active and the partial object matches an previously seen object, the centroid position is "smoothed".
        '''
        for object in objects.objects:
            dis_list = []
            dis = math.hypot(point[0] - object.position[0], point[1] - object.position[1]) 
            dis_list.append(dis)
            '''
            if center point(xyz) distance is less than threshold directly pass the distance
            else check for corners too for redundancy of Obs.
            '''
            for corners in object.bounding_box_3d.corners:
                dis = math.hypot(point[0] - corners.kp[0], point[1] - corners.kp[1]) 
                dis_list.append(dis)     
            zed_obs_dis_data.append(min(dis_list))
        if len(zed_obs_dis_data) > 0:
            min_dis_index = np.argmin(zed_obs_dis_data)
            return min_dis_index, zed_obs_dis_data[min_dis_index]
        else:
            return -1, float('inf')

    def scan_callback(self, data):
        start = time.time()
        self.laser_data_in_time = time.time()
        points = self.laser_geo_obj.projectLaser(data)
        tf_points = transform_cloud(points, data.header.frame_id, "map")
        if tf_points:
            self.laser_np_3d = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(tf_points, remove_nans=False)
            self.laser_np_2d = np.delete(self.laser_np_3d, -1, axis=1)
            if len(self.laser_np_2d.tolist()) == 0:
                self.laser_np_2d = np.array([[100, 100],[100,100]])
            self.scan_data_received = True 
        else:
            self.scan_data_received = False 

        rospy.logdebug(f"time taken for laser scan callback: {time.time() - start} ")

        # self.publish_points(self.pc_np)
        # self.collision_points_publisher.publish(tf_points)
        # try:
        #     self.tree = KDTree(self.pc_np, leaf_size=2)
        # except:
        #     rospy.logwarn("Could not fill KDtree")

    def global_traj_callback(self, data):
        self._traj_in = data
        self._traj_manager.update(data)
        self._traj_end_index = self._traj_manager.get_len()

    def pcl_bboxes_callback(self, data):
        if data.header.frame_id == "map":
            self.bboxes = data
            # self.publish_bboxs(self.bboxes)
        else:
            self.bboxes = transform_lidar_objects(data, "map")
            rospy.logwarn("Bounding boxes are in  map frame")

    def publish_bbox(self, bbox):
        polygon = PolygonStamped()
        polygon.header.frame_id = "map"
        box_list = bbox_to_corners(bbox)
        for x, y in box_list:
            pt = Point()
            pt.x = x
            pt.y = y
            polygon.polygon.points.append(pt)
        pt = Point()
        pt.x = box_list[0][0]
        pt.y = box_list[0][1]
        polygon.polygon.points.append(pt)
        self.collision_points_polygon.publish(polygon)

    def publish_bboxs(self, bboxes):
        polygon = PolygonStamped()
        polygon.header.frame_id = "map"
        for bbox in bboxes.boxes:
            box_list = bbox_to_corners(bbox)

            for x, y in box_list:
                pt = Point()
                pt.x = x
                pt.y = y
                polygon.polygon.points.append(pt)
            pt = Point()
            pt.x = box_list[0][0]
            pt.y = box_list[0][1]
            polygon.polygon.points.append(pt)
        self.box_corner_pub.publish(polygon)

    def find_close_object(self, bboxes, point):
        dis_list = []
        for box in bboxes.boxes:
            # dis = self.min_distance_to_object(bbox, point)
            dis = math.hypot(point[0] - box.pose.position.x, point[1] - box.pose.position.y)
            dis_list.append(dis)
        close_bbox_id = np.argmin(dis_list)
        return close_bbox_id, dis_list[close_bbox_id]

    def min_distance_to_object(self, box, point):
        """f
        returns minimum  distance to the point
        """
        # print("box", box)
        # box_list = bbox_to_corners(box)
        dis_list = []
        # print("point", point)
        # print("first", point[0])
        dis = math.hypot((point[0] - box.pose.position.x) ** 2 + (point[1] - box.pose.position.y) ** 2)
        dis_list.append(dis)
        # for x, y in box_list:
        #     dis = math.hypot((point[0] - x) ** 2 + (point[1] - y) ** 2)
        #     dis_list.append(dis)
        # print("dis_LIST: <dis_list)
        return min(dis_list)

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

    def publish_points(self, collision_points_index):
        try:
            obstacle_points = np.take(self.laser_np_3d, collision_points_index, 0)
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map"
            scaled_polygon_pcl = pcd2.create_cloud_xyz32(header, obstacle_points)
            rospy.logdebug("happily publishing sample pointcloud.. !")
            self.collision_points_publisher.publish(scaled_polygon_pcl)

        except Exception as error:
            rospy.logwarn("not able publish collision points %s", str(error))

    def publish_velocity_marker(self, trajectory):
        marker_arr_msg = MarkerArray()
        i = 0
        marker = Marker()
        marker.header.frame_id = trajectory.header.frame_id
        marker.action = marker.DELETEALL
        marker_arr_msg.markers.append(marker)
        for traj_point in trajectory.points:
            marker = Marker()
            marker.header.frame_id = trajectory.header.frame_id
            marker.type = marker.TEXT_VIEW_FACING
            # marker.type = marker.LINE_STRIP
            marker.text = str(round(traj_point.longitudinal_velocity_mps, 2))
            # marker.text = str(i-1)
            marker.id = i
            i += 1
            marker.action = marker.ADD
            if traj_point.longitudinal_velocity_mps == 0:
                marker.scale.x = 0.4
                marker.scale.y = 0.4
                marker.scale.z = 0.4
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif traj_point.longitudinal_velocity_mps < self.robot_max_speed_th : 
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            else:
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            # marker.lifetime = rospy.Duration.from_sec(1)
            marker.pose = traj_point.pose
            marker_arr_msg.markers.append(marker)
        # print(marker_arr_msg)

        self.velocity_marker_publisher.publish(marker_arr_msg)
        print("marker published")

if __name__ == "__main__":
    rospy.init_node('obstacle_stop_planner')
    set_rospy_log_lvl(rospy.DEBUG)
    obj = ObstacleStopPlanner()
    rospy.spin()