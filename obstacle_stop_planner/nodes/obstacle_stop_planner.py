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
    import ros_numpy
    import rospy
    import tf2_ros
    import time
    import sys

    # ros messages
    from nav_msgs.msg import Path, Odometry
    from jsk_recognition_msgs.msg import BoundingBoxArray
    from zed_interfaces.msg import ObjectsStamped, Object
    from geometry_msgs.msg import Point, PoseArray, Pose, TransformStamped, PoseStamped
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import Float32MultiArray, Header
    from sensor_msgs.msg import PointCloud2, LaserScan

    # utils
    from laser_geometry import LaserProjection
    import tf2_geometry_msgs
    import sensor_msgs.point_cloud2 as pcd2
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
    from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

    # autopilot related imports
    from autopilot_utils.tf_helper import current_robot_pose, convert_point, transform_cloud
    from autopilot_utils.pose_helper import distance_btw_poses, get_yaw
    from autopilot_utils.trajectory_smoother import TrajectorySmoother
    from vehicle_common.vehicle_config import vehicle_data
    from autopilot_utils.trajectory_common import TrajectoryManager
    from autopilot_msgs.msg import Trajectory, TrajectoryPoint
    from velocity_planner import VelocityPlanner

except Exception as e:
    import rospy

    rospy.logerr("Module error %s", str(e))
    exit()


def min_distance_to_object(pose, corners):
    dist_list = []
    for corner in corners:
        dis = math.hypot(pose.position.x - corner[0], pose.position.y - corner[1])
        dis.append(dist_list)
    return min(dist_list)


class ObstacleStopPlanner:
    def __init__(self):
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
        self.laser_geo_obj = LaserProjection()
        # self._a_max, self._slow_speed, self._stop_line_buffer = 1, 0.5, 3.5
        sigma = rospy.get_param("gaussian_velocity_filter/sigma", 1)
        kernal_size = rospy.get_param("gaussian_velocity_filter/kernal_size", 11)
        self.robot_min_speed_th = rospy.get_param("obstacle_stop_planner/robot_min_speed_th", 0.8)
        self._smoother = TrajectorySmoother(sigma, kernal_size, self.robot_min_speed_th)
        self._traj_manager = TrajectoryManager()
        self._stop_line_buffer = rospy.get_param("obstacle_stop_planner/stop_line_buffer", 3.0)

        # ros parameters for Obstacle stop planner
        # TODO accept form patrol application if available else take from patrol params.
        radial_off_set_to_vehicle_width = rospy.get_param("obstacle_stop_planner/radial_off_set_to_vehicle_width", 0.5)
        self._trajectory_resolution = rospy.get_param("obstacle_stop_planner/trajectory_resolution", 0.5)
        self._lookup_collision_distance = rospy.get_param("obstacle_stop_planner/lookup_collision_distance", 20)
        self._vis_collision_points = rospy.get_param("/obstacle_stop_planner/vis_collision_points", True)
        self._vis_trajectory_rviz = rospy.get_param("/obstacle_stop_planner/vis_trajectory_rviz", True)
        self._robot_base_frame = rospy.get_param("robot_base_frame", "base_link")
        self._mission_repeat = rospy.get_param("/obstacle_stop_planner/mission_continue", True)
        self._time_to_wait_at_ends = rospy.get_param("patrol/wait_time_on_mission_complete", 20)
        self._max_look_ahead_dis = rospy.get_param("/pure_pursuit/max_look_ahead_dis", 6)

        self._TIME_OUT_FROM_LASER = 2  # in secs
        self._TIME_OUT_FROM_ODOM = 2
        # TODO consider vehicle diagonal to check for collision detection radius
        # distance within below value to laser point would make collision.
        self._radius_to_search = vehicle_data.dimensions.overall_width / 2 + radial_off_set_to_vehicle_width

        self._base_to_front = vehicle_data.dimensions.wheel_base + vehicle_data.dimensions.front_overhang

        # ros subscribers
        global_traj_topic = rospy.get_param("obstacle_stop_planner/traj_in", "global_gps_trajectory")
        scan_topic = rospy.get_param("obstacle_stop_planner/scan_in", "laser_scan")
        odom_topic = rospy.get_param("patrol/odom_topic", "vehicle/odom", )
        rospy.Subscriber(global_traj_topic, Trajectory, self.global_traj_callback)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        # ros publishers
        local_traj_in_topic = rospy.get_param("obstacle_stop_planner/traj_out", "local_gps_trajectory")
        self.local_traj_publisher = rospy.Publisher(local_traj_in_topic, Trajectory, queue_size=10)
        self.collision_points_publisher = rospy.Publisher('obstacle_stop_planner/collision_points', PointCloud2,
                                                          queue_size=10)
        self.velocity_marker_publisher = rospy.Publisher('obstacle_stop_planner/collision_velocity_marker', MarkerArray,
                                                         queue_size=10)
        self.close_pose_pub = rospy.Publisher("obstacle_stop_planner/close_point", PoseStamped, queue_size=1)
        self.front_pose_pub = rospy.Publisher("obstacle_stop_planner/front_point", PoseStamped, queue_size=1)

        self.main_loop()

    def main_loop(self):
        # TODO: publish stop, slow_down margin's circle

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # robot_pose = current_robot_pose("map", self.robot_base_frame)

            if self.scan_data_received and self._traj_manager.get_len() > 0 and self.robot_pose:
                rospy.loginfo("scan data, global path and robot_pose  are received")
                break
            else:
                rospy.logwarn(
                    f"waiting for data  scan :{self.scan_data_received}, global traj: {self._traj_manager.get_len() > 0}, odom: {self.robot_pose}")
                rate.sleep()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            loop_start_time = time.time()
            # checks whether data from sensors are updated.
            if loop_start_time - self.odom_data_in_time > self._TIME_OUT_FROM_ODOM:
                rospy.logwarn("No update on odom (robot position)")
                rate.sleep()
                continue
            if loop_start_time - self.laser_data_in_time > self._TIME_OUT_FROM_LASER:
                rospy.logwarn(f"No update on laser data from last {loop_start_time - self.laser_data_in_time}")
                rate.sleep()
                continue

            # check for the close index on the trajectory
            if self._close_idx is None:
                angle_th = 90
                found, index = self._traj_manager.find_closest_idx_with_dist_ang_thr(self.robot_pose,
                                                                                     self._max_look_ahead_dis, angle_th)
                if found:
                    self._close_idx = index
                else:
                    rospy.logwarn(f"No close point found dist_thr: {self._max_look_ahead_dis}, angle_thr: {angle_th}")
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
                PoseStamped(header=Header(frame_id="map"), pose=self._traj_manager.get_traj_point(front_tip_idx).pose))

            # filling Kd true

            try:
                kd_tree = KDTree(self.laser_np_2d, leaf_size=2)
            except:
                rospy.logerr("Could not fill KDtree")
                pass
                # rate.sleep()
                # continue
            prev_processed_ind = self._close_idx
            obstacle_found = False
            trajectory_msg = Trajectory()
            trajectory_msg.header.frame_id = "map"
            trajectory_msg.home_position = self._traj_in.home_position

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
                    pose_xy = np.array([[path_pose.position.x, path_pose.position.y]])  # , path_pose.position.z]])
                    try:
                        collision_points = kd_tree.query_radius(pose_xy, r=self._radius_to_search)
                        prev_processed_ind = ind
                    except Exception as error:
                        rospy.logwarn(f"could not query KD tree,{error}")
                    if len(list(collision_points[0])) > 0:
                        obstacle_found = True
                        break
            collision_index = ind
            collision_points = list(collision_points[0])
            print("self.index_old after loop", self._close_idx)
            if obstacle_found:
                dis_to_obstacle = abs(self._traj_in.points[collision_index].accumulated_distance_m -
                                      self._traj_in.points[self._close_idx].accumulated_distance_m)
                dis_from_front_to_obs = abs(self._traj_in.points[collision_index].accumulated_distance_m -
                                            self._traj_in.points[front_tip_idx].accumulated_distance_m)
                rospy.logwarn(f"obstacle found at {dis_to_obstacle} meters ")
                # if obstacle distance is less than _stop_line_buffer -> hard stop
                if dis_to_obstacle < self._stop_line_buffer + self._base_to_front:

                    # TODO apply break directly to pilot
                    rospy.logwarn("obstacle is very close, applying hard breaking")
                    for i in range(self._close_idx, collision_index + 1):
                        traj_point = copy.deepcopy(self._traj_in.points[i])
                        traj_point.longitudinal_velocity_mps = 0.0
                        trajectory_msg.points.append(traj_point)
                    # traj_out = trajectory_msg
                else:
                    rospy.loginfo("obstacle dis is more than the stop_distance")
                    # find the stop index
                    stop_index = collision_index
                    temp_dist = 0.0
                    # Compute the index at which we should stop.
                    while temp_dist < self._stop_line_buffer and stop_index > self._close_idx:
                        temp_dist = abs(self._traj_in.points[collision_index].accumulated_distance_m -
                                        self._traj_in.points[stop_index].accumulated_distance_m)
                        stop_index -= 1
                    # Our trajectory starts from close_index to stop index
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
                rospy.loginfo("No obstacle found")
                for i in range(self._close_idx, collision_index):
                    trajectory_msg.points.append(copy.deepcopy(self._traj_in.points[i]))
                # trajectory_msg.points[-1].longitudinal_velocity_mps = 0.0
                # if self.robot_speed < self.robot_min_speed_th:
                #     trajectory_msg.points[0].longitudinal_velocity_mps = self.robot_min_speed_th
                # else:
                #     trajectory_msg.points[0].longitudinal_velocity_mps = self.robot_speed
                # traj_out = self._smoother.filter(trajectory_msg)

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
        self.robot_speed = math.hypot(data.twist.twist.linear.x ** 2, data.twist.twist.linear.y ** 2)
        # print("self.robot_speed", self.robot_speed)
        self.odom_data_in_time = time.time()
        pose_heading = get_yaw(data.pose.pose.orientation)
        self.robot_head_pose.position.x = data.pose.pose.position.x + self._base_to_front * np.cos(pose_heading)
        self.robot_head_pose.position.y = data.pose.pose.position.y + self._base_to_front * np.sin(pose_heading)
        self.robot_head_pose.position.z = data.pose.pose.position.z
        self.robot_head_pose.orientation = data.pose.pose.orientation

    def scan_callback(self, data):
        start = time.time()
        self.laser_data_in_time = time.time()
        points = self.laser_geo_obj.projectLaser(data)
        tf_points = transform_cloud(points, data.header.frame_id, "map")
        if tf_points:
            self.laser_np_3d = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(tf_points, remove_nans=False)
            self.laser_np_2d = np.delete(self.laser_np_3d, -1, axis=1)
            if len(self.laser_np_2d.tolist()) == 0:
                self.laser_np_2d = np.array([100, 100])
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
    rospy.init_node('obstacle_stop_planner_node')
    obj = ObstacleStopPlanner()
    rospy.spin()
