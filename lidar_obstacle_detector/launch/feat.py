#!/usr/bin/env python3
"""
Collision checking on path with laser scanner Approach is inspired from
https://tier4.github.io/obstacle_stop_planner_refine/pr-check/pr-30/site/obstacle_stop_planner_refine/ 
"""

try:
    # python3 general packages
    import math
    import numpy as np
    from numpy import argmin, zeros
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
    from geometry_msgs.msg import Point, PoseArray, Pose
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import Float32MultiArray, Header
    from sensor_msgs.msg import PointCloud2, LaserScan
    from jsk_recognition_msgs.msg import BoundingBoxArray

    # utils
    from laser_geometry import LaserProjection
    import tf2_geometry_msgs
    import sensor_msgs.point_cloud2 as pcd2
    from tf.transformations import euler_from_quaternion, quaternion_from_euler

    # autopilot related imports
    from autopilot_utils.tf_helper import current_robot_pose, convert_point, transform_cloud
    from autopilot_utils.pose_helper import distance_btw_poses, get_yaw
    from vehicle_common.vehicle_config import vehicle_data
    from autopilot_msgs.msg import Trajectory, TrajectoryPoint

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
        self.path_end_index = None
        self.index_old = None
        self.traj_in = None
        self.laser_data_in_time = None
        self.scan_data_received = None
        self.pc_np = None
        self.tree = None
        self.laser_geo_obj = LaserProjection()

        # ros parameters for Obstacle stop planner
        self.max_slow_down_velocity = rospy.get_param('obstacle_stop_planner/max_slow_down_velocity', 1.5)
        self.min_slow_down_velocity = rospy.get_param('obstacle_stop_planner/min_slow_down_velocity', 0.8)
        self.stop_margin = rospy.get_param('obstacle_stop_planner/stop_margin', 3)
        self.slow_down_margin = rospy.get_param("obstacle_stop_planner/slow_down_margin", 5)
        self.radial_off_set_to_vehicle_width = rospy.get_param("obstacle_stop_planner/radial_off_set_to_vehicle_width",
                                                               0.5)
        self.trajectory_resolution = rospy.get_param("obstacle_stop_planner/trajectory_resolution", 0.5)
        self.lookup_collision_distance = rospy.get_param("obstacle_stop_planner/lookup_collision_distance", 10)
        self.robot_base_frame = rospy.get_param("robot_base_frame", "base_link")
        self.mission_repeat = rospy.get_param("/obstacle_stop_planner/mission_continue", True)
        self.vis_collision_points = rospy.get_param("/obstacle_stop_planner/vis_collision_points", True)
        self.vis_trajectory_rviz = rospy.get_param("/obstacle_stop_planner/vis_trajectory_rviz", True)
        time_to_wait_at_ends = rospy.get_param("patrol/wait_time_on_mission_complete", 20)
        self.min_look_ahead_dis = rospy.get_param("/pure_pursuit/min_look_ahead_dis", 3)
        time_out_from_laser = 2  # in secs
        radius_to_search = vehicle_data.dimensions.overall_width / 2 + self.radial_off_set_to_vehicle_width

        # ros subscribers
        global_traj_topic = rospy.get_param("obstacle_stop_planner/traj_in", "global_gps_trajectory")
        scan_topic = rospy.get_param("obstacle_stop_planner/scan_in", "zed/laser_scan")
        rospy.Subscriber(global_traj_topic, Trajectory, self.global_traj_callback)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber("/filtered_detector/jsk_bboxes",BoundingBoxArray,self.Bounding_Cb)

        # ros publishers
        local_traj_in_topic = rospy.get_param("obstacle_stop_planner/traj_out", "local_gps_trajectory")
        self.local_traj_publisher = rospy.Publisher('local_gps_trajectory', Trajectory, queue_size=10)
        self.collision_points_publisher = rospy.Publisher('collision_points', PointCloud2, queue_size=10)
        self.velocity_marker_publisher = rospy.Publisher('collision_velocity_marker', MarkerArray, queue_size=10)

        # TODO: publish stop, slow_down margin's circle

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            robot_pose = current_robot_pose("map", self.robot_base_frame)
            if self.scan_data_received and self.traj_in and robot_pose:
                rospy.loginfo("scan data, global path and robot_pose  are received")
                break
            else:
                rospy.logwarn("waiting for scan data or global path or robot_pose ")
                rate.sleep()
        count = 0
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            robot_pose = current_robot_pose("map", self.robot_base_frame)
            if not robot_pose:
                rospy.logwarn("No TF between %s and %s", "map", self.robot_base_frame)
                rate.sleep()
                continue
            if time.time() - self.laser_data_in_time > time_out_from_laser:
                rospy.logwarn("No update on laser data from last %s", str(time.time() - self.laser_data_in_time))
                rate.sleep()
                continue
            if self.index_old is None:
                self.index_old, dis, heading_ok = self.calc_nearest_ind(robot_pose)
            else:
                self.index_old, dis, heading_ok = self.find_close_point(robot_pose, self.index_old)
            if not heading_ok:
                rospy.logwarn("Close point heading and vehicle are not on same side")
                rate.sleep()
                continue
            if self.traj_in.points[-1].accumulated_distance_m - \
                    self.traj_in.points[self.index_old].accumulated_distance_m < self.min_look_ahead_dis:
                if self.mission_repeat:
                    rospy.logwarn("mission count %s", str(count))
                    count = count + 1
                    rospy.logwarn("waiting for %s seconds", str(time_to_wait_at_ends))
                    time.sleep(time_to_wait_at_ends)
                    self.index_old = 1
                    continue
                else:
                    rospy.logwarn("mission completed")
                    sys.exit("mission completed")

            collision_points_list = []
            trajectory_msg = Trajectory()
            trajectory_msg.header.frame_id = "map"
            trajectory_msg.home_position = self.traj_in.home_position
            close_dis = self.traj_in.points[self.index_old].accumulated_distance_m
            prev_processed_ind = self.index_old
            for ind in range(self.index_old, self.path_end_index):
                path_acc_distance = self.traj_in.points[ind].accumulated_distance_m - close_dis
                # print(path_acc_distance)
                if path_acc_distance > self.lookup_collision_distance:
                    break

                collision_points = [np.array([])]
                if self.traj_in.points[ind].accumulated_distance_m - \
                        self.traj_in.points[prev_processed_ind].accumulated_distance_m > radius_to_search / 2:
                    # print("distance added:", self.traj_in.points[ind].accumulated_distance_m -
                    #       self.traj_in.points[prev_processed_ind].accumulated_distance_m)
                    path_pose = self.traj_in.points[ind].pose # find distance here
                    pose_xyz = np.array([[path_pose.position.x, path_pose.position.y, path_pose.position.z]])

                    self.from_bb = 0
                    for i in range(0,self.size-1):
                        self.box_msg = self.box_list[i]
                        pos_x = self.box_msg.pose.position.x
                        pos_y = self.box_msg.pose.position.y
                        pos_z = self.box_msg.pose.position.z

                        dimX = self.box_msg.dimensions.x
                        dimY = self.box_msg.dimensions.y

                        other_coordinates = self.GetCoordinates(pos_x,pos_y,dimX,dimY)
                        obs_dist = math.sqrt((path_pose.position.x - pos_x)**2+(path_pose.position.y - pos_y)**2)
                        obs_dist1 = self.getDistance(path_pose.position.x,path_pose.position.y,other_coordinates[0][0],other_coordinates[0][1])
                        obs_dist2 = self.getDistance(path_pose.position.x,path_pose.position.y,other_coordinates[1][0],other_coordinates[1][1])
                        obs_dist3 = self.getDistance(path_pose.position.x,path_pose.position.y,other_coordinates[2][0],other_coordinates[2][1])
                        obs_dist4 = self.getDistance(path_pose.position.x,path_pose.position.y,other_coordinates[3][0],other_coordinates[3][1])

                        if obs_dist < 3 or obs_dist1 < 3 or obs_dist2 < 3 or obs_dist3 < 3 or obs_dist4 < 4:
                            self.from_bb = 1
                            break

                    try:
                        collision_points = self.tree.query_radius(pose_xyz, r=radius_to_search)
                        prev_processed_ind = ind
                    except Exception as error:
                        rospy.logwarn("could not query KD tree", str(error))

                collision_points_list.extend(list(collision_points[0]))
                trajectory_point_msg = TrajectoryPoint()
                # print(len(list(collision_points[0])))

                if len(list(collision_points[0])) > 0 or self.from_bb:
                    # print("path_acc_distance: ",path_acc_distance)
                    # print("stop_margin",self.stop_margin )
                    if self.stop_margin > path_acc_distance:
                        # return 0
                        rospy.logwarn("collision detected in stop margin")
                        trajectory_point_msg = copy.deepcopy(self.traj_in.points[ind])
                        trajectory_point_msg.longitudinal_velocity_mps = 0
                        trajectory_msg.points.append(trajectory_point_msg)

                        # for i in range(ind, self.index_old-ind, -1):
                        for k in range(len(trajectory_msg.points) - 1, -1, -1):
                            trajectory_msg.points[k].longitudinal_velocity_mps = 0
                    elif self.stop_margin < path_acc_distance < self.slow_down_margin:
                        # TODO: look for slow down margin and how to control velocity
                        # reverse the traj till the end of slow down margin

                        dis_diff = abs(self.slow_down_margin - self.stop_margin)
                        vel_diff = abs(self.max_slow_down_velocity - self.max_slow_down_velocity)
                        pass

                        # for k in range(len(trajectory_msg.points) - 1, 0, -1):
                        #     trajectory_msg.points[k].longitudinal_velocity_mps = 0
                    else:
                        pass
                else:
                    # print(self.traj_in.points[ind].longitudinal_velocity_mps)
                    trajectory_point_msg = copy.deepcopy(self.traj_in.points[ind])

                trajectory_msg.points.append(trajectory_point_msg)
            if self.vis_collision_points:
                self.publish_points(collision_points_list)
            if self.vis_trajectory_rviz:
                self.publish_velocity_marker(trajectory_msg)
            self.local_traj_publisher.publish(trajectory_msg)

            rate.sleep()

    def getDistance(self,c1x,c1y,c2x,c2y):
        return math.sqrt((c1x-c2x)**2 + (c1y-c2y)**2)

    def GetCoordinates(self,x, y,dimensionsX,dimensionsY):
        yaw = 0
        cos_th = math.cos(yaw)
        sin_th = math.sin(yaw)

        all_coordinates = []

        co1X = x - (dimensionsX * cos_th - dimensionsY * sin_th)
        co1Y = y - (dimensionsX * sin_th + dimensionsY * cos_th)

        co2X = x + (dimensionsX * cos_th - dimensionsY * sin_th)
        co2Y = y + (dimensionsX * sin_th + dimensionsY * cos_th)

        co3X = x + (dimensionsX * cos_th + dimensionsY * sin_th)
        co3Y = y + (dimensionsX * sin_th - dimensionsY * cos_th)

        co4X = x - (dimensionsX * cos_th + dimensionsY * sin_th)
        co4Y = y - (dimensionsX * sin_th - dimensionsY * cos_th)

        c1 = []
        c1.append(co1X)
        c1.append(co1Y)

        c2 = []
        c2.append(co2X)
        c2.append(co2Y)

        c3 = []
        c3.append(co3X)
        c3.append(co3Y)

        c4 = []
        c4.append(co4X)
        c4.append(co4Y)

        all_coordinates.append(c1)
        all_coordinates.append(c2)
        all_coordinates.append(c3)
        all_coordinates.append(c4)

        return all_coordinates

    def Bounding_Cb(self,data):
        self.box_list = data.boxes
        self.size = len(self.box_list)
        

    def scan_callback(self, data):
        pass
        self.laser_data_in_time = time.time()
        points = self.laser_geo_obj.projectLaser(data)
        tf_points = transform_cloud(points, data.header.frame_id, "map")
        self.pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(tf_points, remove_nans=True)
        self.scan_data_received = True
        # self.publish_points(self.pc_np)
        # self.collision_points_publisher.publish(tf_points)
        try:
            self.tree = KDTree(self.pc_np, leaf_size=2)
        except:
            rospy.logwarn("Could not fill KDtree")

    def global_traj_callback(self, data):
        self.traj_in = data
        self.path_end_index = len(data.points)

    def find_close_point(self, robot_pose, old_close_index):
        close_dis = distance_btw_poses(robot_pose, self.traj_in.points[old_close_index].pose)
        for ind in range(old_close_index + 1, self.path_end_index):
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
        return self.path_end_index, 0, True

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
            a = np.take(self.pc_np, list(collision_points_index), 0)
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map"
            scaled_polygon_pcl = pcd2.create_cloud_xyz32(header, a)
            rospy.logdebug("happily publishing sample pointcloud.. !")
            self.collision_points_publisher.publish(scaled_polygon_pcl)
        except Exception as error:
            rospy.logwarn("not able publish collision points %s", str(error))

    def publish_velocity_marker(self, trajectory):
        marker_arr_msg = MarkerArray()
        i = 0
        for traj_point in trajectory.points:

            marker = Marker()
            marker.header.frame_id = trajectory.header.frame_id
            marker.type = marker.TEXT_VIEW_FACING
            marker.text = str(round(traj_point.longitudinal_velocity_mps, 2))
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
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            marker.pose = traj_point.pose
            marker_arr_msg.markers.append(marker)
        self.velocity_marker_publisher.publish(marker_arr_msg)


if __name__ == "__main__":
    rospy.init_node('obstacle_stop_planner_node')
    obj = ObstacleStopPlanner()
    rospy.spin()
