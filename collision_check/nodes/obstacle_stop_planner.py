#!/usr/bin/env python3
"""Collision checking on path with laser scanner Approach is inspired from
https://tier4.github.io/obstacle_stop_planner_refine/pr-check/pr-30/site/obstacle_stop_planner_refine/ """

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

    # ros messages
    from nav_msgs.msg import Path, Odometry
    from jsk_recognition_msgs.msg import BoundingBoxArray
    from zed_interfaces.msg import ObjectsStamped, Object
    from geometry_msgs.msg import Point, PoseArray, Pose
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import Float32MultiArray, Header
    from sensor_msgs.msg import PointCloud2, LaserScan

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


def distance_to_robot(pose):
    robot_pose = current_robot_pose()
    return distance_btw_poses(robot_pose, pose)


class ObstacleStopPlanner:
    def __init__(self):
        self.path_end_index = None
        self.index_old = None
        self.traj_in = None
        self.laser_data_in_time = None
        self.pc_np = None
        self.tree = None
        self.laser_geo_obj = LaserProjection()

        # ros parameters for Obstacle stop planner
        self.max_slow_down_velocity = rospy.get_param('max_slow_down_velocity', 1.5)
        self.min_slow_down_velocity = rospy.get_param('min_slow_down_velocity', 0.8)
        self.stop_margin = rospy.get_param('stop_margin', 9)
        self.slow_down_margin = rospy.get_param("slow_down_margin", 7)
        self.radial_off_set_to_vehicle_width = rospy.get_param("radial_off_set_to_vehicle_width", 0.5)
        self.trajectory_resolution = rospy.get_param("trajectory_resolution", 0.5)
        self.lookup_collision_distance = rospy.get_param("lookup_collision_distance", 10)
        self.robot_base_frame = rospy.get_param("robot_base_frame", "base_link")
        time_out_from_laser = 2  # in secs
        radius_to_search = vehicle_data.dimensions.overall_width / 2 + self.radial_off_set_to_vehicle_width

        # ros subscribers
        global_traj_topic = rospy.get_param("obstacle_stop_planner/traj_in", "global_gps_trajectory")
        scan_topic = rospy.get_param("obstacle_stop_planner/scan_in", "zed/laser_scan")
        rospy.Subscriber(global_traj_topic, Trajectory, self.global_traj_callback)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)

        # ros publishers
        self.local_traj_publisher = rospy.Publisher('local_gps_trajectory', Trajectory, queue_size=10)
        self.collision_points_publisher = rospy.Publisher('collision_points', PointCloud2, queue_size=10)
        self.velocity_marker_publisher = rospy.Publisher('collision_velocity_marker', MarkerArray, queue_size=10)
        # TODO: publish stop, slow_down margin's circle

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            robot_pose = current_robot_pose("map", self.robot_base_frame)
            if self.tree and self.traj_in and robot_pose:
                rospy.loginfo("scan data, global path and robot_pose  are received")
                break
            else:
                rospy.logwarn("waiting for scan data or global path or robot_pose ")
                rate.sleep()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            robot_pose = current_robot_pose("map", self.robot_base_frame)
            if not robot_pose:
                rospy.logwarn("No TF between %s and %s", "map", self.robot_base_frame)
                rate.sleep()
                continue
            if time.time() - self.laser_data_in_time > time_out_from_laser:
                rospy.logwarn("No update on laser data from last %s", str(time.time() - laser_data_in_time))
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

            collision_points_list = []
            trajectory_msg = Trajectory()
            trajectory_msg.header.frame_id = "map"
            trajectory_msg.home_position = self.traj_in.home_position
            close_dis = self.traj_in.points[self.index_old].accumulated_distance_m

            for ind in range(self.index_old, self.path_end_index):
                path_acc_distance = self.traj_in.points[ind].accumulated_distance_m - close_dis
                # print(path_acc_distance)
                if path_acc_distance > self.lookup_collision_distance:
                    break

                path_pose = self.traj_in.points[ind].pose
                pose_xyz = np.array([[path_pose.position.x, path_pose.position.y, path_pose.position.z]])
                collision_points = self.tree.query_radius(pose_xyz, r=radius_to_search)
                collision_points_list.extend(list(collision_points[0]))
                trajectory_point_msg = TrajectoryPoint()
                # print(len(list(collision_points[0])))

                if len(list(collision_points[0])) > 0:
                    # print("path_acc_distance: ",path_acc_distance)
                    # print("stop_margin",self.stop_margin )
                    if self.stop_margin > path_acc_distance:
                        # return 0
                        trajectory_point_msg = copy.deepcopy(self.traj_in.points[ind])
                        trajectory_point_msg.longitudinal_velocity_mps = 0
                        trajectory_msg.points.append(trajectory_point_msg)

                        # for i in range(ind, self.index_old-ind, -1):
                        for k in range(len(trajectory_msg.points)-1, -1, -1):
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

            self.publish_points(collision_points_list)
            self.publish_velocity_marker(trajectory_msg)
            self.local_traj_publisher.publish(trajectory_msg)

            rate.sleep()

    def scan_callback(self, data):
        self.laser_data_in_time = time.time()
        points = self.laser_geo_obj.projectLaser(data)
        tf_points = transform_cloud(points, self.robot_base_frame, "map")
        self.pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(tf_points, remove_nans=True)
        self.tree = KDTree(self.pc_np, leaf_size=2)

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

        a = np.take(self.pc_np, list(collision_points_index), 0)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        scaled_polygon_pcl = pcd2.create_cloud_xyz32(header, a)
        rospy.loginfo("happily publishing sample pointcloud.. !")
        self.collision_points_publisher.publish(scaled_polygon_pcl)

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


'''
        self.laser_data_in = None
        self.pc_np = None
        self.tree = None
        self.path_end_index = None
        self.obstacle_aware_vel = None
        self.stop_index = None
        self.global_vel = None
        self.close_index = None
        self.revived_path = None
        self.process_data = None
        self.target_frame = "map"
        self.index_old = None
        self.path = None
        self.modified_obj_data = ObjectsStamped()
        self.modified_obj_data.header.frame_id = self.target_frame
        cam_name = rospy.get_param("camera_name", "zed2")
        rospy.Subscriber(f"{cam_name}/zed_node/obj_det/objects", ObjectsStamped, self.objects_cb)
        self.obj_map_frame_pub = rospy.Publisher("/zed_objects_map_frame", ObjectsStamped, queue_size=1)
        self.vel_update_pub = rospy.Publisher("/vel_updates", Float32MultiArray, queue_size=1)

        rospy.Subscriber('/odom_path', Path, self.path_callback)
        rospy.Subscriber('/curvature_velocity_profile', Float32MultiArray, self.global_vel_cb)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # parameters

        self.obstacle_threshold_radius = rospy.get_param("obstacle_threshold_radius", 1.5)
        self.max_cte = rospy.get_param("max_cte_to_interpolate", 3)
        self.forward_collision_check_dis = rospy.get_param("forward_collision_check_dis", 15)  # 15 meters
        self.stop_distance = rospy.get_param("spot_distance_before_collision_ind", 3)
        self.collision_index_dis  = 2

        local_cost_map_topic = rospy.get_param("local_cost_map_topic", "/semantics/costmap_generator/occupancy_grid")
        self.base_frame = rospy.get_param("/patrol/base_frame", "ego_vehicle")
        self.lookup_collision_dis = 100

        # self.ogm = OccupancyGridManager('/semantics/costmap_generator/occupancy_grid',
        #                            subscribe_to_updates=False, base_frame=self.base_frame)
        self.left = -1.0 * vehicle_data.dimensions.rear_overhang
        self.right = vehicle_data.dimensions.overall_length
        self.top = vehicle_data.dimensions.overall_width / 2
        self.bottom = - vehicle_data.dimensions.overall_width / 2
        # self.resolution = self.ogm.resolution
        self.laser_geo_obj = LaserProjection()
        sub = rospy.Subscriber("/zed/laser_scan", LaserScan, self.laser_cb)

        # pose array pub
        self.rotated_poses_pub = rospy.Publisher("/rotated_poses", PoseArray, queue_size=10)
        self.point_cloud_pub = rospy.Publisher("collision_point_cloud", PointCloud2, queue_size=10)
        self.trajectory_publisher = rospy.Publisher("collision_trajectory",Trajectory , queue_size=10)
        self.trajectory_marker_publisher = rospy.Publisher("trajectory_marker", MarkerArray, queue_size=10)


        self.pose_arr_msg = PoseArray()
        self.pose_arr_msg.header.frame_id = 'map'
        self.main_loop()

        # try:
        #     transform = self.tf_buffer.lookup_transform(
        #         self.parent_frame, self.child_frame, rospy.Time())
        #     rate.sleep()
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
        #         tf2_ros.ExtrapolationException):
        #     rate.sleep()
        #     continue

    def objects_cb(self, data):
        print("data.header.frame_id", data.header.frame_id)
        object_corner_list = []
        if data.header.frame_id == self.target_frame:
            # no Transform needed
            self.modified_obj_data = data
        else:
            # need to transform from object frame to target frame
            self.modified_obj_data = copy.deepcopy(data)
            self.modified_obj_data.header.frame_id = self.target_frame
            self.modified_obj_data.header.stamp = rospy.Time.now()
            for i, obj_data in enumerate(data.objects):
                # https://www.stereolabs.com/docs/ros/object-detection/

                pos = convert_point(obj_data.position, data.header.frame_id, self.target_frame)
                self.modified_obj_data.objects[i].position = pos

                for j, corner in enumerate(obj_data.bounding_box_3d.corners):
                    print('c', corner.kp)
                    print(data.header.frame_id)
                    point = convert_point(corner.kp, data.header.frame_id, self.target_frame)
                    print('corner', point, corner.kp)
                    self.modified_obj_data.objects[i].bounding_box_3d.corners[j].kp = point

            self.to_visualization_marker_array(self.modified_obj_data)
            # print(data.objects[0].position, modified_obj_data.objects[0].position)

    def path_callback(self, data):
        rospy.logdebug("path data data received of length %s", str(len(data.poses)))
        self.path = data.poses
        self.path_end_index = len(self.path) - 1

    def global_vel_cb(self, data):

        self.global_vel = data.data
        self.obstacle_aware_vel = list(copy.deepcopy(self.global_vel))

    def find_close_point(self, prev_close_ind):

        close_dis = distance_to_robot(self.revived_path[prev_close_ind])
        for ind in range(prev_close_ind + 1, len(self.revived_path)):
            dis = distance_to_robot(self.revived_path[ind])
            if close_dis >= dis:
                close_dis = dis
            else:
                return ind - 1, close_dis

    def find_close_point(self, robot_pose, old_close_index):
        # n = min(100, len(range(index_old, self.path_end_index)))
        # distance_list = [self.calc_distance(robot, ind) for ind in range(index_old, index_old + n)]
        # ind = np.argmin(distance_list)
        # final = ind + index_old
        # dis = distance_list[ind]
        # return final, dis
        close_dis = self.calc_distance(robot_pose, old_close_index)
        for ind in range(old_close_index + 1, self.path_end_index):
            dis = self.calc_distance(robot_pose, ind)
            if close_dis >= dis:
                close_dis = dis
            else:
                # print("find close", index_old, ind)
                return ind - 1, close_dis
        return self.path_end_index, 0

    def calc_nearest_ind(self, robot_pose):
        """
        calc index of the nearest point to current position
        Args:
            robot_pose: pose of robot
        Returns:
            close_index , distance
        """
        distance_list = [self.calc_distance(robot_pose, ind) for ind in range(len(self.path))]
        ind = np.argmin(distance_list)
        self.index_old = ind
        dis = distance_list[ind]
        return ind, dis

    def distance_to_next_index(self, ind):
        return math.hypot(self.path[ind].pose.position.x - self.path[ind + 1].pose.position.x,
                          self.path[ind].pose.position.y - self.path[ind + 1].pose.position.y)

    def calc_distance(self, robot_pose, ind):
        return math.hypot(robot_pose.position.x - self.path[ind].pose.position.x, robot_pose.position.y -
                          self.path[ind].pose.position.y)

    def calc_distance_idx(self, close, next_id):
        return math.hypot(self.path[close][0] - self.path[next_id][0], self.path[close][1] - self.path[next_id][1])

    def laser_cb(self, data):
        self.laser_data_in = True

        points = self.laser_geo_obj.projectLaser(data)
        tf_points = transform_cloud(points, "ego_vehicle", "map")
        self.pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(tf_points, remove_nans=True)
        self.tree = KDTree(self.pc_np, leaf_size=2)
        # cloud_points = []
        # for i, rang in enumerate(ranges):
        #     if rang == np.inf:
        #         continue
        #     angle = data.angle_min + (i * data.angle_increment)
        #     x, y = pol2cart(rang, angle)
        #     map_frame_point = convert_point([x, y, 0], data.header.frame_id, "map")
        #     cloud_points.append(map_frame_point)
        # cloud_points = np.array(cloud_points)
        # self.tree = KDTree(cloud_points, leaf_size=2)

    def check_collision_on_path(self, robot_pose, index_old):
        cost_list = []

        for i in range(index_old, index_old + 30):

            path_pose = self.path[i].pose
            base_x, base_y = self.ogm.get_costmap_x_y(path_pose.position.x, path_pose.position.y)
            print("base_x: %s, base_y: %s", base_x, base_y)
            origin = self.ogm.origin
            angle = math.atan2(origin.position.y, origin.position.x)  # * (180 / Math.PI)

            cos_theta = math.cos(get_yaw(path_pose.orientation))
            sin_theta = math.sin(get_yaw(path_pose.orientation))
            # cos_theta = math.cos(angle)
            # sin_theta = math.sin(angle)
            # for i in np.arange(0.5, 1.5, 0.2):
            # prev_b = path_pose.position.x
            # self.rotated_poses_pub

            """
            data.pose.pose.position.x - fcu_offset_vehicle * math.cos(yaw)
            """
            poses_list = []
            # min_cost

    def main_loop(self):
        rate = rospy.Rate(200)

        while not rospy.is_shutdown():
            robot_pose = current_robot_pose("map", self.base_frame)
            # print("robot_pose",robot_pose)
            if robot_pose is None:
                rospy.logerr("waiting for tf between map and %s", self.base_frame)
                rate.sleep()
                continue
            if self.tree is None:
                rospy.logwarn("waiting for Laser data")
                rate.sleep()
                continue

            if self.index_old is None:
                self.index_old, cross_track_dis = self.calc_nearest_ind(robot_pose)
            else:
                self.index_old, cross_track_dis = self.find_close_point(robot_pose, self.index_old)
            collision_points_list = []
            dis  = 0
            trajectory_msg = Trajectory()
            for ind in range(self.index_old, self.path_end_index+1):
                dis =  self.calc_distance(robot_pose, ind)
                dis += dis

                if dis > self.lookup_collision_dis:
                    break

                path_pose = self.path[ind].pose
                pose_xyz = np.array([[path_pose.position.x, path_pose.position.y, path_pose.position.z]])
                width = vehicle_data.dimensions.overall_width / 2
                width = 2
                collision_points = self.tree.query_radius(pose_xyz, r=width)
                collision_points_list.extend(list(collision_points[0]))
                trajectory_point_msg = TrajectoryPoint()

                if len(list(collision_points[0])) > 0:

                    trajectory_point_msg.pose = self.path[ind].pose
                    trajectory_point_msg.lateral_velocity_mps = 0
                    trajectory_msg.points.append(trajectory_point_msg)
                    for k in range(len(trajectory_msg.points)-1,len(trajectory_msg.points)-10,-1):
                        trajectory_msg.points[k].lateral_velocity_mps =0


                    break

                else:
                    trajectory_point_msg.pose = self.path[ind].pose
                    trajectory_point_msg.lateral_velocity_mps = self.global_vel[ind]

                trajectory_msg.points.append(trajectory_point_msg)

            self.publish_points(collision_points_list)
            self.publish_velocity_marker(trajectory_msg)
            self.trajectory_publisher.publish(trajectory_msg)

            rate.sleep()

    def publish_velocity_marker(self, trajectory):
        marker_arr_msg = MarkerArray()
        i = 0
        for traj_point in trajectory.points:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.TEXT_VIEW_FACING
            marker.text = str(round(traj_point.lateral_velocity_mps, 2))
            marker.id =  i
            i+=1
            marker.action = marker.ADD
            if traj_point.lateral_velocity_mps == 0:

                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

            marker.pose = traj_point.pose
            marker_arr_msg.markers.append(marker)
        self.trajectory_marker_publisher.publish(marker_arr_msg)




    def publish_points(self, collision_points_index):

        a = np.take(self.pc_np, list(collision_points_index), 0)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        scaled_polygon_pcl = pcd2.create_cloud_xyz32(header, a)
        rospy.loginfo("happily publishing sample pointcloud.. !")
        self.point_cloud_pub.publish(scaled_polygon_pcl)
    def velocity_smoother(self, robot_pose, ind, close_idx):
        """ Depending upon distance between the robot and the collision point, will set the speed and publish into
        velocity profile for path tracker """
        # dis = distance_btw_poses(robot_pose, self.path[ind].pose)
        # acc_dis = 0
        # for i, k in enumerate(range(ind, 0, -1)):
        #     acc_dis += distance_btw_poses(self.path[i].pose, self.path[i - 1].pose)
        #     if acc_dis > 2:
        #         stop_index = i
        #         break

        self.obstacle_aware_vel = list(copy.deepcopy(self.global_vel))

        object_dis = distance_btw_poses(robot_pose, self.path[ind].pose)
        if object_dis <= self.stop_distance:
            self.obstacle_aware_vel[ind:] = np.zeros(len(self.obstacle_aware_vel[ind:])).tolist()
            msg = Float32MultiArray()
            msg.data = self.obstacle_aware_vel

            self.vel_update_pub.publish(msg)
        else:
            acc_dis = 0
            vel_list = []
            for i in range(ind, close_idx, -1):
                acc_dis += distance_btw_poses(self.path[i].pose, self.path[i-1].pose)
                if acc_dis > self.collision_index_dis:
                    # self.stop_index = j
                    acc_dis = 0
                    for j in range(i, close_idx, -1):
                        acc_dis += distance_btw_poses(self.path[j].pose, self.path[j - 1].pose)
                        if acc_dis > 1:
                            diff_ind = j - i
                            for n, x in enumerate(range(j, i)):
                                vel = min((self.global_vel[x]/diff_ind) * (diff_ind-n), self.global_vel[x])
                                vel_list.append(vel)
                            self.obstacle_aware_vel[j:i] = vel_list
                            self.obstacle_aware_vel[i:] = np.zeros(len(self.obstacle_aware_vel[i:])).tolist()
                            msg = Float32MultiArray()
                            msg.data = self.obstacle_aware_vel

                            self.vel_update_pub.publish(msg)

                            break
                    break

        # print("global vel", len(self.obstacle_aware_vel))
        # print(type(self.global_vel))
        # # print(a)
        # self.obstacle_aware_vel = list(copy.deepcopy(self.global_vel))
        # for j in range(ind - 20, ind):
        #     self.obstacle_aware_vel[j] = 0.0
        #
        # # self.obstacle_aware_vel[ind-20:100] = tuple(np.zeros(120).tolist())
        # msg = Float32MultiArray()
        # msg.data = self.obstacle_aware_vel
        # # self.vel_update_pub.publish(msg)
        # print('data', self.obstacle_aware_vel[ind - 30:ind + 30])
        # print("ind", ind)
        #
        # rospy.loginfo("updated vel  published")
        # print(a)

        # if dis < self.spot_distance_before_collision_ind:
        #     pass
        # else:
        #     acc_dis = 0
        #     for j in range(ind, 0, -1):
        #         acc_dis += distance_btw_poses(self.revived_path[j], self.revived_path[j - 1])
        #         if acc_dis > self.spot_distance_before_collision_ind:
        #             self.stop_index = j
        #             break
        #     for i, k in enumerate(range(self.stop_index, 0, -1)):
        #         pass

    def velocity_marker(self, start_index, vel_list):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.TEXT_VIEW_FACING
        marker.text = str(round(vel, 2))
        marker.id = i
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.3
        marker.color.g = 1.0
        marker.color.b = 1.0
        # marker.lifetime = rospy.Duration(duration)
        marker.pose.orientation = Quaternion(odom_orientation['x'], odom_orientation['y'], odom_orientation['z'],
                                             odom_orientation['w'])
        marker.pose.position.x, marker.pose.position.y = odom_position['x'], odom_position['y']
        marker_arr_msg.markers.append(marker)

    def in_collision_marker_at_index(self, i):
        """ A Marker with connected points, with read colour indication collision will happen will be published"""
        pass

    def no_collision_markers_at_index(self, i):
        """ No collision with green colour box """
        pass

    def to_visualization_marker_array(self, modified_obj_data):
        marker_array = MarkerArray()
        self.obj_map_frame_pub.publish(modified_obj_data)
        print("punlished")

        # // create
        # markers
        # for bounding boxes
        #     Marker
        #     marker
        #     {};
        # marker.header = bboxes.header;
        # marker.ns = "bounding_box";
        # marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        # marker.action = Marker::ADD;
        # marker.lifetime = time_utils::to_message(std::chrono::nanoseconds(100000));
        # for o in object_corner_list:

'''
if __name__ == "__main__":
    rospy.init_node('obstacle_stop_planner_node')
    obj = ObstacleStopPlanner()
    rospy.spin()

'''

    self.parent_frame = parent_frame
    self.child_frame = child_frame
    self.bagfile = bagfile
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    self.lookup_frequency = lookup_frequency
    self.output_topic = output_topic
    self.append = append


    def run(self):
        msg_count = 0
        try:
            bag = rosbag.Bag(self.bagfile, mode='a' if self.append else 'w')
            rate = rospy.Rate(self.lookup_frequency)
            last_stamp = rospy.Time()
            while not rospy.is_shutdown():
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.parent_frame, self.child_frame, rospy.Time())
                    rate.sleep()
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException):
                    rate.sleep()
                    continue
                if last_stamp == transform.header.stamp:
                    continue
                pose = transformstamped_to_posestamped(transform)
                bag.write(self.output_topic, pose, t=pose.header.stamp)
                msg_count += 1
                last_stamp = transform.header.stamp
                rospy.loginfo_throttle(
                    10, "Recorded {} PoseStamped messages.".format(msg_count))

        except rospy.ROSInterruptException:
            pass
        finally:
            bag.close()
            rospy.loginfo("Finished recording.")


'''
