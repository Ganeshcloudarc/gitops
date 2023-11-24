#!/usr/bin/env python3

try:
    # python3 general packages
    import math
    from math import cos, sin
    import numpy as np

    import copy

    np.float = np.float64
    import ros_numpy
    import rospy, rospkg, logging
    import tf2_ros
    import time
    import sys, json
    import random

    # ros messages
    from nav_msgs.msg import Path, Odometry
    from jsk_recognition_msgs.msg import BoundingBoxArray, PolygonArray
    from zed_interfaces.msg import ObjectsStamped, Object
    from geometry_msgs.msg import Point, PoseArray, Pose, TransformStamped, PoseStamped, PolygonStamped, Point
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import Float32MultiArray, Header, Float32
    from sensor_msgs.msg import PointCloud2, LaserScan
    from std_msgs.msg import Float32
    from mavros_msgs.msg import HomePosition
    from geographic_msgs.msg import GeoPointStamped, GeoPose

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
    from autopilot_utils.geonav_conversions import ll2xy, xy2ll
    from autopilot_msgs.msg import Trajectory, TrajectoryPoint
    # from velocity_planner import VelocityPlanner
    from utils.auto_nav_utils import Pose2D, ganerate_random_pairs, xy_to_pointcloud2, two_points_to_path, path_to_traj, \
        GetAngle, min_distance_to_object, circumradius, Line, get_tie, pose_array_to_pose_stamped, \
        dubins_path_to_ros_path, two_points_to_line
    from utils.bboxes_utils import inside_radius
    from utils.dwa_path_candidate_generator import DwaPathGenerator
    from utils.polygon_collision_check import PolygonCheck
    from utils.ransac_impl import RansacParallelLineFit

    from utils.sub_handler import SubscriberHandler
    from utils.pub_handler import PublisherHandler
    from utils.follow_gap import get_the_vanish_points

    # python packages
    from sklearn.linear_model import LinearRegression
    import dubins
    import random


except Exception as e:
    import rospy

    rospy.logerr("Module error %s", str(e))
    exit()

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


def set_rospy_log_lvl(log_level):
    logger = logging.getLogger('rosout')
    logger.setLevel(rospy.impl.rosout._rospy_to_logging_levels[log_level])


def get_coords_from_geojson(route_file):
    ros_pack = rospkg.RosPack()
    try:
        geojson_file_dir = ros_pack.get_path('auto_nav') + "/routes/" + str(route_file)

    except Exception as error:
        rospy.logerr(str(error))
        return None
    print(geojson_file_dir)
    try:
        data = json.load(open(geojson_file_dir))
    except Exception as error:
        rospy.logerr(str(error))
        return None
    long_lat_list = data["features"][0]["geometry"]['coordinates']
    return long_lat_list


def route_to_path(route, frame_id="map"):
    path_msg = Path()
    path_msg.header.frame_id = frame_id
    for path in route:
        for point in path:
            pose_st = PoseStamped()
            pose_st.header.frame_id = frame_id
            pose_st.pose.position.x = point[0]
            pose_st.pose.position.y = point[1]
            pose_st.pose.orientation.z = 1
            path_msg.poses.append(pose_st)
    return path_msg


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


def dwa_paths_marker_array(dwa_paths, target_frame="base_link"):
    marker_arr_msg = MarkerArray()
    # marker_arr_msg.header.frame_id = target_frame
    count = 0
    length = len(dwa_paths)
    for path in dwa_paths:
        marker = Marker()
        marker.header.frame_id = target_frame
        # marker.header.stamp = rospy.Time.now()
        marker.type = marker.LINE_STRIP
        marker.id = count
        count += 1
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 0.5
        marker.color.r = random.random()
        marker.color.g = random.random()
        marker.color.b = random.random()
        for point in path:
            pt = Point()
            pt.x = point[0]
            pt.y = point[1]
            marker.points.append(pt)
        marker_arr_msg.markers.append(marker)
    return marker_arr_msg


class SetHome:
    def __init__(self):
        # to set mavros home position
        self.starting_point_pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped,
                                                  queue_size=1, latch=True)
        self.home_position_pub = rospy.Publisher('/mavros/global_position/home', HomePosition,
                                                 queue_size=1, latch=True)

    def set_home_position(self, home_lat, home_long, home_alt=-60):
        geo_point = GeoPointStamped()
        geo_point.position.latitude = home_lat
        geo_point.position.longitude = home_long
        geo_point.position.altitude = home_alt
        home_position_msg = HomePosition()
        home_position_msg.geo = geo_point.position
        self.starting_point_pub.publish(geo_point)
        self.home_position_pub.publish(home_position_msg)
        rospy.logdebug("Home position is published")
        return geo_point.position


class AutoNavRoutePlanning:
    def __init__(self):
        """
        Get params
        get the robot location
        find the closest point on the line
        find the center line, headed in the direction of route lines
        constuct 3d matrx with coords row lenght and skip rows and update every time
        use ransac to fit line in the direction of route.

        """
        # rospy.get_param("route_planning")
        self.route_file_name = rospy.get_param("route_planning/route_file_name", "example_route.geojson")
        self.row_spacing = rospy.get_param("route_planning/row_spacing", 6)
        self.tree_width = rospy.get_param("route_planning/tree_width", 6)
        self.calculate_row_heading_from_coord = rospy.get_param("route_planning/calculate_row_heading_from_coord", True)
        self.row_heading = rospy.get_param("route_planning/row_heading", 90)
        self.headland_width = rospy.get_param("route_planning/Headland_width", 8)
        self.skip_rows = rospy.get_param("route_planning/skip_rows", 2)
        self.minimin_turing_radius_of_vehicle = rospy.get_param("route_planning/minimin_turing_radius_of_vehicle", 5)
        self.max_number_of_rows_to_cover = rospy.get_param("route_planning/max_number_of_rows_to_cover", 10)
        self.reversing_enabled = rospy.get_param("route_planning/reversing_enabled", False)
        self.ransac_max_iterations = rospy.get_param("route_planning/ransac_max_iterations", 50)
        print("self.route_file_nam", self.route_file_name)
        long_lat_list = get_coords_from_geojson(self.route_file_name)
        if long_lat_list is None:
            rospy.signal_shutdown("no points ")

        # long_lat_list = long_lat_list[:2]
        print("long_lat_list", long_lat_list)
        lat_long_list = [[data[1], data[0]] for data in long_lat_list]
        print("lat_long_list", lat_long_list)
        start_lat, start_lon = home_lat, home_lon = lat_long_list[0][0], lat_long_list[0][1]

        end_lat, end_lon = lat_long_list[1][0], lat_long_list[1][1]
        set_home = SetHome()
        set_home.set_home_position(home_lat, home_lon)
        del set_home
        self.sub_handler = SubscriberHandler()
        self.pub_handler = PublisherHandler()
        print("end_lat", end_lat,end_lon )
        self.first_row_start_point = [0,0]
        self.first_row_end_point = list(ll2xy(end_lat, end_lon, home_lat, home_lon))

        final_path ,_= self.route_plan(start_point=self.first_row_start_point, end_point=self.first_row_end_point)
        row_direction = math.atan2(self.first_row_end_point[1] - self.first_row_start_point[1], self.first_row_end_point[0] - self.first_row_start_point[0])


        rate = rospy.Rate(1)
        rate.sleep()

        """ Uncomment on actual implementation
        while not rospy.is_shutdown():
            status = self.sub_handler.data_received()
            if status[0]:
                rospy.loginfo(status[1])
                break
            else:
                rospy.logwarn(status[1])
                rate.sleep()
                continue
        """

        # check for the distance between robot and first line and compare headings
        first_row_line = two_points_to_line(final_path[0], final_path[1])

        print("first_row_line", first_row_line)
        robot_pose = Pose2D(self.sub_handler.get_odom().pose.pose.position.x, self.sub_handler.get_odom().pose.pose.position.y,
                            get_yaw(self.sub_handler.get_odom().pose.pose.orientation))
        print("robot_pose", robot_pose)
        dis_to_first_row = first_row_line.distance_to_point(robot_pose)
        close_point_on_first_line = first_row_line.intersct_point_to_line(robot_pose)
        close_point_on_first_line = Pose2D(close_point_on_first_line[0], close_point_on_first_line[1], row_direction)
        print("dis to line", dis_to_first_row)
        print("close_point_on_first_line ", close_point_on_first_line)
        if not robot_pose.heading_check(close_point_on_first_line):
            rospy.logerr("Robot heading is not the direction of line")
            # rospy.signal_shutdown("Robot heading is not the direction of line")
        else:
            rospy.loginfo("Robot heading and First_row_line headings are in the same direction")

        if dis_to_first_row >= self.row_spacing:
            rospy.logerr("Robot is not close to first row line")
            # rospy.signal_shutdown("Robot is not close to first row line")
        else:
            rospy.loginfo(f"Robot to  First_row_line distance is less than {self.row_spacing} ")

        # initialise Dwa planner
        wheel_base = vehicle_data.dimensions.wheel_base
        steering_angle_max = vehicle_data.motion_limits.max_steering_angle
        steering_inc = 1
        max_path_length = 6
        path_resolution = 1.0
        dwa_path_gen = DwaPathGenerator(wheel_base, steering_angle_max, steering_inc, max_path_length, path_resolution)
        self.dwa_paths = dwa_path_gen.generate_paths()
        self.dwa_marker = dwa_paths_marker_array(self.dwa_paths)
        self.pub_handler.dwa_paths_pub.publish(self.dwa_marker)
        rospy.logdebug("DWA paths are published as MarkerArray")
        # from steering_angle_max to steering_angle_max, with  steering_inc



        self.ranscan = RansacParallelLineFit(parallel_offset=self.row_spacing,
                                        max_number_of_iterations=self.ransac_max_iterations,
                                        offset=self.tree_width)
        # self.main()

        self.main_dwa()

    def main(self):
        rate = rospy.Rate(10)
        robot_pose = self.sub_handler.get_odom().pose.pose
        robot_start_point_2d = Pose2D(robot_pose.position.x, robot_pose.position.y, get_yaw(robot_pose.orientation))
        print("robot_start_point", robot_start_point_2d)
        from_start_dis = 0
        while not rospy.is_shutdown():

            robot_pose = self.sub_handler.get_odom().pose.pose
            robot_pose_2d = Pose2D(robot_pose.position.x, robot_pose.position.y, get_yaw(robot_pose.orientation))
            print("robot_pose_2d", robot_pose_2d)
            from_start_dis = robot_start_point_2d.distance(robot_pose_2d)
            print("from_start_dis", from_start_dis)

            rospy.logdebug("-----------------------------")
            self.pub_handler.dwa_paths_pub.publish(self.dwa_marker)
            rospy.logdebug("DWA paths are published as MarkerArray")
            bboxes = self.sub_handler.get_bboxes("base_link")
            laser_cloud = self.sub_handler.get_scan()
            if laser_cloud is not None:
                print("laser_cloud", type(laser_cloud))
                print("laser_cloud", laser_cloud.shape)
                print("laser_raw", len(self.sub_handler.laser_scan_data.ranges))

            # rate.sleep()
            # continue

            if bboxes:
                pass
            else:
                rospy.logwarn("no Bounding boxes")
                rate.sleep()
                continue

            print("bboxes len ", len(bboxes.boxes))
            radius_filtered_bboxes = inside_radius(bboxes, 2 * self.row_spacing)
            print("radius_filtered_bboxes len ", len(radius_filtered_bboxes))
            polygon_arr_msg = PolygonArray()
            polygon_arr_msg.header.frame_id = "base_link"
            for bbox in radius_filtered_bboxes:
                polygon = PolygonStamped()

                polygon.header.frame_id = "base_link"
                box_list = bbox_to_corners(bbox, scale=1.2)
                for x, y in box_list:
                    pt = Point()
                    pt.x = x
                    pt.y = y
                    polygon.polygon.points.append(pt)
                pt = Point()
                pt.x = box_list[0][0]
                pt.y = box_list[0][1]
                polygon.polygon.points.append(pt)
                polygon_arr_msg.polygons.append(polygon)
            self.pub_handler.enlarged_bboxes_pub.publish(polygon_arr_msg)

            # collision checking between Bboxes and DWA paths
            pc = PolygonCheck(polygon_arr_msg)
            is_inside = pc.is_point_inside([4, 2])
            print("point_inside", is_inside)
            collision_free_paths = []
            for path in self.dwa_paths:
                in_collision_flag = False
                for point in path:
                    is_inside = pc.is_point_inside(point)
                    if is_inside:
                        in_collision_flag = True
                        break
                    else:
                        pass
                if in_collision_flag:
                    continue
                else:
                    collision_free_paths.append(path)
            print("len of collision free paths ", len(collision_free_paths))
            self.pub_handler.dwa_collision_free_paths_pub.publish(dwa_paths_marker_array(collision_free_paths))
            dis_list = []
            """
            path = two_points_to_path(start_point, end_point, "map")
            first_row_points_in_base_link = convert_path(path, "base_link")
            if not first_row_points_in_base_link:
                rospy.logwarn("errr in first_row_points_in_base_link ")
                rate.sleep()
                continue

            first_row_line_base_link = two_points_to_line([first_row_points_in_base_link.poses[0].pose.position.x,
                                                           first_row_points_in_base_link.poses[0].pose.position.y],
                                                          [first_row_points_in_base_link.poses[1].pose.position.x,
                                                           first_row_points_in_base_link.poses[1].pose.position.y])
            """
            # if from_start_dis > 2 * row_length:
            # compute center line.
            point_cloud_local_map = self.sub_handler.get_local_laser_scan("base_link")
            # point_cloud_local_map = self.sub_handler.get_scan("base_link")

            if point_cloud_local_map is None:
                rospy.logwarn("errr in first_row_points_in_base_link ")
                rate.sleep()
                continue
            end_pose_in_laser_frame = convert_point(point=[self.first_row_end_point[0], self.first_row_end_point[1], 0], from_frame="map",
                                                    to_frame="rslidar")
            if end_pose_in_laser_frame is None:
                rospy.logwarn("errr in end_pose_in_base_link ")
                rate.sleep()
                continue
            rospy.logerr(f"end_pose_in_base_link : {end_pose_in_laser_frame}")

            # rospy.logwarn("vanish point not crossed the end point")
            # Follow gap method
            data = self.sub_handler.get_raw_scan()
            vanish_pose = get_the_vanish_points(data, end_pose_in_laser_frame)

            vanish_pose_base = convert_pose(vanish_pose.pose, "rslidar", "base_link")
            end_point_2d = Pose2D(vanish_pose_base.position.x, vanish_pose_base.position.y)
            print("end pose", end_point_2d)

            self.vanish_pose_map = convert_pose(vanish_pose.pose, "rslidar", "map")
            self.vanish_pose_2d_map = Pose2D(self.vanish_pose_map.position.x, self.vanish_pose_map.position.y)
            self.pub_handler.vanish_pose_pub.publish(self.vanish_pose_2d_map.to_posestamped("map"))
            rospy.logdebug("published vanish_pose_pub")
            modified_center_line = two_points_to_line([0, 0], self.vanish_pose_2d_map.to_list())
            end_point = modified_center_line.intersct_point_to_line(
                Pose2D(self.first_row_end_point[0], self.first_row_end_point[1]))

            path = self.route_plan([0, 0], end_point)
            end_point2d = Pose2D(end_point[0], end_point[1])
            ## Finding distance between robot to next point.
            distance_to_end_point = robot_pose_2d.distance(end_point2d)
            if distance_to_end_point < 3:

                rospy.logerr("turn detected")



            end_point = convert_point([end_point[0], end_point[1], 0], "map", "base_link")
            print("end_point", end_point)

            coffes, left_inliers_index, right_inliers_index = self.ranscan.compute_based_of_target_pose(
                cloud_points=point_cloud_local_map, forward_point=end_point, row_spacing=self.row_spacing)

            print("coffes", coffes)
            val = 30

            # left_line = Line(coffes[0], coffes[1])
            # right_line = Line(coffes[0], coffes[2])
            # val = 30
            # p1 = [val, coffes[0] * val + coffes[2]]
            # p2 = [-val, coffes[0] * -val + coffes[2]]
            # path = two_points_to_path(p1, p2, "base_link")
            # self.pub_handler.right_line_pub.publish(path)
            # rospy.logdebug("right path published")
            # p1 = [val, coffes[0] * val + coffes[1]]
            # p2 = [-val, coffes[0] * -val + coffes[1]]
            # path = two_points_to_path(p1, p2, "base_link")
            # self.pub_handler.left_line_pub.publish(path)

            print("left_inliers_index", left_inliers_index)
            print("right_inliers_index", right_inliers_index)

            left_cloud, right_cloud = np.take(point_cloud_local_map, left_inliers_index, axis=0), np.take(
                point_cloud_local_map, right_inliers_index, axis=0)
            self.pub_handler.left_inliers_pub.publish(xy_to_pointcloud2(left_cloud, "base_link"))
            self.pub_handler.right_inliers_pub.publish(xy_to_pointcloud2(right_cloud, "base_link"))
            rospy.logwarn("left and right inliers are publisued")
            model = LinearRegression()
            model.fit(left_cloud[:, 0, np.newaxis],
                      left_cloud[:, 1, np.newaxis])
            left_line_slope_base_link, left_y_intercept_base_link = model.coef_[0][0], model.intercept_[0]
            p1 = [val, left_line_slope_base_link * val + left_y_intercept_base_link]
            p2 = [-val, left_line_slope_base_link * -val + left_y_intercept_base_link]
            path = two_points_to_path(p1, p2, "base_link")
            self.pub_handler.left_line_pub.publish(path)
            model = LinearRegression()
            model.fit(right_cloud[:, 0, np.newaxis],
                      right_cloud[:, 1, np.newaxis])
            right_line_slope_base_link, right_y_intercept_base_link = model.coef_[0][0], model.intercept_[0]
            p1 = [val, right_line_slope_base_link * val + right_y_intercept_base_link]
            p2 = [-val, right_line_slope_base_link * -val + right_y_intercept_base_link]
            path = two_points_to_path(p1, p2, "base_link")
            self.pub_handler.right_line_pub.publish(path)

            center_line_slope_base_link = (left_line_slope_base_link + right_line_slope_base_link) / 2
            center_line_y_intercept_base_link = (left_y_intercept_base_link + right_y_intercept_base_link) / 2
            center_line = Line(center_line_slope_base_link, center_line_y_intercept_base_link)
            p1 = [val, center_line.m * val + center_line.c]
            p2 = [-val, center_line.m * -val + center_line.c]
            path = two_points_to_path(p1, p2, "base_link")
            self.pub_handler.center_line_pub.publish(path)
            # if math.hypot(robot_start_point[0]- )
            # print("first_row_in_base_link", first_row_line_base_link)

            for i in range(len(collision_free_paths)):
                path = collision_free_paths[i]
                # print("path[-1]", path[-1])
                # print(type(path[-1].tolist()))
                dis = center_line.distance_to_point(path[-1].tolist())
                dis_list.append(abs(dis))
            # print("dis_list", dis_list)
            if not len(dis_list) > 0:
                rospy.logwarn("could not found collision free path")
                rate.sleep()
                continue

            min_dis_idx = np.argmin(dis_list)
            print("min_dis_idx", min_dis_idx)
            close_path_to_first_row = collision_free_paths[min_dis_idx]

            dwa_output = points_to_path(close_path_to_first_row, "base_link")
            # self.pub_handler.dwa_collision_free_and_close_to_line_pub.publish(dwa_output)
            dwa_output_map = convert_path(dwa_output, "map")
            self.pub_handler.dwa_collision_free_and_close_to_line_pub.publish(dwa_output_map)
            traj_msg = Trajectory()
            traj_msg.header.frame_id = "map"
            speed = 1
            for i in range(len(dwa_output_map.poses)):
                traj_point = TrajectoryPoint()
                traj_point.pose = dwa_output_map.poses[i].pose
                traj_point.index = i
                traj_point.longitudinal_velocity_mps = speed

                if i == 0:
                    traj_point.accumulated_distance_m = 0
                else:
                    acc_dis = math.hypot(traj_msg.points[-1].pose.position.x - dwa_output_map.poses[i].pose.position.x,
                                         traj_msg.points[-1].pose.position.y - dwa_output_map.poses[i].pose.position.y)
                    traj_point.accumulated_distance_m = traj_msg.points[-1].accumulated_distance_m + acc_dis
                traj_msg.points.append(traj_point)
            self.pub_handler.local_trajectory_pub.publish(traj_msg)

            rate.sleep()
    def main_dwa(self):
        start_point = [0.1, 0]
        end_point = [0, 110]
        path, snake_path = self.route_plan(start_point, end_point)
        snake_path_ros = points_to_path(snake_path)
        self.pub_handler.snake_path_pub.publish(snake_path_ros)
        rospy.logdebug("snake_path  published")
        count = 0
        rate = rospy.Rate(10)
        while count < 2 * self.max_number_of_rows_to_cover:
            rospy.logdebug("-----------------------------")
            rospy.logdebug(f"count : {count}")
            robot_pose = self.sub_handler.get_odom().pose.pose
            robot_pose_2d = Pose2D(robot_pose.position.x, robot_pose.position.y, get_yaw(robot_pose.orientation))

            p1 = convert_point(snake_path[count], "map", "base_link")
            p2 = convert_point(snake_path[count+1], "map", "base_link")
            if p1 and p2:
                pass
            else:
                rospy.logerr("error in tf")
                rate.sleep()
                continue
            current_line = two_points_to_line(p1, p2)

            curr_path = two_points_to_path(p1, p2, "base_link")
            self.pub_handler.center_line_pub.publish(curr_path)
            rospy.logdebug("current line published")

            dis = math.hypot(robot_pose_2d.x - snake_path[count+1][0], robot_pose_2d.y - snake_path[count+1][1])
            print("distance", dis)

            if dis < 3:
                count = count + 1
                rate.sleep()
                continue
            # rate.sleep()
            # continue


            print("current line", current_line)
            next_line = two_points_to_line(snake_path[count+1], snake_path[count+2])

            self.pub_handler.dwa_paths_pub.publish(self.dwa_marker)
            rospy.logdebug("DWA paths are published as MarkerArray")
            bboxes = self.sub_handler.get_bboxes("base_link")

            if bboxes is None:
                rospy.logwarn("no Bounding boxes")
                rate.sleep()
                continue

            print("bboxes len ", len(bboxes.boxes))
            radius_filtered_bboxes = inside_radius(bboxes, 2 * self.row_spacing)
            print("radius_filtered_bboxes len ", len(radius_filtered_bboxes))
            polygon_arr_msg = PolygonArray()
            polygon_arr_msg.header.frame_id = "base_link"
            for bbox in radius_filtered_bboxes:
                polygon = PolygonStamped()

                polygon.header.frame_id = "base_link"
                box_list = bbox_to_corners(bbox, scale=1.2)
                for x, y in box_list:
                    pt = Point()
                    pt.x = x
                    pt.y = y
                    polygon.polygon.points.append(pt)
                pt = Point()
                pt.x = box_list[0][0]
                pt.y = box_list[0][1]
                polygon.polygon.points.append(pt)
                polygon_arr_msg.polygons.append(polygon)
            self.pub_handler.enlarged_bboxes_pub.publish(polygon_arr_msg)

            # collision checking between Bboxes and DWA paths
            pc = PolygonCheck(polygon_arr_msg)
            is_inside = pc.is_point_inside([4, 2])
            print("point_inside", is_inside)
            collision_free_paths = []
            for path in self.dwa_paths:
                in_collision_flag = False
                for point in path:
                    is_inside = pc.is_point_inside(point)
                    if is_inside:
                        in_collision_flag = True
                        break
                    else:
                        pass
                if in_collision_flag:
                    continue
                else:
                    collision_free_paths.append(path)
            print("len of collision free paths ", len(collision_free_paths))
            self.pub_handler.dwa_collision_free_paths_pub.publish(dwa_paths_marker_array(collision_free_paths))
            dis_list = []
            heading_diff_list = []
            total_list = []
            for i in range(len(collision_free_paths)):
                path = collision_free_paths[i]
                # print("path[-1]", path[-1])
                # print(type(path[-1].tolist()))
                dis = current_line.distance_to_point(path[-1].tolist())
                angle_diff = current_line.heading() - path[-1].tolist()[2]
                heading_diff_list.append(angle_diff)
                total_list.append(angle_diff+angle_diff)

                dis_list.append(abs(dis))
            print("dis_list", dis_list)
            if not len(total_list) > 0:
                rospy.logwarn("could not found collision free path")
                rate.sleep()
                continue

            min_dis_idx = np.argmin(dis_list)
            print("min_dis_idx", min_dis_idx)
            close_path_to_first_row = collision_free_paths[min_dis_idx]

            dwa_output = points_to_path(close_path_to_first_row, "base_link")
            # self.pub_handler.dwa_collision_free_and_close_to_line_pub.publish(dwa_output)
            dwa_output_map = convert_path(dwa_output, "map")
            self.pub_handler.dwa_collision_free_and_close_to_line_pub.publish(dwa_output_map)
            traj_msg = Trajectory()
            traj_msg.header.frame_id = "map"
            speed = 1
            for i in range(len(dwa_output_map.poses)):
                traj_point = TrajectoryPoint()
                traj_point.pose = dwa_output_map.poses[i].pose
                traj_point.index = i
                traj_point.longitudinal_velocity_mps = speed

                if i == 0:
                    traj_point.accumulated_distance_m = 0
                else:
                    acc_dis = math.hypot(traj_msg.points[-1].pose.position.x - dwa_output_map.poses[i].pose.position.x,
                                         traj_msg.points[-1].pose.position.y - dwa_output_map.poses[i].pose.position.y)
                    traj_point.accumulated_distance_m = traj_msg.points[-1].accumulated_distance_m + acc_dis
                traj_msg.points.append(traj_point)
            self.pub_handler.local_trajectory_pub.publish(traj_msg)




            rate.sleep()










    def route_plan(self, start_point, end_point):
        # Creation of routes
        start_point = start_point
        end_point = end_point
        # end_point =
        print("end_point", end_point)
        row_direction = math.atan2(end_point[1] - start_point[1], end_point[0] - start_point[0])
        row_length = math.hypot(end_point[1] - start_point[1], end_point[0] - start_point[0])
        print("row_direction ", math.degrees(row_direction))
        print("distance ", row_length)
        direction = "left"
        if direction == "left":
            heading_change = np.pi / 2
        else:
            heading_change = -np.pi / 2
        route = [[start_point, end_point]]
        # route = [[ [start_point[0], start_point[1], row_direction], [end_point[0], end_point[1], row_direction + heading_change]]]
        for i in range(1, self.max_number_of_rows_to_cover):
            theta = heading_change
            rot = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
            vect = get_tie(row_direction) * self.row_spacing * self.skip_rows * i
            dot_pro = np.dot(rot, vect)
            start_point_i = [start_point[0] + dot_pro[0], start_point[1] + dot_pro[1]]
            turn_end_point_yaw = normalize_angle(row_direction + np.pi)
            end_point_i = [end_point[0] + dot_pro[0], end_point[1] + dot_pro[1]]
            route.append([start_point_i, end_point_i])
        # print("route", route)
        route_path = route_to_path(route)
        self.pub_handler.route_path_pub.publish(route_path)
        rospy.logdebug("route path published")

        snake_patten = []
        for i in range(len(route)):
            if i % 2 == 0:
                for j in range(len(route[i])):
                    # print("i", i)
                    # print("route[i][j]", route[i][j])
                    # snake_patten.append(route[i][j].append(row_direction))
                    snake_patten.append([route[i][j][0], route[i][j][1], row_direction])
            else:
                for j in range(len(route[i]) - 1, -1, -1):
                    # snake_patten.append(route[i][j].append(row_direction))
                    snake_patten.append([route[i][j][0], route[i][j][1], normalize_angle(row_direction + np.pi)])
        # print("snake_patten", snake_patten)
        snake_path_ros = points_to_path(snake_patten)
        self.pub_handler.snake_path_pub.publish(snake_path_ros)
        rospy.logdebug("snake_path  published")

        # assigning headings

        # connect the end points

        i = 0
        final_path = []
        while i < len(snake_patten) - 1:
            # if even pass, if odd, connect the next one with dubins curves
            if i % 2 == 0:
                final_path.append(snake_patten[i])
            else:

                # dubins_path = dubins.path([snake_patten[i][0], snake_patten[i][1], snake_patten[i][2]],
                #                           [snake_patten[i+1][0], snake_patten[i+1][1], snake_patten[i+1][2]],
                #                           self.minimin_turing_radius_of_vehicle, 3)
                # shortest_path
                dubins_path = dubins.shortest_path([snake_patten[i][0], snake_patten[i][1], snake_patten[i][2]],
                                                   [snake_patten[i + 1][0], snake_patten[i + 1][1],
                                                    snake_patten[i + 1][2]],
                                                   self.minimin_turing_radius_of_vehicle)
                step_size = 0.3
                dubins_path = dubins_path.sample_many(step_size)[0]
                # print(dubins_path)
                final_path.extend(dubins_path)
            i = i + 1
        # print("final_path", final_path)

        final_dubins_path_ros = points_to_path(final_path)
        self.pub_handler.dubins_path_pub.publish(final_dubins_path_ros)
        rospy.logdebug("dubins_path published")
        return final_path, snake_patten


if __name__ == "__main__":
    rospy.init_node("auto_nav_route_planning")
    set_rospy_log_lvl(rospy.DEBUG)
    RoutePlanning = AutoNavRoutePlanning()
    rospy.spin()
