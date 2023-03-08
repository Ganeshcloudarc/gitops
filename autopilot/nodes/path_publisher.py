#!/usr/bin/env python3

try:
    import rospy
    import rospkg
    from nav_msgs.msg import Path
    from geographic_msgs.msg import GeoPointStamped, GeoPose
    from mavros_msgs.msg import HomePosition
    from geometry_msgs.msg import Pose, PoseStamped, Quaternion, PolygonStamped
    from std_msgs.msg import Float32MultiArray, Int32MultiArray, Bool, Int16, String, Int8, UInt16
    from visualization_msgs.msg import Marker, MarkerArray
    import json, time, math, sys, os
    import numpy as np
    from vehicle_common.vehicle_config import vehicle_data
    from autopilot_msgs.msg import Trajectory, TrajectoryPoint
    from autopilot_utils.geonav_conversions import xy2ll, ll2xy
    from autopilot_utils.pose_helper import distance_btw_poses, get_yaw, angle_btw_poses, yaw_to_quaternion
    from autopilot_utils.trajectory_helper import trajectory_to_path, trajectory_to_marker
    from autopilot_utils.trajectory_common import TrajectoryManager
    from fastkml import kml
    from fastkml import geometry
    import geometry_msgs.msg as gmsg


except Exception as e:
    import rospy

    rospy.logerr("module named %s", str(e))
    exit()


class GlobalGpsPathPub:
    def __init__(self, mission_file_dir):
        self._traj_manager = TrajectoryManager()
        # parameters for path publisher
        self.max_forward_speed = rospy.get_param('/patrol/max_forward_speed', 1.5)
        self.min_forward_speed = rospy.get_param("/patrol/min_forward_speed", 0.3)
        distance_to_slowdown_on_endsdistance_to_slowdown_on_ends = rospy.get_param("/path_publisher/distance_to_slowdown_on_ends", 3)
        self.mission_continue = rospy.get_param("patrol/mission_continue", True)
        self.max_dis_btw_points = rospy.get_param("path_publisher/max_dis_btw_points", 0.5)
        self.path_resolution = rospy.get_param("path_publisher/path_resolution", 0.1)
        self.steering_limits_to_slow_down = rospy.get_param("path_publisher/steering_limits_to_slow_down", 10)
        self.speed_reduce_factor = rospy.get_param("path_publisher/speed_reduce_factor", 0.7)
        self.min_look_ahead = rospy.get_param("pure_pursuit/min_look_ahead_dis", 3)
        self.max_look_ahead = rospy.get_param("pure_pursuit/max_look_ahead_dis", 6)
        self.avg_lhd = (self.min_look_ahead + self.max_look_ahead) / 2
        self.rear_axle_center_height_from_ground = vehicle_data.dimensions.tyre_radius
        self.mission_file_dir = mission_file_dir

        # publishers
        self.global_trajectory_pub = rospy.Publisher('global_gps_trajectory', Trajectory, queue_size=1, latch=True)
        self.gps_path_pub = rospy.Publisher('/global_gps_path', Path, queue_size=1, latch=True)
        # to set mavros home position
        self.starting_point_pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped,
                                                  queue_size=1, latch=True)
        self.home_position_pub = rospy.Publisher('/mavros/global_position/home', HomePosition,
                                                 queue_size=1, latch=True)
        self.trajectory_velocity_marker_pub = rospy.Publisher('/global_gps_trajectory_velocity_marker', MarkerArray,
                                                              queue_size=1, latch=True)

        # debug
        self.polygon_xy_pub = rospy.Publisher('/xy_polygon', PolygonStamped,
                                           queue_size=1, latch=True)
        self.polygon_xy_filled_pub = rospy.Publisher('/polygon_xy_filled_', PolygonStamped,
                                           queue_size=1, latch=True)

    def to_polygon(self, xy_list):
        polygon_st = PolygonStamped()
        polygon_st.header.frame_id = 'map'
        for x, y in xy_list:
            point = gmsg.Point()
            point.x, point.y = x, y
            polygon_st.polygon.points.append(point)
        return polygon_st

    def from_json(self):
        data = None
        try:
            data = json.load(open(self.mission_file_dir))
        except Exception as error:
            rospy.logerr('Error In Reading mission file ' + str(error))
            rospy.signal_shutdown('Error In Reading mission file ' + str(error))
        if 'coordinates' in data.keys() and "odometry" in data.keys():
            rospy.loginfo("odometry values found")
            self.from_odometry(data)

        elif 'coordinates' in data.keys():
            long_lat_list = data['coordinates']
            if len(long_lat_list) >= 2:
                self.publish_path_from_long_lat(long_lat_list)
            else:
                rospy.logerr("No points available in mission file")
                rospy.signal_shutdown("No points available in mission file")
        elif 'features' in data.keys():
            feature = data['features'][0]
            # first feature
            if feature['geometry']['type'] == "LineString":
                rospy.loginfo("type matched to LineString")

            else:
                rospy.logerr(f"geometry does not match, Required LineString, given :{feature['geometry']['type']}")
                rospy.signal_shutdown(
                    f"geometry does not match, Required LineString, given :{feature['geometry']['type']}")

            long_lat_list = feature['geometry']['coordinates']
            if len(long_lat_list) >= 2:
                self.publish_path_from_long_lat(long_lat_list)
            else:
                rospy.logerr("No points available in mission file")
                rospy.signal_shutdown("No points available in mission file")
        else:
            rospy.logerr("could not found proper fields in mission file")
            rospy.signal_shutdown("No points available in mission file")

    def from_kml(self):

        try:
            with open(self.mission_file_dir, 'r') as geo_fence:
                doc = geo_fence.read().encode('utf-8')
        except Exception as error:
            rospy.logerr('Error In Reading mission file ' + str(error))
            rospy.signal_shutdown('Error In Reading mission file ' + str(error))

        k = kml.KML()
        k.from_string(doc)
        long_lat_list = []
        for i in k.features():
            for j in i.features():
                if isinstance(j.geometry, geometry.LineString):
                    polygon = j.geometry
                    for coordinates in polygon.coords:
                        # lat, long format
                        long_lat_list.append([coordinates[0], coordinates[1]])
                else:
                    rospy.logerr(f"KML file format is not LineString")
                    rospy.signal_shutdown("KML file format is not LineString")
        if len(long_lat_list) >= 2:
            self.publish_path_from_long_lat(long_lat_list)
        else:
            rospy.logerr("No points available in mission file")
            rospy.signal_shutdown("No points available in mission file")

    def set_home_position(self, home_lat, home_long, home_alt=-60):
        geo_point = GeoPointStamped()
        geo_point.position.latitude = home_lat
        geo_point.position.longitude = home_long
        geo_point.position.altitude = home_alt
        home_position_msg = HomePosition()
        home_position_msg.geo = geo_point.position
        self.starting_point_pub.publish(geo_point)
        self.home_position_pub.publish(home_position_msg)
        return geo_point.position

    def publish_path_from_long_lat(self, long_lat_list):
        home_lat = long_lat_list[0][1]
        home_long = long_lat_list[0][0]
        home_position = self.set_home_position(long_lat_list[0][1], long_lat_list[0][0])
        rospy.loginfo('Origin point set')
        time.sleep(0.5)
        xy_list = []
        for i in range(len(long_lat_list)):
            x, y = ll2xy(long_lat_list[i][1], long_lat_list[i][0], home_lat, home_long)
            xy_list.append([x, y])
        self.polygon_xy_pub.publish(self.to_polygon(xy_list))

        x_y_filled = [xy_list[0]]

        for i in range(len(xy_list) - 1):
            dis = math.hypot(xy_list[i + 1][0] - xy_list[i][0], xy_list[i + 1][1] - xy_list[i][1])
            if dis < self.max_dis_btw_points:

                x_y_filled.append(xy_list[i])
            else:
                # x_y_filled.append(xy_list[i])
                number_of_points = int(dis / self.path_resolution)
                angle = math.atan2(xy_list[i + 1][1] - xy_list[i][1], xy_list[i + 1][0] - xy_list[i][0])
                for _ in range(number_of_points):
                    px = x_y_filled[-1][0]
                    py = x_y_filled[-1][1]
                    px_updated = px + self.path_resolution * math.cos(angle)
                    py_updated = py + self.path_resolution * math.sin(angle)
                    x_y_filled.append([px_updated, py_updated])
        self.polygon_xy_filled_pub.publish(self.to_polygon(x_y_filled))

        # yaw calculation and traj pub
        traj_msg = Trajectory()
        traj_msg.header.frame_id = "map"
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.home_position.position = home_position
        dis = 0.0

        for i in range(len(x_y_filled) - 1):
            # print(i, len(x_y_filled))
            trajectory_point = TrajectoryPoint()
            trajectory_point.pose.position.x = x_y_filled[i][0]
            trajectory_point.pose.position.y = x_y_filled[i][1]
            trajectory_point.pose.position.z = self.rear_axle_center_height_from_ground

            if i != len(x_y_filled):
                # print("uygyug",xy_list[i+1][0] - xy_list[i][0], xy_list[i+1][1] - xy_list[i][1])
                yaw = math.atan2(x_y_filled[i + 1][1] - x_y_filled[i][1], x_y_filled[i + 1][0] - x_y_filled[i][0])
                trajectory_point.pose.orientation = yaw_to_quaternion(yaw)
            if i == 0:
                dis = 0.0
            else:
                dis += math.hypot(x_y_filled[i][1] - x_y_filled[i - 1][1],
                                  x_y_filled[i][0] - x_y_filled[i - 1][0])
            trajectory_point.accumulated_distance_m = dis

            # print(x_y_filled[i][0], x_y_filled[i][1], home_lat, home_long)
            lat, lng = xy2ll(x_y_filled[i][0], x_y_filled[i][1], home_lat, home_long)
            trajectory_point.gps_pose.position.latitude = lat
            trajectory_point.gps_pose.position.longitude = lng

            traj_msg.points.append(trajectory_point)

        # velocity profile
        traj_length = len(traj_msg.points)
        for i, traj_point in enumerate(traj_msg.points):

            for lhd_index in range(i, traj_length):
                path_acc_distance = traj_msg.points[lhd_index].accumulated_distance_m - \
                                    traj_point.accumulated_distance_m
                if path_acc_distance > self.avg_lhd:
                    break
            look_ahead = path_acc_distance
            # print("i", i)
            # print("lhd", lhd_index)
            # print("dis", look_ahead)
            slope = angle_btw_poses(traj_msg.points[lhd_index].pose, traj_point.pose)
            alpha = slope - get_yaw(traj_point.pose.orientation)
            delta = math.atan2(2.0 * vehicle_data.dimensions.wheel_base * math.sin(alpha), look_ahead)
            delta_degrees = -math.degrees(delta)
            # print("delta angle",delta_degrees )
            if abs(delta_degrees) <= self.steering_limits_to_slow_down:
                traj_msg.points[i].longitudinal_velocity_mps = self.max_forward_speed
            else:
                traj_msg.points[i].longitudinal_velocity_mps = max(
                    self.speed_reduce_factor * np.interp(abs(delta_degrees),
                                                         [self.steering_limits_to_slow_down,
                                                          vehicle_data.speed.max_steering_angle],
                                                         [self.max_forward_speed,
                                                          self.min_forward_speed]), self.min_forward_speed)
            # print("speed",traj_msg.points[i].longitudinal_velocity_mps )

        self.global_trajectory_pub.publish(traj_msg)
        self._traj_manager.update(traj_msg)
        self.gps_path_pub.publish(self._traj_manager.to_path())
        self.trajectory_velocity_marker_pub.publish(trajectory_to_marker(traj_msg, 1.5))

    def from_odometry(self, data):
        data_keys = data.keys()
        data_len = len(data['coordinates'])
        odom_key_name = "odometry"
        gps_key_name = "coordinates"
        if gps_key_name in data_keys and odom_key_name in data_keys:
            rospy.loginfo("gps coordinates and Odometry fields are  in  mission file")
        else:
            rospy.logwarn("No gps coordinates and Odometry fields are  in  mission file")
            sys.exit('No gps coordinates and Odometry fields are  in  mission file')

        # Setting home position for mavros node
        home_lat = data['coordinates'][0][1]
        home_long = data['coordinates'][0][0]
        home_alt = data['gps_coordinates'][0]['altitude']
        home_position = self.set_home_position(home_lat, home_lat, home_alt)
        rospy.loginfo('Origin point set')
        time.sleep(0.5)

        # Ros messages
        trajectory_msg = Trajectory()
        trajectory_msg.header.frame_id = "map"
        trajectory_msg.header.stamp = rospy.Time.now()
        trajectory_msg.home_position.position = home_position

        # Filling the Trajectory_msg
        accumulated_distance = 0
        prev_pose = Pose()
        for i in range(len(data['coordinates'])):

            # Odom based path
            lon, lat = data['coordinates'][i][0], data['coordinates'][i][1]
            odom_position = data['odometry'][i]['pose']['pose']['position']
            odom_orientation = data['odometry'][i]['pose']['pose']['orientation']
            odom_pose = Pose()
            odom_pose.position.x, odom_pose.position.y, odom_pose.position.z = odom_position['x'], odom_position['y'], \
                                                                               self.rear_axle_center_height_from_ground
            odom_pose.orientation = Quaternion(odom_orientation['x'], odom_orientation['y'], odom_orientation['z'],
                                               odom_orientation['w'])
            if i == 0:
                prev_pose = odom_pose

            dis = distance_btw_poses(odom_pose, prev_pose)
            accumulated_distance = accumulated_distance + dis
            prev_pose = odom_pose
            if dis >= self.max_dis_btw_points:
                # TODo interpolate
                pass

            # Trajectory msg filling
            traj_pt_msg = TrajectoryPoint()
            traj_pt_msg.pose = odom_pose
            traj_pt_msg.gps_pose.position.latitude = lat
            traj_pt_msg.gps_pose.position.longitude = lon
            # traj_pt_msg.longitudinal_velocity_mps =
            traj_pt_msg.index = i
            traj_pt_msg.accumulated_distance_m = accumulated_distance
            trajectory_msg.points.append(traj_pt_msg)
        # connecting the first and last way points
        i = i + 1
        if self.mission_continue:
            distance = distance_btw_poses(trajectory_msg.points[-1].pose, trajectory_msg.points[1].pose)
            rospy.loginfo("distance between first and last way point %s", str(distance))
            if distance > self.avg_lhd:
                rospy.logwarn("path's end and start points are %s meters apart , Interpolating them ", str(distance))
                n_points = distance // self.path_resolution
                angle = angle_btw_poses(trajectory_msg.points[1].pose, trajectory_msg.points[-1].pose)
                # print()
                # print("angle", angle)
                for j in range(0, int(n_points)):
                    # odom path
                    px = trajectory_msg.points[-1].pose.position.x
                    py = trajectory_msg.points[-1].pose.position.y
                    px_updated = px + self.path_resolution * math.cos(angle)
                    py_updated = py + self.path_resolution * math.sin(angle)
                    odom_pose = Pose()
                    odom_pose.position.x, odom_pose.position.y, odom_pose.position.z = px_updated, py_updated, \
                                                                                       self.rear_axle_center_height_from_ground
                    odom_pose.orientation = yaw_to_quaternion(angle)
                    # Trajectory
                    lat, lng = xy2ll(px_updated, py_updated, home_lat, home_long)

                    dis = distance_btw_poses(odom_pose, prev_pose)
                    accumulated_distance = accumulated_distance + dis
                    prev_pose = odom_pose
                    # Trajectory msg filling
                    traj_pt_msg = TrajectoryPoint()
                    traj_pt_msg.pose = odom_pose

                    traj_pt_msg.gps_pose.position.latitude = lat
                    traj_pt_msg.gps_pose.position.longitude = lng
                    # traj_pt_msg.longitudinal_velocity_mps =
                    traj_pt_msg.index = i + j
                    traj_pt_msg.accumulated_distance_m = accumulated_distance
                    trajectory_msg.points.append(traj_pt_msg)
            else:
                rospy.loginfo("path end and start points are close , no interpolation needed ")
        else:
            distance = distance_btw_poses(trajectory_msg.points[-1].pose, trajectory_msg.points[1].pose)
            rospy.logdebug("distance between first and last way point %s", str(distance))
            rospy.logdebug("Mission continue not selected")

        traj_length = len(trajectory_msg.points)
        # velocity profile
        for i, traj_point in enumerate(trajectory_msg.points):

            for lhd_index in range(i, traj_length):
                path_acc_distance = trajectory_msg.points[lhd_index].accumulated_distance_m - \
                                    traj_point.accumulated_distance_m
                if path_acc_distance > self.avg_lhd:
                    break
            look_ahead = path_acc_distance

            slope = angle_btw_poses(trajectory_msg.points[lhd_index].pose, traj_point.pose)
            alpha = slope - get_yaw(traj_point.pose.orientation)
            delta = math.atan2(2.0 * vehicle_data.dimensions.wheel_base * math.sin(alpha), look_ahead)
            delta_degrees = -math.degrees(delta)
            if abs(delta_degrees) <= self.steering_limits_to_slow_down:
                trajectory_msg.points[i].longitudinal_velocity_mps = self.max_forward_speed
            else:
                trajectory_msg.points[i].longitudinal_velocity_mps = max(
                    self.speed_reduce_factor * np.interp(abs(delta_degrees),
                                                         [self.steering_limits_to_slow_down,
                                                          vehicle_data.speed.max_steering_angle],
                                                         [self.max_forward_speed,
                                                          self.min_forward_speed]), self.min_forward_speed)
                # trajectory_msg.points[i].longitudinal_velocity_mps =  np.interp(abs(delta_degrees),
                #                                                     [self.steering_limits_to_slow_down,
                #                                                     vehicle_data.speed.max_steering_angle],
                #                                                     [self.max_forward_speed,
                #                                                     self.min_forward_speed])

        marker_arr = trajectory_to_marker(trajectory_msg, self.max_forward_speed)
        self.trajectory_velocity_marker_pub.publish(marker_arr)
        path = trajectory_to_path(trajectory_msg)
        self.gps_path_pub.publish(path)
        self.global_trajectory_pub.publish(trajectory_msg)
        rospy.loginfo("global_trajectory_published")
        rospy.loginfo('Details of ' + str(i) + " are published from file " + str(mission_file))

    def target_index(self, robot_pose, close_point_ind):
        """
        search index of target point in the reference path. The following implementation was inspired from
        http://dyros.snu.ac.kr/wp-content/uploads/2021/02/Ahn2021_Article_AccuratePathTrackingByAdjustin-1.pdf
        Args:
            robot_pose:  pose of robot
            close_point_ind : index of close point to the vehicle
        Returns:
            close_index, target_index, lookahead_distance, cross_track_dis,
        """
        lhd = self.compute_lookahead_distance(abs(self.robot_state.twist.twist.linear.x))
        close_dis = self.trajectory_data.points[close_point_ind].accumulated_distance_m
        for ind in range(close_point_ind, len(self.trajectory_data.points)):
            path_acc_distance = self.trajectory_data.points[ind].accumulated_distance_m - close_dis
            if path_acc_distance > lhd:
                return ind, distance_btw_poses(robot_pose, self.trajectory_data.points[ind].pose)
        return ind, distance_btw_poses(robot_pose, self.trajectory_data.points[ind].pose)


if __name__ == "__main__":
    rospy.init_node("global_gps_path_publisher")
    mission_file = rospy.get_param('/patrol/mission_file', 'default.json')
    if "/" in mission_file:
        mission_file_dir = mission_file
        rospy.loginfo(f"mission_file_dir : {mission_file_dir}")
    else:
        try:
            ros_pack = rospkg.RosPack()
            mission_file_dir = ros_pack.get_path('autopilot') + "/mission_files/" + str(mission_file)
            rospy.loginfo(f"mission_file_dir : {mission_file_dir}")
        except Exception as e:
            rospy.logerr(f"Could not found autopilot package, consider souring it, ERROR: {e}")
            rospy.signal_shutdown(f"Could not found autopilot package, consider souring it, ERROR: {e}")

    fail_exists = os.path.isfile(mission_file_dir)
    if not fail_exists:
        rospy.logerr(f"mission file does not exist, path: {mission_file_dir}")
        rospy.signal_shutdown(f"mission file does not exist, path: {mission_file_dir}")
    else:
        rospy.loginfo("mission file exists in file directory")

    gps_path_pub = GlobalGpsPathPub(mission_file_dir)

    if '.json' in mission_file or ".geojson" in mission_file:
        gps_path_pub.from_json()

    elif ".kml" in mission_file:
        gps_path_pub.from_kml()
    else:
        rospy.logerr(f"No proper extension to input mission file, path: {mission_file_dir}")
        rospy.signal_shutdown(f"No proper extension to input mission file, path: {mission_file_dir}")

    rospy.spin()
