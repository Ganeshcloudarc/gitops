#!/usr/bin/env python3

try:
    import rospy
    import rospkg
    from nav_msgs.msg import Path
    from geographic_msgs.msg import GeoPointStamped, GeoPose
    from mavros_msgs.msg import HomePosition
    from geometry_msgs.msg import Pose, PoseStamped, Quaternion
    from std_msgs.msg import Float32MultiArray, Int32MultiArray, Bool, Int16, String, Int8, UInt16
    from visualization_msgs.msg import Marker, MarkerArray
    import json, time, math, sys
    import numpy as np
    from vehicle_common.vehicle_config import vehicle_data
    from autopilot_msgs.msg import Trajectory, TrajectoryPoint
    from autopilot_utils.geonav_conversions import xy2ll
    from autopilot_utils.pose_helper import distance_btw_poses, get_yaw, angle_btw_poses, yaw_to_quaternion
    from autopilot_utils.trajectory_helper import trajectory_to_path, trajectory_to_marker
except Exception as e:
    import rospy
    rospy.logerr("module named %s", str(e))
    exit()


class PathPubGps:
    def __init__(self, mission_file):
        # parameters for path publisher
        self.max_forward_speed = rospy.get_param('/patrol/max_forward_speed', 1.5)
        self.min_forward_speed = rospy.get_param("/patrol/min_forward_speed", 0.3)
        distance_to_slowdown_on_ends = rospy.get_param("/path_publisher/distance_to_slowdown_on_ends", 3)
        self.mission_continue = rospy.get_param("patrol/mission_continue", True)
        self.max_dis_btw_points = rospy.get_param("path_publisher/max_dis_btw_points", 0.5)
        self.path_resolution = rospy.get_param("path_publisher/path_resolution", 0.1)
        self.steering_limits_to_slow_down = rospy.get_param("path_publisher/steering_limits_to_slow_down", 10)
        self.min_look_ahead = rospy.get_param("pure_pursuit/min_look_ahead_dis", 3)
        self.max_look_ahead = rospy.get_param("pure_pursuit/max_look_ahead_dis", 6)
        self.avg_lhd = (self.min_look_ahead + self.max_look_ahead) / 2
        rear_axle_center_height_from_ground = vehicle_data.dimensions.tyre_radius

        try:
            data = json.load(open(mission_file))
        except Exception as error:
            rospy.logerr('Error In Reading mission file ' + str(error))
            sys.exit('Error In Reading mission file ' + str(error))
        data_keys = data.keys()
        data_len = len(data['coordinates'])
        odom_key_name = "odometry"
        gps_key_name = "coordinates"
        if gps_key_name in data_keys and odom_key_name in data_keys:
            rospy.loginfo("gps coordinates and Odometry fields are  in  mission file")
        else:
            rospy.logwarn("No gps coordinates and Odometry fields are  in  mission file")
            sys.exit('No gps coordinates and Odometry fields are  in  mission file')

        # publishers
        global_trajectory_pub = rospy.Publisher('global_gps_trajectory', Trajectory, queue_size=1, latch=True)
        gps_path_pub = rospy.Publisher('/global_gps_path', Path, queue_size=1, latch=True)
        starting_point_pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped,
                                             queue_size=1, latch=True)
        home_position_pub = rospy.Publisher('/mavros/global_position/home', HomePosition,
                                            queue_size=1, latch=True)
        trajectory_velocity_marker_pub = rospy.Publisher('/global_gps_trajectory_velocity_marker', MarkerArray,
                                                         queue_size=1, latch=True)

        # Setting home position for mavros node
        home_lat = data['coordinates'][0][1]
        home_long = data['coordinates'][0][0]
        geo_point = GeoPointStamped()
        geo_point.position.latitude = home_lat
        geo_point.position.longitude = home_long
        geo_point.position.altitude = data['gps_coordinates'][0]['altitude']
        starting_point_pub.publish(geo_point)
        home_position_msg = HomePosition()
        home_position_msg.geo = geo_point.position
        home_position_pub.publish(home_position_msg)
        rospy.loginfo('Origin point set')
        time.sleep(0.5)
        # Ros messages
        trajectory_msg = Trajectory()
        trajectory_msg.header.frame_id = "map"
        trajectory_msg.header.stamp = rospy.Time.now()
        trajectory_msg.home_position = geo_point.position

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
                                                                               rear_axle_center_height_from_ground
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
            rospy.logdebug("distance between first and last way point %s", str(distance))
            if distance > self.avg_lhd:
                rospy.logwarn("path's end and start points are %s meters apart , Interpolating them ", str(distance))
                n_points = distance // self.path_resolution
                angle = angle_btw_poses(trajectory_msg.points[0].pose, trajectory_msg.points[-1].pose)
                for j in range(0, int(n_points)):
                    # odom path
                    px = trajectory_msg.points[-1].pose.position.x
                    py = trajectory_msg.points[-1].pose.position.y
                    px_updated = px + self.path_resolution * math.cos(angle)
                    py_updated = py + self.path_resolution * math.sin(angle)
                    odom_pose = Pose()
                    odom_pose.position.x, odom_pose.position.y, odom_pose.position.z = px_updated, py_updated, \
                                                                                       rear_axle_center_height_from_ground
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
                trajectory_msg.points[i].longitudinal_velocity_mps = np.interp(abs(delta_degrees),
                                                                               [self.steering_limits_to_slow_down,
                                                                                vehicle_data.speed.max_steering_angle],
                                                                               [self.max_forward_speed,
                                                                                self.min_forward_speed])

        marker_arr = trajectory_to_marker(trajectory_msg, self.max_forward_speed)
        trajectory_velocity_marker_pub.publish(marker_arr)
        path = trajectory_to_path(trajectory_msg)
        gps_path_pub.publish(path)
        global_trajectory_pub.publish(trajectory_msg)
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
    rospy.init_node("gps_path_publisher")
    mission_file = rospy.get_param('/patrol/mission_file', 'default.json')
    if '.json' in mission_file:
        pass
    else:
        mission_file = mission_file + '.json'
        rospy.set_param('/patrol/mission_file', mission_file)
    try:
        ros_pack = rospkg.RosPack()
        mission_file_dir = ros_pack.get_path('autopilot') + "/mission_files/" + str(mission_file)
    except Exception as e:
        rospy.logwarn("Please source autopilot package" + str(e))
        sys.exit("Please source autopilot package" + str(e))
    a = PathPubGps(mission_file_dir)
    rospy.spin()
