#!/usr/bin/env python3
try:
    import rospy
    import rospkg
    from nav_msgs.msg import Path
    from geographic_msgs.msg import GeoPointStamped, GeoPose
    from mavros_msgs.msg import HomePosition
    from geometry_msgs.msg import Pose, PoseStamped, Quaternion
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
    from std_msgs.msg import Float32MultiArray, Int32MultiArray, Bool, Int16, String, Int8, UInt16
    from visualization_msgs.msg import Marker, MarkerArray
    import json
    import time
    import math
    import sys
    import numpy as np
    from vehicle_common.vehicle_common import VehicleData
    from autopilot_msgs.msg import Trajectory, TrajectoryPoint
    from autopilot_utils.geonav_conversions import *
# /home/boson/costmap_ws/src/autopilot_boson/autopilot_utils/src/autopilot_utils/geonav_conversions.py
except Exception as e:
    print("No module named", str(e))
    exit()


def circum_radius(x_vals, y_vals):
    """
        Calculates the circum radius for three 2D points
    """
    x1, x2, x3, y1, y2, y3 = x_vals[0], x_vals[1], x_vals[2], y_vals[0], y_vals[1], y_vals[2]
    den = 2 * ((x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2))
    num = ((((x2 - x1) ** 2) + ((y2 - y1) ** 2)) * (((x3 - x2) ** 2) + ((y3 - y2) ** 2)) * (
            ((x1 - x3) ** 2) + ((y1 - y3) ** 2))) ** 0.5
    if den == 0:
        print('Failed: points are either collinear or not distinct')
        return 0
    radius = abs(num / den)
    return radius


def distance_btw_poses(pose1, pose2):
    """2d distance between two poses"""
    return math.hypot(pose1.position.x - pose2.position.x, pose1.position.y - pose2.position.y)


class PathPubGps:
    def __init__(self, mission_file):
        self.vehicle = VehicleData()
        # parameters for path publisher'
        rc_max = rospy.get_param("/path_publisher/radius_of_curvature_max", 5)
        if rc_max >= self.vehicle.minimum_turning_radius:
            pass
        else:
            rospy.logwarn("provided radius_of_curvature_max is less than vehicle's radius_of_curvature_max")
            rospy.loginfo("assigning rc_max to vehicle's min turing radius")
            rc_max = self.vehicle.minimum_turning_radius
        max_forward_speed = rospy.get_param('/patrol/max_forward_speed', 1.5)
        if max_forward_speed > self.vehicle.max_forward_speed:
            rospy.logwarn("provided forward speed is more than vehicle's max forward speed")
            rospy.loginfo("assigning max_forward to vehicles max_forward speed")
            max_forward_speed = self.vehicle.max_forward_speed
        if max_forward_speed < self.vehicle.min_forward_speed:
            rospy.logwarn("provided max_forward_speed  less than the vehicles min speed")
            rospy.loginfo("assigning max_forward speed to vehicles max_forward speed")
        min_forward_speed = rospy.get_param("/patrol/min_forward_speed", 0.3)
        if min_forward_speed < self.vehicle.min_forward_speed:
            rospy.logwarn("provided min_forward_speed  less than the vehicles min speed")
            rospy.loginfo("assigning min_forward_speed speed to vehicle's min_forward_speed")
            min_forward_speed = self.vehicle.min_forward_speed
        if min_forward_speed > self.vehicle.max_forward_speed:
            rospy.logwarn("invalid min speed %s", min_forward_speed)
            rospy.loginfo("assigning to vehicle's min forward speed ")
            min_forward_speed = self.vehicle.min_forward_speed

        self.max_forward_speed = max_forward_speed
        self.min_forward_speed = min_forward_speed
        self.rc_max = rc_max
        self.rc_min = 1  # always use 1
        distance_to_slowdown_on_ends = rospy.get_param("/path_publisher/distance_to_slowdown_on_ends", 3)

        odom_key_name = "odometry"
        gps_key_name = "coordinates"
        try:
            data = json.load(open(mission_file))
        except Exception as e:
            rospy.logerr('Error In Reading mission file ' + str(e))
            sys.exit('Error In Reading mission file ' + str(e))
        self.data = data
        data_keys = data.keys()
        data_len = len(data['coordinates'])
        if gps_key_name in data_keys and odom_key_name in data_keys:
            rospy.loginfo("gps coordinates and Odometry fields are  in  mission file")
        else:
            rospy.logwarn("No gps coordinates and Odometry fields are  in  mission file")
            sys.exit('No gps coordinates and Odometry fields are  in  mission file')

        global_trajectory_pub = rospy.Publisher('global_gps_trajectory', Trajectory, queue_size=10, latch=True)
        gps_path_pub = rospy.Publisher('/gps_path', Path, queue_size=10, latch=True)
        odom_path_pub = rospy.Publisher('/odom_path', Path, queue_size=10, latch=True)
        starting_point_pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped,
                                             queue_size=10, latch=True)
        home_position_pub = rospy.Publisher('/mavros/global_position/home', HomePosition,
                                            queue_size=10, latch=True)
        curvature_pub = rospy.Publisher('/curvature_profile', Float32MultiArray, queue_size=10, latch=True)
        velocities_pub = rospy.Publisher('/velocity_profile', Float32MultiArray, queue_size=10, latch=True)
        curvature_velocities_pub = rospy.Publisher('/curvature_velocity_profile', Float32MultiArray, queue_size=10,
                                                   latch=True)
        velocity_marker_pub = rospy.Publisher('/curvature_velocity_profile_markers', MarkerArray, queue_size=10,
                                              latch=True)
        curvature_msg = Float32MultiArray()
        velocity_msg = Float32MultiArray()
        curvature_velocity_msg = Float32MultiArray()
        marker_arr_msg = MarkerArray()
        trajectory_msg = Trajectory()
        trajectory_msg.header.frame_id = "map"

        # msgs
        gps_path_msg = Path()
        gps_path_msg.header.frame_id = 'map'
        gps_path_msg.header.stamp = rospy.Time.now()
        odom_path_msg = Path()
        odom_path_msg.header.frame_id = 'map'

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

        trajectory_msg.home_position.position.latitude = home_lat
        trajectory_msg.home_position.position.longitude = home_long
        trajectory_msg.home_position.position.altitude = data['gps_coordinates'][0]['altitude']

        circum_radius_list = []
        vel_list = []

        accumulated_distance = 0
        prev_pose = Pose()
        for i in range(len(data['coordinates'])):
            # gps_based path
            lon, lat = data['coordinates'][i][0], data['coordinates'][i][1]
            gps_orientation = data['imu'][i]['orientation']
            x, y = ll2xy(lat, lon, home_lat, home_long)
            gps_pose = PoseStamped()
            gps_pose.pose.position.x, gps_pose.pose.position.y = x, y
            gps_pose.pose.orientation = Quaternion(gps_orientation['x'], gps_orientation['y'], gps_orientation['z'],
                                                   gps_orientation['w'])
            gps_pose.header.seq += 1
            gps_pose.header.frame_id = 'map'
            gps_pose.header.stamp = rospy.Time.now()
            gps_path_msg.poses.append(gps_pose)

            # Odom based path
            odom_position = data['odometry'][i]['pose']['pose']['position']
            odom_orientation = data['odometry'][i]['pose']['pose']['orientation']
            odom_pose = PoseStamped()
            odom_pose.pose.position.x, odom_pose.pose.position.y = odom_position['x'], odom_position['y']
            odom_pose.pose.orientation = Quaternion(odom_orientation['x'], odom_orientation['y'], odom_orientation['z'],
                                                    odom_orientation['w'])
            odom_pose.header.seq += 1
            odom_pose.header.stamp = rospy.Time.now()
            odom_pose.header.frame_id = "map"
            odom_path_msg.poses.append(odom_pose)
            # Velocity driven
            velocity = data['odometry'][i]['twist']['twist']['linear']['x']
            velocity_msg.data.append(abs(velocity))

            # velocity calculation based of curvature
            values = [0.1, 0.2, 0.4, 0.2, 0.1]  # weight values for neighbouring points circum radius
            vel = 0
            # inspired from https://github.com/RobeSafe-UAH/Waypoint_Tracking_Controller/blob/d534d527777c9457abd84203e5
            # 9440e62cf009d8/src/controller_node.cpp#L236

            for num, k in enumerate(range(-2, 3)):
                cr, curvature = self.find_curvature_at_index(i + k * 10)
                if k == 0:
                    curvature_msg.data.append(curvature)
                    circum_radius_list.append(cr)
                if cr <= self.rc_min:
                    cr = self.rc_min
                elif cr > self.rc_max:
                    cr = self.rc_max
                vel += values[num] * self.max_forward_speed * cr / self.rc_max

            if i < distance_to_slowdown_on_ends * 10:  # on starting 3 meter min speed applied
                vel = self.min_forward_speed
            elif data_len - i < distance_to_slowdown_on_ends * 10:  # on last 3 meters
                vel = self.min_forward_speed
            vel_list.append(vel)
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
            dis = distance_btw_poses(odom_pose.pose, prev_pose)
            accumulated_distance = accumulated_distance + dis
            prev_pose = odom_pose.pose
            # Trajectory msg filling
            traj_pt_msg = TrajectoryPoint()
            traj_pt_msg.pose = odom_pose.pose
            traj_pt_msg.gps_pose.position.latitude = lat
            traj_pt_msg.gps_pose.position.longitude = lon
            traj_pt_msg.longitudinal_velocity_mps = vel
            traj_pt_msg.index = i
            traj_pt_msg.accumulated_distance_m = accumulated_distance
            trajectory_msg.points.append(traj_pt_msg)

        curvature_velocity_msg.data = vel_list
        gps_path_pub.publish(gps_path_msg)
        rospy.loginfo('GPS path is published')
        odom_path_pub.publish(odom_path_msg)
        rospy.loginfo('ODOM path is published')
        velocities_pub.publish(velocity_msg)
        rospy.loginfo('Velocity profile is published')
        curvature_velocities_pub.publish(curvature_velocity_msg)
        rospy.loginfo('Curvature velocity profile is published')
        curvature_pub.publish(curvature_msg)
        rospy.loginfo('Curvature profile is published')
        velocity_marker_pub.publish(marker_arr_msg)
        rospy.loginfo("Velocity markers are published")
        global_trajectory_pub.publish(trajectory_msg)
        rospy.loginfo("Trajectory are published")
        rospy.loginfo('Details of ' + str(i) + " are published from file " + str(mission_file))

    def find_curvature_at_index(self, i):
        """
        Find curvature at an index
        Args:
            i (int) : index of point on the path
        Returns:
            circum_radius (float)
            curvature (float)
        """

        if i <= 10 or i >= len(self.data['coordinates']) - 10:
            return 0, 0
        else:
            x_vals = [self.data['odometry'][i - 10]['pose']['pose']['position']['x'],
                      self.data['odometry'][i]['pose']['pose']['position']['x'],
                      self.data['odometry'][i + 10]['pose']['pose']['position']['x']]
            y_vals = [self.data['odometry'][i - 10]['pose']['pose']['position']['y'],
                      self.data['odometry'][i]['pose']['pose']['position']['y'],
                      self.data['odometry'][i + 10]['pose']['pose']['position']['y']]
            radius = circum_radius(x_vals, y_vals)
            if radius == 0:
                return radius, inf
            else:
                return radius, 1 / radius

    def velocity_at_index(self, circum_radius_list):
        speed_list = []
        for i in range(len(circum_radius_list) - 3):
            if i < self.vh_config.distance_to_slowdown_on_start * 10:  # on starting 3 meter min speed applied
                vel = self.vh_config.min_speed
            # elif len(circum_radius_list)-i < self.vh_config.distance_to_slowdown_on_final * 10:  # on last 3 meters
            #     vel = self.vh_config.min_speed
            else:
                print('cr', circum_radius_list[i])
                # inspired from https://github.com/RobeSafe-UAH/Waypoint_Tracking_Controller/blob/d534d527777c9457abd8
                # 4203e59440e62cf009d8/src/controller_node.cpp#L236
                vel = 0.1 * self.vh_config.max_speed * circum_radius_list[i - 2] / self.vh_config.rc_max + \
                      0.2 * self.vh_config.max_speed * circum_radius_list[i - 1] / self.vh_config.rc_max + \
                      0.4 * self.vh_config.max_speed * circum_radius_list[i] / self.vh_config.rc_max + \
                      0.2 * self.vh_config.max_speed * circum_radius_list[i + 1] / self.vh_config.rc_max + \
                      0.1 * self.vh_config.max_speed * circum_radius_list[i + 2] / self.vh_config.rc_max

            speed_list.append(vel)
        return speed_list


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
