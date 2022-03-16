#!/usr/bin/env python3
try:
    import rospy
    import rospkg
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import PoseStamped, Quaternion
    from sensor_msgs.msg import NavSatFix, MagneticField, Imu
    from ackermann_msgs.msg import AckermannDrive
    from mavros_msgs.msg import RCIn
    import json
    import math
    import sys
    import time
    import numpy as np
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
    from std_msgs.msg import Float32MultiArray, Int32MultiArray, Bool, Int16, String, Int8, UInt16
    from geographic_msgs.msg import GeoPointStamped
except Exception as e:
    print('No module named :', str(e))
    exit(e)


class Config:
    def __init__(self):
        """Configuration class for pure pursuit"""
        # patrol specific
        self.path_topic = rospy.get_param('/patrol/path_topic', 'odom_path')  # gps_path or odom_path(good one)
        self.odom_topic = rospy.get_param("/patrol/odom_topic", "/mavros/local_position/odom")
        self.start_from_first_point = rospy.get_param('/patrol/from_start', False)  # TODO
        self.min_speed = rospy.get_param('/patrol/min_speed', 0.3)
        self.max_speed = rospy.get_param('/patrol/max_speed', 3)
        self.carla = rospy.get_param("/carla_sim", False)
        if self.carla:
            self.average_speed = rospy.get_param("carla_sim/speed", 0.3)
        else:
            self.average_speed = (self.min_speed + self.max_speed) / 2
        self.ack_pub_topic = rospy.get_param('/patrol/control_topic', '/cmd_drive/pure_pursuit')

        # vehicle specific
        self.wheel_base = rospy.get_param('/vehicle/wheel_base', 2)
        self.dist_front_rear_wheels = rospy.get_param('/vehicle/dist_front_rear_wheels', 1.5)

        # Pure Pursuit specific
        self.min_look_ahead = rospy.get_param('/pure_pursuit/min_look_ahead', 3)
        self.max_look_ahead = rospy.get_param('/pure_pursuit/max_look_ahead', 6)
        self.speed_to_lookahead_ratio = rospy.get_param('/pure_pursuit/speed_to_lookahead_ratio',
                                                        1)  # taken from autoware
        self.curvature_speed_ratio = rospy.get_param('/pure_pursuit/curvature_speed_ratio', 1)


class PurePursuit:
    def __init__(self):
        self.speed = 0.3 # test variable for reverse
        self.config = Config()
        self.robot_state = {
            'x': 0.0,
            'y': 0.0,
            'yaw': 0.0,
            'vel': 0.0
        }
        self.home_gps_location = {
            'latitude': 0.0,
            'longitude': 0.0,
            'altitude': 0.0
        }
        # Publishers
        self.ackermann_publisher = rospy.Publisher(self.config.ack_pub_topic, AckermannDrive, queue_size=10)
        self.vehicle_pose_pub = rospy.Publisher('/vehicle_pose', PoseStamped, queue_size=2)
        self.target_pose_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=2)
        self.mission_count_pub = rospy.Publisher('/mission_count', Int16, queue_size=2)
        self.vehicle_pose_msg = PoseStamped()
        self.target_pose_msg = PoseStamped()
        self.ackermann_msg = AckermannDrive()
        self.path = []  # change it later
        self.velocity_profile = []
        self.curvature_profile = []

        self.ind_end = 0
        self.index_old = None
        self.time_when_odom = None
        self.odom_data = None
        self.revived_path = None
        self.present_look_ahead = None
        self.target_id = None
        self.close_idx, self.cross_track_dis = 0.0, 0.0
        self.count_mission_repeat = 0
        self.mission_complete = True

        path_data, odom_data = None, None
        while not rospy.is_shutdown():
            if not path_data:
                try:
                    path_data = rospy.wait_for_message(self.config.path_topic, Path, timeout=1)
                except:
                    path_data = None
                    rospy.logwarn("Waiting for %s", str(self.config.path_topic))
                else:
                    rospy.logdebug("Topic %s is active", str(self.config.path_topic))
            if not odom_data:
                try:
                    odom_data = rospy.wait_for_message(self.config.odom_topic, Odometry, timeout=1)
                except:
                    odom_data = None
                    rospy.logwarn("Waiting for %s", str(self.config.odom_topic))
                else:
                    rospy.logdebug("Topic %s is active", str(self.config.path_topic))

            if path_data and odom_data:
                if path_data.header.frame_id == odom_data.header.frame_id:

                    self.vehicle_pose_msg.header.frame_id = path_data.header.frame_id
                    self.target_pose_msg.header.frame_id = path_data.header.frame_id
                    rospy.Subscriber(self.config.path_topic, Path, self.path_callback)
                    rospy.Subscriber(self.config.odom_topic, Odometry, self.odom_callback)
                    rospy.Subscriber('/curvature_profile', Float32MultiArray, self.curvature_profile_callback)
                    rospy.Subscriber('/velocity_profile', Float32MultiArray, self.velocity_profile_callback)
                    rospy.Subscriber('/mavros/global_position/set_gp_origin', GeoPointStamped,
                                     self.home_position_callback)
                    rospy.loginfo('All the subscribers defined properly %s %s %s %s %s', self.config.path_topic,
                                  self.config.odom_topic, '/curvature_profile', '/velocity_profile',
                                  '/mavros/global_position/set_gp_origin')
                    break
                else:
                    rospy.logfatal('tf frames of path and odometry are not same: path %s , odom %s',
                                   path_data.header.frame_id, odom_data.header.frame_id)
                    sys.exit('tf frames of path and odometry are not same"')
        time.sleep(1)
        self.main_loop()

    def curvature_profile_callback(self, data):
        rospy.logdebug("curvature data received of length %s", str(len(data.data)))
        self.curvature_profile = data.data

    def velocity_profile_callback(self, data):
        rospy.logdebug("velocity data received of length %s", str(len(data.data)))
        self.velocity_profile = data.data

    def home_position_callback(self, data):
        self.home_gps_location = {
            'latitude': data.position.latitude,
            'longitude': data.position.longitude,
            'altitude': data.position.altitude
        }
        rospy.loginfo("home position data received %s", str(self.home_gps_location))

    def mission_status_callback(self, data):
        self.mission_complete = data.data

    def path_callback(self, data):
        rospy.logdebug("path data data received of length %s", str(len(data.poses)))
        # data.poses.reverse() 
        self.revived_path = data.poses
        for pose in data.poses:
            _, _, heading = euler_from_quaternion(
                [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            heading = math.degrees(heading)
            self.path.append([pose.pose.position.x, pose.pose.position.y, heading])
        self.ind_end = len(self.path) - 1

    def odom_callback(self, data):
        rospy.loginfo_once("Odom data received")
        self.time_when_odom = time.time()
        self.odom_data = data
        _, _, pose_heading = euler_from_quaternion(
            [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
             data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        )
        self.robot_state['x'] = data.pose.pose.position.x - self.config.wheel_base * np.cos(pose_heading)
        self.robot_state['y'] = data.pose.pose.position.y - self.config.wheel_base * np.sin(pose_heading)
        self.robot_state['yaw'] = pose_heading
        self.robot_state['vel'] = abs(data.twist.twist.linear.x)

        # fx = x + self.wheelbase * np.cos(yaw)
        # fy = y + self.wheelbase * np.sin(yaw)

    def compute_lookahead_distance(self, vel):
        # https://gitlab.com/-/ide/project/RamanaBotta/AutowareAuto/tree/master/-/src/control/pure_pursuit/src/pure_pursuit.cpp/#L136
        rospy.loginfo("vel %s ", vel)
        look_ahead = vel * self.config.speed_to_lookahead_ratio
        rospy.loginfo("vel_lookahead: %s", look_ahead)
        final_look_ahead = max(self.config.min_look_ahead, min(look_ahead, self.config.max_look_ahead))
        rospy.loginfo('final_look_ahead %s', final_look_ahead)
        return final_look_ahead

    def compute_velocity_at_point(self, curvature, velocity_at_index):
        # TODO
        # add obstacle velocity_profile.
        rospy.loginfo("curvature %s", curvature)
        try:
            circum_radius = (1 / curvature)
        except:
            circum_radius = 10000
        rospy.loginfo("circum radius %s", circum_radius)
        curvature_vel = circum_radius * self.config.curvature_speed_ratio
        rospy.loginfo("curvature_vel: %s velocity_at_index: %s max_speed: %s", curvature_vel, velocity_at_index,
                       self.config.max_speed)
        final_vel = min(self.config.max_speed, velocity_at_index, curvature_vel)
        final_vel = max(final_vel, self.config.min_speed)
        return final_vel

    def find_close_point(self, robot, index_old):
        n = min(100, len(range(index_old, self.ind_end)))
        distance_list = [self.calc_distance(robot, ind) for ind in range(index_old, index_old + n)]
        ind = np.argmin(distance_list)
        final = ind + index_old
        dis = distance_list[ind]
        return final, dis

    def target_index(self, robot):
        """
        search index of target point in the reference path.
        Args:
            robot: robot_state
        Returns:
            close_index, target_index, lookahead_distance, cross_track_dis,
        """
        if self.index_old is None:
            self.index_old, cross_track_dis = self.calc_nearest_ind(robot)
        else:
            self.index_old, cross_track_dis = self.find_close_point(robot, self.index_old)
            # cross_track_dis = 3
        lhd = self.compute_lookahead_distance(robot['vel'])
        # lhd = 3
        for ind in range(self.index_old, self.ind_end):
            dis = self.calc_distance(robot, ind)
            if dis > lhd:
                return self.index_old, ind, lhd, cross_track_dis
            if ind + 1 >= self.ind_end:
                return ind, ind, lhd, cross_track_dis
                # return ind, ind, lhd,cross_track_dis
        return self.ind_end, self.ind_end, lhd, cross_track_dis

    def calc_distance(self, robot, ind):
        # print(robot)
        return math.hypot(robot['x'] - self.path[ind][0], robot['y'] - self.path[ind][1])

    def calc_distance_idx(self, close, next_id):
        return math.hypot(self.path[close][0] - self.path[next_id][0], self.path[close][1] - self.path[next_id][1])

    def calc_nearest_ind(self, robot):
        """
        calc index of the nearest point to current position
        Args:
            robot:
        """
        distance_list = [self.calc_distance(robot, ind) for ind in range(len(self.path))]
        ind = np.argmin(distance_list)
        self.index_old = ind
        dis = distance_list[ind]
        return ind, dis

    def main_loop(self):
        r = rospy.Rate(1)
        all_data_received = False
        while not all_data_received and not rospy.is_shutdown():
            # print(" Here in inner Loop")
            if len(self.path) < 1:
                rospy.logwarn('No path received')
            if len(self.curvature_profile) < 1:
                rospy.logwarn('No curvature_profile received')
            if len(self.velocity_profile) < 1:
                rospy.logwarn('No velocity_profile received')
            # print(len(self.velocity_profile) > 1 and len(self.curvature_profile) > 1 and len(self.path) > 1)
            if len(self.velocity_profile) > 1 and len(self.curvature_profile) > 1 and len(self.path) > 1:
                rospy.loginfo("Path, velocity_profile and curvature profile are received")
                all_data_received = True
            r.sleep()

        rospy.loginfo('Pure pursuit is started')
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            # print("---------------------------------------------------------------")
            # not necessary
            self.vehicle_pose_msg.pose.position.x = self.robot_state['x']
            self.vehicle_pose_msg.pose.position.y = self.robot_state['y']
            self.vehicle_pose_msg.pose.orientation = self.odom_data.pose.pose.orientation
            self.vehicle_pose_pub.publish(self.vehicle_pose_msg)

            close_idx, target_idx, lhd, cross_track_error = self.target_index(self.robot_state)
            rospy.loginfo("close index %s , target point index %s, lookahead dis %s, ctc %s",
                          str(close_idx), str(target_idx), str(lhd), str(cross_track_error))

            if target_idx + 1 >= self.ind_end:
                self.count_mission_repeat += 0.5
                rospy.loginfo(' MISSION COUNT %s ', self.count_mission_repeat)
                self.mission_count_pub.publish(self.count_mission_repeat)
                self.send_ack_msg(0, 0, 0)
                time.sleep(1)
                self.revived_path.reverse()
                self.path.reverse()
                self.index_old = None
                if self.speed < 0:
                    self.speed = 0.3
                else:
                    self.speed = -0.3
            self.target_pose_msg.pose.position.x = self.path[target_idx][0]
            self.target_pose_msg.pose.position.y = self.path[target_idx][1]
            self.target_pose_msg.pose.orientation = self.revived_path[target_idx].pose.orientation
            self.target_pose_pub.publish(self.target_pose_msg)
            delta_x = self.path[target_idx][0] - self.robot_state['x']
            delta_y = self.path[target_idx][1] - self.robot_state['y']
            slope = math.atan2(delta_y, delta_x)
            alpha = slope - self.robot_state['yaw']
            delta = math.atan2(2.0 * self.config.wheel_base * math.sin(alpha), lhd)
            delta_degrees = -1 * math.degrees(delta)
            steering_angle = np.clip(delta_degrees, -30, 30)
            speed = self.compute_velocity_at_point(self.curvature_profile[target_idx], self.velocity_profile[target_idx])
            rospy.loginfo("cur: %s, vel: %s", self.curvature_profile[target_idx], self.velocity_profile[target_idx])
            rospy.loginfo("steering angle: %s, speed: %s, break: %s", str(steering_angle), str(speed), str(0))
            rospy.loginfo("self.speed %s", self.speed)
            self.send_ack_msg(steering_angle, self.speed, 0)
            r.sleep()

    def mission_mode_selector(self, count_mission_repeat):
        """
        Args:
            count_mission_repeat:

        Returns:
            stop/continue , summary

        """
        if self.MISSION_MODE == 0:
            return False, 'Completed Mission '
        elif self.MISSION_MODE == 1:
            if count_mission_repeat <= 1:
                self.revived_path.reverse()
                print("before", self.path[0])
                self.path.reverse()
                print("after", self.path[0])

                self.index_old = 0
                return True, "Completed Reaching the end , starting back"
            else:
                return False, "Completed Mission, both reached target and retuned to starting"
        elif self.MISSION_MODE == 2:
            if self.mission_trips == 0:
                self.revived_path.reverse()
                self.path.reverse()
                self.index_old = 0
                return True, "Reached one end, starting towards to other end"
            else:
                if self.mission_trips <= count_mission_repeat / 2:
                    return False, 'Completed ' + str(self.mission_reapeat_number) + ' trips'
                else:
                    self.revived_path.reverse()
                    self.path.reverse()
                    self.index_old = 0
                    return True, 'Trips of ' + str(count_mission_repeat / 2) + " are completed " + str(
                        self.mission_trips - count_mission_repeat / 2) + " are yet to completed"
        else:
            return False, "Invalid Mission mode was given, Treating the mission as Mission mode 0"

    def send_ack_msg(self, steering_angle, speed, jerk):
        self.ackermann_msg.steering_angle = steering_angle
        self.ackermann_msg.speed = speed
        self.ackermann_msg.jerk = jerk
        self.ackermann_publisher.publish(self.ackermann_msg)


if __name__ == "__main__":
    rospy.init_node('pure_pursuit1')

    pure_pursuit = PurePursuit()

    rospy.spin()
