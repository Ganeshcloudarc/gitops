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
'''
time out from odometry
'''

class PurePursuit:
    def __init__(self):
        self.load_params()
        self.MISSION_MODE = rospy.get_param('/patrol/mission_mode', 0)
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
        self.ackermann_publisher = rospy.Publisher(self.ack_pub_topic, AckermannDrive, queue_size=10)
        self.vehicle_pose_pub = rospy.Publisher('/vehicle_pose', PoseStamped, queue_size=10)
        self.target_pose_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=10)

        self.rc_stop = False
        self.vehicle_pose_msg = PoseStamped()
        self.target_pose_msg = PoseStamped()
        self.ackermann_msg = AckermannDrive()
        self.vehicle_pose_msg.header.frame_id = 'map'
        self.target_pose_msg.header.frame_id = 'map'
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

        data1, data2, data3 = None, None, None
        while not rospy.is_shutdown():
            if not data1:
                try:
                    data1 = rospy.wait_for_message(self.path_topic, Path, timeout=1.5)
                except:
                    data1 = None
                    rospy.loginfo("waiting for /gps_path")
            if not data2:
                try:
                    data2 = rospy.wait_for_message("/mavros/local_position/odom", Odometry, timeout=1.5)
                except:
                    data2 = None
                    rospy.loginfo("Waiting for /mavros/local_position/odom")

            if data1 and data2:
                rospy.loginfo('Subscribers defined properly')
                rospy.Subscriber(self.path_topic, Path, self.path_callback)
                rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)
                # newly added
                rospy.Subscriber('/curvature_profile', Float32MultiArray, self.curvature_profile_callback)
                rospy.Subscriber('/velocity_profile', Float32MultiArray, self.velocity_profile_callback)
                rospy.Subscriber('/mavros/global_position/set_gp_origin', GeoPointStamped, self.home_position_callback)
                break
        time.sleep(1)
        self.main_loop()

    def load_params(self):
        # IMP PARAMS
        rc_enable = rospy.get_param('patrol/rc_control', False)
        if rc_enable:
            self.ack_pub_topic = '/vehicle/cmd_drive_rc'
        else:
            od_enable = rospy.get_param('patrol/od_enable', False)
            if od_enable:
                self.ack_pub_topic = '/vehicle/cmd_drive_nosafe'
            else:
                self.ack_pub_topic = '/vehicle/cmd_drive_safe'

        self.path_topic = rospy.get_param('/patrol/path_topic', 'odom_path')  # gps_path or odom_path(good one)
        self.start_from_first_point = rospy.get_param('/patrol/from_start', False)  # TODO
        self.throttle_speed = rospy.get_param('/patrol/max_speed', 1.5)
        self.wheel_base = rospy.get_param('/patrol/wheel_base', 2)
        self.given_look_ahead_dis = rospy.get_param('/patrol/look_ahead_distance', 3)
        self.speed_constant = rospy.get_param('/patrol/speed_gain', 0.1)
        self.speed_curvature_gain = rospy.get_param('/patrol/speed_curvature_gain', 0.1)
        self.wait_time_at_ends = rospy.get_param('/patrol/wait_time_at_ends', 5)  # in secs
        self.mission_trips = rospy.get_param('/patrol/mission_trips', 0)

    def curvature_profile_callback(self, data):
        self.curvature_profile = data.data

    def velocity_profile_callback(self, data):
        self.velocity_profile = data.data

    def home_position_callback(self, data):
        self.home_gps_location = {
            'latitude': data.position.latitude,
            'longitude': data.position.longitude,
            'altitude': data.position.altitude
        }

    def path_callback(self, data):
        # data.poses.reverse() 
        self.revived_path = data.poses
        for pose in data.poses:
            _, _, heading = euler_from_quaternion(
                [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            heading = math.degrees(heading)
            self.path.append([pose.pose.position.x, pose.pose.position.y, heading])
        rospy.loginfo('path received')
        self.ind_end = len(self.path) - 1

    def odom_callback(self, data):
        self.time_when_odom = time.time()
        self.odom_data = data
        _, _, pose_heading = euler_from_quaternion(
            [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
             data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        )
        self.robot_state['x'] = data.pose.pose.position.x - self.wheel_base * np.cos(pose_heading)
        self.robot_state['y'] = data.pose.pose.position.y - self.wheel_base * np.sin(pose_heading)
        self.robot_state['yaw'] = pose_heading
        self.robot_state['vel'] = abs(data.twist.twist.linear.x)

        # fx = x + self.wheelbase * np.cos(yaw)
        # fy = y + self.wheelbase * np.sin(yaw)

    def find_close_point(self, robot, index_old):
        n = min(50, len(range(index_old, self.ind_end)))
        distance_list = [self.calc_distance(robot, ind) for ind in range(index_old, index_old + n)]
        ind = np.argmin(distance_list)
        print(" ind", ind)
        final = ind + index_old
        print("final ", final)
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
            self.calc_nearest_ind(robot)
        self.index_old, cross_track_dis = self.find_close_point(robot, self.index_old)
        # self.cross_track_dis = self.calc_distance(robot, self.close_idx)
        print(cross_track_dis)
        # print(self.given_look_ahead_dis / 2)
        if cross_track_dis > self.given_look_ahead_dis:  # try divided by 2
            # Find a point on the path which is look ahead distance from the closest path, then find distance to the
            # robot and give it as a look_ahead_distance to pure pursuit formulae.
            print("from here old")
            for ind in range(self.index_old, self.ind_end):
                print(ind)
                dis = self.calc_distance_idx(self.index_old, ind)
                if dis > self.given_look_ahead_dis:
                    present_look_ahead_dis = self.calc_distance(robot, ind)
                    return self.index_old, ind, self.given_look_ahead_dis, cross_track_dis
                if ind >= self.ind_end:
                    return ind, ind, self.given_look_ahead_dis, cross_track_dis
                    # return ind, ind, lhd,cross_track_dis

        else:
            print("from here")
            lhd = self.given_look_ahead_dis + self.speed_constant * robot['vel']  # evaluate it afterwards
            for ind in range(self.index_old, self.ind_end + 1):
                dis = self.calc_distance(robot, ind)
                if dis > lhd:
                    return self.index_old, ind, lhd, cross_track_dis
                if ind >= self.ind_end:
                    return ind, ind, lhd, cross_track_dis
                    # return ind, ind, lhd,cross_track_dis

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
        # print("Here on Loop")
        r = rospy.Rate(1)

        all_data_received = False
        while not all_data_received and not rospy.is_shutdown():
            # print(" Here in inner Loop")
            if len(self.path) < 1:
                rospy.loginfo('No path received')
            if len(self.curvature_profile) < 1:
                rospy.loginfo('No curvature_profile received')
            if len(self.velocity_profile) < 1:
                rospy.loginfo('No velocity_profile received')
            print(len(self.velocity_profile) > 1 and len(self.curvature_profile) > 1 and len(self.path) > 1)
            if len(self.velocity_profile) > 1 and len(self.curvature_profile) > 1 and len(self.path) > 1:
                rospy.loginfo("Path, velocity_profile and curvature profile are received")
                all_data_received = True
            r.sleep()

        rospy.loginfo('Pure pursuit is started')
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            print("---------------------------------------------------------------")

            # Main Loop
            self.vehicle_pose_msg.pose.position.x = self.robot_state['x']
            self.vehicle_pose_msg.pose.position.y = self.robot_state['y']
            self.vehicle_pose_msg.pose.orientation = self.odom_data.pose.pose.orientation
            self.vehicle_pose_pub.publish(self.vehicle_pose_msg)

            if self.start_from_first_point:
                self.index_old = 0
                self.start_from_first_point = False
                # close_idx, target_idx, lhd, cross_track_error = 0, 0,
            else:

                close_idx, target_idx, lhd, cross_track_error = self.target_index(self.robot_state)
                print('close_idx, target_idx, lhd, cross_track_error')
                print(close_idx, target_idx, lhd, cross_track_error)

                if target_idx >= self.ind_end:
                    rospy.loginfo('MISSION COMPLETED ')
                    self.send_ack_msg(0, 0, 0)
                    self.count_mission_repeat += 1
                    # print('mode ',self.mission_mode_selector(self.count_mission_repeat))
                    state, state_text = self.mission_mode_selector(self.count_mission_repeat)
                    if state:
                        print("------------------")
                        rospy.logwarn(state_text)
                        print("waiting for " + str(self.wait_time_at_ends) + ' secs')
                        time.sleep(self.wait_time_at_ends)
                        print("------------------")
                        continue
                    else:
                        print("------------------")
                        rospy.logwarn(state_text)
                        print("waiting for " + str(self.wait_time_at_ends) + ' secs')
                        time.sleep(self.wait_time_at_ends)
                        print("------------------")
                        break
                # publish target point
                self.target_pose_msg.pose.position.x = self.path[target_idx][0]
                self.target_pose_msg.pose.position.y = self.path[target_idx][1]
                self.target_pose_msg.pose.orientation = self.revived_path[target_idx].pose.orientation
                self.target_pose_pub.publish(self.target_pose_msg)
                delta_x = self.path[target_idx][0] - self.robot_state['x']
                delta_y = self.path[target_idx][1] - self.robot_state['y']
                slope = math.atan2(delta_y, delta_x)
                alpha = slope - self.robot_state['yaw']
                delta = math.atan2(2.0 * self.wheel_base * math.sin(alpha), lhd)
                delta_degrees = -1 * math.degrees(delta)
                steering_angle = np.clip(delta_degrees, -30, 30)
                # Speed calculation
                # speed = min(self.velocity_profile[close_idx],
                #             self.curvature_profile[close_idx] * self.speed_curvature_gain,
                #             self.throttle_speed)

                self.send_ack_msg(steering_angle, self.throttle_speed, 0)
                # print("\r Steering: " + str(round(steering_angle)) + "  Speed: " + str(
                #     round(speed)) + " CTE :" + str(round(cross_track_error,2)) + "  ", end="\t").
                print(" Steering: " + str(round(steering_angle)) + "  Speed: " + str(
                    round(self.throttle_speed)) + " CTE :" + str(round(cross_track_error, 2)) + "  ")
                print('vel: ', self.velocity_profile[close_idx])
                print("cur: ", self.curvature_profile[close_idx])
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

    pure_parsuit = PurePursuit()

    rospy.spin()
