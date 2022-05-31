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
    from std_msgs.msg import Float32MultiArray, Int32MultiArray, Bool, Int16, String, Int8, UInt16, Float32
    from geographic_msgs.msg import GeoPointStamped
    from autopilot_msgs.msg import ControllerDiagnose
    # from vehicle_common.vehicle_common import VehicleData
    from vehicle_common.vehicle_config import vehicle_data
    from vehicle_common.tf_helper import current_robot_pose
except Exception as e:
    print('No module named :', str(e))
    exit(e)


def get_yaw(orientation):
    _, _, heading = euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w])
    return heading


class PurePursuit:
    def __init__(self):

        self.home_gps_location = {
            'latitude': 0.0,
            'longitude': 0.0,
            'altitude': 0.0
        }
        # parameters
        self.max_forward_speed = rospy.get_param("/patrol/max_forward_speed", 1.2)
        self.min_forward_speed = rospy.get_param("/patrol/min_forward_speed", 0.3)
        self.max_backward_speed = rospy.get_param("/patrol/max_backward_speed", -1.2)
        self.min_forward_speed = rospy.get_param("/patrol/min_backward_speed", -0.3)

        self.min_look_ahead_dis = rospy.get_param("/pure_pursuit/min_look_ahead_dis", 3)
        self.max_look_ahead_dis = rospy.get_param("/pure_pursuit/max_look_ahead_dis", 6)

        self.path_topic = rospy.get_param("/patrol/path_topic", 'odom_path')
        self.wait_time_on_mission_complete = rospy.get_param("/patrol/wait_time_on_mission_complete", 10)
        self.mission_continue = rospy.get_param("/patrol/mission_continue", False)

        # Publishers
        self.ackermann_publisher = rospy.Publisher("pure_pursuit/cmd_drive", AckermannDrive, queue_size=10)
        self.vehicle_pose_pub = rospy.Publisher('/vehicle_pose', PoseStamped, queue_size=2)
        self.target_pose_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=2)
        self.mission_count_pub = rospy.Publisher('/mission_count', Float32, queue_size=2)
        self.controller_diagnose_pub = rospy.Publisher("pure_pursuit_diagnose", ControllerDiagnose, queue_size=2)
        self.vehicle_pose_msg = PoseStamped()
        self.target_pose_msg = PoseStamped()
        self.ackermann_msg = AckermannDrive()
        self.path = []  # change it later
        self.velocity_profile = []
        self.curvature_profile = []

        self.ind_end = 0
        self.index_old = None
        self.path_end_index = None
        self.time_when_odom = None
        self.odom_data = None
        self.revived_path = None
        self.present_look_ahead = None
        self.target_id = None
        self.close_idx, self.cross_track_dis = 0.0, 0.0
        self.count_mission_repeat = 0
        self.mission_complete = True
        self.odom_wait_time_limit = 1  # change it
        self.time_when_odom_cb = time.time()

        path_data, odom_data = None, None
        while not rospy.is_shutdown():
            if not path_data:
                try:
                    path_data = rospy.wait_for_message(self.path_topic, Path, timeout=1)
                except:
                    path_data = None
                    rospy.logwarn("Waiting for %s", str(self.path_topic))
                else:
                    rospy.logdebug("Topic %s is active", str(self.path_topic))
            if not odom_data:
                try:
                    odom_data = rospy.wait_for_message(self.odom_topic, Odometry, timeout=1)
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
        print("exited main loop")

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
        self.path_end_index = len(self.revived_path) - 1

    def odom_callback(self, data):
        rospy.loginfo_once("Odom data received")
        self.time_when_odom_cb = time.time()
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
        return self.config.min_look_ahead
        # # https://gitlab.com/-/ide/project/RamanaBotta/AutowareAuto/tree/master/-/src/control/pure_pursuit/src/pure_pursuit.cpp/#L136
        # rospy.loginfo("vel %s ", vel)
        # look_ahead = vel * self.config.speed_to_lookahead_ratio
        # rospy.loginfo("vel_lookahead: %s", look_ahead)
        # final_look_ahead = max(self.config.min_look_ahead, min(look_ahead, self.config.max_look_ahead))
        # rospy.loginfo('final_look_ahead %s', final_look_ahead)
        # return final_look_ahead

    def compute_velocity_at_point(self, curvature, velocity_at_index):
        return self.config.average_speed
        # # TODO
        # # add obstacle velocity_profile.
        # rospy.loginfo("curvature %s", curvature)
        # try:
        #     circum_radius = (1 / curvature)
        # except:
        #     circum_radius = 10000
        # rospy.loginfo("circum radius %s", circum_radius)
        # curvature_vel = circum_radius * self.config.curvature_speed_ratio
        # rospy.loginfo("curvature_vel: %s velocity_at_index: %s max_speed: %s", curvature_vel, velocity_at_index,
        #                self.config.max_speed)
        # final_vel = min(self.config.max_speed, velocity_at_index, curvature_vel)
        # final_vel = max(final_vel, self.config.min_speed)
        # return final_vel

    def find_close_point(self, robot, index_old):
        # n = min(100, len(range(index_old, self.ind_end)))
        # distance_list = [self.calc_distance(robot, ind) for ind in range(index_old, index_old + n)]
        # ind = np.argmin(distance_list)
        # final = ind + index_old
        # dis = distance_list[ind]
        # return final, dis
        close_dis = self.calc_distance(robot, index_old)
        for ind in range(index_old + 1, self.ind_end):
            dis = self.calc_distance(robot, ind)
            if close_dis >= dis:
                close_dis = dis
            else:
                # print("find close", index_old, ind)
                return ind - 1, close_dis

    def target_index(self, robot_pose):
        """
        search index of target point in the reference path.
        Args:
            robot_pose:  pose of robot
        Returns:
            close_index, target_index, lookahead_distance, cross_track_dis,
        """
        # important condition check the
        if self.index_old is None:
            self.index_old, cross_track_dis = self.calc_nearest_ind(robot_pose)

        else:
            self.index_old, cross_track_dis = self.find_close_point(robot_pose, self.index_old)
        # The following implementation was inspired from
        # http://dyros.snu.ac.kr/wp-content/uploads/2021/02/Ahn2021_Article_AccuratePathTrackingByAdjustin-1.pdf
        sum_dis = 0
        for ind in range(self.index_old, self.ind_end):
            sum_dis += self.distance_between_points_by_index(ind)
            # print(sum_dis)
            if sum_dis >= self.min_look_ahead:
                lhd = self.calc_distance(robot_pose, ind)
                return self.index_old, ind, lhd, cross_track_dis
            if ind + 1 >= self.ind_end:
                return ind, ind, 0, cross_track_dis
        return self.ind_end, self.ind_end, 0, 0

    def distance_between_points_by_index(self, ind):
        return math.hypot(self.path[ind][0] - self.path[ind + 1][0], self.path[ind][1] - self.path[ind + 1][1])

    def calc_distance(self, robot_pose, ind):

        return math.hypot(robot_pose.position.x - self.path[ind][0], robot_pose.position.y - self.path[ind][1])

    def calc_distance_idx(self, close, next_id):
        return math.hypot(self.path[close][0] - self.path[next_id][0], self.path[close][1] - self.path[next_id][1])

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
        r = rospy.Rate(30)
        diagnostic_msg = ControllerDiagnose()
        diagnostic_msg.name = "Pure Pursuit Node"
        while not rospy.is_shutdown():

            robot_pose = current_robot_pose()

            close_idx, target_idx, lhd, cross_track_error = self.target_index(robot_pose)
            rospy.loginfo("close index %s , target point index %s, lookahead dis %s, ctc %s",
                          str(close_idx), str(target_idx), str(lhd), str(cross_track_error))

            if target_idx + 1 >= self.ind_end:
                self.count_mission_repeat += 1
                rospy.loginfo(' MISSION COUNT %s ', self.count_mission_repeat)
                self.mission_count_pub.publish(self.count_mission_repeat)
                self.send_ack_msg(0, 0, 0)
                diagnostic_msg.level = diagnostic_msg.OK
                diagnostic_msg.message = 'MISSION COUNT  ' + str(self.count_mission_repeat)
                self.controller_diagnose_pub.publish(diagnostic_msg)
                if self.config.mission_continue:
                    rospy.loginfo("waiting for %s secs", self.config.wait_time_at_end_point)
                    time.sleep(self.config.wait_time_at_end_point)
                    self.index_old = 0

                if self.config.mission_repeat:
                    print("waiting for 5 secs")
                    time.sleep(5)
                    self.revived_path.reverse()
                    self.path.reverse()
                    self.index_old = None
                    if self.speed < 0:
                        self.speed = abs(self.speed)
                    else:
                        self.speed = -self.speed
                else:
                    rospy.loginfo('MISSION COMPLETED')
                    diagnostic_msg.level = diagnostic_msg.OK
                    diagnostic_msg.message = 'MISSION COMPLETED'
                    self.controller_diagnose_pub.publish(diagnostic_msg)
                    break

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
            speed = self.compute_velocity_at_point(self.curvature_profile[target_idx],
                                                   self.velocity_profile[target_idx])
            speed = self.speed
            rospy.loginfo("cur: %s, vel: %s", self.curvature_profile[target_idx], self.velocity_profile[target_idx])
            rospy.loginfo("steering angle: %s, speed: %s, break: %s", str(steering_angle), str(speed), str(0))
            rospy.loginfo("self.speed %s", speed)
            timeout = time.time() - self.time_when_odom_cb

            # diagnose msg
            diagnostic_msg.level = diagnostic_msg.OK
            diagnostic_msg.message = 'Executing path'
            diagnostic_msg.look_ahead = lhd
            diagnostic_msg.cte = cross_track_error
            diagnostic_msg.speed = speed
            diagnostic_msg.steering_angle = steering_angle
            diagnostic_msg.current_point_heading = math.degrees(self.robot_state['yaw'])
            diagnostic_msg.target_point_heading = self.path[target_idx][2]
            diagnostic_msg.current_point_curvature = self.curvature_profile[close_idx]
            diagnostic_msg.target_point_curvature = self.curvature_profile[target_idx]

            if timeout > self.odom_wait_time_limit:
                rospy.logwarn('Time out from Odometry: %s', str(timeout))
                diagnostic_msg.level = diagnostic_msg.WARN
                diagnostic_msg.message = 'Time out from Odometry: ' + str(timeout)
                diagnostic_msg.speed = 0
                diagnostic_msg.steering_angle = 0
                self.controller_diagnose_pub.publish(diagnostic_msg)
                self.send_ack_msg(0, 0, 0)
                r.sleep()
                continue

            self.send_ack_msg(steering_angle, speed, 0)
            self.controller_diagnose_pub.publish(diagnostic_msg)
            r.sleep()

    def send_controller_diagnose(self):
        pass

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
