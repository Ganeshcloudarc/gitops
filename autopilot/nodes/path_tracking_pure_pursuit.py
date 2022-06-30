#!/usr/bin/env python3
try:
    import rospy
    import rospkg
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import PoseStamped, Quaternion
    from sensor_msgs.msg import NavSatFix, MagneticField, Imu
    from ackermann_msgs.msg import AckermannDrive
    import math
    import sys
    import time
    import numpy as np
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
    from std_msgs.msg import Float32MultiArray, Int32MultiArray, Bool, Int16, String, Int8, UInt16, Float32
    from geographic_msgs.msg import GeoPointStamped
    from autopilot_msgs.msg import ControllerDiagnose
    from vehicle_common.vehicle_config import vehicle_data
    from autopilot_utils.tf_helper import current_robot_pose
except Exception as e:
    print('No module named :', str(e))
    exit(e)


def get_yaw(orientation):
    _, _, yaw = euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w])
    return yaw


def get_poses_slope(pose1, pose2):
    delta_x = pose1.position.x - pose2.position.x
    delta_y = pose1.position.y - pose2.position.y
    return math.atan2(delta_y, delta_x)


class PurePursuit:
    def __init__(self):

        self.home_gps_location = {
            'latitude': 0.0,
            'longitude': 0.0,
            'altitude': 0.0
        }
        # parameters
        self.max_forward_speed = rospy.get_param("/patrol/max_forward_speed", 1.8)
        self.min_forward_speed = rospy.get_param("/patrol/min_forward_speed", 0.5)
        self.max_backward_speed = rospy.get_param("/patrol/max_backward_speed", -1.2)
        self.min_backward_speed = rospy.get_param("/patrol/min_backward_speed", -0.3)

        self.min_look_ahead_dis = rospy.get_param("/pure_pursuit/min_look_ahead_dis", 3)
        self.max_look_ahead_dis = rospy.get_param("/pure_pursuit/max_look_ahead_dis", 6)

        self.path_topic = rospy.get_param("/patrol/path_topic", 'odom_path')
        self.odom_topic = rospy.get_param("/patrol/odom_topic", '/mavros/global_position/local')
        self.wait_time_on_mission_complete = rospy.get_param("/patrol/wait_time_on_mission_complete", 10)
        self.mission_continue = rospy.get_param("/patrol/mission_continue", False)
        self.mission_trips = rospy.get_param("/patrol/mission_trips", 0)
        self.base_frame = rospy.get_param("/patrol/base_frame", "base_link")
        self.carla_sim = rospy.get_param("/carla_sim/activate", False)

        if self.carla_sim:
            self.cmd_topic = "pure_pursuit/cmd_drive"
            self.max_forward_speed = rospy.get_param("/patrol/max_forward_speed", 0.3)
            self.min_forward_speed = rospy.get_param("/patrol/min_forward_speed", 0.03)

        else:
            self.cmd_topic = "vehicle/cmd_drive_safe"

        # Publishers
        self.ackermann_publisher = rospy.Publisher(self.cmd_topic, AckermannDrive, queue_size=10)
        self.target_pose_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=2)
        self.mission_count_pub = rospy.Publisher('/mission_count', Float32, queue_size=2)
        self.controller_diagnose_pub = rospy.Publisher("pure_pursuit_diagnose", ControllerDiagnose, queue_size=2)
        self.target_pose_msg = PoseStamped()
        self.ackermann_msg = AckermannDrive()
        self.path = []  # change it later
        self.velocity_profile = []
        self.curvature_profile = []

        self.index_old = None
        self.path_end_index = None
        self.odom_data = None
        self.revived_path = None
        self.curvature_velocity = None
        self.updated_vel = None
        self.present_look_ahead = None
        self.target_id = None
        self.close_idx, self.cross_track_dis = 0.0, 0.0
        self.count_mission_repeat = 0
        self.odom_wait_time_limit = 1  # change it
        self.robot_state = None

        path_data, tf_data = None, None
        while not rospy.is_shutdown():
            if not path_data:
                try:
                    path_data = rospy.wait_for_message(self.path_topic, Path, timeout=1)
                except:
                    path_data = None
                    rospy.logwarn("Waiting for %s", str(self.path_topic))
                else:
                    rospy.logdebug("Topic %s is active", str(self.path_topic))
            if not tf_data:
                try:
                    tf_data = current_robot_pose("map", self.base_frame)
                except:
                    tf_data = None
                    rospy.logwarn("Waiting for TF data between map and base_link")
                else:
                    rospy.logdebug("TF data is available  between map and base_link")

            if path_data and tf_data:
                if path_data.header.frame_id == "map":  # tf_data.header.frame_id:
                    self.target_pose_msg.header.frame_id = path_data.header.frame_id
                    rospy.Subscriber(self.path_topic, Path, self.path_callback)
                    rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
                    rospy.Subscriber('/curvature_profile', Float32MultiArray, self.curvature_profile_callback)
                    rospy.Subscriber('/velocity_profile', Float32MultiArray, self.velocity_profile_callback)
                    rospy.Subscriber("curvature_velocity_profile", Float32MultiArray, self.curvature_velocity_cb)
                    rospy.Subscriber("/vel_updates", Float32MultiArray, self.vel_update_cb)
                    rospy.Subscriber('/mavros/global_position/set_gp_origin', GeoPointStamped,
                                     self.home_position_callback)
                    rospy.loginfo('All the subscribers defined properly %s %s %s %s %s', self.path_topic,
                                  self.odom_topic, '/curvature_profile', '/velocity_profile',
                                  '/mavros/global_position/set_gp_origin')
                    break
                else:
                    rospy.logfatal('tf frames of path and odometry are not same: ')
                    sys.exit('tf frames of path and odometry are not same"')
        time.sleep(1)
        self.main_loop()

    def curvature_profile_callback(self, data):
        rospy.logdebug("curvature data received of length %s", str(len(data.data)))
        self.curvature_profile = data.data

    def velocity_profile_callback(self, data):
        rospy.logdebug("velocity data received of length %s", str(len(data.data)))
        self.velocity_profile = data.data

    def curvature_velocity_cb(self, data):
        self.curvature_velocity = data.data

    def vel_update_cb(self, data):
        self.updated_vel = data.data

    def home_position_callback(self, data):
        self.home_gps_location = {
            'latitude': data.position.latitude,
            'longitude': data.position.longitude,
            'altitude': data.position.altitude
        }
        rospy.loginfo("home position data received %s", str(self.home_gps_location))

    def path_callback(self, data):
        rospy.logdebug("path data data received of length %s", str(len(data.poses)))
        # data.poses.reverse()
        self.path = data.poses
        self.path_end_index = len(self.path) - 1

    def odom_callback(self, data):
        rospy.loginfo_once("Odom data received")
        self.robot_state = data

    def compute_lookahead_distance(self, vel):
        # return 3
        # return self.min_look_ahead_dis
        # https://github.com/bosonrobotics/autopilot_boson/issues/22
        robot_speed = abs(self.robot_state.twist.twist.linear.x)
        if robot_speed > self.max_forward_speed:
            lhd = (robot_speed * self.min_look_ahead_dis) / self.max_forward_speed
            if self.min_look_ahead_dis <= lhd <= self.max_look_ahead_dis:
                return lhd
            elif lhd <= self.min_look_ahead_dis:
                return self.min_look_ahead_dis
            else:
                return self.max_look_ahead_dis
        else:
            return self.min_look_ahead_dis

        # # https://gitlab.com/-/ide/project/RamanaBotta/AutowareAuto/tree/master/-/src/control/pure_pursuit/src/pure_pursuit.cpp/#L136
        # rospy.loginfo("vel %s ", vel)
        # look_ahead = vel * self.config.speed_to_lookahead_ratio
        # rospy.loginfo("vel_lookahead: %s", look_ahead)
        # final_look_ahead = max(self.config.min_look_ahead, min(look_ahead, self.config.max_look_ahead))
        # rospy.loginfo('final_look_ahead %s', final_look_ahead)
        # return final_look_ahead

    def compute_velocity_at_index(self, index):
        # if self.carla_sim:
        #     return 0.3
        if self.updated_vel:
            return self.updated_vel[index]
        else:
            # final_vel = min(self.velocity_profile[index], self.curvature_velocity[index])
            final_vel = max(self.min_forward_speed, self.curvature_velocity[index])
            return final_vel

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
        lhd = self.compute_lookahead_distance(self.index_old)
        for ind in range(self.index_old, self.path_end_index):
            sum_dis += self.distance_to_next_index(ind)
            # print(sum_dis)
            if sum_dis >= lhd:
                lhd = self.calc_distance(robot_pose, ind)
                return self.index_old, ind, lhd, cross_track_dis
            if ind >= self.path_end_index:
                return ind, ind, 0, cross_track_dis
        return self.path_end_index, self.path_end_index, 0, 0

    def distance_to_next_index(self, ind):
        return math.hypot(self.path[ind].pose.position.x - self.path[ind + 1].pose.position.x,
                          self.path[ind].pose.position.y - self.path[ind + 1].pose.position.y)

    def calc_distance(self, robot_pose, ind):
        return math.hypot(robot_pose.position.x - self.path[ind].pose.position.x, robot_pose.position.y -
                          self.path[ind].pose.position.y)

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
        r = rospy.Rate(50)
        diagnostic_msg = ControllerDiagnose()
        diagnostic_msg.name = "Pure Pursuit Node"
        while not rospy.is_shutdown():
            robot_pose = current_robot_pose("map", self.base_frame)
            if robot_pose is None:
                diagnostic_msg.level = diagnostic_msg.WARN
                diagnostic_msg.message = 'Time out from tf'
                diagnostic_msg.speed = 0
                diagnostic_msg.steering_angle = 0
                self.controller_diagnose_pub.publish(diagnostic_msg)
                continue

            close_idx, target_idx, lhd, cross_track_error = self.target_index(robot_pose)
            rospy.loginfo("close index %s , target point index %s, lookahead dis %s, ctc %s",
                          str(close_idx), str(target_idx), str(lhd), str(cross_track_error))

            if close_idx >= self.path_end_index:
                self.count_mission_repeat += 1
                rospy.loginfo(' MISSION COUNT %s ', self.count_mission_repeat)
                self.mission_count_pub.publish(self.count_mission_repeat)
                self.send_ack_msg(0, 0, 0)
                diagnostic_msg.level = diagnostic_msg.OK
                diagnostic_msg.message = 'MISSION COUNT  ' + str(self.count_mission_repeat)
                self.controller_diagnose_pub.publish(diagnostic_msg)
                if self.mission_continue:
                    if self.mission_trips == 0:
                        rospy.loginfo('restarting the mission')
                        diagnostic_msg.level = diagnostic_msg.OK
                        diagnostic_msg.message = 'MISSION COMPLETED AND RESTARTING THE MISSION-' + str(
                            self.count_mission_repeat)
                        self.controller_diagnose_pub.publish(diagnostic_msg)
                        rospy.loginfo("waiting for %s", str(self.wait_time_on_mission_complete))
                        time.sleep(self.wait_time_on_mission_complete)
                        self.index_old = 1
                        continue
                    elif self.count_mission_repeat <= self.mission_trips:
                        rospy.loginfo('completed mission  %s and target is %s', self.count_mission_repeat,
                                      self.mission_trips)
                        diagnostic_msg.level = diagnostic_msg.OK
                        diagnostic_msg.message = "MISSION:" + str(self.count_mission_repeat) + " IS COMPLETED"
                        self.controller_diagnose_pub.publish(diagnostic_msg)
                        rospy.loginfo("waiting for %s", str(self.wait_time_on_mission_complete))
                        time.sleep(self.wait_time_on_mission_complete)
                        self.index_old = 1
                        continue
                    else:
                        rospy.loginfo('completed mission')
                        diagnostic_msg.level = diagnostic_msg.OK
                        diagnostic_msg.message = "MISSION IS COMPLETED"
                        self.controller_diagnose_pub.publish(diagnostic_msg)
                        break
                else:
                    self.send_ack_msg(0, 0, 0)
                    rospy.loginfo('completed mission')
                    diagnostic_msg.level = diagnostic_msg.OK
                    diagnostic_msg.message = "MISSION IS COMPLETED"
                    self.controller_diagnose_pub.publish(diagnostic_msg)
                    break

            else:
                self.target_pose_pub.publish(self.path[target_idx])
                slope = get_poses_slope(self.path[target_idx].pose, robot_pose)
                alpha = slope - get_yaw(robot_pose.orientation)
                delta = math.atan2(2.0 * vehicle_data.dimensions.wheel_base * math.sin(alpha), lhd)
                delta_degrees = -math.degrees(delta)
                steering_angle = np.clip(delta_degrees, -30, 30)
                speed = self.compute_velocity_at_index(target_idx)
                rospy.loginfo("steering angle: %s, speed: %s, break: %s", str(steering_angle), str(speed), str(0))

                # diagnose msg
                diagnostic_msg.level = diagnostic_msg.OK
                diagnostic_msg.message = 'Executing path'
                diagnostic_msg.look_ahead = lhd
                diagnostic_msg.cte = cross_track_error
                diagnostic_msg.speed = speed
                diagnostic_msg.steering_angle = steering_angle
                diagnostic_msg.current_point_heading = get_yaw(robot_pose.orientation)
                diagnostic_msg.target_point_heading = get_yaw(self.path[target_idx].pose.orientation)
                diagnostic_msg.current_point_curvature = 0  # self.curvature_profile[close_idx]
                diagnostic_msg.target_point_curvature = 0  # self.curvature_profile[target_idx]

                self.send_ack_msg(steering_angle, speed, 0)
                self.controller_diagnose_pub.publish(diagnostic_msg)
            r.sleep()
            print("sleep")

    def send_ack_msg(self, steering_angle, speed, jerk):
        self.ackermann_msg.steering_angle = steering_angle
        self.ackermann_msg.speed = speed
        self.ackermann_msg.jerk = jerk
        self.ackermann_publisher.publish(self.ackermann_msg)


if __name__ == "__main__":
    rospy.init_node('pure_pursuit1')

    pure_pursuit = PurePursuit()

    rospy.spin()
