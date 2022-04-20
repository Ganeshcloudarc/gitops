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
    from std_msgs.msg import Float32MultiArray
    from geographic_msgs.msg import GeoPointStamped
except Exception as e:
    rospy.loginfo('No module named :', str(e))
    exit(e)


class Config:
    def __init__(self):
        self.MISSION_MODE = rospy.get_param('/patrol/mission_mode', 0)
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
        self.odom_topic = rospy.get_param("/patrol/odom_topic", "/mavros/local_position/odom")
        self.start_from_first_point = rospy.get_param('/patrol/from_start', False)  # TODO
        self.throttle_speed = rospy.get_param('/patrol/max_speed', 1.5)
        self.wheel_base = rospy.get_param('/patrol/wheel_base', 2)
        self.given_look_ahead_dis = rospy.get_param('/patrol/look_ahead_distance', 3)
        self.speed_curvature_gain = rospy.get_param('/patrol/speed_curvature_gain', 0.1)
        self.wait_time_at_ends = rospy.get_param('/patrol/wait_time_at_ends', 5)  # in secs
        self.mission_trips = rospy.get_param('/patrol/mission_trips', 0)
        # Pure Pursuit specific
        self.min_look_ahead = rospy.get_param('/patrol/min_look_ahead', 3)
        self.max_look_ahead = rospy.get_param('/patrol/max_look_ahead', 6)
        self.speed_to_lookahead_ratio = rospy.get_param('/patrol/speed_to_lookahead_ratio', 2)  # taken from autoware

        self.min_speed = rospy.get_param('/patrol/min_speed', 1)
        self.max_speed = rospy.get_param('/patrol/max_speed', 2)
        self.curvature_speed_ratio = rospy.get_param('/patrol/curvature_speed_ratio', 2)


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle


class PurePursuit:
    def __init__(self):
        # self.load_params(
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
                    data1 = rospy.wait_for_message(self.config.path_topic, Path, timeout=1.5)
                except:
                    data1 = None
                    rospy.loginfo("waiting for /gps_path")
            if not data2:
                try:
                    data2 = rospy.wait_for_message(self.config.odom_topic, Odometry, timeout=1.5)
                except:
                    data2 = None
                    rospy.loginfo("Waiting for" + str(self.config.odom_topic))

            if data1 and data2:
                rospy.loginfo('Subscribers defined properly')
                rospy.Subscriber(self.config.path_topic, Path, self.path_callback)
                rospy.Subscriber(self.config.odom_topic, Odometry, self.odom_callback)
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
        self.speed_to_lookahead_ratio = rospy.get_param('/patrol/speed_to_lookahead_ratio', 2 ) # taken from autoware
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
        self.robot_state['x'] = data.pose.pose.position.x - self.config.wheel_base * np.cos(pose_heading)
        self.robot_state['y'] = data.pose.pose.position.y - self.config.wheel_base * np.sin(pose_heading)
        self.robot_state['yaw'] = pose_heading
        self.robot_state['vel'] = abs(data.twist.twist.linear.x)

        # fx = x + self.wheelbase * np.cos(yaw)
        # fy = y + self.wheelbase * np.sin(yaw)


    def compute_velocity_at_point(self, ind):
        #  depends upon
        return (self.config.min_speed + self.config.max_speed)/2

    def find_close_point(self, robot, index_old):

        n = min(100, len(range(index_old, self.ind_end)))
        distance_list = [self.calc_distance(robot, ind) for ind in range(index_old, index_old + n)]
        ind = np.argmin(distance_list)
        print(" ind", ind)
        final = ind + index_old
        print("final ", final)
        dis = distance_list[ind]
        return final, dis

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

    def stanley_control(self, target_idx, error_front_axle):
        # theta_e corrects the heading error
        print("target_idx", target_idx)
        # a = arctan2((self.path[target_idx][1]-self.path[target_idx-1][1]),(self.path[target_idx][0]-self.path[target_idx-1][0]))
        # print('path angle',math.degrees(a))
        # print('path heading',math.degrees(self.path[target_idx][2]))
        theta_e = normalize_angle(self.path[target_idx][2] - self.robot['yaw'])
        # theta_e = self.normalize_angle(a - self.robot['yaw'])

        print("heading error: ", math.degrees(theta_e))
        # theta_d corrects the cross track error
        ks = 1
        theta_d = np.arctan2(self.control_gain_k * error_front_axle, ks + self.robot['vel'])
        print("ctc componet: ", math.degrees(theta_d))

        # Steering control
        delta = theta_e + theta_d
        print("delta", delta)

        return delta

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
        close_idx, _ = self.calc_nearest_ind(self.robot, 0)
        while not rospy.is_shutdown():
            close_idx, cte = self.find_close_point(self.robot, close_idx)

            print('close_idx, cte')
            print(close_idx, cte)

            front_axle_vector = [np.sin(self.robot_state['yaw']), -np.cos(self.robot_state['yaw'])]
            nearest_path_vector = [self.path[close_idx][0], self.path[close_idx][1]]
            crosstrack_error = np.sign(np.dot(nearest_path_vector, front_axle_vector)) * cte
            print("signed ctc", crosstrack_error)

            self.target_pose_msg.pose.position.x = self.path[close_idx][0]
            self.target_pose_msg.pose.position.y = self.path[close_idx][1]
            self.target_pose_msg.pose.orientation = self.revived_path[close_idx].pose.orientation
            self.target_pose_pub.publish(self.target_pose_msg)

            if close_idx + 1 >= self.ind_end:
                rospy.loginfo('MISSION COMPLETED ')
                self.send_ack_msg(0, 0, 0)
                break

            delta = self.stanley_control(close_idx, cte)
            print("raw steering: ", math.degrees(delta))

            steering_angle = np.clip(math.degrees(delta), -self.max_steer, self.max_steer)
            print('steering_angle: ', steering_angle)
            self.send_ack_msg(steering_angle, self.throttle_speed, 0)
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
