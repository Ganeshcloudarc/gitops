#!/usr/bin/env python3
"""
pure pursuit controller implementation
"""
try:
    # python3 general packages
    import math
    import numpy as np
    from numpy import argmin, zeros
    import ros_numpy
    import rospy
    import tf2_ros
    import time

    # ros messages
    from nav_msgs.msg import Path, Odometry
    from geometry_msgs.msg import Point, PoseArray, PoseStamped
    from std_msgs.msg import Header
    from geographic_msgs.msg import GeoPointStamped
    from ackermann_msgs.msg import AckermannDrive
    from sensor_msgs.msg import NavSatFix

    from tf.transformations import euler_from_quaternion, quaternion_from_euler

    # autopilot related imports
    from autopilot_utils.tf_helper import current_robot_pose, convert_point, transform_cloud
    from autopilot_utils.pose_helper import distance_btw_poses, get_yaw, angle_btw_poses
    from vehicle_common.vehicle_config import vehicle_data
    from autopilot_msgs.msg import Trajectory, TrajectoryPoint
    from autopilot_msgs.msg import ControllerDiagnose

except Exception as e:
    import rospy

    rospy.logerr("No module %s", str(e))
    exit(e)


class PurePursuitController:
    def __init__(self):
        self.gps_robot_state = None
        self.robot_state = None
        self.trajectory_data = None
        self.updated_traj_time = time.time()
        prev_steering_angle = 0
        prev_time = 0
        prev_speed = 0

        # ros parameters
        self.max_forward_speed = rospy.get_param("/patrol/max_forward_speed", 1.8)
        self.min_forward_speed = rospy.get_param("/patrol/min_forward_speed", 0.5)
        self.min_look_ahead_dis = rospy.get_param("/pure_pursuit/min_look_ahead_dis", 3)
        self.max_look_ahead_dis = rospy.get_param("/pure_pursuit/max_look_ahead_dis", 6)
        self.time_out_from_input_trajectory = rospy.get_param("/pure_pursuit/time_out", 3)
        trajectory_in_topic = rospy.get_param("/trajectory_in", "/local_gps_trajectory")
        odom_topic = rospy.get_param("/patrol/odom_topic", "/vehicle/odom")
        gps_topic = rospy.get_param("/patrol/gps_topic", "/mavros/global_position/global")
        self.robot_base_frame = rospy.get_param("robot_base_frame", "base_link")

        failsafe_enable = rospy.get_param("/patrol/failsafe_enable", True)

        if failsafe_enable:
            cmd_topic = rospy.get_param("patrol/cmd_topic", "/vehicle/cmd_drive_safe")
        else:
            cmd_topic = rospy.get_param("patrol/pilot_cmd_in", "/vehicle/cmd_drive_safe")

        # Publishers
        self.ackermann_publisher = rospy.Publisher(cmd_topic, AckermannDrive, queue_size=10)
        target_pose_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=2)
        close_pose_pub = rospy.Publisher('/close_pose', PoseStamped, queue_size=2)

        controller_diagnose_pub = rospy.Publisher("pure_pursuit_diagnose", ControllerDiagnose, queue_size=2)

        # ros subscribers

        target_pose_msg = PoseStamped()
        self.ackermann_msg = AckermannDrive()

        trajectory_data, tf_data = None, None
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if not trajectory_data:
                try:
                    trajectory_data = rospy.wait_for_message(trajectory_in_topic, Trajectory, timeout=1)
                except:
                    trajectory_data = None
                    rospy.logwarn("Waiting for %s", str(trajectory_in_topic))
                else:
                    rospy.logdebug("Topic %s is active", str(trajectory_in_topic))
            if not tf_data:
                try:
                    tf_data = current_robot_pose("map", self.robot_base_frame)
                except:
                    tf_data = None
                    rospy.logwarn("Waiting for TF data between map and %s", self.robot_base_frame)
                else:
                    rospy.logdebug("TF data is available  between map and %s", self.robot_base_frame)

            if trajectory_data and tf_data:
                if trajectory_data.header.frame_id == "map":  # tf_data.header.frame_id:
                    rospy.Subscriber(trajectory_in_topic, Trajectory, self.trajectory_callback)
                    rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
                    rospy.Subscriber(gps_topic, NavSatFix, self.gps_callback)

                    rospy.loginfo("data received on tf and %s ", trajectory_in_topic)
                    break
                else:
                    rospy.logfatal('tf frames of path and odometry are not same: ')
                    sys.exit('tf frames of path and odometry are not same"')
                    rate.sleep()
        time.sleep(1)
        while not rospy.is_shutdown():
            if self.trajectory_data and self.robot_state and self.gps_robot_state:
                rospy.loginfo("trajectory_data and robot_state and gps_robot_state are avilable")
                break
            else:
                rospy.loginfo("waiting for trajectory_data or robot_state or gps_robot_state")
                rate.sleep()

        diagnostic_msg = ControllerDiagnose()
        diagnostic_msg.name = "Pure Pursuit Node"
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            try:

                # robot_pose = current_robot_pose("map", self.robot_base_frame)
                robot_pose = self.robot_state.pose.pose
                if not robot_pose:
                    rospy.logwarn("No tf between map and %s", self.robot_base_frame)
                    diagnostic_msg.level = diagnostic_msg.WARN
                    diagnostic_msg.message = 'Time out from tf'
                    diagnostic_msg.stamp = rospy.Time.now()
                    controller_diagnose_pub.publish(diagnostic_msg)
                    rate.sleep()
                    continue
                if time.time() - self.updated_traj_time > self.time_out_from_input_trajectory:
                    rospy.logwarn("timeout from input trajectory by %s", str(time.time() - self.updated_traj_time))
                    diagnostic_msg.level = diagnostic_msg.WARN
                    diagnostic_msg.message = "Timeout from input trajectory by " + str(time.time() - self.updated_traj_time)
                    diagnostic_msg.stamp = rospy.Time.now()
                    controller_diagnose_pub.publish(diagnostic_msg)
                    rate.sleep()
                    continue

                close_point_ind, close_dis = self.calc_nearest_ind(robot_pose)
                if close_point_ind == -1:
                    self.send_ack_msg(0, 0, 0)
                    rospy.loginfo("Reached end of local trajectory")
                    diagnostic_msg.level = diagnostic_msg.WARN
                    diagnostic_msg.message = "Reached end of local trajectory"
                    diagnostic_msg.stamp = rospy.Time.now()
                    controller_diagnose_pub.publish(diagnostic_msg)
                    rate.sleep()
                    continue
                else:
                    target_point_ind, lhd = self.target_index(robot_pose, close_point_ind)
                if target_point_ind == -1:
                    self.send_ack_msg(0, 0, 0)
                    rospy.loginfo("Lhd is less than min_look_ahead distance, reached end of local_traj")
                    diagnostic_msg.level = diagnostic_msg.WARN
                    diagnostic_msg.message = "Lhd is less than min_look_ahead distance, reached end of local_traj"
                    diagnostic_msg.stamp = rospy.Time.now()
                    controller_diagnose_pub.publish(diagnostic_msg)
                    rate.sleep()
                    continue
                target_pose = PoseStamped()
                target_pose.header.frame_id = "map"
                target_pose.pose = self.trajectory_data.points[target_point_ind].pose
                target_pose_pub.publish(target_pose)
                close_pose = PoseStamped()
                close_pose.header.frame_id = "map"
                close_pose.pose = self.trajectory_data.points[close_point_ind].pose
                close_pose_pub.publish(close_pose)


                slope = angle_btw_poses(self.trajectory_data.points[target_point_ind].pose, robot_pose)
                alpha = slope - get_yaw(robot_pose.orientation)
                delta = math.atan2(2.0 * vehicle_data.dimensions.wheel_base * math.sin(alpha), lhd)
                delta_degrees = -math.degrees(delta)
                steering_angle = np.clip(delta_degrees, -30, 30)
                speed = self.trajectory_data.points[close_point_ind].longitudinal_velocity_mps
                rospy.loginfo("steering angle: %s, speed: %s, break: %s", str(steering_angle), str(speed), str(0))
                if speed <= 0 :
                    self.send_ack_msg(steering_angle, speed, 1)
                else:
                    self.send_ack_msg(steering_angle, speed, 0)

                # fill the control diagnose topic
            
                diagnostic_msg.level = diagnostic_msg.OK
                diagnostic_msg.message = "Tracking path"
                diagnostic_msg.stamp = rospy.Time.now()
                diagnostic_msg.look_ahead = lhd
                diagnostic_msg.cte = close_dis
                diagnostic_msg.longitudinal_velocity_mps = speed
                diagnostic_msg.steering_angle = steering_angle
                diagnostic_msg.lateral_velocity_dps = steering_angle - prev_steering_angle / time.time() - prev_time
                diagnostic_msg.acceleration_mps2 = speed - prev_speed / time.time() - prev_time
                diagnostic_msg.target_pose = target_pose.pose
                diagnostic_msg.target_gps_pose = self.trajectory_data.points[target_point_ind].gps_pose
                diagnostic_msg.vehicle_pose = robot_pose
                diagnostic_msg.vehicle_gps_pose = self.gps_robot_state
                controller_diagnose_pub.publish(diagnostic_msg)
                prev_steering_angle = steering_angle
                prev_time = time.time()
                prev_speed = speed
                rate.sleep()
            except IndexError:
                rospy.logerr("index error occured")
                rate.sleep()
                continue
            except Exception as err:
                rospy.logerr(f"Error occured :{err}")
                rate.sleep()
                continue

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

        if distance_btw_poses(robot_pose, self.trajectory_data.points[ind].pose) < self.min_look_ahead_dis:
            return -1, -1
        else:
            return ind, distance_btw_poses(robot_pose, self.trajectory_data.points[ind-1].pose)

    def calc_nearest_ind(self, robot_pose):
        """
        calc index of the nearest point to current position
        Args:
            robot_pose: pose of robot
        Returns:
            close_index , distance
        """
        distance_list = [distance_btw_poses(robot_pose, point.pose) for point in self.trajectory_data.points]
        if len(distance_list) > 0:
            ind = np.argmin(distance_list)
            dis = distance_list[ind]
            if ind == len(self.trajectory_data.points)-1:
                return -1, -1
            else:
                return ind, dis
        else:
            return 0, 0


    def trajectory_callback(self, data):
        self.trajectory_data = data
        self.updated_traj_time = time.time()

    def odom_callback(self, data):
        rospy.loginfo_once("Odom data received")
        self.robot_state = data

    def gps_callback(self, data):
        self.gps_robot_state = data


    def compute_lookahead_distance(self, robot_speed):
        # return 3
        # return self.min_look_ahead_dis
        # https://github.com/bosonrobotics/autopilot_boson/issues/22
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

    def send_ack_msg(self, steering_angle, speed, jerk):
        self.ackermann_msg.steering_angle = steering_angle
        self.ackermann_msg.speed = speed
        self.ackermann_msg.jerk = jerk
        self.ackermann_publisher.publish(self.ackermann_msg)


if __name__ == "__main__":
    rospy.init_node('Pure_pursuit_controller_node')

    pure_pursuit = PurePursuitController()

    rospy.spin()
