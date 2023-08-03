#!/usr/bin/env python3
"""
pure pursuit controller implementation
"""
try:
    # python3 general packages
    import math
    import numpy as np
    from numpy import argmin, zeros
    # import ros_numpy
    import rospy
    import tf2_ros
    import time, sys
    # import dynamic reconfigure related imports
    from dynamic_reconfigure.server import Server
    from autopilot.cfg import pid_Config
    # ros messages
    from nav_msgs.msg import Path, Odometry
    from geometry_msgs.msg import Point, PoseArray, PoseStamped
    from std_msgs.msg import Header
    from geographic_msgs.msg import GeoPointStamped
    from ackermann_msgs.msg import AckermannDrive
    from sensor_msgs.msg import NavSatFix
    from nav_msgs.msg import Path
    from std_msgs.msg import Float32

    from tf.transformations import euler_from_quaternion, quaternion_from_euler

    # autopilot related imports
    from autopilot_utils.tf_helper import current_robot_pose, convert_point, transform_cloud
    from autopilot_utils.pose_helper import distance_btw_poses, get_yaw, angle_btw_poses, normalize_angle
    from vehicle_common.vehicle_config import vehicle_data
    from autopilot_msgs.msg import Trajectory, TrajectoryPoint
    from autopilot_msgs.msg import ControllerDiagnose, FloatKeyValue
except Exception as e:
    import rospy
    rospy.logerr("No module %s", str(e))
    exit(e)


def heading_check(robot_orientation, path_orientation):
    robot_heading = normalize_angle(get_yaw(robot_orientation))
    path_heading = normalize_angle(get_yaw(path_orientation))
    # print(robot_heading- path_heading)
    if abs(robot_heading - path_heading) > math.radians(90):
        # rospy.logwarn("Headings are %s apart ", str(abs(robot_heading - path_heading)))
        heading_ok = False
    else:
        heading_ok = True
    return heading_ok

def getLineNumber():
    return sys._getframe().f_back.f_lineno

class PurePursuitController:
    def __init__(self):
        srv = Server(pid_Config,self.pid_callback)
        self.updated_odom_time = None
        self.trajectory_len = None
        self.gps_robot_state = None
        self.robot_state = None
        self.trajectory_data = None
        self.updated_traj_time = time.time()

        #PID parameters
        self.cte = 0
        self.sum_cte = 0
        self.is_reverse = False
        self.kp_pid = 0.01
        self.ki_pid = 0.01
        self.e1 = 0
        self.e2 = 0
        self.kp_speed = rospy.get_param("/pure_pursuit/kp_speed",0.1)
        self.allowable_cte_for_adjustable_speed = rospy.get_param("/pure_pursuit/allowable_cte_for_adjustable_speed",0.3)
        self.rqt = rospy.get_param("/pure_pursuit/enable_rqt")

        self.is_pp_pid = rospy.get_param("/patrol/pp_with_pid",False)
        # ros parameters
        self.max_speed = rospy.get_param("/patrol/max_forward_speed", 1.8) # default value of max speed is max forward speed.
        self.max_forward_speed = rospy.get_param("/patrol/max_forward_speed", 1.8)
        self.min_forward_speed = rospy.get_param("/patrol/min_forward_speed", 0.5)
        self.max_backward_speed = abs(rospy.get_param("/patrol/max_backward_speed",1.0)) # doing abs() as reversing uses same logic as forward wrt to speed
        self.min_backward_speed = abs(rospy.get_param("/patrol/min_backward_speed",0.8))
        self.min_look_ahead_dis = rospy.get_param("/pure_pursuit/min_look_ahead_dis", 3)
        self.max_look_ahead_dis = rospy.get_param("/pure_pursuit/max_look_ahead_dis", 6)
        self.avg_look_ahead = (self.min_look_ahead_dis + self.max_look_ahead_dis) / 2
        self.time_out_from_input_trajectory = rospy.get_param("/pure_pursuit/time_out", 3)
        trajectory_in_topic = rospy.get_param("/trajectory_in", "/global_gps_trajectory")
        gps_path = rospy.get_param("global_gps_path_topic","/global_gps_path")
        odom_topic = rospy.get_param("/patrol/odom_topic", "vehicle/odom")
        gps_topic = rospy.get_param("/patrol/gps_topic", "/mavros/global_position/local")
        self.robot_base_frame = rospy.get_param("robot_base_frame", "ego_vehicle")
        self.wait_time_at_ends = rospy.get_param("/patrol/wait_time_on_mission_complete", 10)
        self.mission_continue = rospy.get_param("/patrol/mission_continue", False)
        self.mission_trips = rospy.get_param("/patrol/mission_trips", 0)
        self.search_point_distance = 5
        failsafe_enable = rospy.get_param("/patrol/failsafe_enable", True)
        self.allow_reversing = rospy.get_param("/patrol/allow_reversing", True)
        self.enable_cte_based_speed_control = rospy.get_param("/patrol/enable_cte_based_speed_control", False)

        if failsafe_enable:
            cmd_topic = rospy.get_param("patrol/cmd_topic", "pure_pursuit/cmd_drive")
        else:
            cmd_topic = rospy.get_param("patrol/pilot_cmd_in", "/vehicle/cmd_drive_safe")

        # Publishers
        self.ackermann_publisher = rospy.Publisher(cmd_topic, AckermannDrive, queue_size=10)
        self.target_pose_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=2)
        self.close_pose_pub = rospy.Publisher('/close_pose', PoseStamped, queue_size=2)
        self.mission_count_pub = rospy.Publisher('/mission_count', Float32, queue_size=2, latch=True)
        self.controller_diagnose_pub = rospy.Publisher("pure_pursuit_diagnose", ControllerDiagnose, queue_size=2)

        # other utility variables
        self.count_mission_repeat = 0
        self.index_old = None
        self.robot_speed = None

        self.ackermann_msg = AckermannDrive()

        rospy.Subscriber(trajectory_in_topic, Trajectory, self.trajectory_callback)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber(gps_topic, NavSatFix, self.gps_callback)
        rospy.logdebug(f"Subscriber initiated, {trajectory_in_topic} , {odom_topic}, {gps_topic}")
        time.sleep(1)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.trajectory_data and self.robot_state and self.gps_robot_state:
                rospy.loginfo("trajectory_data and robot_state and gps_robot_state are available")
                break
            else:
                rospy.logwarn("waiting for trajectory_data or robot_state or gps_robot_state")
                rate.sleep()
        self.main_loop()

    def main_loop(self):

        diagnostic_msg = ControllerDiagnose()
        diagnostic_msg.name = "Pure Pursuit Node"
        rate = rospy.Rate(20)

        prev_steering_angle = 0
        prev_speed = 0
        prev_time = time.time()

        target_pose_msg = PoseStamped()
        target_pose_msg.header.frame_id = "map"
        close_pose_msg = PoseStamped()
        close_pose_msg.header.frame_id = "map"
        log_tracking_message = "Initialized log"
        stop_on_command = False # To check whether to stop incase of any anamoly
        # main loop starts
        # robot_pose = current_robot_pose("map", self.robot_base_frame)
        while not rospy.is_shutdown():
            # TODO
            # remove this current robot pose with odom callback, it increases speeed
            # robot_pose = current_robot_pose("map", self.robot_base_frame)
            robot_pose = self.robot_state.pose.pose
            if time.time() - self.updated_odom_time > 1:
                rospy.logwarn("Time out from Odometry, no update from last %s seconds",
                              str(time.time() - self.updated_odom_time))
                diagnostic_msg = ControllerDiagnose()
                diagnostic_msg.level = diagnostic_msg.WARN
                diagnostic_msg.message = "Time out from Odometry, no update from last %s seconds" + \
                                         str(time.time() - self.updated_odom_time)
                diagnostic_msg.stamp = rospy.Time.now()
                self.controller_diagnose_pub.publish(diagnostic_msg)
                rate.sleep()
                continue
                # return 0

            # if not robot_pose:
            #     rospy.logwarn("No tf between map and %s", self.robot_base_frame)
            #     diagnostic_msg = ControllerDiagnose()
            #     diagnostic_msg.level = diagnostic_msg.WARN
            #     diagnostic_msg.message = 'Time out from tf'
            #     diagnostic_msg.stamp = rospy.Time.now()
            #     self.controller_diagnose_pub.publish(diagnostic_msg)
            #     # rate.sleep()
            #     # continue
            #     return 0

            if self.index_old is None:
                id_heading_okay, reason, close_idx, cross_track_dis = self.find_first_close_point(robot_pose)

                if id_heading_okay:
                    diagnostic_msg = ControllerDiagnose()
                    diagnostic_msg.name = "Pure Pursuit Node"
                    diagnostic_msg.level = diagnostic_msg.OK
                    diagnostic_msg.message = reason
                    self.controller_diagnose_pub.publish(diagnostic_msg)
                    self.index_old = close_idx
                    rospy.loginfo(reason)
                    self.send_ack_msg(0, 0, 0)
                    rate.sleep()
                    continue
                    # return 0
                else:
                    diagnostic_msg = ControllerDiagnose()
                    diagnostic_msg.name = "Pure Pursuit Node"
                    diagnostic_msg.level = diagnostic_msg.ERROR
                    diagnostic_msg.message = reason
                    self.controller_diagnose_pub.publish(diagnostic_msg)
                    self.index_old = None
                    rospy.logerr(reason)
                    self.send_ack_msg(0, 0, 0)
                    rate.sleep()
                    continue
                    # return 0cte

            # close_point_ind, close_dis = self.calc_nearest_ind(robot_pose)
            self.index_old, close_dis = self.find_close_point_by_distance(robot_pose, self.index_old)
            # self.cte = close_dis

            #  COMMETING FOR DEBUG
            # if close_dis > self.avg_look_ahead:
            #     # TODO
            #     # Check for cross track error limit
            #     diagnostic_msg = ControllerDiagnose()
            #     diagnostic_msg.level = diagnostic_msg.WARN
            #     diagnostic_msg.message = "Distance to close point is more than " + str(self.avg_look_ahead)
            #     rospy.logwarn("Distance to close point is more than " + str(self.avg_look_ahead))
            #     self.controller_diagnose_pub.publish(diagnostic_msg)
            #     self.send_ack_msg(0, 0, 0)
            #     self.index_old = None
            #     continue
            #     # return 0
            

            lhd = self.compute_lookahead_distance(self.robot_speed)
            
            # dist_to_direction_change = self.findDirectionChange(self.robot_state)

            
            # if dist_to_direction_change < lhd:
            #     lhd = dist_to_direction_change
                # rospy.logerr(lhd)
                # if target_pose_msg.pose.position.x >= 0.0:
                #     rospy.logwarn("Forward")
                #     self.is_reverse = False
                # else:
                #     self.is_reverse = True
                #     rospy.logerr("Reverse")
            
            target_point_ind, lhd = self.find_target_index(robot_pose, self.index_old, lhd)
            close_point_ind = self.index_old

            if target_point_ind >= self.trajectory_len - 1:
                self.count_mission_repeat += 1
                diagnostic_msg = ControllerDiagnose()
                rospy.loginfo(' Mission count %s ', self.count_mission_repeat)
                self.mission_count_pub.publish(self.count_mission_repeat)
                diagnostic_msg.level = diagnostic_msg.OK
                diagnostic_msg.message = 'Mission count  ' + str(self.count_mission_repeat)
                self.controller_diagnose_pub.publish(diagnostic_msg)
                if self.mission_continue:
                    if self.mission_trips == 0:
                        diagnostic_msg = ControllerDiagnose()
                        diagnostic_msg.level = diagnostic_msg.WARN
                        diagnostic_msg.message = 'Mission completed and restarting the plan ' + str(
                            self.count_mission_repeat) + " waiting for : " + str(self.wait_time_at_ends) + " secs"
                        self.controller_diagnose_pub.publish(diagnostic_msg)
                        # TODO
                        # publish this message for the complete duration of wait time
                        self.send_ack_msg(0, 0, 0)
                        rospy.logwarn('Mission completed and restarting the plan' + str(
                            self.count_mission_repeat))
                        rospy.loginfo("waiting for %s", str(self.wait_time_at_ends))
                        time.sleep(self.wait_time_at_ends)
                        self.index_old = 1
                        rate.sleep()
                        continue
                    elif self.count_mission_repeat <= self.mission_trips:
                        diagnostic_msg = ControllerDiagnose()
                        diagnostic_msg.level = diagnostic_msg.WARN
                        diagnostic_msg.message = 'completed mission ' + str(
                            self.count_mission_repeat) + 'and target is' + str(
                            self.mission_trips) + " waiting for : " + str(self.wait_time_at_ends) + " secs"
                        self.controller_diagnose_pub.publish(diagnostic_msg)
                        self.send_ack_msg(0, 0, 0)
                        rospy.logwarn('completed mission  %s and target is %s', self.count_mission_repeat,
                                      self.mission_trips)
                        rospy.loginfo("waiting for %s", str(self.wait_time_at_ends))
                        time.sleep(self.wait_time_at_ends)
                        self.index_old = 1
                        rate.sleep()
                        continue
                    else:
                        rospy.logwarn("mission repeats of :" + str(self.mission_trips) + " are completed")
                        diagnostic_msg = ControllerDiagnose()
                        diagnostic_msg.level = diagnostic_msg.WARN
                        diagnostic_msg.message = "mission repeats of :" + str(self.mission_trips) + " are completed"
                        self.controller_diagnose_pub.publish(diagnostic_msg)
                        self.send_ack_msg(0, 0, 0)
                        rate.sleep()
                        break
                else:
                    self.send_ack_msg(0, 0, 0)
                    rospy.logwarn('completed mission')
                    diagnostic_msg = ControllerDiagnose()
                    diagnostic_msg.level = diagnostic_msg.WARN
                    diagnostic_msg.message = "completed mission"
                    self.controller_diagnose_pub.publish(diagnostic_msg)
                    break

            # Publishing close and target poses.
            close_pose_msg.pose = self.trajectory_data.points[close_point_ind].pose
            self.close_pose_pub.publish(close_pose_msg)
            target_pose_msg.pose = self.trajectory_data.points[target_point_ind].pose
            self.target_pose_pub.publish(target_pose_msg)

            target_point_angle = angle_btw_poses(self.trajectory_data.points[target_point_ind].pose, robot_pose)
            alpha = -(target_point_angle - get_yaw(robot_pose.orientation))
           
            # robot_position = (3, 3)
            robot_position = (robot_pose.position.x,robot_pose.position.y) 
            # nearest_point = (5, 5)
            nearest_point = (close_pose_msg.pose.position.x,close_pose_msg.pose.position.y)
            # lookahead_point = (7, 6)
            lookahead_point = (target_pose_msg.pose.position.x,target_pose_msg.pose.position.y)

            cross_product = self.calculate_cross_product(robot_position, nearest_point, lookahead_point)
            if cross_product > 0:
                result = "right"
                # return "Right"
            elif cross_product < 0:
                close_dis = -close_dis
                result = "Left"
                # return "Left"  
            else:
                result = "on path"
                # return "On Path" 
            self.cte = close_dis
            rospy.logdebug(f'Robot is {result} of the path.')
            
            dot_vector = self.findLookaheadPos(robot_pose, target_pose_msg)

            if dot_vector >= 0:
                self.is_reverse = False
                rospy.loginfo_throttle(10, "Forward")
            elif dot_vector < 0:
                if self.allow_reversing: # safety check to stop on reverse conditions when allow_reversing is set to false.
                    self.is_reverse = True
                    rospy.loginfo_throttle(10, "Reverse")
                else:
                    self.is_reverse = False
                    stop_on_command = True
            # elif dot_vector == 0:
            #     rospy.logerr("Stopping the Robot")
            #     self.send_ack_msg(0, 0, 0)
            #     self.is_reverse = False
            else:
                pass

            if self.is_pp_pid:
                rospy.logdebug("running with pid")
                delta_degrees = self.pp_with_pid(lhd=lhd,alpha=alpha)
                delta_degrees = math.degrees(delta_degrees)

            else:
                rospy.logdebug("running without pid")
                delta = math.atan2(2.0 * vehicle_data.dimensions.wheel_base * math.sin(alpha), lhd)
                delta_degrees = math.degrees(delta)

            steering_angle = np.clip(delta_degrees, -30, 30)
            speed = self.trajectory_data.points[close_point_ind].longitudinal_velocity_mps
            if self.enable_cte_based_speed_control:
                adjusted_speed = self.adjust_speed_based_on_cte(speed, abs(self.cte),self.kp_speed)
            else:
                adjusted_speed = speed
            if stop_on_command:
                speed = 0
                log_tracking_message = "allow_reversing flag not set. Stopping the Robot"
                rospy.logerr_throttle(10, "allow_reversing flag not set. Stopping the Robot")
                self.send_ack_msg(steering_angle, speed, 1)
            elif self.is_reverse:
                if adjusted_speed < self.min_backward_speed and adjusted_speed > 0:
                    adjusted_speed = self.min_backward_speed
                speed = -min(adjusted_speed, self.max_backward_speed) # To avoid max speed from path_publisher.
                log_tracking_message = f'Tracking path in Reverse with speed {speed}'
                self.send_ack_msg(steering_angle, speed, 0)
            else:
                if adjusted_speed < self.min_forward_speed and adjusted_speed > 0:
                    speed = self.min_forward_speed
                else:
                    speed = adjusted_speed
                log_tracking_message = f'Tracking path in Forward with speed {speed}'
                # stop vehicle is speed is negative when is_reverse is false.(just a safety check)
                if speed <= 0:
                    self.send_ack_msg(steering_angle, speed, 1)
                else:
                    self.send_ack_msg(steering_angle, speed, 0)
            rospy.loginfo("steering angle: %s, speed: %s, break: %s", str(steering_angle), str(speed), str(0))
            rospy.loginfo('lhd: %s, alpha: %s , robot_speed: %s ', str(lhd), str(alpha), str(self.robot_speed))
            
            

            # fill the control diagnose topic
            diagnostic_msg = ControllerDiagnose()
            diagnostic_msg.name = "Pure Pursuit Node"
            diagnostic_msg.level = diagnostic_msg.OK
            diagnostic_msg.message = log_tracking_message # "Tracking path"
            diagnostic_msg.stamp = rospy.Time.now()
            diagnostic_msg.look_ahead = lhd
            diagnostic_msg.cte = close_dis
            diagnostic_msg.longitudinal_velocity_mps = speed
            diagnostic_msg.steering_angle = steering_angle
            diagnostic_msg.lateral_velocity_dps = steering_angle - prev_steering_angle / time.time() - prev_time
            diagnostic_msg.acceleration_mps2 = speed - prev_speed / time.time() - prev_time
            diagnostic_msg.target_pose = target_pose_msg.pose
            diagnostic_msg.target_gps_pose = self.trajectory_data.points[target_point_ind].gps_pose
            diagnostic_msg.vehicle_pose = robot_pose
            diagnostic_msg.vehicle_gps_pose = self.gps_robot_state
            k1 = FloatKeyValue("close_index", close_point_ind)
            k2 = FloatKeyValue("target_index", target_point_ind)
            path_percent = (self.trajectory_data.points[close_point_ind].accumulated_distance_m /
                            self.trajectory_data.points[-1].accumulated_distance_m) * 100
            k3 = FloatKeyValue("path_completed_percentage", round(path_percent, 3))
            diagnostic_msg.values.extend([k1, k2, k3])
            # diagnostic_msg.values["close_index"] = 10
            self.controller_diagnose_pub.publish(diagnostic_msg)
            prev_steering_angle = steering_angle
            prev_time = time.time()
            prev_speed = speed
            # removing this sleep increasing the speed
            diagnostic_msg = None
            rate.sleep()
            # return 0

    def pp_with_pid(self,lhd,alpha):
        '''
        trying different formula for appying 
        pid on the pure pursuit controller
        '''
        #0.6,0.1,0.3 //0.65,0.25 //
        # kp = rospy.get_param("kp_pid",0.01)
        # ki = rospy.get_param("ki_pid",0.01)
        if self.rqt:
            kp = self.kp_pid
            ki = self.ki_pid
        else:
            kp = rospy.get_param("/pure_pursuit/kp_pid",0.01)
            ki = rospy.get_param("/pure_pursuit/ki_pid",0.01)
        kd = 0.5
        kpp = 1 - kp - ki
        delta = math.atan2(2.0 * vehicle_data.dimensions.wheel_base * math.sin(alpha), lhd)
        delp = kp*(self.cte + (vehicle_data.dimensions.wheel_base+lhd)*math.sin(alpha))
        deli = ki*(self.sum_cte + self.cte)
        delpp = kpp*delta
        self.e2 = self.e1
        deld = self.e2-self.e1
        self.e1 = self.cte
        strAngle = delp + deli + delpp
        
        return strAngle

    def find_first_close_point(self, robot_pose):
        index_list = []
        for ind in range(self.trajectory_len):
            dis = distance_btw_poses(robot_pose, self.trajectory_data.points[ind].pose)
            if dis < self.avg_look_ahead:
                heading_ok = heading_check(robot_pose.orientation, self.trajectory_data.points[ind].pose.orientation)
                # print("heading_ok", heading_ok)
                # exit()
                if heading_ok:
                    return heading_ok, "Heading is okay", ind, dis
                else:
                    index_list.append(ind)
        if len(index_list) == 0:
            return False, "No close point found", 0, 0
        else:
            if self.allow_reversing:
                return True, "Found " + str(
                    len(index_list)) + "points are close, But No Heading is not okay for them", 0, 0
            return False, "Found " + str(
                len(index_list)) + "points are close, But No Heading is not okay for them", 0, 0

    # def find_close_point(self, robot_pose, old_close_index):
    #     close_dis = distance_btw_poses(robot_pose, self.trajectory_data.points[old_close_index].pose)
    #     for ind in range(old_close_index + 1, self.trajectory_len):
    #         dis = distance_btw_poses(robot_pose, self.trajectory_data.points[ind].pose)
    #         if close_dis >= dis:
    #             close_dis = dis
    #         else:
    #             return ind - 1, close_dis
    #     return self.trajectory_len, 0

    def find_close_point_by_distance(self, robot_pose, old_close_index):
        distance_list = []
        acc_dis = 0
        if old_close_index >= self.trajectory_len - 1:
            dis = distance_btw_poses(robot_pose, self.trajectory_data.points[self.trajectory_len - 1].pose)
            return self.trajectory_len - 1, dis
        for ind in range(old_close_index, self.trajectory_len-1):
            acc_dis += distance_btw_poses(self.trajectory_data.points[ind].pose,
                                          self.trajectory_data.points[ind + 1].pose)
            if acc_dis < self.search_point_distance:
                dis = distance_btw_poses(robot_pose, self.trajectory_data.points[ind].pose)
                distance_list.append(dis)
            else:
                break
        if len(distance_list) > 0:
            i = np.argmin(distance_list)
            dis = distance_list[i]
            return old_close_index + i, dis
        else:
            return self.trajectory_len - 1, 0

    def find_target_index(self, robot_pose, close_point_ind, lhd):
        """
        search index of target point in the reference path. The following implementation was inspired from
        http://dyros.snu.ac.kr/wp-content/uploads/2021/02/Ahn2021_Article_AccuratePathTrackingByAdjustin-1.pdf
        Args:
            robot_pose:  pose of robot
            close_point_ind : index of close point to the vehicle
            lhd : look ahead distance
        Returns:
            target_index, distance
        """

        close_dis = self.trajectory_data.points[close_point_ind].accumulated_distance_m
        for ind in range(close_point_ind, self.trajectory_len):
            path_acc_distance = self.trajectory_data.points[ind].accumulated_distance_m - close_dis
            if path_acc_distance > lhd:
                return ind, distance_btw_poses(robot_pose, self.trajectory_data.points[ind].pose)
        return ind, distance_btw_poses(robot_pose, self.trajectory_data.points[ind].pose)

    def calc_nearest_ind(self, robot_pose):
        """
        calc index of the nearest point to current position
        Args:
            robot_pose: pose of robot
        Returns:
            close_index , distance
        """
        distance_list = [distance_btw_poses(robot_pose, point.pose) for point in self.trajectory_data.points]
        ind = np.argmin(distance_list)
        dis = distance_list[ind]
        return ind, dis

    def trajectory_callback(self, data):
        self.trajectory_data = data
        self.trajectory_len = len(self.trajectory_data.points)
        self.updated_traj_time = time.time()

    def odom_callback(self, data):
        rospy.loginfo_once("Odom data received")
        self.robot_state = data
        self.robot_speed = math.sqrt(data.twist.twist.linear.x ** 2 + data.twist.twist.linear.y ** 2)
        self.updated_odom_time = time.time()

    def gps_callback(self, data):
        self.gps_robot_state = data
        
    def gps_path_cb(self,data):
        self.global_plan = data

    def findDirectionChange(self,robot_pose):
        for pose_id in range(1,len(self.global_plan.poses)):
            oa_x = self.global_plan.poses[pose_id].pose.position.x - self.global_plan.poses[pose_id - 1].pose.position.x
            oa_y = self.global_plan.poses[pose_id].pose.position.y - self.global_plan.poses[pose_id - 1].pose.position.y
            ab_x = self.global_plan.poses[pose_id + 1].pose.position.x - self.global_plan.poses[pose_id].pose.position.x
            ab_y = self.global_plan.poses[pose_id + 1].pose.position.y - self.global_plan.poses[pose_id].pose.position.y
            # rospy.logwarn(f'{oa_x},{oa_y},{ab_x},{ab_y},{pose_id}')
            # rospy.logwarn("DOT PRODUCT: {}".format((oa_x * ab_x) + (oa_y * ab_y)))
            
            if ((oa_x * ab_x) + (oa_y * ab_y) < 0.0):
                x = self.global_plan.poses[pose_id].pose.position.x - robot_pose.pose.pose.position.x
                y = self.global_plan.poses[pose_id].pose.position.y - robot_pose.pose.pose.position.y
                rospy.logwarn(math.hypot(x,y))
                return math.hypot(x,y)
        rospy.logerr(float('inf'))
        return float('inf')
    
    def findLookaheadPos(self,robot_pose,lookAheadPose,reverse_threshold=0.1,stop_threshold=0.1):
        # robot_pose = robot_pose.pose.pose
        theta = get_yaw(robot_pose.orientation)
        rx = robot_pose.position.x
        ry = robot_pose.position.y
        lx = lookAheadPose.pose.position.x
        ly = lookAheadPose.pose.position.y
        ruv_x,ruv_y = math.cos(theta), math.sin(theta)
        # print(ruv_x,ruv_y)
        # robot, lookahead x, robot, lookahead y
        # rl_x, rl_y =  = x2-x1, y2-y1
        rl_x = lx-rx
        rl_y = ly-ry
        a = [ruv_x,ruv_y]
        b = [rl_x,rl_y]
        # self.ruvDotvector = np.dot(a,b)
        dot_product = np.dot(a,b)
        
        # Check conditions for reversing or stopping
        # distance = math.sqrt((lx-rx)**2 + (ly-ry)**2)
        # if dot_product < -reverse_threshold:
        #     # Lookahead point is behind robot and threshold is exceeded - reverse
        #     return -1
        # elif abs(dot_product) < stop_threshold and distance < stop_threshold:
        #     # Robot has reached lookahead point - stop
        #     return 0
        # else:
        #     # Robot can continue moving forward
        #     return 1
        return dot_product
    
    def calculate_cross_product(self,robot_position, nearest_point, lookahead_point):
        # Calculate vectors P and Q
        vector_P = np.array([lookahead_point[0] - robot_position[0], lookahead_point[1] - robot_position[1]])
        vector_Q = np.array([nearest_point[0] - robot_position[0], nearest_point[1] - robot_position[1]])

        # Calculate the cross product (P x Q)
        cross_product = np.cross(vector_P, vector_Q)
        rospy.logdebug(cross_product)
        
        return cross_product
        # if cross_product > 0.3:
        #     return "Right"  
        # elif cross_product < 0.3:
        #     return "Left"  
        # else:
        #     return "On Path" 
    def compute_lookahead_distance(self, robot_speed):
        # return 3
        # return self.min_look_ahead_dis
        # https://github.com/bosonrobotics/autopilot_boson/issues/22
        
        if self.is_reverse:
            self.max_speed = self.max_backward_speed
        else:
            self.max_speed = self.max_forward_speed
        if robot_speed > self.max_speed:
            lhd = (robot_speed * self.min_look_ahead_dis) / self.max_speed
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
    
    def pid_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {kp_pid}, {ki_pid}""".format(**config))
        self.kp_pid = config['kp_pid']
        self.ki_pid = config['ki_pid']
        rospy.loginfo(f'Kp and Ki elements are {self.kp_pid},{self.ki_pid}')
        return config
    
    def adjust_speed_based_on_cte(self, speed, cte, kp_speed):

        # speed : pure pursuit calculated speed
        # CTE : cross track error (positive)
        # kp_speed : speed adjustment factor or speed gain.
        
        if speed == 0 or abs(cte) < self.allowable_cte_for_adjustable_speed:
            return speed
        adjusted_speed = speed - kp_speed * cte
        
        return adjusted_speed

if __name__ == "__main__":
    rospy.init_node('Pure_pursuit_controller_node')
    pure_pursuit = PurePursuitController()
    rospy.spin()

