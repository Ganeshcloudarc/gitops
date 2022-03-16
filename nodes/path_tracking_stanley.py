#!/usr/bin/env python3
try:
    import rospy
    import rospkg
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import PoseStamped,Quaternion
    from sensor_msgs.msg import NavSatFix,MagneticField,Imu
    from ackermann_msgs.msg import AckermannDrive
    from mavros_msgs.msg import RCIn
    import json
    import math
    from geonav_conversions import *
    import sys
    import time
    from numpy import clip,argmin,pi,arctan2
    import numpy as np
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
except Exception as e:
    exit(e)


class StanlyControl:
    def __init__(self):
        
        od_enable = rospy.get_param('patrol/od_enable', False)
        if od_enable:
            ack_pub_topic = '/cmd_drive/pure_pursuit'
        else:
            ack_pub_topic = '/cmd_drive/pure_pursuit'

        self.ack_pub_safe = rospy.Publisher(ack_pub_topic, AckermannDrive, queue_size=10)
        self.vehicle_pose_pub = rospy.Publisher('vehicle_pose', PoseStamped, queue_size=10)
        self.target_pose_pub = rospy.Publisher('target_pose', PoseStamped, queue_size=10)


        # rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        # rospy.Subscriber('/mavros/imu/mag', MagneticField, self.heading_callback)
        rospy.Subscriber('/mavros/rc/in', RCIn, self.rc_callback)
        self.rc_stop = False
        self.vehicle_pose_msg = PoseStamped()
        self.target_pose_msg = PoseStamped()
        self.ack_msg = AckermannDrive()
        self.vehicle_pose_msg.header.frame_id = 'map'
        self.target_pose_msg.header.frame_id = 'map'
        self.path = []
        self.robot={}
        self.robot['x'] = 0.0
        self.robot['y'] = 0.0
        self.robot['yaw'] = 0.0
        self.robot['vel'] = 0.0
        

        self.ind_end = 0
        self.index_old = None

        # IMP PARAMS
        self.throttle_speed = rospy.get_param('/patrol/speed',1.5)
        self.wheel_base     = rospy.get_param('/patrol/wheel_base',2.9)
        self.look_ahead_dis = rospy.get_param('/patrol/look_ahead_distance',4)
        self.control_gain_k = 8  # control gain
        self.max_steer = 30.0  # [rad] max steering angle


        data1, data2, data3 = None, None, None
        while not rospy.is_shutdown():
            try:
                data1 = rospy.wait_for_message("/odom_path", Path, timeout=2)
            except:
                data1 = None
                rospy.loginfo("waiting for /odom_path")
            try:
                data2 = rospy.wait_for_message("/carla/ego_vehicle/odometry", Odometry, timeout=2)
            except:
                data2 = None
                rospy.loginfo("Waiting for /carla/ego_vehicle/odometry")
            
            if data1 and data2:
                rospy.Subscriber('/odom_path', Path, self.path_callback)
                rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odom_callback)
                break

        self.main_loop()
    def rc_callback(self, data):
        if data.channels[4]== 1000 or data.channels[4]== 1001:
            self.rc_stop = False
        else:
            self.rc_stop = True
        


    def path_callback(self, data):
        self.revived_path = data.poses
    
        for pose in  data.poses:
            _,_,heading = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            # heading = math.degrees(heading)
            self.path.append([pose.pose.position.x,pose.pose.position.y,heading])
        rospy.loginfo('path received')
        self.ind_end = len(self.path) - 1
        
    def odom_callback(self, data):
        self.robot['x'] = data.pose.pose.position.x
        self.robot['y'] = data.pose.pose.position.y
        _, _, self.pose_heading = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        self.robot['yaw'] = self.pose_heading
        self.robot['vel'] = data.twist.twist.linear.x

    def target_index(self, robot):
        """
        search index of target point in the reference path.
        the distance between target point and current position is ld
        :param node: current information
        :return: index of target point
        """

        if self.index_old is None:
            self.calc_nearest_ind(robot)

        # Lf = C.kf * node.v + C.Ld
        # Look ahead distance
        Lf = self.look_ahead_dis
        

        for ind in range(self.index_old, self.ind_end + 1):
            dis = self.calc_distance(robot, ind) 
            if dis > Lf :
                self.index_old = ind
                return ind, Lf,dis

        self.index_old = self.ind_end
        dis = self.calc_distance(robot, elf.ind_end) 
        return self.index_old, Lf,dis
    
    def calc_distance(self, robot, ind):
        # print(robot)
        return math.hypot(robot['x'] - self.path[ind][0], robot['y'] - self.path[ind][1])

    def calc_nearest_dis(self, robot):
        """
        calc index of the nearest point to current position
        :param node: current information
        :return: index of nearest point
        """
    
        distance_list =  [self.calc_distance(robot,ind) for ind in range(len(self.path)) ]
       
        ind = np.argmin(distance_list)
        # print(ind)
        # self.index_old = ind
        # print('dis',distance_list[ind])
        # print('close point',self.path[ind])
        dis = distance_list[ind]
        return ind,dis

    def normalize_angle(self,angle):
        """
        Normalize an angle to [-pi, pi].

        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > pi:
            angle -= 2.0 * pi

        while angle < -pi:
            angle += 2.0 * pi

        return angle

    def calc_nearest_ind(self, robot, last_target_idx):

        distance_list =  [self.calc_distance(robot,ind) for ind in range(len(self.path)) ]
        current_target_idx = argmin(distance_list)

        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        error_front_axle = distance_list[current_target_idx]
        ## TODO
        # Calc front axle position
        # fx = state.x + L * np.cos(state.yaw)
        # fy = state.y + L * np.sin(state.yaw)
        return current_target_idx, error_front_axle

    def stanley_control(self, target_idx, error_front_axle):
        # theta_e corrects the heading error
        print("target_idx",target_idx)
        # a = arctan2((self.path[target_idx][1]-self.path[target_idx-1][1]),(self.path[target_idx][0]-self.path[target_idx-1][0]))
        # print('path angle',math.degrees(a))
        # print('path heading',math.degrees(self.path[target_idx][2]))
        theta_e = self.normalize_angle(self.path[target_idx][2] - self.robot['yaw'])
        # theta_e = self.normalize_angle(a - self.robot['yaw'])

        print("heading error: ", math.degrees(theta_e))
        # theta_d corrects the cross track error
        ks = 1
        theta_d = arctan2(self.control_gain_k * error_front_axle, ks + self.robot['vel'])
        print("ctc componet: ", math.degrees(theta_d ))
        
        # Steering control
        delta = theta_e + theta_d
        print("delta", delta)

        return delta

    def find_close_point(self, robot, index_old):

        n = min(100, len(range(index_old, self.ind_end)))
        distance_list = [self.calc_distance(robot, ind) for ind in range(index_old, index_old + n)]
        ind = np.argmin(distance_list)
        print(" ind", ind)
        final = ind + index_old
        print("final ", final)
        dis = distance_list[ind]
        return final, dis

    def main_loop(self):
        r = rospy.Rate(1)
        while not len(self.path) > 1 and not rospy.is_shutdown():
            rospy.loginfo('No path received')
            r.sleep()
        rospy.loginfo('path received')
        rospy.loginfo('Stanley started is started')
        r = rospy.Rate(5)
        close_idx, _ = self.calc_nearest_ind(self.robot, 0)
        while not rospy.is_shutdown():
            close_idx, cte = self.find_close_point(self.robot, close_idx)
            print('close_idx, cte')
            print(close_idx, cte)

            front_axle_vector = [np.sin(self.robot['yaw']), -np.cos(self.robot['yaw'])]
            nearest_path_vector = [self.path[close_idx][0], self.path[close_idx][1]]
            crosstrack_error = np.sign(np.dot(nearest_path_vector, front_axle_vector)) * cte
            print("signed ctc", crosstrack_error)
            self.target_pose_msg.pose.position.x = self.path[close_idx][0]
            self.target_pose_msg.pose.position.y = self.path[close_idx][1]
            self.target_pose_msg.pose.orientation = self.revived_path[close_idx].pose.orientation
            self.target_pose_pub.publish(self.target_pose_msg)

            if close_idx+1 >= self.ind_end:
                rospy.loginfo('MISSION COMPLETED ')
                self.send_ack_msg(0, 0, 0)
                break

            delta = self.stanley_control(close_idx, cte)
            print("raw steering: ",math.degrees(delta))

            steering_angle = clip(math.degrees(delta), -self.max_steer, self.max_steer)
            print('steering_angle: ', steering_angle)
            self.send_ack_msg(steering_angle, self.throttle_speed, 0)

            r.sleep()


            
            
    def send_ack_msg(self,steering_angle, speed, jerk):
        if abs(steering_angle)> 30:
            steering_angle = 30*abs(steering_angle)//steering_angle
        self.ack_msg.steering_angle =  steering_angle
        self.ack_msg.speed = speed
        self.ack_msg.jerk = jerk
        # print(self.ack_msg)
        # if self.rc_stop:
        #     print("NOT sending commads RC stoped")
        #     self.ack_msg.steering_angle=0.0
        #     self.ack_msg.speed = 0.0
        #     self.ack_msg.jerk = 1
        #     self.ack_pub_safe.publish( self.ack_msg)
        # else:
        self.ack_pub_safe.publish( self.ack_msg)


if __name__ =="__main__":
    rospy.init_node('stanly_control')

    pure_parsuit = StanlyControl()

    rospy.spin()

    
        
        

