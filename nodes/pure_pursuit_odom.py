#!/usr/bin/env python
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
    import numpy as np
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
except Exception as e:
    sys.exit(e)

def ConvertTo180Range( deg):
    deg = ConvertTo360Range(deg)
    if deg > 180.0:
        deg = -(180.0 - (deg%180.0))

    return deg

def ConvertTo360Range(deg):

	# if deg < 0.0:
	deg = deg%360.0

	return deg
def find_smallest_diff_ang(goal, cur):

	## goal is in 180ranges, we need to convert to 360ranges first

	diff_ang1 = abs(ConvertTo360Range(goal) - cur)

	if diff_ang1 > 180.0:

		diff_ang = 180.0 - (diff_ang1%180.0)
	else:
		diff_ang = diff_ang1

	## check closet direction
	compare1 = ConvertTo360Range(ConvertTo360Range(goal) - ConvertTo360Range(cur + diff_ang))
	compare2 = ConvertTo180Range(goal - ConvertTo180Range(cur + diff_ang))
	# print(compare1, compare2)
	if (abs(compare1) < 0.5) or (compare1 == 360.0) or (abs(compare2) < 0.5) or (compare2 == 360.0):
		sign = 1.0 # clockwise count from current hdg to target
	else:
		sign = -1.0 # counter-clockwise count from current hdg to target

	return diff_ang, sign

class PurePursuit:
    def __init__(self):
        
        od_enable = rospy.get_param('patrol/od_enable',False)
        if od_enable:
            ack_pub_topic = '/vehicle/cmd_drive_nosafe'
        else:
            ack_pub_topic = '/vehicle/cmd_drive_safe'

        self.ack_pub_safe = rospy.Publisher(ack_pub_topic,AckermannDrive, queue_size=10)
        self.vehicle_pose_pub = rospy.Publisher('/vehicle/pose',PoseStamped, queue_size=10)
        self.vehicle_tarpose_pub = rospy.Publisher('/vehicle/target_pose',PoseStamped, queue_size=10)


        # rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        # rospy.Subscriber('/mavros/imu/mag', MagneticField, self.heading_callback)
        rospy.Subscriber('/mavros/rc/in', RCIn, self.rc_callback)
        self.rc_stop = False
        self.vehicle_pose_msg = PoseStamped()
        self.target_point_msg = PoseStamped()
        self.ack_msg = AckermannDrive()
        self.vehicle_pose_msg.header.frame_id = 'map'
        self.target_point_msg.header.frame_id = 'map'
        self.path = []
        self.robot={}
        self.robot['x'] = 0.0
        self.robot['y'] = 0.0
        self.robot['yaw'] = 0.0
        self.robot['vel'] = 0.0
        

        self.ind_end = 0
        self.index_old = None

        # IMP PARAMS
        self.throttle_speed = rospy.get_param('/patrol/max_speed',1.5)
        self.wheel_base     = rospy.get_param('/patrol/wheel_base',2)
        self.look_ahead_dis = rospy.get_param('/patrol/look_ahead_distance',3)
        self.speed_constant = rospy.get_param('/patrol/speed_gain',0.1)

        data1, data2, data3 = None, None, None
        while not rospy.is_shutdown():
            try:
                data1 = rospy.wait_for_message("/gps_path", Path, timeout=2)
            except:
                data1 = None
                rospy.loginfo("waiting for /gps_path")
            try:
                data2 = rospy.wait_for_message("/mavros/local_position/odom", Odometry, timeout=2)
            except:
                data2 = None
                rospy.loginfo("Waiting for /mavros/local_position/odom")
            
            if data1 and data2:
                rospy.Subscriber('/gps_path', Path, self.path_callback)
                rospy.Subscriber('/mavros/local_position/odom', Odometry,self.odom_callback)
                break

        self.main_loop()
    def rc_callback(self, data):
        if data.channels[4]== 1000 or data.channels[4]== 1001:
            self.rc_stop = False
        else:
            self.rc_stop = True
        


    def path_callback(self, data):
    
        for pose in  data.poses:
            _,_,heading = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            heading = math.degrees(heading)
            self.path.append([pose.pose.position.x,pose.pose.position.y,heading])
        rospy.loginfo('path received')
        self.ind_end = len(self.path) - 1
        
    def odom_callback(self, data):
        
        _,_,self.pose_heading = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        self.robot['yaw'] = self.pose_heading
        self.robot['x'] = data.pose.pose.position.x - self.wheel_base * np.cos(self.pose_heading)
        self.robot['y'] = data.pose.pose.position.y - self.wheel_base * np.sin(self.pose_heading)
        self.robot['vel'] = data.twist.twist.linear.x 

        


        # fx = x + self.wheelbase * np.cos(yaw)
        # fy = y + self.wheelbase * np.sin(yaw)

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
        Lf = self.look_ahead_dis# + self.speed_constant * self.robot['vel']
        
        if self.calc_distance(robot, self.index_old) > Lf:

            pass

        for ind in range(self.index_old, self.ind_end + 1):
            dis = self.calc_distance(robot, ind) 
            if dis > Lf :
                self.index_old = ind
                return ind, Lf,dis

        self.index_old = self.ind_end
        dis = self.calc_distance(robot, self.ind_end) 
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

    def calc_nearest_ind(self, robot):
        """
        calc index of the nearest point to current position
        :param node: current information
        :return: index of nearest point
        """
    
        # print(robot)
        distance_list =  [self.calc_distance(robot,ind) for ind in range(len(self.path)) ]
        # for i in range(10):
        #     print(distance_list[i])
        # distance_list =  [print(ind) for ind in range(len(self.path)) ]

        ind = np.argmin(distance_list)
        # print(ind)
        self.index_old = ind
        # print('dis',distance_list[ind])
        # print('close point',self.path[ind])
        dis = distance_list[ind]
        return ind,dis


    def main_loop(self):
        r = rospy.Rate(1)
        while not len(self.path) > 1 and not rospy.is_shutdown():
            rospy.loginfo('No path received')
            r.sleep()
        rospy.loginfo('path received')
        rospy.loginfo('Pure pursit is started')
        r = rospy.Rate(5)
        while not rospy.is_shutdown(): 
            ## distance fun test
            # ind,dis = self.calc_nearest_ind(self.robot)
            # print('point ',self.path[ind])
            # print('dis', dis)
            self.vehicle_pose_msg.pose.position.x  =  self.robot['x']
            self.vehicle_pose_msg.pose.position.y = self.robot['y']
            self.vehicle_pose_msg.pose.orientation = Quaternion(*quaternion_from_euler(0,0, self.robot['yaw']))

            self.vehicle_pose_pub.publish(self.vehicle_pose_msg)
            # find target point  test
            ind, Lf,dis = self.target_index(self.robot)

            if ind ==self.ind_end:
                rospy.loginfo('MISSION COMPLETED ')
                self.send_ack_msg(0, 0, 0)
                break
            print('robot', self.robot)
            print('point ',self.path[ind])  
            self.target_point_msg.pose.position.x  =self.path[ind][0]
            self.target_point_msg.pose.position.y = self.path[ind][1]
            
            self.vehicle_tarpose_pub.publish(self.target_point_msg)




            tx = self.path[ind][0]
            ty = self.path[ind][1]
            # # heading_diff = self.robot['yaw'] - self.path[ind][2]
            # heading_diff ,sign= find_smallest_diff_ang(self.path[ind][2],self.robot['yaw'])

            # # delta_x =  self.robot['x'] - self.path[ind][0] 
            # # delta_y =  self.robot['y'] - self.path[ind][1] 
            delta_x =    self.path[ind][0] - self.robot['x'] 
            delta_y =  self.path[ind][1] - self.robot['y']
            slope = atan2(delta_y, delta_x)
            alpha  = atan2(delta_y, delta_x)-  self.robot['yaw']
            delta = math.atan2(2.0 * self.wheel_base  * math.sin(alpha), Lf)
            delta_degrees = -1 *math.degrees(delta)
            if abs(delta_degrees) > 30 :
                steering_angle = 30 *np.sign(delta_degrees)
            else:
                steering_angle  = delta_degrees

            self.send_ack_msg(steering_angle, self.throttle_speed, 0)
            
            print('robot heading:',math.degrees(self.robot['yaw'])%360)
            print('Slope        :',math.degrees(slope)%360)
            print('Steering angl:', steering_angle)
            r.sleep()
            
            
    def send_ack_msg(self,steering_angle, speed, jerk):
        if abs(steering_angle)> 30:
            steering_angle = 30*abs(steering_angle)//steering_angle
        self.ack_msg.steering_angle =  steering_angle
        self.ack_msg.speed = speed
        self.ack_msg.jerk = jerk
        # print(self.ack_msg)
        if self.rc_stop:
            print("NOT sending commads RC stoped")
            self.ack_msg.steering_angle=0.0
            self.ack_msg.speed = 0.0
            self.ack_msg.jerk = 1
            self.ack_pub_safe.publish( self.ack_msg)
        else:
            self.ack_pub_safe.publish( self.ack_msg)
            

if __name__ =="__main__":
    rospy.init_node('pure_pursuit1')

    pure_parsuit = PurePursuit()

    rospy.spin()

    
        
        

