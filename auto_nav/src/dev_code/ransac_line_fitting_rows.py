#!/usr/bin/env python3

'''line fitting for bounding box markersarray'''
import rospy
from sklearn import linear_model
from sklearn.linear_model import LogisticRegression
import numpy as np
import numpy as np
import rospy
import math
from visualization_msgs.msg import MarkerArray, Marker
from jsk_recognition_msgs.msg import BoundingBoxArray
from geometry_msgs.msg import Point,PoseStamped
#path message
from nav_msgs.msg import Path,Odometry
import tf2_ros
import tf2_geometry_msgs  
from tf_helper_frame import *
from scipy.signal import medfilt

from autopilot_msgs.msg import Trajectory, TrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Bool



def distance_btw_poses(pose1, pose2):
    """
    Calculates distance between poses.
        Parameters:
            pose1(geometry_msgs/Pose.msg): pose one.
            pose2(geometry_msgs/Pose.msg): pose two.
        Returns:
            distance(float): distance between the poses
    """
    distance = math.hypot(pose1.position.x - pose2.position.x, pose1.position.y - pose2.position.y)
    return distance

# def ConvertTo180Range( deg):
#     deg = ConvertTo360Range(deg)
#     if deg > 180.0:
#         deg = -(180.0 - (deg%180.0))

#     return deg
# def ConvertTo360Range(deg):

# 	# if deg < 0.0:
# 	deg = deg%360.0

# 	return deg
# def find_smallest_diff_ang(goal, cur):

# 	## goal is in 180ranges, we need to convert to 360ranges first

# 	diff_ang1 = abs(ConvertTo360Range(goal) - cur)

# 	if diff_ang1 > 180.0:

# 		diff_ang = 180.0 - (diff_ang1%180.0)
# 	else:
# 		diff_ang = diff_ang1

# 	## check closet direction
# 	compare1 = ConvertTo360Range(ConvertTo360Range(goal) - ConvertTo360Range(cur + diff_ang))
# 	compare2 = ConvertTo180Range(goal - ConvertTo180Range(cur + diff_ang))
# 	# print(compare1, compare2)
# 	if (abs(compare1) < 0.5) or (compare1 == 360.0) or (abs(compare2) < 0.5) or (compare2 == 360.0):
# 		sign = 1.0 # clockwise count from current hdg to target
# 	else:
# 		sign = -1.0 # counter-clockwise count from current hdg to target

# 	return diff_ang, sign


class BoundingBox_linefitting:
    def __init__(self):
        self.bounding_box_sub = rospy.Subscriber("/bounding_boxes", BoundingBoxArray, self.bounding_box_callback)
        self.curve_sub = rospy.Subscriber("/global_gps_path/is_curve", Bool , self.curve_callback)
        # self.path_sub = rospy.Subscriber("/global_gps_path", Path, self.path_callback)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)
        self.trajectory_pub = rospy.Publisher("/local_gps_trajectory", Trajectory, queue_size=1)


        # self.marker_pub = rospy.Publisher("/markers", MarkerArray, queue_size=1)
        # self.odom_sub = rospy.Subscriber("/vehicle/odom", Odometry, self.odom_callback)

        self.path=[]
        self.odom=None
        self.heading_odom = None
        # self.tfBuffer = tf2_ros.Buffer()
        self.curve = False

        # self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)
        

    # def path_callback(self, data):
    #     for pose in  data.poses:
    #         _,_,heading = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
    #         heading = math.degrees(heading)
    #         self.path.append([pose.pose.position.x,pose.pose.position.y,heading])
    #     rospy.loginfo('path received')
    #     self.ind_end = len(self.path) - 1

    # #find the look ahead point of 10m in the self.path from robot position
    # def find_look_ahead_point(self):
    #     if self.odom is None:
    #         return None
    #     #find the robot position in the path
    #     robot_position = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]
    #     robot_position = np.array(robot_position)
    #     path = np.array(self.path)
    #     path = path[:,0:2]
    #     #find the distance between the robot position and the path
    #     distance = np.linalg.norm(path - robot_position, axis=1)
    #     #find the index of the minimum distance
    #     index = np.argmin(distance)
    #     #find the look ahead point
    #     look_ahead_point = self.path[index]
    #     #find the distance between the robot position and the look ahead point
    #     distance = np.linalg.norm(np.array(look_ahead_point[0:2]) - robot_position)
    #     #find the look ahead point in 10m
    #     while distance < 10:
    #         index += 1
    #         look_ahead_point = self.path[index]
    #         distance = np.linalg.norm(np.array(look_ahead_point[0:2]) - robot_position)
    #     return look_ahead_point
    



    # def odom_callback(self, odom_msg):
    #     self.odom = odom_msg
    #     _,_,self.heading_odom=euler_from_quaternion([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])
    #     steering_angle = self.find_steering_angle(self.find_look_ahead_point())
    #     diff_ang1 = abs(ConvertTo360Range(steering_angle) - self.heading_odom)

    #     if diff_ang1 > 180.0:

    #         diff_ang = 180.0 - (diff_ang1%180.0)
    #     else:
    #         diff_ang = diff_ang1

    #     ## check closet direction
    #     compare1 = ConvertTo360Range(ConvertTo360Range(steering_angle) - ConvertTo360Range(self.heading_odom + diff_ang))
    #     compare2 = ConvertTo180Range(steering_angle - ConvertTo180Range(self.heading_odom + diff_ang))
    #     # print(compare1, compare2)
    #     if (abs(compare1) < 0.5) or (compare1 == 360.0) or (abs(compare2) < 0.5) or (compare2 == 360.0):
    #         sign = 1.0 # clockwise count from current hdg to target
    #     else:
    #         sign = -1.0 # counter-clockwise count from current hdg to target
        
    #     print(diff_ang, sign)


        


    # #find steering angle from the look ahead point and self.heading_odom
    # def find_steering_angle(self, look_ahead_point):
    #     #find the angle between the look ahead point and the robot position
    #     angle = math.atan2(look_ahead_point[1] - self.odom.pose.pose.position.y, look_ahead_point[0] - self.odom.pose.pose.position.x)
    #     #find the difference between the look ahead point angle and the robot heading
    #     diff_ang, sign = find_smallest_diff_ang(math.degrees(angle), math.degrees(self.heading_odom))
    #     #find the steering angle
    #     steering_angle = sign * diff_ang
    #     return steering_angle

    def curve_callback(self, data):
        self.curve = data.data
        rospy.loginfo('curve received')
        if self.curve == 1:
            rospy.loginfo('curve detected')
    def bounding_box_callback(self, bounding_box_msg):
        #convert bounding box to numpy array
        bounding_box = bounding_box_msg.boxes
        #change data from map frame to rslidar frame

        bounding_box = np.array(list(bounding_box))
        #find the center of each bounding box
        bounding_box_centers = []
        for box in bounding_box:
            bounding_box_centers.append([box.pose.position.x, box.pose.position.y, box.pose.position.z])
        #find left and right right side of the rows
        left_side = []
        right_side = []
        for bounding_box_center in bounding_box_centers:
            #if the y value of the center of the bounding box find the closest point to the robot
            if bounding_box_center[1] < 10 and bounding_box_center[1] > 0:
                left_side.append(bounding_box_center)
            elif bounding_box_center[1] > -10 and bounding_box_center[1] < 0:
                right_side.append(bounding_box_center)
        #fit a line to the left and right side of the rows in the map frame from the base_link
        if len(left_side) == 0 or len(right_side) == 0:
            print("no left or right trees found in range of 10 meters to robot")
        else:
            left_line = self.fit_line(left_side)
            right_line = self.fit_line(right_side)
            #find the center line of the rows
            center_line = self.center_line(left_line, right_line)
            #publish the center path
            path2 = []
            interpolation = self.interpolation(center_line[0][0], center_line[1][0], 1)
            i = self.interpolation_x_y(interpolation, center_line[0][1])
            
            
            for j in range(3,len(i)):
                path2.append([j,i[0][1]])
            #publish the path in the map frame
            print(path2)
            ax=[0,0]
            ay=[path2[0][0],path2[0][1]]
            xy=self.draw_curve(ax,ay)
            path2 = np.concatenate((xy, path2), axis=0)
            print(path2)
            #publish the path in the map frame
            path = Path()
            path.header.frame_id = "rslidar"
            # path.header.stamp = rospy.get_rostime()
            path.header.stamp = rospy.Time.now()
            for point in path2:
                pose = PoseStamped()
                pose.header.frame_id = "rslidar"
                # pose.header.stamp = rospy.get_rostime()
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = 0
                # try:
                #     point_odom = self.tf_buffer.transform(pose, 'map',rospy.Duration(0.1))#, type='geometry_msgs/PoseStamped')
                # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                #     rospy.sleep(1) # sleep for a bit to allow the tf buffer to fill up
                #     raise
                #     print("error in transform")
                path.poses.append(pose)
            self.path_pub.publish(path)
            #convert the path positions x and y to a numpy array
            x, y = [], []
            tra=path
            trajectory_message = Trajectory()
            trajectory_message.header.frame_id = "map"
            # trajectory_message.header.stamp = rospy.get_rostime()
            trajectory_message.header.stamp = rospy.Time.now()
            dis = 0
            
            for i,point in enumerate(tra.poses):
                trajectory_point = TrajectoryPoint()
                trajectory_point.pose.position.x = point.pose.position.x
                trajectory_point.pose.position.y = point.pose.position.y
                trajectory_point.pose.position.z = point.pose.position.z
                trajectory_point.longitudinal_velocity_mps = 1
                if i == 0:
                    dis = 0
                else:
                    dis += distance_btw_poses(point.pose,tra.poses[i-1].pose)
                trajectory_point.accumulated_distance_m = dis

                trajectory_message.points.append(trajectory_point)

            # trajectory_message.longitudinal_velocity_mps =1
            # trajectory_message.lateral_velocity_mps = 1
            if self.curve != True:
                self.trajectory_pub.publish(trajectory_message)
                print("trajectory published")
            else:
                rospy.loginfo('curve received')
    def interpolation_x_y(self,x_list, y):
        path=[]
        for i in x_list:
            path.append([i,y,0])
        return path


    def interpolation(self, low, high, step):
        return np.arange(low, high + step, step)

    def draw_curve(self,p1, p2):
        a = (p2[1] - p1[1]) / (np.cosh(p2[0]) - np.cosh(p1[0]))
        b = p1[1] - a * np.cosh(p1[0])
        x = np.linspace(p1[0], p2[0], 20)
        y = a * np.cosh(x) + b
        xy = np.vstack((x, y)).T
        return xy

    def fit_line(self, bounding_box_centers):
        #fit a line to the left and right side of the rows in the map frame from the base_link
        bounding_box_centers = np.array(bounding_box_centers)
        x = bounding_box_centers[:, 0]
        y = bounding_box_centers[:, 1]
        z = bounding_box_centers[:, 2]
        x = x[:, np.newaxis]
        y = y[:, np.newaxis]
        z = z[:, np.newaxis]
        model = linear_model.LinearRegression()
        model.fit(x, y)
        a = model.coef_[0][0]
        b = model.intercept_[0]
        x1 = np.min(x)
        y1 = a * x1 + b
        x2 = np.max(x)
        y2 = a * x2 + b
        return [[1, y1, 0], [40, y2, 0]]

    def center_line(self, left_line, right_line):
        #find the center of the left and right line for y1 and y2
        y1 = (left_line[0][1] + right_line[0][1]) / 2
        y2 = (left_line[1][1] + right_line[1][1]) / 2
        #find the x1 and x2 for the center line
        
        return [[0, y1, 0], [40, y2, 0]]
    
    def convert_path(self,path, to_frame):
        """
        :param path: list or geometry_msgs/poses returns the same format
        :param to_frame:
        :return: transformed path in to_frame
        """
        if path.header.frame_id == to_frame:
            return path

        if self.tfBuffer is None or self.listener is None:
            print('tfBuffer or listener is None')
        try:
            trans = self.tfBuffer.lookup_transform(to_frame, path.header.frame_id, rospy.Time.now(), rospy.Duration(2.0))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('FAILED TO GET TRANSFORM FROM %s to %s' % (to_frame, path.header.frame_id))
            rospy.logerr(str(e))
            return None
        for i in range(len(path.poses)):
            path.poses[i] = tf2_geometry_msgs.do_transform_pose(path.poses[i], trans)
        path.header.frame_id = to_frame
        return path
    


if __name__ == '__main__':
    rospy.init_node('bounding_box_line_fitting')
    bounding_box_line_fitting = BoundingBox_linefitting()
    rospy.spin()