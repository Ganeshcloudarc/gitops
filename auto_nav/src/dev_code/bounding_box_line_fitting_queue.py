#!/usr/bin/env python3
'''line fitting for bounding box markersarray with map frame reference'''
import rospy
from sklearn import linear_model
from sklearn.linear_model import LogisticRegression
import numpy as np
import numpy as np
import rospy
import math
from visualization_msgs.msg import MarkerArray, Marker
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
from geometry_msgs.msg import Point,PoseStamped
#path message
from nav_msgs.msg import Path,Odometry
import tf2_ros
import tf2_geometry_msgs  
from tf_helper_frame import *
from autopilot_msgs.msg import Trajectory, TrajectoryPoint
from std_msgs.msg import Bool
from autopilot_utils.pose_helper import get_yaw, angle_btw_poses
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from scipy.signal import medfilt

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
def distance_btw_poses_y(pose1, pose2):
    """
    Calculates distance between poses.
        Parameters:
            pose1(geometry_msgs/Pose.msg): pose one.
            pose2(geometry_msgs/Pose.msg): pose two.
        Returns:
            distance(float): distance between the poses
    """
    distance = math.hypot(pose1.position.y - pose2.position.y)
    return distance

class PID:
    def __init__(self, P=0.0, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.last_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output

class BoundingBox_map:
    def __init__(self):
        self.bounding_box_sub = rospy.Subscriber("/filtered_detector/jsk_bboxes", BoundingBoxArray, self.bounding_box_callback)
        self.curve_sub = rospy.Subscriber("/global_gps_path/is_curve", Bool , self.curve_callback)
        self.path_sub = rospy.Subscriber("/global_gps_path", Path, self.path_callback)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)
        self.path_repub = rospy.Publisher("/local_gps_path", Path, queue_size=1)
        self.trajectory_pub = rospy.Publisher("/local_gps_trajectory", Trajectory, queue_size=1)
        self.odom_sub = rospy.Subscriber("/vehicle/odom", Odometry, self.odom_callback)
        self.line_pub = rospy.Publisher("/markers", MarkerArray, queue_size=1)
        self.odom=None
        self.vehicle_position=None
        self.left_queue = []
        self.right_queue = []
        self.left_queue_x = []
        self.left_queue_y = []
        self.right_queue_x = []
        self.right_queue_y = []
        self.path = None
        self.path2 = None
        self.is_curve = False
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)


    def path_callback(self, data):
        self.path = data
        print("path",self.path)

         
    def odom_callback(self,odom):
        self.odom=odom
        self.vehicle_pose = self.odom.pose.pose 
        self.vehicle_yaw = get_yaw(self.vehicle_pose.orientation)
        #get the next 10m of the path in front of the vehicle
        path2 = self.get_path(self.vehicle_pose, self.path)
        #publish the path message to rviz
        path= Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        path.poses = path2
        self.path_repub.publish(path)
        self.path2 = path

    def get_path(self, vehicle_pose, path):
        path2 = []
        # print("path",path)
        for pose in path.poses:

            if distance_btw_poses(pose.pose, vehicle_pose) < 10:
                #append only the poses in front of the vehicle
                if pose.pose.position.x > vehicle_pose.position.x:
                     
                    path2.append(pose)
        return path2

    #distence between two path points in the path message in meters 
    


    def curve_callback(self, curve):
        self.is_curve = curve.data

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
        path2 = []
        left_queue_x = []
        left_queue_y = []
        right_queue_x = []
        right_queue_y = []
        dis = []
        
        # print("bounding_box_centers",bounding_box_centers)
        for box in bounding_box_centers:
                
                if box[1] > 0:
                        left_side.append(box)
                else:
                        right_side.append(box)
        self.left_queue.append(left_side)
        self.right_queue.append(right_side)
        if len(self.left_queue) > 5:
                self.left_queue.pop(0)
        if len(self.right_queue) > 5:
                self.right_queue.pop(0)
        #array for only the x and y coordinates in queue for left and right side
        
        for left in self.left_queue:
                for box in left:
                        left_queue_x.append(box[0])
                        left_queue_y.append(box[1])
        for right in self.right_queue:
                for box in right:
                        right_queue_x.append(box[0])
                        right_queue_y.append(box[1])
        #fit a line to the left and right side of the rows
        left_x1, left_y1, left_x2, left_y2 = self.line_fitting(left_queue_x, left_queue_y)
        right_x1, right_y1, right_x2, right_y2 = self.line_fitting(right_queue_x, right_queue_y)
        #center of the two lines
        center_x1,center_x2,center_y1,center_y2 = self.center_line(left_x1, left_y1, left_x2, left_y2, right_x1, right_y1, right_x2, right_y2)
        self.publish_line(left_x1, left_y1, left_x2, left_y2, right_x1, right_y1, right_x2, right_y2, center_x1,center_x2,center_y1,center_y2)
        # print("center_x1",center_x1)
        # print("center_x2",center_x2)
        # print("center_y1",center_y1, "center_y2",center_y2)
        # print("center_x1",center_x1, "center_x2",center_x2)
        if center_x2 < center_y2:
            m= center_y2 - center_x2
            m=m/10
            i = np.arange(center_x2,center_y2,m)
        else :
            m= center_x2 - center_y2
            m=m/10
            i = np.arange(center_y2,center_x2,m)
        if i[0] == center_x2:
            pass
        else:
            i = i[::-1]
        
        
        i=medfilt(i,13)

        # for j in range(0,len(i)):
        #     path2.append([j,i[j]])

        for j in range(5,len(i)):
            path2.append([j,i[j]])
        #publish the path in the map frame
        ax=[0,0]
        ay=[path2[0][0],path2[0][1]]
        xy=self.draw_curve(ax,ay)
        path2 = np.concatenate((xy, path2), axis=0)
        # print(path2)
        #publish the path in the map frame
        path = Path()
        path.header.frame_id = "rslidar"
        # path.header.stamp = rospy.get_rostime()
        path.header.stamp = rospy.Time.now()
        for i,point in enumerate(path2):
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
            #find distance between two paths and publish the self.path2 and xy
            distance = distance_btw_poses_y(pose.pose,self.path2.poses[i].pose)
            dis.append(distance)
            path.poses.append(pose)
        
        
        result = [val for val in dis if val>1]
        
        if len(result) > 0 or self.is_curve == True:
            self.path_pub.publish(self.path2)
            tra=self.path2
        else:
            self.path_pub.publish(path)
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
            trajectory_point.longitudinal_velocity_mps = 1.5
            if i == 0:
                dis = 0
            else:
                dis += distance_btw_poses(point.pose,tra.poses[i-1].pose)
            trajectory_point.accumulated_distance_m = dis

            trajectory_message.points.append(trajectory_point)

        # trajectory_message.longitudinal_velocity_mps =1
        # trajectory_message.lateral_velocity_mps = 1
        if self.is_curve != True:
            self.trajectory_pub.publish(trajectory_message)
            print("trajectory published")
        else:
            rospy.loginfo('curve received')

    def draw_curve(self,p1, p2):
        a = (p2[1] - p1[1]) / (np.cosh(p2[0]) - np.cosh(p1[0]))
        b = p1[1] - a * np.cosh(p1[0])
        x = np.linspace(p1[0], p2[0], 24)
        y = a * np.cosh(x) + b
        xy = np.vstack((x, y)).T
        return xy        
        
    def line_fitting(self, x, y):
        #line fitting with ransac algorithm
        x = np.array(x)
        y = np.array(y)
        x = x[:, np.newaxis]
        y = y[:, np.newaxis]
        model = linear_model.LinearRegression()
        model.fit(x, y)
        a = model.coef_[0][0]
        b = model.intercept_[0]
        x1 = np.min(x)
        y1 = a * x1 + b
        x2 = np.max(x)
        y2 = a * x2 + b
        return x1, y1, x2, y2
    
    def center_line(self, left_x1, left_y1, left_x2, left_y2, right_x1, right_y1, right_x2, right_y2):
        #find the center line between the left and right side of the rows
        center_x1 = (left_x1 + right_x1) / 2
        center_y1 = (left_y1 + right_y1) / 2
        center_x2 = (left_x2 + right_x2) / 2
        center_y2 = (left_y2 + right_y2) / 2
        return center_x1, center_y1, center_x2, center_y2

    def publish_line(self, left_x1, left_y1, left_x2, left_y2, right_x1, right_y1, right_x2, right_y2, center_x1, center_y1, center_x2, center_y2):
        #create a marker array for the left and right side of the rows
        marker_array = MarkerArray()
        marker_left = Marker()
        marker_left.header.frame_id = "rslidar"
        marker_left.header.stamp = rospy.Time.now()
        marker_left.ns = "left_line"
        marker_left.id = 0
        marker_left.type = Marker.LINE_STRIP
        marker_left.action = Marker.ADD
        marker_left.scale.x = 0.1
        marker_left.color.a = 1.0
        marker_left.color.r = 1.0
        marker_left.color.g = 0.0
        marker_left.color.b = 0.0
        marker_left.pose.orientation.w = 1.0
        marker_left.points.append(Point(left_x1, left_y1, 0))
        marker_left.points.append(Point(left_x2, left_y2, 0))
        marker_array.markers.append(marker_left)
        marker_right = Marker()
        marker_right.header.frame_id = "rslidar"
        marker_right.header.stamp = rospy.Time.now()
        marker_right.ns = "right_line"
        marker_right.id = 0
        marker_right.type = Marker.LINE_STRIP
        marker_right.action = Marker.ADD
        marker_right.scale.x = 0.1
        marker_right.color.a = 1.0
        marker_right.color.r = 0.0
        marker_right.color.g = 1.0
        marker_right.color.b = 0.0
        marker_right.pose.orientation.w = 1.0
        marker_right.points.append(Point(right_x1, right_y1, 0))
        marker_right.points.append(Point(right_x2, right_y2, 0))
        marker_array.markers.append(marker_right)
        marker_center = Marker()
        marker_center.header.frame_id = "rslidar"
        marker_center.header.stamp = rospy.Time.now()
        marker_center.ns = "center_line"
        marker_center.id = 0
        marker_center.type = Marker.LINE_STRIP
        marker_center.action = Marker.ADD
        marker_center.scale.x = 0.1
        marker_center.color.a = 1.0
        marker_center.color.r = 0.0
        marker_center.color.g = 0.0
        marker_center.color.b = 1.0
        marker_center.pose.orientation.w = 1.0
        marker_center.points.append(Point(center_x1, center_y1, 0))
        marker_center.points.append(Point(center_x2, center_y2, 0))
        marker_array.markers.append(marker_center)
        self.line_pub.publish(marker_array)



if __name__ == '__main__':
    rospy.init_node('bounding_box_line_fitting_map_reference')

    bounding_box_map = BoundingBox_map()
    rospy.spin()