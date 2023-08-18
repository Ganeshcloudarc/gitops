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


class BoundingBox_map:
    def __init__(self):
        self.bounding_box_sub = rospy.Subscriber("/filtered_detector/jsk_bboxes", BoundingBoxArray, self.bounding_box_callback)
        self.curve_sub = rospy.Subscriber("/global_gps_path/is_curve", Bool , self.curve_callback)
        # self.path_sub = rospy.Subscriber("/global_gps_path", Path, self.path_callback)
        # self.path_pub = rospy.Publisher("/path", Path, queue_size=1)
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


    def odom_callback(self,odom):
        self.odom=odom
        self.vehicle_pose = self.odom.pose.pose 
        self.vehicle_yaw = get_yaw(self.vehicle_pose.orientation)
        # self.vehicle_[self.odom.pose.pose.position.x,self.odom.pose.pose.position.y, get_yaw(self.odom.pose.pose.orientation)]


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
        
        # print("bounding_box_centers",bounding_box_centers)
        for box in bounding_box_centers:
                # prinr()
                pose_unit = [np.cos(self.vehicle_yaw), np.sin(self.vehicle_yaw)]
                bbox_vect = [box[0] - self.vehicle_pose.position.x   , box[1] - self.vehicle_pose.position.y  ]

                area =np.cross(pose_unit, bbox_vect)
                if area > 0:
                        left_side.append(box)
                else:
                        right_side.append(box)        
        self.left_queue.append(left_side)
        self.right_queue.append(right_side)
        if len(self.left_queue) > 1000:
                self.left_queue.pop(0)
        if len(self.right_queue) > 1000:
                self.right_queue.pop(0)
        #array for only the x and y coordinates in queue for left and right side
        
        for left in self.left_queue:
                for box in left:
                        self.left_queue_x.append(box[0])
                        self.left_queue_y.append(box[1])
        for right in self.right_queue:
                for box in right:
                        self.right_queue_x.append(box[0])
                        self.right_queue_y.append(box[1])
        #fit a line to the left and right side of the rows
        left_x1, left_y1, left_x2, left_y2 = self.line_fitting(self.left_queue_x, self.left_queue_y)
        right_x1, right_y1, right_x2, right_y2 = self.line_fitting(self.right_queue_x, self.right_queue_y)
        self.publish_line(left_x1, left_y1, left_x2, left_y2, right_x1, right_y1, right_x2, right_y2)
        
        

#     def line_fitting(self, x, y):
#         x = np.array(x)
#         y = np.array(y)
#         x = x[:, np.newaxis]
#         y = y[:, np.newaxis]
#         model = linear_model.LinearRegression()
#         model.fit(x, y)
#         a = model.coef_[0][0]
#         b = model.intercept_[0]
#         x1 = x[0]
#         y1 = a * x1 + b
#         x2 = max(x[-10:]) + 10
#         y2 = a * x2 + b
#         return x1, y1, x2, y2
    def line_fitting(self, x, y):
        #line fitting with ransac algorithm
        x = np.array(x)
        y = np.array(y)
        x = x[:, np.newaxis]
        y = y[:, np.newaxis]
        model_ransac = linear_model.RANSACRegressor(linear_model.LinearRegression())
        model_ransac.fit(x, y)
        inlier_mask = model_ransac.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)
        line_X = np.arange(x.min(), x.max())[:, np.newaxis]
        line_y_ransac = model_ransac.predict(line_X)
        a = model_ransac.estimator_.coef_[0][0]
        b = model_ransac.estimator_.intercept_[0]
        x1 = x[0]
        y1 = a * x1 + b
        x2 = max(x[-10:]) + 10
        y2 = a * x2 + b
        return x1, y1, x2, y2
    def publish_line(self, left_x1, left_y1, left_x2, left_y2, right_x1, right_y1, right_x2, right_y2):
        #create a marker array for the left and right side of the rows
        marker_array = MarkerArray()
        marker_left = Marker()
        marker_left.header.frame_id = "map"
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
        marker_right.header.frame_id = "map"
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
        self.line_pub.publish(marker_array)



if __name__ == '__main__':
    rospy.init_node('bounding_box_line_fitting_map_reference')

    bounding_box_map = BoundingBox_map()
    rospy.spin()


