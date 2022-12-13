#!/usr/bin/env python3 
from turtle import pos
import rospy
import math
from jsk_recognition_msgs.msg import BoundingBoxArray
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

import numpy as np
from numpy import argmin, zeros
from sklearn.neighbors import KDTree
import copy
import ros_numpy
import rospy
import tf2_ros
import time
import sys

# ros messages
import geometry_msgs.msg as gmsg
from nav_msgs.msg import Path, Odometry
from jsk_recognition_msgs.msg import BoundingBoxArray
# from zed_interfaces.msg import ObjectsStamped, Object
from geometry_msgs.msg import Point, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import PointCloud2, LaserScan
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import BoundingBox
from geometry_msgs.msg import Polygon, PolygonStamped
# utils
from laser_geometry import LaserProjection
import tf2_geometry_msgs
import sensor_msgs.point_cloud2 as pcd2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from vehicle_common.vehicle_config import vehicle_data
# # autopilot related imports
# from autopilot_utils.tf_helper import current_robot_pose, convert_point, transform_cloud
# from autopilot_utils.pose_helper import distance_btw_poses, get_yaw
# from vehicle_common.vehicle_config import vehicle_data
from autopilot_msgs.msg import Trajectory, TrajectoryPoint
tfBuffer = None
listener = None

class obs_detection:
    def __init__(self):
        self.pub1 = rospy.Subscriber("/filtered_detector/jsk_bboxes",BoundingBoxArray,self.Bounding_Cb)
        self.pub2 = rospy.Publisher("/vehicle_emergency_stop",Bool,queue_size=1)
        self.box_msg = BoundingBoxArray()
        self.obs = 0
        self.vehicle = AckermannDrive()
        self.speed = 0
        self.obs_inf = 0
        self.acceleration = 0
        self.x = 0
        self.y = 0
        self.z = 0

        self.emer = Bool

    def convert_point_by_transform(self,point, trans):
        if isinstance(point, gmsg.Point):
            p2 = tf2_geometry_msgs.do_transform_point(gmsg.PointStamped(point=point), trans).point
            return p2
        elif isinstance(point, list) or isinstance(point, tuple) and len(point) == 3:
            point = gmsg.Point(point[0], point[1], point[2])
            p2 = tf2_geometry_msgs.do_transform_point(gmsg.PointStamped(point=point), trans).point
            return p2.x, p2.y, p2.z

    def _init_tf(self):
        # Create buffer and listener
        # Something has changed in tf that means this must happen after init_node
        global tfBuffer, listener
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        
    def transform_zed_objects(self,object_data, to_frame):
        """
        Transforms zed objects data to
        :param object_data: (zed_interfaces/ObjectsStamped) object data you want to transform
        :param to_frame: frame to transform
        :return transformed_object_data (zed_interfaces/ObjectsStamped)
        """
        if object_data.header.frame_id == to_frame:
            return object_data
        global tfBuffer, listener
        if tfBuffer is None or listener is None:
            self._init_tf()
        try:
            trans = tfBuffer.lookup_transform(to_frame, object_data.header.frame_id, rospy.Time.now(), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('FAILED TO GET TRANSFORM FROM %s to %s' % (to_frame, object_data.header.frame_id))
            return None
        for i in range(len(object_data.boxes)):
            object_data.boxes[i].pose.position = self.convert_point_by_transform(object_data.boxes[i].pose.position, trans)
            # for j in range(len(object_data.objects[i].bounding_box_3d.corners)):
            #     object_data.objects[i].bounding_box_3d.corners[j].kp = \
            #         self.convert_point_by_transform(object_data.objects[i].bounding_box_3d.corners[j].kp, trans)
        object_data.header.frame_id = to_frame
        object_data.header.stamp = rospy.Time.now()
        return object_data  

    def pose_cb(self,data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z

        print(self.x,self.y,self.z)

    def Bounding_Cb(self,data):
        print(data.header.frame_id)

        data = self.transform_zed_objects(data,'map')
        box_list = data.boxes
        size = len(box_list)

        print(data.header.frame_id)


        for i in range(0,size-1):
            self.box_msg = box_list[i]
            pos_x = self.box_msg.pose.position.x
            pos_y = self.box_msg.pose.position.y
            pos_z = self.box_msg.pose.position.z

            dimX = self.box_msg.dimensions.x
            dimY = self.box_msg.dimensions.y

            print("CenterX: ",pos_x)
            print("CenterY: ",pos_y)

            
            other_coordinates = self.GetCoordinates(pos_x,pos_y,dimX,dimY)

            print(other_coordinates[0][0],end=" ")
            print(other_coordinates[0][1])

            print(other_coordinates[1][0],end=" ")
            print(other_coordinates[1][1])

            print(other_coordinates[2][0],end=" ")
            print(other_coordinates[2][1])

            print(other_coordinates[3][0],end=" ")
            print(other_coordinates[3][1])

            print("..........................................................................")

            obs_dist = self.distance(pos_x,pos_y,pos_z)
            self.dist_from_path = math.sqrt((pos_x - self.x)*(pos_x-self.x) + (pos_y - self.y)*(pos_y-self.y)+ (pos_z - self.z)*(pos_z-self.z))
            if(obs_dist < 10):
                angle = self.angle(pos_y,pos_x)
                if((obs_dist < 10 and obs_dist > 0.6)):
                    # print("Distance: ",obs_dist)
                    # print("From path: ",self.dist_from_path)
                    self.obs = 1
                    break

                else:
                    self.obs = 0

        self.pub2.publish(self.obs)

    def distance(self,x,y,z):
        return math.sqrt(x*x+y*y+z*z)

    def angle(self,x,y):
        return math.degrees(math.atan2(y,x))

    def GetCoordinates(self,x, y,dimensionsX,dimensionsY):
        yaw = 0
        cos_th = math.cos(yaw)
        sin_th = math.sin(yaw)

        all_coordinates = []

        co1X = x - (dimensionsX * cos_th - dimensionsY * sin_th)
        co1Y = y - (dimensionsX * sin_th + dimensionsY * cos_th)

        co2X = x + (dimensionsX * cos_th - dimensionsY * sin_th)
        co2Y = y + (dimensionsX * sin_th + dimensionsY * cos_th)

        co3X = x + (dimensionsX * cos_th + dimensionsY * sin_th)
        co3Y = y + (dimensionsX * sin_th - dimensionsY * cos_th)

        co4X = x - (dimensionsX * cos_th + dimensionsY * sin_th)
        co4Y = y - (dimensionsX * sin_th - dimensionsY * cos_th)

        c1 = []
        c1.append(co1X)
        c1.append(co1Y)

        c2 = []
        c2.append(co2X)
        c2.append(co2Y)

        c3 = []
        c3.append(co3X)
        c3.append(co3Y)

        c4 = []
        c4.append(co4X)
        c4.append(co4Y)

        all_coordinates.append(c1)
        all_coordinates.append(c2)
        all_coordinates.append(c3)
        all_coordinates.append(c4)

        return all_coordinates

if __name__=="__main__":
    rospy.init_node("obstacle_data")
while not rospy.is_shutdown():
    obs_detection()
    rospy.spin()
