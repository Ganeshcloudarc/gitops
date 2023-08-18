#generate path from kml file and publish it as a path message with dubins curve path planning

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Header
import numpy as np
from geographic_msgs.msg import GeoPoint
import utm
from dubins import *
from geometry_msgs.msg import PoseStamped

class dubins_path():
    def __init__(self):
        self.path = Path()
        self.path.header = Header()
        self.path.header.frame_id = "map"
        self.path.header.stamp = rospy.Time.now()
        self.path_pub = rospy.Publisher('path', Path, queue_size=1)
        self.sub = rospy.Subscriber('/kml_path', GeoPoint, self.callback)
        self.path_points = []
        self.path_poses = []

    def callback(self, msg):
        self.path_points.append(msg)
        if len(self.path_points) > 1:
            self.path.header.stamp = rospy.Time.now()
            self.path.header.frame_id = "map"
            self.path.poses = []
            self.path_poses = []
            for i in range(len(self.path_points)-1):
                start = (self.path_points[i].latitude, self.path_points[i].longitude)
                end = (self.path_points[i+1].latitude, self.path_points[i+1].longitude)
                path = dubins_path_planning(start, end)
                for j in range(len(path)):
                    self.path_poses.append(path[j])
            for i in range(len(self.path_poses)):
                pose = PoseStamped()
                pose.header = self.path.header
                pose.pose.position.x = self.path_poses[i][0]
                pose.pose.position.y = self.path_poses[i][1]
                self.path.poses.append(pose)
            self.path_pub.publish(self.path)
    
    def dubins_path_planning(start, end):
        start_utm = utm.from_latlon(start[0], start[1])
        end_utm = utm.from_latlon(end[0], end[1])
        start_utm = (start_utm[0], start_utm[1], start_utm[2])
        end_utm = (end_utm[0], end_utm[1], end_utm[2])
        path = []
        path.append(start_utm)
        path.append(end_utm)
        return path

