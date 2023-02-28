#!/usr/bin/env python

# import matplotlib.pyplot as plt
import rospy
# from RDP import rdp
import rospy
from std_msgs.msg import String, Bool
from nav_msgs.msg import Path
import math

from nav_msgs.msg import Odometry
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from autopilot_msgs.msg import Trajectory, TrajectoryPoint
from autopilot_msgs.msg import ControllerDiagnose, FloatKeyValue
from autopilot_utils.pose_helper import distance_btw_poses, get_yaw, angle_btw_poses, normalize_angle

import rospy
from std_msgs.msg import String
import numpy as np
class FindDir:
    def __init__(self):
        self.trajectory_data = None
        self.robot_state = None
        self.ruvDotvector = None
        self.coordinates = []
        rospy.Subscriber("/global_gps_trajectory", Trajectory, self.trajectory_callback)
        rospy.Subscriber("vehicle/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/global_gps_path", Path, self.gps_path_cb)
        rospy.Subscriber("/target_pose", PoseStamped, self.target_pose_cb)
        
    def target_pose_cb(self,data):
        self.target_pose = data
        
    def trajectory_callback(self,data):
        self.trajectory_data = data
    
    def odom_callback(self,data):
        self.robot_state = data
        self.robot_pose = self.robot_state.pose.pose
        # self.findDirectionChange(self.robot_state)
        self.findLookaheadPos(self.robot_pose, self.target_pose)

    def gps_path_cb(self,data):
        self.global_plan = data
        
        for pose in data.poses:
                # _, _, heading = euler_from_quaternion(
                #     [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
                # heading = math.degrees(heading)
                # line.append((pose.pose.position.x, pose.pose.position.y))
                self.coordinates.append([pose.pose.position.x,pose.pose.position.y])
        print(self.coordinates)
        self.findDirChange()

    def findLookaheadPos(self,robot_pose,lookAheadPose):
        # robot_pose = robot_pose.pose.pose
        theta = get_yaw(robot_pose.orientation)
        rx = robot_pose.position.x
        ry = robot_pose.position.y
        lx = lookAheadPose.pose.position.x
        ly = lookAheadPose.pose.position.y
        ruv_x,ruv_y = rx*round(math.cos(math.radians(theta)),2), ry*round(math.sin(math.radians(theta)),2)
        # print(ruv_x,ruv_y)
        # robot, lookahead x, robot, lookahead y
        # rl_x, rl_y =  = x2-x1, y2-y1
        rl_x = lx-rx
        rl_y = ly-ry
        a = [ruv_x,ruv_y]
        b = [rl_x,rl_y]
        self.ruvDotvector = np.dot(a,b)
        print(self.ruvDotvector)
        return self.ruvDotvector
        
    def findDirectionChange(self,robot_pose):
        for pose_id in range(1,len(self.global_plan.poses)):
            oa_x = self.global_plan.poses[pose_id].pose.position.x - self.global_plan.poses[pose_id - 1].pose.position.x
            oa_y = self.global_plan.poses[pose_id].pose.position.y - self.global_plan.poses[pose_id - 1].pose.position.y
            ab_x = self.global_plan.poses[pose_id + 1].pose.position.x - self.global_plan.poses[pose_id].pose.position.x
            ab_y = self.global_plan.poses[pose_id + 1].pose.position.y - self.global_plan.poses[pose_id].pose.position.y
            print(oa_x)
            
            if ((oa_x * ab_x) + (oa_y * ab_y) < 0.0):
                x = self.global_plan.poses[pose_id].pose.position.x - robot_pose.pose.pose.position.x
                y = self.global_plan.poses[pose_id].pose.position.y - robot_pose.pose.pose.position.y
                rospy.logwarn(math.hypot(x,y))
                return math.hypot(x,y)
        rospy.logerr(float('inf'))
        return float('inf')
    
    def findDirChange(self):
        coordinates = np.array(self.coordinates)
        # Get the input - array of coordinates

        # Calculate the angle between the points
        angles = []
        for i in range(len(coordinates)-1):
            x_diff = coordinates[i+1,0] - coordinates[i,0]
            y_diff = coordinates[i+1,1] - coordinates[i,1]
            angle = np.arctan2(y_diff, x_diff)
            angles.append(angle)

        # Calculate the difference between the angles
        angle_differences = []
        for i in range(len(angles)-1):
            angle_diff = angles[i+1] - angles[i]
            angle_differences.append(angle_diff)

        # Detect any changes in direction
        direction_changes_right = []
        direction_changes_left = []
        for i in range(len(angle_differences)):
            if angle_differences[i] > np.pi/2:
                print(angle_differences[i])
                direction_changes_right.append(i)
            elif angle_differences[i] < -np.pi/2:
                direction_changes_left.append(i)
        # Output the changes in direction
        print('The direction changed at points:', len(direction_changes_right),len(direction_changes_left))

        
if __name__ == '__main__':
    rospy.init_node('findDirection')
    cd = FindDir()
    rospy.spin()

