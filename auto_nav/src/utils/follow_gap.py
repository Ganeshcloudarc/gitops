import rospy
from sensor_msgs.msg import LaserScan
import math
from visualization_msgs.msg import Marker, MarkerArray
import random
from geometry_msgs.msg import Point, PoseArray, Pose, TransformStamped, PoseStamped, PolygonStamped, Point
from autopilot_utils.tf_helper import convert_pose
import numpy as np

def get_the_vanish_points(data, end_point):
    target_point_heading = math.atan2(end_point[1], end_point[0])

    start_gap_ind = -1
    start_found = False
    end_found = False
    end_gap_ind = 0
    ind_list = []
    for i in range(len(data.ranges)):
        # print("angle ", data.angle_min + i*data.angle_increment)
        #     print("ange", data.ranges[i])
        if data.ranges[i] >= data.range_max:
            # print("inside", data.ranges[i], i, math.degrees(data.angle_min + i * data.angle_increment))
            # print("angle ", math.degrees(data.angle_min + i * data.angle_increment))
            if start_gap_ind == -1:
                start_gap_ind = i
                start_found = True

            else:
                end_gap_ind = i
                end_found = True
        else:
            if start_found and end_found:
                ind_list.append([start_gap_ind, end_gap_ind])
                start_gap_ind = -1
                end_gap_ind = -1
                end_found = False
                start_found = False
    diff_list = []
    for idx in ind_list:
        mid_id = (idx[0] + idx[1]) / 2
        mid_angle = (data.angle_min + mid_id * data.angle_increment)
        diff = abs(mid_angle - target_point_heading)
        diff_list.append(diff)
    m = np.argmin(diff_list)
    print("diff_list", diff_list)
    m = np.argmin(diff_list)
    closest_idx = ind_list[m]
    print("closest_idx", closest_idx)
    mid_i = (closest_idx[0] + closest_idx[1]) // 2
    pst = PoseStamped()
    pst.header.frame_id = data.header.frame_id
    pst.pose.position.x = data.ranges[mid_i] * math.cos(data.angle_min + mid_i * data.angle_increment)
    pst.pose.position.y = data.ranges[mid_i] * math.sin(data.angle_min + mid_i * data.angle_increment)
    pst.pose.orientation.w = 1
    return pst



class FollowGap:
    def __init__(self):
        rospy.Subscriber("laser_scan", LaserScan, self.laserscan_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.target_point_callback)

        self.marker_arr_pub = rospy.Publisher("gaps", MarkerArray, queue_size=1,
                                                            latch=True)
        self.vanish_point_pub = rospy.Publisher("vanish_point", PoseStamped, queue_size=1,
                                                            latch=True)
        max_gap = 0
        self.min_range = 10
        self.target_pose = PoseStamped()

    def target_point_callback(self, data):
        self.target_pose = data

    def laserscan_callback(self, data):
        print("---------------------")
        print("target_pose", self.target_pose)
        target_pose = convert_pose(self.target_pose.pose, self.target_pose.header.frame_id, data.header.frame_id)
        # after_conversion_pose
        if target_pose is None:
            print("target_pose is none")
            return 0

        target_point_heading = math.atan2(target_pose.position.y, target_pose.position.x)
        print("target_point_heading", target_point_heading)
        start_gap_ind = -1
        start_found = False
        end_found = False
        end_gap_ind = 0
        current_start = 0
        ind_list = []
        for i in range(len(data.ranges)):
            # print("angle ", data.angle_min + i*data.angle_increment)
        #     print("ange", data.ranges[i])
            if data.ranges[i] >= data.range_max:
                print("inside", data.ranges[i], i, math.degrees(data.angle_min + i * data.angle_increment))
                # print("angle ", math.degrees(data.angle_min + i * data.angle_increment))
                if start_gap_ind == -1:
                    start_gap_ind = i
                    start_found = True

                else:
                    end_gap_ind = i
                    end_found = True
            else:
                if start_found and end_found:
                    ind_list.append([start_gap_ind, end_gap_ind ])
                    start_gap_ind = -1
                    end_gap_ind = -1
                    end_found = False
                    start_found = False

        print("ind_list", ind_list)

        marker_arr_msg = MarkerArray()
        count = 0

        for idx in ind_list:
            marker = Marker()
            marker.header.frame_id = data.header.frame_id
            marker.type = marker.LINE_STRIP
            marker.id = count
            count += 1
            marker.action = marker.ADD
            marker.scale.x = 0.3
            marker.color.a = 0.5
            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 0
            for i in range(idx[0], idx[1]):
                pt = Point()
                pt.x = data.ranges[i] * math.cos(data.angle_min + i * data.angle_increment)
                pt.y = data.ranges[i] * math.sin(data.angle_min + i * data.angle_increment)
                marker.points.append(pt)
            marker_arr_msg.markers.append(marker)
        self.marker_arr_pub.publish(marker_arr_msg)
        diff_list = []
        for idx in ind_list:
            mid_id = (idx[0] + idx[1])/2
            mid_angle = (data.angle_min + mid_id * data.angle_increment)
            print("mid_angle", mid_angle)
            diff = abs(mid_angle - target_point_heading)
            diff_list.append(diff)

        print("diff_list", diff_list)
        m = np.argmin(diff_list)
        closest_idx = ind_list[m]
        print("closest_idx", closest_idx)
        mid_i = (closest_idx[0] + closest_idx[1])//2
        pst = PoseStamped()
        pst.header.frame_id = data.header.frame_id
        pst.pose.position.x = data.ranges[mid_i] * math.cos(data.angle_min + mid_i * data.angle_increment)
        pst.pose.position.y = data.ranges[mid_i] * math.sin(data.angle_min + mid_i * data.angle_increment)
        pst.pose.orientation.w = 1
        self.vanish_point_pub.publish(pst)
        rospy.logwarn("markers published")





    def dwa_paths_marker_array(dwa_paths, target_frame="base_link"):
        marker_arr_msg = MarkerArray()
        # marker_arr_msg.header.frame_id = target_frame
        count = 0
        length = len(dwa_paths)
        for path in dwa_paths:
            marker = Marker()
            marker.header.frame_id = target_frame
            # marker.header.stamp = rospy.Time.now()
            marker.type = marker.LINE_STRIP
            marker.id = count
            count += 1
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.color.a = 0.5
            marker.color.r = random.random()
            marker.color.g = random.random()
            marker.color.b = random.random()
            for point in path:
                pt = Point()
                pt.x = point[0]
                pt.y = point[1]
                marker.points.append(pt)
            marker_arr_msg.markers.append(marker)
        return marker_arr_msg


if __name__ == "__main__":
    rospy.init_node("follow_gap_node")
    fg = FollowGap()
    rospy.spin()







# import rospy
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist
# import math
# class FollowGap:
#     def __init__(self):
#         rospy.init_node('follow_gap')
#         self.scan_sub = rospy.Subscriber('/laser_scan', LaserScan, self.scan_callback)
#         self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#         self.cmd_vel = Twist()
#
#         self.min_gap_angle = 10  # Minimum angle in degrees to consider a gap
#         self.min_gap_distance = 0.5  # Minimum distance to consider a gap
#
#     def scan_callback(self, data):
#         ranges = data.ranges
#         num_ranges = len(ranges)
#         half_num_ranges = num_ranges // 2
#
#         # Find the gap
#         min_distance = float('inf')
#         gap_start = 0
#         gap_end = 0
#         current_start = 0
#
#         for i in range(num_ranges):
#             if ranges[i] > self.min_gap_distance:
#                 if i - current_start > half_num_ranges:
#                     current_start = i
#                 elif i - current_start > self.min_gap_angle:
#                     gap_start = current_start
#                     gap_end = i
#                     break
#
#         # Calculate the angle to the center of the gap
#         gap_center = (gap_start + gap_end) / 2
#         gap_angle = (gap_center - half_num_ranges) * data.angle_increment
#         print("gap_angle", math.degrees(gap_angle))
#         # Publish twist command to follow the gap
#         self.cmd_vel.linear.x = 0.2  # Forward linear velocity
#         self.cmd_vel.angular.z = -0.5 * gap_angle  # Angular velocity proportional to gap angle
#         self.cmd_pub.publish(self.cmd_vel)
#
#     def run(self):
#         rospy.spin()
#
# if __name__ == '__main__':
#     try:
#         follower = FollowGap()
#         follower.run()
#     except rospy.ROSInterruptException:
#         pass


