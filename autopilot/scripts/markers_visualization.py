#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA, String

wheel_base = rospy.get_param('/vehicle/wheel_base', 2)
dist_front_rear_wheels = rospy.get_param('/vehicle/dist_front_rear_wheels', 1.5)


def vehicle_pose_callback(data):
    global vehicle_pose_marker
    marker = Marker(
        type=Marker.SPHERE,
        id=0,
        lifetime=rospy.Duration(1),
        pose=data.pose,
        scale=Vector3(0.3, 0.3, 0.3),
        header=Header(frame_id='map'),
        color=ColorRGBA(0, 255, 0, 1))
    rospy.logdebug("target_pose_callback")
    vehicle_pose_marker.publish(marker)


def target_pose_callback(data):
    global target_pose_marker
    marker = Marker(
        type=Marker.SPHERE,
        id=0,
        lifetime=rospy.Duration(1),
        pose=data.pose,
        scale=Vector3(0.3, 0.3, 0.3),
        header=Header(frame_id='map'),
        color=ColorRGBA(255, 255, 255, 1))
    rospy.logdebug("target_pose_callback")
    target_pose_marker.publish(marker)


if __name__ == "__main__":
    rospy.init_node("visualization_marker_pure_pursuit")
    rospy.loginfo("visualization_marker_pure_pursuit Node started ")
    rospy.Subscriber("/close_pose", PoseStamped, vehicle_pose_callback)
    rospy.Subscriber("/target_pose", PoseStamped, target_pose_callback)
    vehicle_pose_marker = rospy.Publisher("/close_pose_marker", Marker, queue_size=2)
    target_pose_marker = rospy.Publisher("/target_pose_marker", Marker, queue_size=2)
    rospy.spin()
