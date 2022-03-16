#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA, String

wheel_base = rospy.get_param('/vehicle/wheel_base', 2)
dist_front_rear_wheels = rospy.get_param('/vehicle/dist_front_rear_wheels', 1.5)


def vehicle_pose_callback(data):
    global vehicle_pose_marker, vehicle_pose_id
    marker = Marker(
        type=Marker.CUBE,
        id=0,
        lifetime=rospy.Duration(1),
        pose=data.pose,
        scale=Vector3(wheel_base+0.5, dist_front_rear_wheels, 1),
        header=Header(frame_id='map'),
        color=ColorRGBA(1, 1, 0, 0.5))
    vehicle_pose_id += target_id
    rospy.logdebug("vehicle_pose_callback")
    vehicle_pose_marker.publish(marker)


def target_pose_callback(data):
    global target_pose_marker, target_id
    marker = Marker(
        type=Marker.SPHERE,
        id=target_id,
        lifetime=rospy.Duration(1),
        pose=data.pose,
        scale=Vector3(0.3, 0.3, 0.3),
        header=Header(frame_id='map'),
        color=ColorRGBA(255, 255, 255, 1))
    target_id += target_id
    rospy.logdebug("target_pose_callback")
    target_pose_marker.publish(marker)


if __name__ == "__main__":
    rospy.init_node("visualization_marker_pure_pursuit")
    rospy.loginfo("visualization_marker_pure_pursuit Node started ")
    rospy.Subscriber("/vehicle_pose", PoseStamped, vehicle_pose_callback)
    rospy.Subscriber("/target_pose", PoseStamped, target_pose_callback)
    vehicle_pose_marker = rospy.Publisher("/vehicle_pose_marker", Marker, queue_size=2)
    target_pose_marker = rospy.Publisher("/target_pose_marker", Marker, queue_size=2)
    target_id, vehicle_pose_id = 0, 0
    rospy.spin()
