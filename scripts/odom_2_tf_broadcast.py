#!usr/bin/env python3
'''
A node to publish tf between base link to map by subscribing odometry msg from mavros.
'''
import math
import rospy
import tf2_ros
import tf_conversions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


# parameters
dis_x = rospy.get_param("/tf/dis_from_real_wheel_center_x", 2)  # assuming only x translation is available between
# sensor and real axle
child_frame_name = rospy.get_param('/tf/child_frame', 'base_link')  # frame_id of vehicle base
odom_topic = rospy.get_param("/tf/odom_topic", '/mavros/local_position/odom')
send_odom = rospy.get_param("/tf/send_odom", True)  # send odom from base link

tf_broad_caster = tf2_ros.TransformBroadcaster()
tf_msg = TransformStamped()


def odom_callback(data):
    """ subscriber for odom, it publishes tf and odom from vehicle base link(rear axil center)"""

    _, _, pose_heading = tf_conversions.transformations.euler_from_quaternion(
        [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
         data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    )
    x_pos = data.pose.pose.position.x - dis_x * math.cos(pose_heading)  # transom sensor location to rear axle in x
    y_pos = data.pose.pose.position.y - dis_x * math.sin(pose_heading)  # transom sensor location to rear axle in y

    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.header.frame_id = data.header.frame_id
    tf_msg.child_frame_id = child_frame_name
    tf_msg.transform.translation.x = x_pos
    tf_msg.transform.translation.y = y_pos
    tf_msg.transform.translation.z = 0.0  # making z zero
    tf_msg.transform.rotation = data.pose.pose.orientation
    tf_broad_caster.sendTransform(tf_msg)
    rospy.logdebug("transform sent")
    if send_odom:
        odom_msg = Odometry()
        odom_msg = data
        odom_msg.pose.pose.position.x = x_pos
        odom_msg.pose.pose.position.y = y_pos
        odom_msg.child_frame_id = child_frame_name
        odom_publisher.publish(odom_msg)
        rospy.logdebug("Odom sent")


if __name__ == '__main__':
    rospy.init_node('tf2__broadcaster')
    rospy.loginfo("tf broadcaster node started for mavros")
    if send_odom:
        odom_publisher = rospy.Publisher("/base_2_map_odom", Odometry, queue_size=2)
    rospy.Subscriber(odom_topic, Odometry, odom_callback)
    rospy.spin()
