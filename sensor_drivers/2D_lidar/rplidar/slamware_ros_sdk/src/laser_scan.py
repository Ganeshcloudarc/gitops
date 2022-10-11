#!/usr/bin/env python3
from sensor_msgs.msg import LaserScan
import rospy


def scan_cb(data):
    global scan_pub
    scan_data = data
    scan_data.header.frame_id = 'laser'
    scan_pub.publish(scan_data)
    rospy.loginfo('published')
rospy.init_node('scan_node')
rospy.Subscriber("/slamware_ros_sdk_server_node/scan", LaserScan, scan_cb)
scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)

rospy.spin()