#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix, MagneticField, Imu
from geometry_msgs.msg import PoseStamped
import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion

global zed_cam_heading, pwd_heading


def ConvertTo360Range(deg):
    # if deg < 0.0:
    deg = deg % 360.0

    return deg


def pwk_pose_callback(data):
    # print(data)
    _, _, pose_heading = euler_from_quaternion(
        [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
    pose_heading = math.degrees(pose_heading) % 360
    diff = pose_heading - pwd_heading
    print("--------------------------")
    print('pose heading :', pose_heading)
    print('mag heading  :', pwd_heading)
    print('diff         :', diff)


def pwk_mag_callback(data):
    compass_angle = math.atan2(data.magnetic_field.y, data.magnetic_field.x)
    global pwd_heading
    pwd_heading = math.degrees(compass_angle) % 360  #



rospy.init_node('test_magnetometer')
rospy.Subscriber('/mavros/imu/mag', MagneticField, pwk_mag_callback)
# rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pwk_pose_callback)
# while True:


# print('diff',zed_cam_heading)
rospy.spin()