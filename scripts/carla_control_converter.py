#!/usr/bin/env python3
import rospy
from carla_msgs.msg import CarlaEgoVehicleControl
from ackermann_msgs.msg import AckermannDrive
import math


def ackremann_callback(data):
    rospy.loginfo_once("/cmd_drive/pure_pursuit data received" )
    global carla_msg_pub, carla_msg
    if data.speed < 0:
        carla_msg.throttle = abs(data.speed)
        carla_msg.reverse = True
    else:
        carla_msg.throttle = data.speed
        carla_msg.reverse = False

    carla_msg.steer = math.radians(data.steering_angle)
    carla_msg.brake = data.jerk
    carla_msg_pub.publish(carla_msg)


if __name__ == "__main__":
    rospy.init_node('carla_control_converter')
    carla_msg_pub = rospy.Publisher("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, queue_size=2)
    carla_msg = CarlaEgoVehicleControl()
    rospy.Subscriber("/cmd_drive/pure_pursuit", AckermannDrive, ackremann_callback)
    rospy.loginfo("carla_control_converter started ")
    rospy.spin()
