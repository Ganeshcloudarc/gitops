#!/usr/bin/env python3
import rospy
from carla_msgs.msg import CarlaEgoVehicleControl
from ackermann_msgs.msg import AckermannDrive
import math


def ackremann_callback(data):
    rospy.loginfo_once("/cmd_drive/pure_pursuit data received" )
    global carla_msg_pub, carla_ack_msg_pub
    '''
    carla_msg =CarlaEgoVehicleControl()
    if data.speed < 0:
        carla_msg.throttle = abs(data.speed)
        carla_msg.reverse = True
    else:
        carla_msg.throttle = data.speed
        carla_msg.reverse = False
    if data.speed == 0:
        carla_msg.brake = 1
    else:
        data.speed = 0.3

    if data.jerk:
        carla_msg.brake = data.jerk
    carla_msg.steer = math.radians(data.steering_angle)
    # carla_msg.brake = data.jerk
    carla_msg_pub.publish(carla_msg)
    '''
    data.steering_angle = -math.radians(data.steering_angle)
    data.acceleration = 0.5
    carla_ack_msg_pub.publish(data)



if __name__ == "__main__":
    rospy.init_node('carla_control_converter')
    carla_msg_pub = rospy.Publisher("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, queue_size=2)
    carla_ack_msg_pub = rospy.Publisher("/carla/ego_vehicle/ackermann_cmd",AckermannDrive, queue_size=1)
    carla_msg = CarlaEgoVehicleControl()
    failsafe_enable = rospy.get_param("/patrol/failsafe_enable", True)

    if failsafe_enable:
        cmd_topic = rospy.get_param("patrol/cmd_topic", "pure_pursuit/cmd_drive")
    else:
        cmd_topic = rospy.get_param("patrol/pilot_cmd_in", "/vehicle/cmd_drive_safe")
    rospy.Subscriber(cmd_topic, AckermannDrive, ackremann_callback)
    rospy.loginfo("carla_control_converter started ")
    rospy.spin()
