#!/usr/bin/env python3
import rospy
# from my_robot_common.vehicle_common import say_it_works
from vehicle_common.vehicle_common import VehicleData
from tf_helper.tf_helper import list_to_quaternion
from vehicle_common.vehicle_config import vehicle_data
if __name__ == '__main__':
    rospy.init_node('test_node')
    vehicle = VehicleData()
    print(vehicle.wheel_base)
    print("rear_cg", vehicle.rear_axle_to_center_of_gravity)
    print("turning radius ", vehicle.min_acceleration)
    vehicle.view_data()
    # print(vehicle.check())

    print(list_to_quaternion([1,2,4,4]))
