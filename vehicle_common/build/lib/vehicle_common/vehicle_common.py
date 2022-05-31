#!/usr/bin/env python3
"""
A base class for vehicle information
"""
import rospy


class VehicleData:
    def __init__(self):
        self.wheel_base = rospy.get_param("/vehicle/wheel_base", 2)
        self.wheel_axle_length = rospy.get_param("/vehicle/wheel_axle_length", 1.2)
        self.minimum_turning_radius = rospy.get_param("/vehicle/minimum_turning_radius", 5)
        self.width = rospy.get_param("/vehicle/width", 1.5)
        self.length = rospy.get_param("/vehicle/length", 2.5)
        self.mass = rospy.get_param("/vehicle/mass", 500)
        self.front_axle_to_center_of_gravity = rospy.get_param("/vehicle/front_axle_to_center_of_gravity", 0.8)
        self.rear_axle_to_center_of_gravity = rospy.get_param("/vehicle/rear_axle_to_center_of_gravity", 1.2)

        self.max_forward_speed = rospy.get_param("/vehicle/max_forward_speed", 3)
        self.min_forward_speed = rospy.get_param("/vehicle/min_forward_speed", 0.3)
        self.max_backward_speed = rospy.get_param("/vehicle/max_backward_speed", -3)
        self.min_backward_speed = rospy.get_param("/vehicle/min_backward_speed", -0.3)
        self.max_acceleration = rospy.get_param('/vehicle/max_acceleration', 0.5)
        self.min_acceleration = rospy.get_param('/vehicle/min_acceleration', -0.5)
        self.max_steering_angle = rospy.get_param("/vehicle/max_steering_angle", 30)
        self.min_steering_angle = rospy.get_param("/vehicle/min_steering_angle", -30)
        self.gear_ratio = rospy.get_param("/vehicle/gear_ratio", 29)

    def view_data(self):
        print(
            f"wheel_base: {self.wheel_base}, wheel_axle_length: {self.wheel_axle_length}"

        )

