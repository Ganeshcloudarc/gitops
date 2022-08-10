#!/usr/bin/env python3
"""
A node which manages failsafe, and stops the vehicle
"""
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray
import rospy
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive
import time
OK = DiagnosticStatus.OK
ERROR = DiagnosticStatus.ERROR
WARN = DiagnosticStatus.WARN


class FailSafeAutoPilot:
    def __init__(self):
        self.time_on_obj = None
        self.time_to_wait_after_obstacle = rospy.get_param("time_to_wait_after_obstacle", 5) # in secs
        # publishers
        pilot_cmd_in = rospy.get_param("/patrol/pilot_cmd_in", "/vehicle/cmd_drive_safe")
        self.cmd_pulisher = rospy.Publisher(pilot_cmd_in, AckermannDrive, queue_size=1)


        # subscribers
        cmd_topic = rospy.get_param("patrol/cmd_topic","pure_pursuit/cmd_drive")
        rospy.Subscriber(cmd_topic, AckermannDrive, self.vehicle_cmd_cb)
        self.status_dict = {}
        use_zed = rospy.get_param("/failsafe_manager/use_zed_camera", True)
        if use_zed:
            self.status_dict['zed_obstacle_exist_status'] = False
            rospy.Subscriber("/zed_obj_status", Bool, self.zed_obstacle_status_cb, 'zed_obstacle_exist_status')

        use_oak = rospy.get_param("/failsafe_manager/use_oak_camera", False)
        if use_oak:
            self.status_dict['oak_obstacle_exist_status'] = False
            rospy.Subscriber("/oak_obj_status", Bool, self.oak_obstacle_status_cb, 'oak_obstacle_exist_status')

        use_vehicle_safety = rospy.get_param("/failsafe_manager/vehicle_safety_diagnostics", True) 
        if use_vehicle_safety:
            self.status_dict['vehicle_safety_status'] = False
            rospy.Subscriber("/vehicle_safety_diagnostics", DiagnosticArray, self.vehicle_safety_diagnose_cb, 'vehicle_safety_status')


    def vehicle_safety_diagnose_cb(self, data, key):
        RTK_fail_status, emergency_stop_pressed, out_of_geofence, CTE_fail_status = None, None, None, None
        for field in data.status:
            if field.name == "vehicle_safety_diagnostics: GPS":
                if field.level == ERROR:
                    RTK_fail_status = True
                else:
                    RTK_fail_status = False
            if field.name == "vehicle_safety_diagnostics: Emergency":
                if field.level == ERROR:
                    emergency_stop_pressed = True
                else:
                    emergency_stop_pressed = False
            if field.name == "vehicle_safety_diagnostics: GeoFence":
                if field.level == ERROR:
                    out_of_geofence = True
                else:
                    out_of_geofence = False
            if field.name == "vehicle_safety_diagnostics: CTE":
                if field.level == ERROR:
                    CTE_fail_status = True
                    rospy.logerr("CTE_fail_status")
                else:
                    CTE_fail_status = False
                    
        if RTK_fail_status or emergency_stop_pressed or out_of_geofence or CTE_fail_status:
            self.status_dict[key] = True
        else:
            self.status_dict[key] = False

    def zed_obstacle_status_cb(self, data, key):
        self.status_dict[key] = data.data
    
    def oak_obstacle_status_cb(self, data, key):
        self.status_dict[key] = data.data

        
    def vehicle_cmd_cb(self, data):
        if True in self.status_dict.values():
            rospy.logwarn("vehicle stop command sent")
            rospy.logwarn(self.status_dict)
            self.cmd_pulisher.publish(AckermannDrive(steering_angle =data.steering_angle,jerk=1))
        else:
            rospy.loginfo("Forwarding the commands")
            rospy.loginfo(self.status_dict)
            self.cmd_pulisher.publish(data)

if __name__ == "__main__":
    rospy.init_node('failsafe_intigator')
    fs = FailSafeAutoPilot()
    rospy.spin()
    
