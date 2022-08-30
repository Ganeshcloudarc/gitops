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
        self.fail_status = None
        self.status = []
        self.whitelist = rospy.get_param('/vehicle_safety/WHITE_LIST')
        self.whitelist = ["vehicle_safety_diagnostics: " + s for s in self.whitelist]
        self.blacklist = rospy.get_param('/vehicle_safety/BLACK_LIST')
        self.blacklist = ["vehicle_safety_diagnostics: " + s for s in self.blacklist]
        print(self.whitelist)
        print(self.blacklist)
        use_vehicle_safety = rospy.get_param("/failsafe_manager/vehicle_safety_diagnostics", True) 
        if use_vehicle_safety:
            rospy.Subscriber("/vehicle_safety_diagnostics", DiagnosticArray, self.vehicle_safety_diagnose_cb)


    def vehicle_safety_diagnose_cb(self, data):
        for field in data.status:
            if field.name in self.whitelist and field.name not in self.blacklist:
                if field.level == ERROR:
                    self.status.append(True)
                else:
                    self.status.append(False)
            else:
                # print("Black list",field.name,field.level)
                pass
        if any(self.status):
            self.fail_status = True
            print(True)
        else:
            self.fail_status = False
            print(False)
        self.status = []
    
    def send_stop_command(self):
        if self.fail_status:
            print("Stopping the Vehicle")
        else:
            pass
        
if __name__ == "__main__":
    rospy.init_node('failsafe_intigator')
    fs = FailSafeAutoPilot()
    rospy.spin()
    
