#!/usr/bin/env python3
"""
A node which manages failsafe, and stops the vehicle
"""
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray
import rospy
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive
import time
from pilot.msg import vehicle_stop_command
OK = DiagnosticStatus.OK
ERROR = DiagnosticStatus.ERROR
WARN = DiagnosticStatus.WARN

class FailSafeAutoPilot:
    def __init__(self):
        self.fail_status = None
        self.status = {}
        
        self.whitelist = rospy.get_param('/vehicle_safety/WHITE_LIST')
        self.whitelist = ["vehicle_safety_diagnostics: " + s for s in self.whitelist]
        self.blacklist = rospy.get_param('/vehicle_safety/BLACK_LIST')
        self.blacklist = ["vehicle_safety_diagnostics: " + s for s in self.blacklist]
        use_vehicle_safety = rospy.get_param("/failsafe_manager/vehicle_safety_diagnostics", True)
        self.stop_cmd_publisher = rospy.Publisher("/vehicle/stop_command", vehicle_stop_command , queue_size=10) 
        if use_vehicle_safety:
            rospy.Subscriber("/vehicle_safety_diagnostics", DiagnosticArray, self.vehicle_safety_diagnose_cb)
        self.stop_command_data = vehicle_stop_command()

    def vehicle_safety_diagnose_cb(self, data):
        reason = []
        for field in data.status:
            if field.name in self.whitelist and field.name not in self.blacklist:
                if field.level == ERROR:
                    self.status.update({field.name:True})
                    reason.append(field.name)
                else:
                    self.status.update({field.name:False})
            else:
                # print("Black list",field.name,field.level)
                pass
        # print(self.status.values())
        if any(self.status.values()):
            self.fail_status = True
        else:
            self.fail_status = False
        
        if self.fail_status == True:
            rospy.logerr("Stopping the Vehicle")
            self.stop_command_data.node = rospy.get_name()
            self.stop_command_data.message = "ERROR : {}".format(reason)
            self.stop_command_data.status = True
            rospy.logerr(self.stop_command_data.message)
            
        else:
            rospy.loginfo("Running")
            self.stop_command_data.node = rospy.get_name()
            self.stop_command_data.message = "OK"
            self.stop_command_data.status = False
            # rospy.loginfo("OK from Vehicle Safety")

        self.stop_cmd_publisher.publish(self.stop_command_data)
        self.status = {}

if __name__ == "__main__":
    rospy.init_node('vehicle_safety_analyzer')
    fs = FailSafeAutoPilot()
    rospy.spin()
    
