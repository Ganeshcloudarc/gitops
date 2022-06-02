#!/usr/bin/env python3
"""
A node which manages failsafe, and stops the vehicle
"""
import rospy
from std_msgs.msg import Bool
from zed_interfaces.msg import ObjectsStamped
from ackermann_msgs.msg import AckermannDrive
from mavros_msgs.msg import GPSRAW
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from zed_interfaces.msg import ObjectsStamped
from std_msgs.msg import Bool
from autopilot_msgs.msg import ControllerDiagnose
import rosnode

# rosnode.get_node_names()
'''

gps failsafe
cte failsafe
emergency stop
object detection
sensor update and fault monitoring
heading failsafe
speed failsafe

ERROR CODES

## NEXT
Nodes nomonitoring: ensuring proper kill and exit from the launch files
ros without network
NRU memory handling

'''


class FailSafeAutoPilot:
    def __init__(self):
        # defining subscribers for all the nessesary topics
        self.emergency_stop = None
        self.pp_diagnose_data = None
        self.odom_data = None
        self.main_gps_data = None
        self.gps2_data = None
        self.gps1_data = None
        rospy.Subscriber("/mavros/gpsstatus/gps1/raw", GPSRAW, self.gps1_callback)  # to see fix type
        rospy.Subscriber("/mavros/gpsstatus/gps2/raw", GPSRAW, self.gps2_callback)  # to see fix type
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.main_callback)  # to see covariance
        cam_name = rospy.get_param("camera_name", "zed2i")
        # rospy.Subscriber(f"{cam_name}/zed_node/obj_det/objects", ObjectsStamped, self.obj_callback)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/pure_pursuit_diagnose", ControllerDiagnose, self.path_track_diagnose_callback)
        emergency_stop_topic = rospy.get_param('/failsafe/emergency_stop_topic')
        rospy.Subscriber(emergency_stop_topic, Bool, self.emergency_callback)

        self.main_loop()

    # callback functions
    def gps1_callback(self, data):
        self.gps1_data = data

    def gps2_callback(self, data):
        self.gps2_data = data

    def main_callback(self, data):
        self.main_gps_data = data

    def odom_callback(self, data):
        self.odom_data = data

    def path_track_diagnose_callback(self, data):
        self.pp_diagnose_data = data

    def emergency_callback(self, data):
        self.emergency_stop = data.data

    # failsafe functions
    def gps_failsafe(self):
        pass

    def rtk_failsafe(self):
        pass

    def cte_failsafe(self):
        pass

    def emergency_stop_failsafe(self):
        pass

    def sensor_update_fault_monitoring(self):
        pass

    def object_detection_failsafe(self):
        pass

    def heading_failsafe(self):
        pass

    def main_loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            pass
