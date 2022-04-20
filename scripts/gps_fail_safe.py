#!usr/bin/env python3
"""
A Node to listen to GPS1 and GPS2 and report on their gps status.
"""
import time
import rospy
from mavros_msgs.msg import GPSRAW
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import NavSatFix


class GpsFixMonitor:
    def __init__(self):
        self.gps1_fix = None
        self.gps2_fix = None
        self.RTK_FIX_NUMBER = 6
        self.GPS_FIX_TYPE = {
            0: "NO GPS",
            1: "NO FIX",
            2: "2D FIX",
            3: "3D FIX",
            4: "DGPS",
            5: "RTK FLOAT",
            6: "RTK FIXED",
            7: "STATIC",
            8: "PPP"
        }
        self.diagnose_pub = rospy.Publisher("/gps_fail_safe/gps_status", DiagnosticStatus, queue_size=2)
        self.rtk_fix_status_pub = rospy.Publisher("/gps_fail_safe/gps_rtk_status", String, queue_size=2)  # for Rviz
        rospy.Subscriber("/mavros/gpsstatus/gps1/raw", GPSRAW, self.gps1_callback)  # to see fix type
        rospy.Subscriber("/mavros/gpsstatus/gps2/raw", GPSRAW, self.gps2_callback)  # to see fix type
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)  # to see covariance
        rate = rospy.Rate(1)
        time.sleep(1)
        while not rospy.is_shutdown():
            if self.gps1_fix is None or self.gps2_fix is None:
                rospy.loginfo("Waiting GPS1 and GPS2 callback ")
            else:
                break
            rate.sleep()
        self.main_loop()

    def gps1_callback(self, data):
        self.gps1_fix = data.fix_type

    def gps2_callback(self, data):
        self.gps2_fix = data.fix_type

    def gps_callback(self, data):
        pass

    def main_loop(self):
        rospy.logdebug("main_loop started")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            diagnose_status_msg = DiagnosticStatus()
            # print("elf.gps1_fix", self.gps1_fix)
            # print("self.gps2_fix", self.gps2_fix)

            if self.gps1_fix == self.RTK_FIX_NUMBER and self.gps2_fix == self.RTK_FIX_NUMBER:
                diagnose_status_msg.level = diagnose_status_msg.OK
                diagnose_status_msg.name = "gps_fix_monitor"
                diagnose_status_msg.message = "Both RTK FIX"
                gps1_keyval = KeyValue()
                gps1_keyval.key = "GPS1"
                gps1_keyval.value = self.GPS_FIX_TYPE[self.gps1_fix]
                gps2_keyval = KeyValue()
                gps2_keyval.key = "GPS2"
                gps2_keyval.value = self.GPS_FIX_TYPE[self.gps2_fix]
                diagnose_status_msg.values.append(gps1_keyval)
                diagnose_status_msg.values.append(gps2_keyval)
            else:
                diagnose_status_msg.level = diagnose_status_msg.WARN
                diagnose_status_msg.name = "gps_fix_monitor"
                diagnose_status_msg.message = "NO RTK FIX"
                gps1_keyval = KeyValue()
                gps1_keyval.key = "GPS1"
                gps1_keyval.value = self.GPS_FIX_TYPE[self.gps1_fix]
                gps2_keyval = KeyValue()
                gps2_keyval.key = "GPS2"
                gps2_keyval.value = self.GPS_FIX_TYPE[self.gps2_fix]
                diagnose_status_msg.values.append(gps1_keyval)
                diagnose_status_msg.values.append(gps2_keyval)
            self.rtk_fix_status_pub.publish(diagnose_status_msg.message)
            self.diagnose_pub.publish(diagnose_status_msg)
            rospy.logdebug("published diagnose")
            del diagnose_status_msg, gps2_keyval, gps1_keyval
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('gps_fix_monitor')
    gps_mon = GpsFixMonitor()
    rospy.spin()
