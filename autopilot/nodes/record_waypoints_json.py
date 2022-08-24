#!/usr/bin/env python3
try:
    import rospy
    import rospkg
    import sys
    import math
    from sensor_msgs.msg import NavSatFix, MagneticField, Imu
    from nav_msgs.msg import Odometry
    from geographic_msgs.msg import GeoPointStamped
    from mavros_msgs.msg import HomePosition
    from json import dump
    from geojson import LineString
    import numpy as np
    import time
    from rospy_message_converter import message_converter
    from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray

except Exception as e:
    print("No module named", str(e))
    exit()

# TODO
'''
1. Depending upon the accuracy of the first gps location we need to decide on recording points.
2. Warn user when, high speed drive.
3. Warn user when, gps data is less accurate.
4. when gps speed is less call a service call to speed up the sensor data
'''

# TODO
"""
do some thing when time from gps/odom
"""

OK = DiagnosticStatus.OK
ERROR = DiagnosticStatus.ERROR
WARN = DiagnosticStatus.WARN

def get_distance(lat1, lon1, lat2, lon2):
    r = 6371.0 * 1000.0
    lat_start = np.radians(lat1)
    lon_start = np.radians(lon1)
    lat_end = np.radians(lat2)
    lon_end = np.radians(lon2)
    dLat = lat_end - lat_start
    dLon = lon_end - lon_start
    a = np.sin(dLat / 2.0) * np.sin(dLat / 2.0) + np.cos(lat_start) * np.cos(lat_end) * np.sin(dLon / 2.0) * np.sin(
        dLon / 2.0)
    c = 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    d = c * r
    return d


class SaveWayPoints:
    def __init__(self, mission_file_dir):

        self.mission_file_dir = mission_file_dir
        self.prev_lat = 0
        self.prev_long = 0
        self.is_first_point = True
        self.final_waypoints_list = []
        self.imu_list = []
        self.odometry_list = []
        self.gps_list = []
        self.heading = 0
        self.odom_data = None
        self.odom_data_msg = None
        self.imu_data = None
        self.gps_data = None
        self.gps_data_msg = None
        self.time_at_odom = None
        self.time_at_gps = None
        self.wait_time_limit = 1
        self.RTK_fail_status  = None

        # IMP PARAMS
        self.min_dis_between_waypoints = rospy.get_param('/patrol/min_dis_between_waypoints', 0.1)
        gps_topic = rospy.get_param('/patrol/gps_topic', "/mavros/global_position/global")
        imu_topic = rospy.get_param('/patrol/imu_topic', "/mavros/imu/data")
        odom_topic = rospy.get_param('/patrol/odom_topic', "/mavros/global_position/local")
        data = None
        # TODO
        # wait for odom
        while not rospy.is_shutdown():
            try:
                data = rospy.wait_for_message(gps_topic, NavSatFix, timeout=2)
            except:
                data = None
                rospy.loginfo("Waiting for " + str(gps_topic))
            if data:
                rospy.Subscriber(gps_topic, NavSatFix, self.gps_callback)
                rospy.Subscriber(imu_topic, Imu, self.imu_callback)
                rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
                rospy.Subscriber("/vehicle_safety_diagnostics", DiagnosticArray, self.vehicle_safety_diagnose_cb)
                self.starting_point_pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped,
                                                          queue_size=10, latch=True)
                self.home_position_pub = rospy.Publisher('/mavros/global_position/home', HomePosition,
                                                          queue_size=10, latch=True)
                break
        # time.sleep(1)
        self.main_loop()
        if len(self.final_waypoints_list) == 0:
            rospy.logwarn('No points to save, Exiting without saving')
            exit()

        line_string = LineString(self.final_waypoints_list)
        line_string['imu'] = self.imu_list
        line_string['odometry'] = self.odometry_list
        line_string['gps_coordinates'] = self.gps_list
        with open(mission_file_dir, 'w') as f:
            dump(line_string, f)
        print(" ")
        rospy.loginfo("saved %s points to %s", str(len(self.final_waypoints_list)), self.mission_file_dir)

    def odom_callback(self, data):
        self.odom_data_msg = data
        self.odom_data = message_converter.convert_ros_message_to_dictionary(data)
        self.time_at_odom = time.time()

    def imu_callback(self, data):
        self.imu_data = message_converter.convert_ros_message_to_dictionary(data)

    def gps_callback(self, data):
        self.gps_data_msg = data
        self.gps_data = message_converter.convert_ros_message_to_dictionary(data)
        self.time_at_gps = time.time()
    
    def vehicle_safety_diagnose_cb(self, data):
        for field in data.status:
            if field.name == "vehicle_safety_diagnostics: GPS":
                if field.level == ERROR:
                    self.RTK_fail_status = True
                else:
                    self.RTK_fail_status = False
    def main_loop(self):
        now = time.time()
        r = rospy.Rate(50)
        try:
            while not rospy.is_shutdown():
                if not self.RTK_fail_status:
                    if self.is_first_point:
                        if self.gps_data_msg and self.odom_data and self.imu_data:
                            # TODO
                            #  Depending upon the accuracy(covariance of gps) of the first gps location we need to decide on
                            #  recording points if gps_data_msg.position_covariance[0]
                            # https://en.wikipedia.org/wiki/Standard_error
                            rospy.loginfo("Origin gps location:- longitude: %s latitude: %s altitude: %s covariance:%s",
                                        str(self.gps_data_msg.longitude), str(self.gps_data_msg.latitude),
                                        str(self.gps_data_msg.altitude), str(self.gps_data_msg.position_covariance))
                            self.final_waypoints_list.append([self.gps_data_msg.longitude, self.gps_data_msg.latitude])
                            self.imu_list.append(self.imu_data)
                            odom_msg = Odometry()
                            odom_msg.pose.pose.orientation = self.odom_data_msg.pose.pose.orientation
                            self.odometry_list.append(
                                message_converter.convert_ros_message_to_dictionary(odom_msg))  # change it to self.odom_msg
                            self.gps_list.append(self.gps_data)
                            self.prev_long = self.gps_data_msg.longitude
                            self.prev_lat = self.gps_data_msg.latitude
                            geo_point = GeoPointStamped()
                            geo_point.position.latitude = self.gps_data_msg.latitude
                            geo_point.position.longitude = self.gps_data_msg.longitude
                            geo_point.position.altitude = self.gps_data_msg.altitude  # 29 # change it to actual latitude
                            self.starting_point_pub.publish(geo_point)
                            home_position_msg = HomePosition()
                            home_position_msg.geo = geo_point.position
                            self.home_position_pub.publish(home_position_msg)

                            rospy.loginfo('Origin point set')
                            rospy.loginfo('Home location was saved, drive the vehicle')
                            self.is_first_point = False
                            time.sleep(0.5)
                        else:
                            rospy.logwarn("Waiting for data")
                            r.sleep()
                            continue
                    else:
                        now = time.time()
                        if now - self.time_at_gps > self.wait_time_limit:
                            rospy.logwarn('Time out from GPS: %s secs', str(now - self.time_at_gps))
                            r.sleep()
                            continue
                        if now - self.time_at_odom > self.wait_time_limit:
                            rospy.logwarn('Time out from Odometry: %s secs', str(now - self.time_at_odom))
                            r.sleep()
                            continue
                        dis = get_distance(self.gps_data_msg.latitude, self.gps_data_msg.longitude, self.prev_lat,
                                        self.prev_long)
                        if dis >= self.min_dis_between_waypoints:
                            print("\rSaved: " + str(round(self.gps_data_msg.longitude, 5)) + " " + str(
                                round(self.gps_data_msg.latitude, 5)) + ", saved points count :" + str(
                                len(self.final_waypoints_list)) + "  ", end="\t")
                            # print(len(self.final_waypoints_list))
                            self.final_waypoints_list.append([self.gps_data_msg.longitude, self.gps_data_msg.latitude])
                            self.imu_list.append(self.imu_data)
                            self.odometry_list.append(self.odom_data)
                            self.gps_list.append(self.gps_data)
                            self.prev_long = self.gps_data_msg.longitude
                            self.prev_lat = self.gps_data_msg.latitude
                        else:
                            #  distance less than min_dis_between_waypoints
                            pass
                else:
                    rospy.logwarn("NO RTK, not saving PATH")
                r.sleep()
        except Exception as e:
            print('exception', e)
            pass


if __name__ == "__main__":
    rospy.init_node('way_point_saver_node')
    mission_file = rospy.get_param('/patrol/mission_file', 'default.json')
    if '.json' in mission_file:
        pass
    else:
        mission_file = mission_file + '.json'
        rospy.set_param('/patrol/mission_file', mission_file)
    try:
        ros_pack = rospkg.RosPack()
        mission_file_dir = ros_pack.get_path('autopilot') + "/mission_files/" + str(mission_file)
    except Exception as e:
        rospy.logwarn("Please source autopilot package" + str(e))
        sys.exit()
    save_points = SaveWayPoints(mission_file_dir)
    rospy.spin()
