#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import geometry_msgs.msg as gmsg
from autopilot_utils.geonav_conversions import *
from geographic_msgs.msg import GeoPointStamped

class geoFencePub:
    def __init__(self):
        self.home_gps_location = False
        self.geo_polygon_pub = rospy.Publisher(
            '/geo_fence_polygon_2', gmsg.PolygonStamped, queue_size=1, latch=True)
        rospy.Subscriber('/mavros/global_position/set_gp_origin', GeoPointStamped,
                             self.home_position_callback)
        if rospy.has_param('/vehicle_safety/no_go_zone_coordinates'):
            self.no_go_geo_fence_coordinates = rospy.get_param('/vehicle_safety/no_go_zone_coordinates')
                
    def home_position_callback(self, data):
            self.home_gps_location = {
            'latitude': data.position.latitude,
            'longitude': data.position.longitude,
            'altitude': data.position.altitude
            }
            rospy.logdebug("home position data received %s",
                        str(self.home_gps_location))
            self.publish_geo_fence()

    def publish_geo_fence(self):
        
        if not self.home_gps_location:
            return False
        else:
            polygon_st = gmsg.PolygonStamped()
            polygon_st.header.frame_id = 'map'
            home_lat = self.home_gps_location['latitude']
            home_long = self.home_gps_location['longitude']

            # print(no_go_geo_fence_coordinates[0])
            for i in self.no_go_geo_fence_coordinates:
                print(i)
                for j in i:
                    lat, lon = j[0], j[1]
                    # print(lat, lon)
                    point = gmsg.Point()
                    point.x, point.y = ll2xy(lat, lon,home_lat , home_long)
                    polygon_st.polygon.points.append(point)
                self.geo_polygon_pub.publish(polygon_st)
            print("Published")

if __name__ == "__main__":
    rospy.init_node('geo_fence_publisher')
    fs = geoFencePub()
    rospy.spin()