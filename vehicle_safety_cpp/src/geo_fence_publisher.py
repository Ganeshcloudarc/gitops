#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import geometry_msgs.msg as gmsg
from autopilot_utils.geonav_conversions import *
from geographic_msgs.msg import GeoPointStamped
from vehicle_safety_cpp.srv import trigger,triggerResponse  
#srv on trigger on the updated coordinates in publishment


class geoFencePub:
    def __init__(self):
        self.funcall()
        self.home_gps_location = False 
        self.use_geo_fence = rospy.get_param(
            '/vehicle_safety/use_geo_fence', False)
        rospy.logdebug("service created")       
        self.geo_fence_coordinates = rospy.get_param(
            '/vehicle_safety/geo_fence_coordinates') 
        if self.use_geo_fence: 
            self.geofence_polygon_pub = rospy.Publisher(
                '/geo_fence_polygon', gmsg.PolygonStamped, queue_size=1, latch=True)
        else: 
            rospy.logerr(f"use_geo_fence: {self.use_geo_fence}") 
            rospy.signal_shutdown(f"use_geo_fence is: {self.use_geo_fence}")
        self.nogozone_polygon_pub = rospy.Publisher(
            '/geo_fence_polygon_2', gmsg.PolygonStamped, queue_size=1, latch=True)
        rospy.Subscriber('/mavros/global_position/set_gp_origin', GeoPointStamped,
                             self.home_position_callback)
        self.no_go_geo_fence_coordinates = None
                
    def home_position_callback(self, data):
        if rospy.has_param('/vehicle_safety/no_go_zone_coordinates'):
            self.no_go_geo_fence_coordinates = rospy.get_param('/vehicle_safety/no_go_zone_coordinates')
        self.home_gps_location = {
        'latitude': data.position.latitude,
        'longitude': data.position.longitude,
        'altitude': data.position.altitude
        }
        rospy.logdebug("home position data received %s",
                    str(self.home_gps_location)) 
        self.publish_geo_fence()

    #service call no updating the no_go_zone_coordinates 
    def service_request(self,req): 
        
        if rospy.has_param('/vehicle_safety/no_go_zone_coordinates'):
            self.no_go_geo_fence_coordinates = rospy.get_param('/vehicle_safety/no_go_zone_coordinates')
            rospy.loginfo("updating the new params in geofence") 
            self.publish_geo_fence() 
            rospy.loginfo("updated")
            return triggerResponse(response = 1)
            
        
        

    def publish_geo_fence(self):
        
        if not self.home_gps_location:
            return False
        else:
            polygon_st = gmsg.PolygonStamped()
            polygon_st.header.frame_id = 'map'
            home_lat = self.home_gps_location['latitude']
            home_long = self.home_gps_location['longitude']

            for lat, lon in self.geo_fence_coordinates:
                point = gmsg.Point()
                point.x, point.y = ll2xy(lat, lon, home_lat, home_long)
                polygon_st.polygon.points.append(point)
            self.geofence_polygon_pub.publish(polygon_st)
            rospy.logdebug("published Geofence")
            
            polygon_st = gmsg.PolygonStamped() #reset the polygon
            polygon_st.header.frame_id = 'map'
            
            # print(no_go_geo_fence_coordinates[0])
            if self.no_go_geo_fence_coordinates is not None: #None = param is not there. 
                for i in self.no_go_geo_fence_coordinates:
                    print(i)
                    for j in i:
                        lat, lon = j[0], j[1]
                        # print(lat, lon) 
                        point = gmsg.Point()
                        point.x, point.y = ll2xy(lat, lon,home_lat , home_long)
                        polygon_st.polygon.points.append(point)
                    self.nogozone_polygon_pub.publish(polygon_st)
                rospy.logdebug("Published No Go Zone")

    def funcall(self):  
        rospy.logdebug("service started :)") 
        rospy.Service('no_go_geo_fence_coordinates',trigger,self.service_request)



if __name__ == "__main__":
    rospy.init_node('geo_fence_publisher')
    fs = geoFencePub()

    rospy.spin()