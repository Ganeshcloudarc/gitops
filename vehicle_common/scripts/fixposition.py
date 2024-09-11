#!/usr/bin/python3
try:
    import rospy
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import Header,Float64
    from fixposition_driver_ros1.msg import gphdt,llh,odomenu,gpgga,odometry
    from nav_msgs.msg import Odometry 
    from geographic_msgs.msg import GeoPointStamped
    from mavros_msgs.msg import GPSRAW
    from pyproj import Proj, transform
    from autopilot_utils.geonav_conversions import xy2ll, ll2xy
    import requests
except ImportError as e:
    print("No module named ".format(e))
#fixposition driver

class visionConverter:
    def __init__(self):
        self.odomerty_msg=None
        self.navsatfix_msg=None
        self.home_pose_lat=None
        self.home_pose_lon=None
        self.lat=None
        self.lon=None
        self.xy_points = None
        self.gps1_fixtype = None
        self.gps2_fixtype = None
        self.is_home_pose_published = False
        self.satellite_visible = None
        self.hdop_value = None
        self.correct_heading = None
        self.ip=rospy.get_param("/fixposition_driver_ros1/fp_output/ip")
        self.is_enu_mode = rospy.get_param('/fixposition_enu_mode')

        # Subscriber to fixposition  topics
        rospy.Subscriber('/fixposition/fpa/odomenu', odomenu, callback=self.odometry_msgs)
        rospy.Subscriber('/fixposition/fpa/llh',llh,callback=self.llh_data)
        rospy.Subscriber('/mavros/global_position/set_gp_origin',GeoPointStamped,callback=self.home_position_cb)
        rospy.Subscriber('/fixposition/nmea/gphdt',gphdt,callback=self.heading_data)
        rospy.Subscriber('/fixposition/gnss1',NavSatFix,callback=self.gps1_data)
        rospy.Subscriber('/fixposition/gnss1',NavSatFix,callback=self.gps2_data)
        rospy.Subscriber('/fixposition/nmea/gpgga',gpgga,callback=self.satellite_data)
        

        # Publisher for mavros topic
        self.odom_pub = rospy.Publisher('/fixposition/odometry', Odometry, queue_size=1)
        self.llh_pub = rospy.Publisher('/fixposition/global_position/global',NavSatFix,queue_size=1)
        self.home_pose_publisher = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=10, latch=True)
        self.heading_publisher = rospy.Publisher('/fixposition/global_position/compass_hdg',Float64,queue_size=1)
        self.gps1_publisher = rospy.Publisher('/fixposition/gpsstatus/gps1/raw',GPSRAW,queue_size=1)
        self.gps2_publisher = rospy.Publisher('/fixposition/gpsstatus/gps2/raw',GPSRAW,queue_size=1)
        


    def home_position_cb(self,data):
        # set the home location
        self.home_pose_lat = data.position.latitude
        self.home_pose_lon = data.position.longitude
        self.home_pose_alt = data.position.altitude
    

        # Define the WGS84 (LLA) and ECEF coordinate systems
        lla_proj = Proj(proj="latlong", datum="WGS84")
        ecef_proj = Proj(proj="geocent", datum="WGS84")

        # Example coordinates in LLA (Latitude, Longitude, Altitude)
        lat, lon, alt = self.home_pose_lat, self.home_pose_lon, self.home_pose_alt  # Coordinates for the Eiffel Tower
        
              # Convert from LLA to ECEF
        x, y, z = transform(lla_proj, ecef_proj, lon, lat, alt)

    
        headers = {
            'Content-Type': 'application/json'
        }

        json_data = {
            'params': {
                'io': {
                    'tf': {
                        'enu0_mode': 'auto',
                    }
                }
            }
        }

        try:
            response = requests.post(f'http://{self.ip}/api/v2/params/config/set', headers=headers, json=json_data)
            response.raise_for_status()  # Check for HTTP request errors
        except requests.RequestException as e:
            rospy.logerr(f"Failed to send request: {e}")
    def llh_data(self, data):
        # Retrieve lat, lon, and alt
        lat = data.position.x
        lon = data.position.y
        alt = data.position.z
        

        if not self.is_home_pose_published:
            geo_point = GeoPointStamped()
            geo_point.position.latitude = lat
            geo_point.position.longitude = lon
            geo_point.position.altitude = alt  # or set to 0 if altitude is not needed

            self.home_pose_publisher.publish(geo_point)
            self.is_home_pose_published = True
        if self.home_pose_lat is not None:
            self.xy_points = ll2xy(lat, lon, self.home_pose_lat, self.home_pose_lon)
            
           
                

        # Create a NavSatFix message
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header = Header()
        navsatfix_msg.header.stamp = rospy.Time.now()
        navsatfix_msg.header.frame_id = "gps_link"

        # Set latitude, longitude, and altitude
        navsatfix_msg.latitude = lat
        navsatfix_msg.longitude = lon
        navsatfix_msg.altitude = alt
        covariance_7 = data.covariance[7] if data.covariance[7] >= 0 else 0.0

        # Set position covariance and type
        navsatfix_msg.position_covariance = [
            data.covariance[0], data.covariance[1], data.covariance[2],
            data.covariance[6], covariance_7, data.covariance[8],
            data.covariance[12], data.covariance[13], data.covariance[14]
        ]
        navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        # Publish the message
        self.llh_pub.publish(navsatfix_msg)
    
        
     
    
    
    def odometry_msgs(self, data):
        # Create an Odometry message
        odometry_msg = Odometry()
        odometry_msg.header = Header()
        odometry_msg.header.stamp = rospy.Time.now()  # Set the current time
        odometry_msg.header.frame_id = "odom"  # Example frame_id, change as needed
        odometry_msg.child_frame_id = "base_link"
        
        if self.is_enu_mode:
            odometry_msg.pose.pose.position.x= data.pose.pose.position.x
            odometry_msg.pose.pose.position.y= data.pose.pose.position.y
        else:
            if self.xy_points is not None:
                odometry_msg.pose.pose.position.x = self.xy_points[0]
                odometry_msg.pose.pose.position.y = self.xy_points[1]
            else:
                rospy.logwarn("XY points are None, cannot set odometry position.")
            
        odometry_msg.pose.pose.position.z= data.pose.pose.position.z
        odometry_msg.pose.pose.orientation.x = data.pose.pose.orientation.x
        odometry_msg.pose.pose.orientation.y = data.pose.pose.orientation.y
        odometry_msg.pose.pose.orientation.z = data.pose.pose.orientation.z
        odometry_msg.pose.pose.orientation.w = data.pose.pose.orientation.w
        odometry_msg.pose.covariance = data.pose.covariance
        odometry_msg.twist.twist.linear.x=data.velocity.twist.linear.x
        odometry_msg.twist.twist.linear.y=data.velocity.twist.linear.y
        odometry_msg.twist.twist.linear.z=data.velocity.twist.linear.z
        odometry_msg.twist.twist.angular.x=data.velocity.twist.angular.x
        odometry_msg.twist.twist.angular.y=data.velocity.twist.angular.y
        odometry_msg.twist.twist.angular.z=data.velocity.twist.angular.z
        odometry_msg.twist.covariance=data.velocity.covariance
        
        self.gps1_fixtype = data.gnss1_status
        self.gps2_fixtype = data.gnss2_status

        self.odom_pub.publish(odometry_msg)
    

    def heading_data(self,data):
        heading=round(data.heading)
        self.heading_publisher.publish(heading)

    def satellite_data(self,data):
        self.satellite_visible = data.num_sv
        self.hdop_value = data.hdop
        

    def gps1_data(self,data):
        gps1_raw = GPSRAW()
        gps1_raw.header.frame_id = 'gps1 '
        gps1_raw.header.stamp = rospy.Time.now()
        gps1_raw.lat = int(data.latitude*1e7)
        gps1_raw.lon = int(data.longitude*1e7)
        gps1_raw.alt = int(data.altitude*1e3)
        uere = 0.35

        if self.gps1_fixtype == 8:
            try:
                gps1_raw.fix_type = gps1_raw.GPS_FIX_TYPE_RTK_FIXED
            except AttributeError:
                gps1_raw.fix_type = gps1_raw.GPS_FIX_TYPE_RTK_FIXEDR
        elif self.gps1_fixtype == 7:
            try:
                gps1_raw.fix_type = gps1_raw.GPS_FIX_TYPE_RTK_FLOAT
            except AttributeError:
                gps1_raw.fix_type = gps1_raw.GPS_FIX_TYPE_RTK_FLOATR
        elif self.gps1_fixtype == 5 :
            gps1_raw.fix_type = gps1_raw.GPS_FIX_TYPE_3D_FIX
        elif self.gps1_fixtype == 6 :
            gps1_raw.fix_type = gps1_raw.GPS_FIX_TYPE_DGPS
        else:
            gps1_raw.fix_type = gps1_raw.GPS_FIX_TYPE_NO_FIX
            
        if self.hdop_value is not None:
            gps1_raw.h_acc = int(float(self.hdop_value * 1e2) * float(uere))  # h_acc = HDOP * UERE
        else:
            rospy.logwarn("HDOP value is None, cannot calculate h_acc.")
            gps1_raw.h_acc = 0

        gps1_raw.satellites_visible = int(self.satellite_visible)
        self.gps1_publisher.publish(gps1_raw)

    def gps2_data(self,data):
       uere = 0.35
       gps2_raw = GPSRAW()
       gps2_raw.header.frame_id = 'gps2'
       gps2_raw.header.stamp = rospy.Time.now()
       gps2_raw.lat = int(data.latitude*1e7)
       gps2_raw.lon = int(data.longitude*1e7)
       gps2_raw.alt = int(data.altitude*1e3)

       if self.gps2_fixtype == 8:
            try:
               gps2_raw.fix_type =gps2_raw.GPS_FIX_TYPE_RTK_FIXED
            except AttributeError:
               gps2_raw.fix_type =gps2_raw.GPS_FIX_TYPE_RTK_FIXEDR
       elif self.gps2_fixtype == 7:
            try:
               gps2_raw.fix_type =gps2_raw.GPS_FIX_TYPE_RTK_FLOAT
            except AttributeError:
               gps2_raw.fix_type =gps2_raw.GPS_FIX_TYPE_RTK_FLOATR
       elif self.gps2_fixtype == 5 :
           gps2_raw.fix_type =gps2_raw.GPS_FIX_TYPE_3D_FIX
       elif self.gps2_fixtype == 6 :
           gps2_raw.fix_type =gps2_raw.GPS_FIX_TYPE_DGPS
       else:
           gps2_raw.fix_type =gps2_raw.GPS_FIX_TYPE_NO_FIX
           
       if self.hdop_value is not None:
            gps2_raw.h_acc = int(float(self.hdop_value * 1e2) * float(uere))  # h_acc = HDOP * UERE
       else:
            rospy.logwarn("HDOP value is None, cannot calculate h_acc.")
            gps2_raw.h_acc = 0


       gps2_raw.satellites_visible = int(self.satellite_visible)
       self.gps2_publisher.publish(gps2_raw)
  
        
if __name__ == "__main__":
    rospy.init_node('fixposition_to_mavros')
    fp = visionConverter()
    rospy.spin()