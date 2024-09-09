#!/usr/bin/env python3
'''
A ros node that subscribes to GPS Coordinates and publishes data relative to the autopilot stack.
Reference output topics : 

~global_position/global (sensor_msgs/NavSatFix)
    GPS Fix.

~global_position/local (nav_msgs/Odometry)

'''
try:

    import rospy
    # https://docs-a1.readthedocs.io/en/latest/communication_messaging.html#ascii-data-output-messages
    from sensor_msgs.msg import NavSatFix
    from nav_msgs.msg import Odometry
    from ackermann_msgs.msg import AckermannDrive
    from geometry_msgs.msg import Quaternion
    import math
    import tf_conversions
    from autopilot_utils.geonav_conversions import xy2ll, ll2xy
    from geographic_msgs.msg import GeoPointStamped
    from autopilot_utils.pose_helper import yaw_to_quaternion 
    from geometry_msgs.msg import TwistStamped, QuaternionStamped
    from std_msgs.msg import Float64
except ImportError as e:
    print("No module named ".format(e))

def calculate_gps_center(coord1, coord2):
    # Convert latitude and longitude from degrees to radians
    lat1, lon1 = map(math.radians, coord1)
    lat2, lon2 = map(math.radians, coord2)
    
    # Calculate the differences
    dlon = lon2 - lon1
    
    # Calculate the midpoint
    Bx = math.cos(lat2) * math.cos(dlon)
    By = math.cos(lat2) * math.sin(dlon)
    lat3 = math.atan2(math.sin(lat1) + math.sin(lat2),
                      math.sqrt((math.cos(lat1) + Bx) ** 2 + By ** 2))
    lon3 = lon1 + math.atan2(By, math.cos(lat1) + Bx)
    
    # Convert the midpoint from radians to degrees
    lat3 = math.degrees(lat3)
    lon3 = math.degrees(lon3)
    
    return (lat3, lon3)

class standaloneGPSConverter:
    def __init__(self):
        self.north_velocity = None  # North velocity in NED Frame
        self.east_velocity = None   # East velocity in NED Frame
        self.down_velocity = None
        self.ekf_lat = None
        self.ekf_lon = None


        self.standalone_lat = None
        self.standalone_lon = None
        self.standalone_linear_vel_x = None
        self.standalone_linear_vel_y = None
        self.standalone_linear_vel_z = None
        self.standalone_angular_vel_x = None
        self.standalone_angular_vel_y = None
        self.standalone_angular_vel_z = None

        self.standalone_heading_quaternion = None
        
        self.heading_degrees = None
        self.dual_heading_deg = None
        self.ins_heading = None
        self.home_pose_lat = None
        self.home_pose_lon = None
        self.hacc = None
        self.vacc = None

        self.is_home_pose_published = False

        rospy.Subscriber("/mavros/global_position/set_gp_origin", GeoPointStamped, self.home_position_cb)
        rospy.Subscriber('/gps1', NavSatFix, self.standalone_gps1_cb) #/gps/global_position/global
        rospy.Subscriber('/gps2', NavSatFix, self.standalone_gps2_cb) #/gps/global_position/global
        rospy.Subscriber('/gps/global_position/global', NavSatFix, self.standalone_gps_cb) #/gps/global_position/global
        
        rospy.Subscriber('/gps/global_position/vel', TwistStamped, self.standalone_gps_vel_cb) #/gps/global_position/vel
        rospy.Subscriber('/gps/global_position/heading', QuaternionStamped, self.standalone_heading_cb) #/gps/global_position/heading

        # rospy.Subscriber("/vehicle/drive_feedback", AckermannDrive, self.vehicle_callback)

        # self.navsat_fix_publisher = rospy.Publisher('/gps/global_position/global', NavSatFix, queue_size=10) # nmea driver will publish
        self.standalone_local_odom_publisher = rospy.Publisher('/gps/global_position/local', Odometry, queue_size=10)
        self.standalone_local_odom_publisher1 = rospy.Publisher('/gps/global_position/local1', Odometry, queue_size=10)
        self.standalone_local_odom_publisher2 = rospy.Publisher('/gps/global_position/local2', Odometry, queue_size=10)
        
        self.home_pose_publisher = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=10, latch=True)
        self.compass_heading_publisher = rospy.Publisher('/gps/global_position/compass_hdg',Float64, queue_size=1)
        self.center_gps_publisher = rospy.Publisher('/gps/global_position/global_center',NavSatFix, queue_size=1)

    def standalone_heading_cb(self, data):

        self.standalone_heading_quaternion = data.quaternion

    def standalone_gps_vel_cb(self, data):
        self.standalone_linear_vel_x = data.twist.linear.x
        self.standalone_linear_vel_y = data.twist.linear.y
        self.standalone_linear_vel_z = data.twist.linear.z

        self.standalone_angular_vel_x = data.twist.angular.x
        self.standalone_angular_vel_y = data.twist.angular.y
        self.standalone_angular_vel_z = data.twist.angular.z

    def standalone_gps1_cb(self, data):
            self.gps1_lat = data.latitude
            self.gps1_lon = data.longitude

            # Do LL TO XY Here
            if self.home_pose_lat is not None:
                self.gps1_XY_points = ll2xy(self.gps1_lat, self.gps1_lon, self.home_pose_lat, self.home_pose_lon)

                self.publish_local_odom_from_standalone_gps(self.gps1_XY_points,1)
    def standalone_gps2_cb(self, data):
        self.gps2_lat = data.latitude
        self.gps2_lon = data.longitude
        # Do LL TO XY Here
        if self.home_pose_lat is not None:
            self.gps2_XY_points = ll2xy(self.gps2_lat, self.gps2_lon, self.home_pose_lat, self.home_pose_lon)

            self.publish_local_odom_from_standalone_gps(self.gps2_XY_points,2)


    def standalone_gps_cb(self, data):
        self.standalone_lat = data.latitude
        self.standalone_lon = data.longitude

        if not self.is_home_pose_published:
            '''
            If home position is not set from save path, temporarily set the current position
            If again home position updates, conversions will happen from there.
            Using the same topic as mavros for home position /mavros/global_position/set_gp_origin.
            so, no changes in save path node and path publisher are needed. 
            - http://docs.ros.org/en/lunar/api/mavros/html/group__plugin.html#ga10172da1406175808ae2aa8fb84bd8af
            - https://ardupilot.org/dev/docs/mavlink-get-set-home-and-origin.html#set-gps-global-origin
            - @iam-vishnu

            '''
            geo_point = GeoPointStamped()
            geo_point.position.latitude = self.standalone_lat
            geo_point.position.longitude = self.standalone_lon
            geo_point.position.altitude = -60

            self.home_pose_publisher.publish(geo_point)
            self.is_home_pose_published = True

        # Do LL TO XY Here
        if self.home_pose_lat is not None:
            # XY_points = ll2xy(self.standalone_lat, self.standalone_lon, self.home_pose_lat, self.home_pose_lon)
            x_center = (self.gps1_XY_points[0] + self.gps2_XY_points[0]) / 2
            y_center = (self.gps2_XY_points[1] + self.gps2_XY_points[1]) / 2
            XY_points = (x_center, y_center)
            self.publish_local_odom_from_standalone_gps(XY_points)
            
            
            center_lat, center_lon = calculate_gps_center((self.gps1_lat, self.gps1_lon), (self.gps2_lat, self.gps2_lon))
            navsat_msg = NavSatFix()
            # navsat_msg.header = data.header
            navsat_msg.header.stamp = rospy.Time.now()
            navsat_msg.header.frame_id = 'base_link'
            navsat_msg.latitude = center_lat
            navsat_msg.longitude = center_lon
            navsat_msg.altitude = data.altitude
            navsat_msg.position_covariance = data.position_covariance
            navsat_msg.position_covariance_type = data.position_covariance_type
            
            # Publish the NavSatFix message
            self.center_gps_publisher.publish(navsat_msg)

    def home_position_cb(self, data):

        self.home_pose_lat = data.position.latitude
        self.home_pose_lon = data.position.longitude
    
    # fill up position data and twist data for odom and publish as ~/global_position/local

    def publish_local_odom_from_standalone_gps(self,xy_points, gps_ant=0):

        '''
        Collects data from different topics, compile and post as 
        ~/global_position/global and ~/global_position/local


        '''
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link" 

        odom_msg.pose.pose.position.x = xy_points[0] # LL to XY  ;  # utm_points.easting # UTM Coordinates
        odom_msg.pose.pose.position.y = xy_points[1] # LL to XY ;   # utm_points.northing # UTM Coordinnates
        
        # odom_msg.pose.pose.orientation = self.heading_to_quaternion(self.heading_degrees)

        # Get the fused Heading data from APINS and normalize to 0-360
        # ins_heading_360 = (self.ins_heading+360)%360

        # adjust 90 degree shift and publish orientation. Maybe ENU conversion ? Need to check.

        # odom_msg.pose.pose.orientation = yaw_to_quaternion(math.radians((90-ins_heading_360)%360))

        # odom_msg.pose.pose.orientation = self.heading_to_quaternion((self.ins_heading+360)%360)
        # print((90-heading_360)%360)

        odom_msg.pose.pose.orientation = self.standalone_heading_quaternion
        # odom_msg.pose.pose.orientation.z = self.standalone_heading_quaternion.z
        if self.standalone_heading_quaternion is not None:
            _, _, yaw = tf_conversions.transformations.euler_from_quaternion(
                            [self.standalone_heading_quaternion.x, self.standalone_heading_quaternion.y,
                            self.standalone_heading_quaternion.z,self.standalone_heading_quaternion.w]
                        )
            deg = (math.degrees(yaw) + 360) % 360
            # print(f'YAW IS {deg}, corrected Yaw is {(deg-90)%360}')
            corrected_heading = (deg-90)%360
        
            self.compass_heading_publisher.publish(corrected_heading)

            _, _, yaw2 = tf_conversions.transformations.euler_from_quaternion(
                            [self.standalone_heading_quaternion.x, self.standalone_heading_quaternion.y,
                            self.standalone_heading_quaternion.z,-self.standalone_heading_quaternion.w]
                        )

            odom_msg.pose.pose.orientation = yaw_to_quaternion(math.radians((math.degrees(yaw2)-180)%360)) # need to validate this

        # odom_msg.pose.pose.orientation.z = -odom_msg.pose.pose.orientation.z
        if self.standalone_linear_vel_x is not None:
            odom_msg.twist.twist.linear.x = self.standalone_linear_vel_x
            odom_msg.twist.twist.linear.y = self.standalone_linear_vel_y
            odom_msg.twist.twist.linear.z = self.standalone_linear_vel_z

            odom_msg.twist.twist.angular.x = self.standalone_angular_vel_x #self.deg_to_rad(self.angular_x)
            odom_msg.twist.twist.angular.y = self.standalone_angular_vel_y  #self.deg_to_rad(self.angular_y)
            odom_msg.twist.twist.angular.z = self.standalone_angular_vel_z # self.deg_to_rad(self.angular_z)
        try:
            # publish gps1 local coordinates
            if gps_ant == 1:
                self.standalone_local_odom_publisher1.publish(odom_msg)
            # publish gps1 local coordinates
            elif gps_ant == 2:
                self.standalone_local_odom_publisher2.publish(odom_msg)
            # publish center gps local coordinates
            else:
                self.standalone_local_odom_publisher.publish(odom_msg)
            rospy.logdebug("Odom sent")
        except Exception as e:
            rospy.logerr_throttle(10,{e})

    
    def deg_to_rad(self,deg):
        return deg * (math.pi / 180.0)

    def heading_to_quaternion(self,heading_degrees):
        '''
        NOTE : Depricated Function in the use of 
        yaw_to_quaternion from autopilot_utils
        ---
        Takes heading in degress and returns quaternion. 

        '''
        # Convert heading from degrees to radians
        heading_radians = math.radians(heading_degrees)

        # Calculate quaternion from heading
        quaternion = Quaternion()
        quaternion.w = math.cos(heading_radians / 2)
        quaternion.x = 0
        quaternion.y = 0
        quaternion.z = math.sin(heading_radians / 2)

        # check the quaternion yaw degree is correct against the topic /APGPS/heading

        # _, _, yaw = tf_conversions.transformations.euler_from_quaternion(
        #                 [quaternion.x, quaternion.y,
        #                 quaternion.z,quaternion.w]
        #             )
        # deg = (math.degrees(yaw) + 360) % 360
        # print(f'YAW IS {deg}')
        
        return quaternion

if __name__ == "__main__":
    rospy.init_node('standalone_gps_converter')
    fs = standaloneGPSConverter()
    rospy.spin()
    