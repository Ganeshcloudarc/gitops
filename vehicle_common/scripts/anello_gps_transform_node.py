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
    from anello_ros_driver.msg import APGPS, APIMU, APINS, APHDG, APODO
    from sensor_msgs.msg import NavSatFix
    from nav_msgs.msg import Odometry
    from ackermann_msgs.msg import AckermannDrive
    from geometry_msgs.msg import Quaternion
    import math
    import tf_conversions
    from autopilot_utils.geonav_conversions import xy2ll, ll2xy
    from geographic_msgs.msg import GeoPointStamped
    from autopilot_utils.pose_helper import yaw_to_quaternion 
except ImportError as e:
    print("No module named ".format(e))



class anelloGPSConverter:
    def __init__(self):
        self.north_velocity = None  # North velocity in NED Frame
        self.east_velocity = None   # East velocity in NED Frame
        self.down_velocity = None
        self.ekf_lat = None
        self.ekf_lon = None
        self.heading_degrees = None
        self.dual_heading_deg = None
        self.ins_heading = None
        self.home_pose_lat = None
        self.home_pose_lon = None
        self.hacc = None
        self.vacc = None
        rospy.Subscriber("/APGPS", APGPS, self.anello_gps_cb)
        rospy.Subscriber("/APIMU", APIMU, self.anello_imu_cb)
        rospy.Subscriber("/APINS", APINS, self.anello_ins_cb)
        rospy.Subscriber("/APHDG", APHDG, self.anello_hdg_cb)
        self.is_home_pose_published = False
        rospy.Subscriber("/mavros/global_position/set_gp_origin", GeoPointStamped, self.home_position_cb)
        rospy.Subscriber("/vehicle/drive_feedback", AckermannDrive, self.vehicle_callback)

        self.navsat_fix_publisher = rospy.Publisher('/anello/global_position/global', NavSatFix, queue_size=10)
        self.local_odom_publisher = rospy.Publisher('/anello/global_position/local', Odometry, queue_size=10)
        self.navsat_fix_publisher_ins = rospy.Publisher('/anello_ins/global_position/global', NavSatFix, queue_size=10)
        self.local_odom_publisher_ins = rospy.Publisher('/anello_ins/global_position/local', Odometry, queue_size=10)
        self.home_pose_publisher = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=10, latch=True)
        self.anello_odom_publisher = rospy.Publisher('/APODO', APODO, queue_size=10)

    def vehicle_callback(self,data):
        '''
        It is the feedback data from the motor rpm converted in to m/s speed. 
        We have to publish the speed back to anello for better data fusion 
        https://github.com/bosonrobotics/pilot/blob/460600402cde09392334abf2e1df0bf9970d365a/pilot/scripts/pilot_vehicle_control.py#L554

        From Anello Documentation:
            When an APODO message is received with a reverse direction indication, 
            the unit will assume the vehicle is in reverse until a packet is received with a forward direction. 
            The units of the speed in the APODO message is user configurable to m/s (default), mile/hr, km/hr, ft/s
        
        '''
        vehicle_speed = data.speed
        ap_odom_msg = APODO()
        ap_odom_msg.odo_speed = vehicle_speed
        rospy.loginfo_throttle(100,"Publishing speed to Anello Device {}".format(vehicle_speed))
        self.anello_odom_publisher.publish(ap_odom_msg)

    def home_position_cb(self, data):

        self.home_pose_lat = data.position.latitude
        self.home_pose_lon = data.position.longitude

    def anello_hdg_cb(self, data):

        self.dual_heading_deg = data.rel_pos_heading
        # print(self.dual_heading_deg)

    # subscribe to gps message and convert to navsat fix and publish back
    def anello_gps_cb(self, data):
        '''
        Anello GPS Callback subscribes to gps and publish as NavSatFix message type. 
        Calculates UTM coorindates from lat, lon and pass to odometry publisher. 
        '''
        lat = data.lat
        lon = data.lon
        self.hacc = data.hacc
        self.vacc = data.vacc
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
            geo_point.position.latitude = lat
            geo_point.position.longitude = lon
            geo_point.position.altitude = 0

            # self.home_pose_publisher.publish(geo_point)
            # self.is_home_pose_published = True

        self.heading_degrees = data.heading # 0 to 360 degree
        navsat_msg = NavSatFix()
        # navsat_msg.header = data.header
        navsat_msg.header.stamp = rospy.Time.now()
        navsat_msg.header.frame_id = 'base_link'
        navsat_msg.latitude = lat
        navsat_msg.longitude = lon
        navsat_msg.altitude = data.alt_msl
        navsat_msg.position_covariance = [
            data.hacc**2, 0, 0,
            0, data.hacc**2, 0,
            0, 0, data.vacc**2
        ]
        navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        # Publish the NavSatFix message
        self.navsat_fix_publisher.publish(navsat_msg)

        # Do UTM conversion here. 
        # self.utm_points = utm.fromLatLong(lat, lon)
        # print(self.utm_points)
        # self.publish_local_odom(self.utm_points)

        # Do LL TO XY Here
        if self.home_pose_lat is not None:
            XY_points = ll2xy(lat, lon, self.home_pose_lat, self.home_pose_lon)

            self.publish_local_odom(XY_points,False)

    # subscribe to imu message to publish in ros imu message
    def anello_imu_cb(self,data):

        self.accel_x = data.ax # accelerations m/s2
        self.accel_y = data.ay
        self.accel_z = data.az
        self.angular_x = data.wx # deg/s
        self.angular_y = data.wy
        self.angular_z = data.wz

        # ToDo - compile as ros imu message and publish back as ~/imu
    
    # subscribe to ins message and get velocities to publish in odom message along with converted gps message as UTM coordinates. 
    def anello_ins_cb(self,data):
        '''
        The APINS message is the Kalman filter position, 
        velocity, and attitude solution output from the EVK and GNSS INS units.
        '''

        self.ekf_lat = data.lat # Latitude, ‘+’: north, ‘-’: south
        self.ekf_lon = data.lon # Longitude, ‘+’: east, ‘-’: west
        self.north_velocity = data.vn  # North velocity in NED Frame
        self.east_velocity = data.ve   # East velocity in NED Frame
        self.down_velocity = data.vd   # Down velocity in NED Frame
        self.ins_heading = data.heading


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
            geo_point.position.latitude = self.ekf_lat
            geo_point.position.longitude = self.ekf_lon
            geo_point.position.altitude = -60

            self.home_pose_publisher.publish(geo_point)
            self.is_home_pose_published = True

        # self.heading_degrees = data.heading # 0 to 360 degree
        # print(self.hacc)
        navsat_msg = NavSatFix()
        # navsat_msg.header = data.header
        navsat_msg.header.stamp = rospy.Time.now()
        navsat_msg.header.frame_id = 'base_link'
        navsat_msg.latitude = self.ekf_lat
        navsat_msg.longitude = self.ekf_lon
        navsat_msg.altitude = -60
        if self.hacc is not None:
            navsat_msg.position_covariance = [
                self.hacc**2, 0, 0,
                0, self.hacc**2, 0,
                0, 0, self.vacc**2
            ]
            navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        # Publish the NavSatFix message
        self.navsat_fix_publisher_ins.publish(navsat_msg)

        # Do UTM conversion here. 
        # self.utm_points = utm.fromLatLong(lat, lon)
        # print(self.utm_points)
        # self.publish_local_odom(self.utm_points)

        # Do LL TO XY Here
        if self.home_pose_lat is not None:
            XY_points = ll2xy(self.ekf_lat, self.ekf_lon, self.home_pose_lat, self.home_pose_lon)

            self.publish_local_odom(XY_points,True)

    # fill up position data and twist data for odom and publish as ~/global_position/local
    def publish_local_odom(self,xy_points,ins):

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
        ins_heading_360 = (self.ins_heading+360)%360

        # adjust 90 degree shift and publish orientation. Maybe ENU conversion ? Need to check.

        odom_msg.pose.pose.orientation = yaw_to_quaternion(math.radians((90-ins_heading_360)%360))
        # odom_msg.pose.pose.orientation = self.heading_to_quaternion((self.ins_heading+360)%360)
        # print((90-heading_360)%360)
        odom_msg.twist.twist.linear.x = self.north_velocity
        odom_msg.twist.twist.linear.y = self.east_velocity
        odom_msg.twist.twist.linear.z = self.down_velocity

        odom_msg.twist.twist.angular.x = self.deg_to_rad(self.angular_x)
        odom_msg.twist.twist.angular.y = self.deg_to_rad(self.angular_y)
        odom_msg.twist.twist.angular.z = self.deg_to_rad(self.angular_z)

        if ins:
            self.local_odom_publisher_ins.publish(odom_msg)
        else:
            self.local_odom_publisher.publish(odom_msg)
        rospy.logdebug("Odom sent")
    
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
    rospy.init_node('anello_gps_converter')
    fs = anelloGPSConverter()
    rospy.spin()
    