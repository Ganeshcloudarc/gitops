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
    from polyx_nodea.msg import Kalman
    from polyx_nodea.msg import SolutionStatus
    from polyx_nodea.msg import nmeaGGA
    from polyx_nodea.msg import Icd # Compact Navigation Message (High rate) 
    from mavros_msgs.msg import GPSRAW
    from sensor_msgs.msg import NavSatFix
    from nav_msgs.msg import Odometry
    from ackermann_msgs.msg import AckermannDrive
    from geometry_msgs.msg import Quaternion
    from std_msgs.msg import Float64
    import math
    import tf_conversions
    from autopilot_utils.geonav_conversions import xy2ll, ll2xy
    from geographic_msgs.msg import GeoPointStamped
    from autopilot_utils.pose_helper import yaw_to_quaternion 
    import numpy as np
    from scipy.spatial.transform import Rotation as R
except ImportError as e:
    print("No module named ".format(e))



class polynavGPSConverter:
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
        self.position_mode_fix = None
        self. gps1_sat = None
        self.is_home_pose_published = False
        
        # Pubilshers 
        self.navsat_fix_publisher = rospy.Publisher('/polynav/global_position/global', NavSatFix, queue_size=10)
        self.local_odom_publisher = rospy.Publisher('/polynav/global_position/local', Odometry, queue_size=10)
        self.home_pose_publisher = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=10, latch=True)
        self.compass_heading_publisher = rospy.Publisher('/polynav/global_position/compass_hdg',Float64, queue_size=1)
        self.pub_gps1_raw = rospy.Publisher('/polynav/gpsstatus/gps1/raw', GPSRAW, queue_size=1)
        self.pub_gps2_raw = rospy.Publisher('/polynav/gpsstatus/gps2/raw', GPSRAW, queue_size=1)
        
        # Subscribers
        rospy.Subscriber("/polyx_Kalman", Kalman, self.kalman_cb)
        rospy.Subscriber("/polyx_solutionStatus", SolutionStatus, self.solutionStatus_cb)
        rospy.Subscriber("/polyx_ICD", Icd, self.icd_cb)
        rospy.Subscriber("/polyx_nmeaGGA", nmeaGGA, self.nmea_cb)
        rospy.Subscriber("/mavros/global_position/set_gp_origin", GeoPointStamped, self.home_position_cb)
        rospy.Subscriber("/vehicle/drive_feedback", AckermannDrive, self.vehicle_callback)

        
    def vehicle_callback(self,data):
        '''
        It is the feedback data from the motor rpm converted in to m/s speed. 
        We have to publish the speed back to anello for better data fusion 
        https://github.com/bosonrobotics/pilot/blob/460600402cde09392334abf2e1df0bf9970d365a/pilot/scripts/pilot_vehicle_control.py#L554
        '''
        vehicle_speed = data.speed
        rospy.loginfo_throttle(100,"Publishing speed to GPS Device {}".format(vehicle_speed))
        # self.anello_odom_publisher.publish(ap_odom_msg)

    def home_position_cb(self, data):

        self.home_pose_lat = data.position.latitude
        self.home_pose_lon = data.position.longitude
        
    def nmea_cb(self, data):
        
        self.gps1_sat = data.n_sv_used
        gps1raw = self.publish_gps_raw(self.lat, self.lon, self.alt, self.gps1_sat)
        self.pub_gps1_raw.publish(gps1raw)

    def icd_cb(self, data):
        
        lat = math.degrees(data.Latitude)
        lon = math.degrees(data.Longitude)
        alt = data.Altitude
        velocityNed = data.VelocityNED
        position_rms0, position_rms1, position_rms2 = data.PositionRMS
        
        icd_q0, icd_q1, icd_q2, icd_q3 = data.Quaternion # w,x,y,z
        # quaternion = Quaternion(icd_q1, icd_q2, icd_q3, icd_q0) # x,y,z,w - ROS

        # Define the original quaternion
        # icd_q0, icd_q1, icd_q2, icd_q3 = data.Quaternion # w,x,y,z
        quaternion_old = np.array([icd_q0, icd_q1, icd_q2, icd_q3]) # w,x,y,z

        # Define the 90-degree rotation quaternion (z-axis)
        q_90 = np.array([np.sqrt(2)/2, 0, 0, np.sqrt(2)/2]) # w,x,y,z

        # Compute the inverse of the 90-degree rotation quaternion
        q_90_inv = np.array([np.sqrt(2)/2, 0, 0, -np.sqrt(2)/2]) # w,x,y,z

        # Perform the quaternion multiplication (q_90_inv * quaternion)
        q_90_inv_rot = R.from_quat(q_90_inv)
        quaternion_rot = R.from_quat(quaternion_old)
        corrected_quaternion = q_90_inv_rot * quaternion_rot

        # Convert back to w,x,y,z format
        corrected_quaternion = corrected_quaternion.as_quat() # x,y,z,w
        corrected_quaternion_ros = [corrected_quaternion[3], corrected_quaternion[0], corrected_quaternion[1], corrected_quaternion[2]]
        quaternion = Quaternion(corrected_quaternion[3], corrected_quaternion[0], corrected_quaternion[1], corrected_quaternion[2])

                
        self.h_acc = math.sqrt(position_rms0*position_rms0 + position_rms1*position_rms1)
        
        self.v_acc = position_rms2

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
            geo_point.position.altitude = -60
            
            self.home_pose_publisher.publish(geo_point)
            self.is_home_pose_published = True
        
        # Heading Calculation - Human Format
        
        self.publish_heading(data.Quaternion)
        
        self.publish_navsat_fix([lat, lon, alt, self.h_acc, self.v_acc])
        
        if self.home_pose_lat is not None:
            XY_points = ll2xy(lat, lon, self.home_pose_lat, self.home_pose_lon)
            self.publish_local_odom(XY_points, quaternion, velocityNed)
        
        self.lat = lat
        self.lon = lon
        self.alt = alt
    
    def kalman_cb(self, data):
        self.position_mode_fix = data.PositionMode
    
    def solutionStatus_cb(self, data):
        pass

    def publish_navsat_fix(self,data):
        lat, lon, alt, h_acc, v_acc = data
        navsat_msg = NavSatFix()
        # navsat_msg.header = data.header
        navsat_msg.header.stamp = rospy.Time.now()
        navsat_msg.header.frame_id = 'base_link'
        navsat_msg.latitude = lat
        navsat_msg.longitude = lon
        navsat_msg.altitude = alt
        navsat_msg.position_covariance = [
            h_acc**2, 0, 0,
            0, h_acc**2, 0,
            0, 0, v_acc**2
        ]
        navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        # Publish the NavSatFix message
        self.navsat_fix_publisher.publish(navsat_msg)

    # fill up position data and twist data for odom and publish as ~/global_position/local
    def publish_local_odom(self,xy_points, quaternion, velocityNed):

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
        

        odom_msg.pose.pose.orientation = quaternion

        odom_msg.twist.twist.linear.x = velocityNed[0]
        odom_msg.twist.twist.linear.y = velocityNed[1]
        odom_msg.twist.twist.linear.z = velocityNed[2]

        # odom_msg.twist.twist.angular.x = self.deg_to_rad(self.angular_x)
        # odom_msg.twist.twist.angular.y = self.deg_to_rad(self.angular_y)
        # odom_msg.twist.twist.angular.z = self.deg_to_rad(self.angular_z)

        self.local_odom_publisher.publish(odom_msg)
        rospy.logdebug("Odom sent")
    
    def publish_heading(self, quat):
        
        # Polynav refernce manual section : 5.13

        icd_q0, icd_q1, icd_q2, icd_q3 = quat
        
        c11 = icd_q0**2 + icd_q1**2 - icd_q2**2 - icd_q3**2
        
        c21 = 2*(icd_q1*icd_q2+icd_q0*icd_q3) 
        
        heading_rad = math.atan2(c21,c11)
        heading_deg = math.degrees(heading_rad)%360
        
        self.compass_heading_publisher.publish(heading_deg)
        
    def publish_gps_raw(self,latitude, longitude, altitude, sat):
        '''
        NMEA Ref : # 7.1.1.3 section of UM980 manual
        Nmea to GPSRAW code ref :
        
        https://github.com/mavlink/mavros/blob/b0da849a06eb1a215b9205b92b8cf39c6d7cf88f/mavros_extras/src/plugins/hil.cpp#L210
        '''
        gps_raw = GPSRAW()
        gps_raw.header.frame_id = 'gps'
        gps_raw.header.stamp = rospy.Time.now()
        
        fix_type = self.position_mode_fix
        
        if fix_type == 6:
            try:
                gps_raw.fix_type = gps_raw.GPS_FIX_TYPE_RTK_FIXED
            except AttributeError:
                gps_raw.fix_type = gps_raw.GPS_FIX_TYPE_RTK_FIXEDR
        elif fix_type == 5:
            try:
                gps_raw.fix_type = gps_raw.GPS_FIX_TYPE_RTK_FLOAT
            except AttributeError:
                gps_raw.fix_type = gps_raw.GPS_FIX_TYPE_RTK_FLOATR
        elif fix_type == 4 :
            gps_raw.fix_type = gps_raw.GPS_FIX_TYPE_DGPS
        elif fix_type == 3 :
            gps_raw.fix_type = gps_raw.GPS_FIX_TYPE_3D_FIX
        elif fix_type == 2 :
            gps_raw.fix_type = gps_raw.GPS_FIX_TYPE_2D_FIX
        elif fix_type == 1 :
            gps_raw.fix_type = gps_raw.GPS_FIX_TYPE_NO_FIX
        else:
            gps_raw.fix_type = gps_raw.GPS_FIX_TYPE_NO_GPS
            
        gps_raw.lat = int(latitude*1e7)
        gps_raw.lon = int(longitude*1e7)
        gps_raw.alt = int(altitude*1e3)
        gps_raw.satellites_visible = sat
        gps_raw.h_acc = int(self.h_acc*1000)
        
        return gps_raw

    
    def deg_to_rad(self,deg):
        return deg * (math.pi / 180.0)
    
    

if __name__ == "__main__":
    rospy.init_node('polynav_gps_converter')
    fs = polynavGPSConverter()
    rospy.spin()
    