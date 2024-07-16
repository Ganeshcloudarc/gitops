#!/usr/bin/env python3
"""
A node to publish tf between base link to map by subscribing odometry msg from mavros.
"""
try:
    import rospy
    import math, os
    import tf2_ros
    import tf_conversions
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped, Pose, Quaternion, Vector3, PoseStamped, Polygon, \
        PolygonStamped, Point
    from vehicle_common.vehicle_config import vehicle_data
    from pyproj import Geod
    from sensor_msgs.msg import NavSatFix
    from autopilot_utils.pose_helper import yaw_to_quaternion
except ImportError as error:
    print("No module named:", error)

# parameters
enable_2d_mode = rospy.get_param('enable_2d_mode', True)
wheel_base = rospy.get_param('/vehicle/dimensions/wheel_base', 1.82)
front_axle_to_fcu_position = rospy.get_param('/vehicle/fcu/position', [0.4, 0, 0])
front_axle_to_fcu_orientation = rospy.get_param('/vehicle/fcu/orientation', [0, 0, 0])

vehicle_length = rospy.get_param("/vehicle/dimensions/overall_length", 3.544)
vehicle_width = rospy.get_param("/vehicle/dimensions/overall_width", 1.460)
vehicle_height = rospy.get_param("/vehicle/dimensions/overall_height", 1.75)

front_overhang = rospy.get_param("/vehicle/dimensions/front_overhang", 1.0)
rear_overhang = rospy.get_param("/vehicle/dimensions/rear_overhang", 0.7)
rear_axle_center_height_from_ground = rospy.get_param("/vehicle/dimensions/tyre_radius", 0.3)

vehicle_frame = rospy.get_param("/base_frame", "base_link")
odom_in_topic = rospy.get_param("/odometry_in", '/mavros/global_position/local')
gps_in_topic = rospy.get_param("/gps_in", '/mavros/global_position/global')

send_odom = rospy.get_param("/send_odom", True)  # send odom from base link
send_gps = rospy.get_param("/send_gps", True)  # send odom from base link

send_foot_print = rospy.get_param("/send_footprint", True)
send_odom_topic_name = rospy.get_param("/odometry_out", "/vehicle/odom")
send_gps_topic_name = rospy.get_param("/gps_out", "/vehicle/gps")
send_footprint_topic_name = rospy.get_param("/footprint_out", "/vehicle/foot_print")
# send_foot_print = rospy.get_param("/send_footprint", True)

standalone_gps = rospy.get_param("standalone_gps", True)

tf_broad_caster = tf2_ros.TransformBroadcaster()
tf_msg = TransformStamped()
fcu_offset_vehicle = (wheel_base + front_axle_to_fcu_position[0])

foot_print_specs = [
    [-rear_overhang, -vehicle_width / 2],
    [wheel_base + front_overhang, -vehicle_width / 2],
    [wheel_base + front_overhang, vehicle_width / 2],
    [-rear_overhang, vehicle_width / 2],
]
geod = Geod(ellps="WGS84")
global yaw 
yaw = None

def odom_callback(data):
    """ subscriber for odom, it publishes tf and odom from vehicle base link(rear axil center)"""
    global yaw
    _, _, yaw = tf_conversions.transformations.euler_from_quaternion(
        [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
         data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    )
    x_pos = data.pose.pose.position.x - fcu_offset_vehicle * math.cos(yaw)  # transom sensor location to rear axle in x
    y_pos = data.pose.pose.position.y - fcu_offset_vehicle * math.sin(yaw)  # transom sensor location to rear axle in y
    
    if standalone_gps:
        # Transform XY by 0.8625 meters when standalone GPS because standalone gps will give coordinates from Main(Left) antenna.
        
        # y_pos = y_pos - 0.8625
        y_pos = y_pos - 0.8765 * math.cos(yaw)
        x_pos = x_pos + 0.8765 * math.sin(yaw)
        

    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.header.frame_id = 'odom'                            #data.header.frame_id
    tf_msg.child_frame_id = vehicle_frame
    tf_msg.transform.translation.x = x_pos
    tf_msg.transform.translation.y = y_pos
    tf_msg.transform.translation.z = rear_axle_center_height_from_ground  # making z zero
    
    if enable_2d_mode:
        orientation_from_yaw = yaw_to_quaternion(yaw)
    else:
        orientation_from_yaw = data.pose.pose.orientation

    tf_msg.transform.rotation = orientation_from_yaw
    tf_broad_caster.sendTransform(tf_msg)
    rospy.logdebug("transform sent")
    if send_odom:
        odom_msg = Odometry()
        odom_msg = data
        odom_msg.pose.pose.orientation = orientation_from_yaw
        odom_msg.pose.pose.position.x = x_pos
        odom_msg.pose.pose.position.y = y_pos
        odom_msg.pose.pose.position.z = rear_axle_center_height_from_ground
        odom_msg.child_frame_id = vehicle_frame
        odom_publisher.publish(odom_msg)
        rospy.logdebug("Odom sent")
    #
    if send_foot_print:
        polygon_msg = transform_footprint(x_pos, y_pos, yaw)
        foot_prin_pub.publish(polygon_msg)
        rospy.logdebug("footprint sent")


def gps_callback(data):
    if yaw:
        # heading to Azumuth 
        deg = 360 - math.degrees(yaw ) + 90
        lon, lat,_ = geod.fwd(lons=data.longitude, lats=data.latitude, az=deg, dist=-fcu_offset_vehicle)
        if standalone_gps:
            # lon, lat,_ = geod.fwd(lons=data.longitude, lats=data.latitude, az=deg+90, dist=0.5)
            # shift main antenna coordinates to center by distance. vehicle width/2 because main antenna is to the left of the vehicle.
            # deg+90 := shift to right side with 0.8625 at 90 degree right angle. Fwd is zero degree.
            lon2, lat2, _ = geod.fwd(lons=lon, lats=lat, az=deg+90, dist=0.8625) 
            data.longitude = lon2
            data.latitude = lat2
        else:
            data.longitude = lon
            data.latitude = lat
        gps_publisher.publish(data)
        rospy.logdebug("gps published")
        


def transform_footprint(x, y, yaw):
    polygon_st = PolygonStamped()
    polygon_st.header.frame_id = 'map'
    cos_th = math.cos(yaw)
    sin_th = math.sin(yaw)

    for foot in foot_print_specs:
        point = Point()
        # point.x = foot[0]
        # point.y = foot[1]
        point.x = x + (foot[0] * cos_th - foot[1] * sin_th)
        point.y = y + (foot[0] * sin_th + foot[1] * cos_th)
        polygon_st.polygon.points.append(point)
    return polygon_st


if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster_mavros')

    rospy.loginfo("tf broadcaster node started for mavros")
    # increasing the speed to 50 hz
    os.system('rosrun mavros mavsys rate --all 50')  # TODO
    if send_odom:
        odom_publisher = rospy.Publisher(send_odom_topic_name, Odometry, queue_size=2)
    if send_foot_print:
        foot_prin_pub = rospy.Publisher(send_footprint_topic_name, PolygonStamped, queue_size=2)
    if send_gps:
        gps_publisher = rospy.Publisher(send_gps_topic_name, NavSatFix, queue_size=2)
    rospy.Subscriber(odom_in_topic, Odometry, odom_callback)
    rospy.Subscriber(gps_in_topic, NavSatFix, gps_callback)

    rospy.spin()
