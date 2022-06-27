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
except ImportError as error:
    print("No module named:", error)

# parameters
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
odom_in_topic = rospy.get_param("/odometry_in", '/mavros/local_position/odom')
send_odom = rospy.get_param("/send_odom", True)  # send odom from base link
send_foot_print = rospy.get_param("/send_footprint", True)
send_odom_topic_name = rospy.get_param("/odometry_out", "/vehicle/odom")
send_footprint_topic_name = rospy.get_param("/foorprint_out", "/vehicle/foot_print")
tf_broad_caster = tf2_ros.TransformBroadcaster()
tf_msg = TransformStamped()
fcu_offset_vehicle = (wheel_base + front_axle_to_fcu_position[0])

foot_print_specs = [
    [-rear_overhang, -vehicle_width / 2],
    [wheel_base + front_overhang, -vehicle_width / 2],
    [wheel_base + front_overhang, vehicle_width / 2],
    [-rear_overhang, vehicle_width / 2],
]


def odom_callback(data):
    """ subscriber for odom, it publishes tf and odom from vehicle base link(rear axil center)"""

    _, _, yaw = tf_conversions.transformations.euler_from_quaternion(
        [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
         data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    )
    x_pos = data.pose.pose.position.x - fcu_offset_vehicle * math.cos(yaw)  # transom sensor location to rear axle in x
    y_pos = data.pose.pose.position.y - fcu_offset_vehicle * math.sin(yaw)  # transom sensor location to rear axle in y

    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.header.frame_id = data.header.frame_id
    tf_msg.child_frame_id = vehicle_frame
    tf_msg.transform.translation.x = x_pos
    tf_msg.transform.translation.y = y_pos
    tf_msg.transform.translation.z = rear_axle_center_height_from_ground  # making z zero
    tf_msg.transform.rotation = data.pose.pose.orientation
    tf_broad_caster.sendTransform(tf_msg)
    rospy.logdebug("transform sent")
    if send_odom:
        odom_msg = Odometry()
        odom_msg = data
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
        foot_prin_pub = rospy.Publisher("/vehicle/foot_print", PolygonStamped, queue_size=2)
    rospy.Subscriber(odom_in_topic, Odometry, odom_callback)
    rospy.spin()
