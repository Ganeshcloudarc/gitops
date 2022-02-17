#!/usr/bin/env python3
try:
    from geonav_conversions import *
    import rospy
    import rospkg
    from nav_msgs.msg import Path
    from geographic_msgs.msg import GeoPointStamped
    from geometry_msgs.msg import Pose, PoseStamped, Quaternion
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
    from std_msgs.msg import Float32MultiArray, Int32MultiArray, Bool, Int16, String, Int8, UInt16
    import json
    import time
    import math
    import sys
except Exception as e:
    print("No module named", str(e))
    exit()


def circum_radius(x_vals, y_vals):
    """
        Calculates the circum radius for three 2D points
    """
    x1, x2, x3, y1, y2, y3 = x_vals[0], x_vals[1], x_vals[2], y_vals[0], y_vals[1], y_vals[2]
    den = 2 * ((x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2))
    num = ((((x2 - x1) ** 2) + ((y2 - y1) ** 2)) * (((x3 - x2) ** 2) + ((y3 - y2) ** 2)) * (
                ((x1 - x3) ** 2) + ((y1 - y3) ** 2))) ** 0.5
    if den == 0:
        print('Failed: points are either collinear or not distinct')
        return 0
    radius = abs(num / den)
    return radius


class PathPubGps:
    def __init__(self, mission_file):
        self.gps_path_pub = rospy.Publisher('/gps_path', Path, queue_size=10, latch=True)
        self.odom_path_pub = rospy.Publisher('/odom_path', Path, queue_size=10, latch=True)
        self.starting_point_pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped,
                                                  queue_size=10, latch=True)
        self.curvature_pub = rospy.Publisher('/curvature_profile', Float32MultiArray, queue_size=10, latch=True)
        self.velocities_pub = rospy.Publisher('/velocity_profile', Float32MultiArray, queue_size=10, latch=True)
        self.curvature_msg = Float32MultiArray()
        self.velocity_msg = Float32MultiArray()
        self.max_speed = rospy.get_param("/patrol/max_speed", 1.5)

        # msgs
        self.gps_path_msg = Path()
        self.gps_path_msg.header.frame_id = 'map'
        self.gps_path_msg.header.stamp = rospy.Time.now()
        self.odom_path_msg = Path()
        self.odom_path_msg.header.frame_id = 'map'

        try:
            data = json.load(open(mission_file))
        except Exception as e:
            rospy.logwarn('Error In Reading mission file ' + str(e))
            sys.exit('Error In Reading mission file ' + str(e))

        if len(data['coordinates']) == 0:
            rospy.logwarn("No way points on mission file " + str(mission_file))
            sys.exit("No way points on mission file" + str(mission_file))
        else:
            rospy.loginfo("Success in reading mission file")
            pass
        self.data_len = len(data['coordinates'])
        self.data = data

        home_lat = data['coordinates'][0][1]
        home_long = data['coordinates'][0][0]
        geo_point = GeoPointStamped()
        geo_point.position.latitude = home_lat
        geo_point.position.longitude = home_long
        geo_point.position.altitude = 29  # change it to actual latitude
        self.starting_point_pub.publish(geo_point)
        rospy.loginfo('Origin point set')
        time.sleep(0.5)

        for i in range(len(data['coordinates'])):
            # gps_based path
            lon, lat = data['coordinates'][i][0], data['coordinates'][i][1]
            gps_orientation = data['imu'][i]['orientation']
            x, y = ll2xy(lat, lon, home_lat, home_long)
            gps_pose = PoseStamped()
            gps_pose.pose.position.x, gps_pose.pose.position.y = x, y
            gps_pose.pose.orientation = Quaternion(gps_orientation['x'], gps_orientation['y'], gps_orientation['z'],
                                                   gps_orientation['w'])
            gps_pose.header.seq += 1
            gps_pose.header.frame_id = 'map'
            gps_pose.header.stamp = rospy.Time.now()
            self.gps_path_msg.poses.append(gps_pose)

            # Odom based path
            odom_position = data['odometry'][i]['pose']['pose']['position']
            odom_orientation = data['odometry'][i]['pose']['pose']['orientation']
            odom_pose = PoseStamped()
            odom_pose.pose.position.x, odom_pose.pose.position.y = odom_position['x'], odom_position['y']
            odom_pose.pose.orientation = Quaternion(odom_orientation['x'], odom_orientation['y'], odom_orientation['z'],
                                                    odom_orientation['w'])
            odom_pose.header.seq += 1
            odom_pose.header.stamp = rospy.Time.now()
            self.odom_path_msg.poses.append(odom_pose)

            # Velocity 
            velocity = data['odometry'][i]['twist']['twist']['linear']['x']
            self.velocity_msg.data.append(velocity)
            # Curvature
            curvature = self.find_curvature_at_index(i)
            self.curvature_msg.data.append(curvature)

        self.gps_path_pub.publish(self.gps_path_msg)
        rospy.loginfo('GPS path is published')
        self.odom_path_pub.publish(self.odom_path_msg)
        rospy.loginfo('ODOM path is published')
        self.velocities_pub.publish(self.velocity_msg)
        rospy.loginfo('Velocity profile is published')
        self.curvature_pub.publish(self.curvature_msg)
        rospy.loginfo('Curvature profile is published')
        rospy.loginfo('Details of ' + str(i) + " are published from file " + str(mission_file))

    def find_curvature_at_index(self, i):
        # find curvature at given index

        if i == 0 or i >= len(self.data['coordinates']) - 1:
            return 0
        else:
            x_vals = [self.data['odometry'][i - 1]['pose']['pose']['position']['x'],
                      self.data['odometry'][i]['pose']['pose']['position']['x'],
                      self.data['odometry'][i + 1]['pose']['pose']['position']['x']]
            y_vals = [self.data['odometry'][i - 1]['pose']['pose']['position']['y'],
                      self.data['odometry'][i]['pose']['pose']['position']['y'],
                      self.data['odometry'][i + 1]['pose']['pose']['position']['y']]
            radius = circum_radius(x_vals, y_vals)
            if radius == 0:
                return 0
            else:
                return 1 / radius


if __name__ == "__main__":
    rospy.init_node("gps_path_publisher")
    mission_file = rospy.get_param('/patrol/mission_file', 'default.json')
    if '.json' in mission_file:
        pass
    else:
        mission_file = mission_file + '.json'
        rospy.set_param('/patrol/mission_file', mission_file)
    try:
        ros_pack = rospkg.RosPack()
        mission_file_dir = ros_pack.get_path('autopilot_boson') + "/mission_files/" + str(mission_file)
    except Exception as e:
        rospy.logwarn("Please source autopilot_boson package" + str(e))
        sys.exit("Please source autopilot_boson package" + str(e))
    a = PathPubGps(mission_file_dir)
    rospy.spin()
