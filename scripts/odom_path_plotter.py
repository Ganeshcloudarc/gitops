#!/usr/bin/env python3
import math

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32

class OdomPathPlotter:
    def __init__(self):
        self.path_pub = rospy.Publisher('/Vehicle_travelled_path', Path, queue_size=10)
        self.heading_pub = rospy.Publisher('/yaw', Float32, queue_size=10)
        carla_status = rospy.get_param("/carla_sim/activate", False)
        if carla_status:
            odom_topic = '/carla/ego_vehicle/odometry'
        else:
            odom_topic = '/mavros/local_position/odom'
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.prev_x, self.prev_y = 0.0, 0.0
        self.path_msg = Path()

    def odom_callback(self, data):
        pose = PoseStamped()
        pose.header.frame_id = data.header.frame_id
        pose.pose.position.x = float(data.pose.pose.position.x)
        pose.pose.position.y = float(data.pose.pose.position.y)
        pose.pose.orientation = data.pose.pose.orientation

        _, _, pose_heading = euler_from_quaternion(
            [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
             data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        )

        if self.prev_x != pose.pose.position.x and self.prev_y != pose.pose.position.y:
            pose.header.seq = self.path_msg.header.seq + 1
            self.path_msg.header.frame_id = 'map'
            self.path_msg.header.stamp = rospy.Time.now()
            pose.header.stamp = self.path_msg.header.stamp
            self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)
        self.heading_pub.publish(math.degrees(pose_heading)%360)
        print("\rPublished: " + str(self.path_msg.header.seq) + " Points ", end="\t")


if __name__ == '__main__':
    rospy.init_node('odom_path_plotter')
    plotter = OdomPathPlotter()
    rospy.spin()
