#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomPathPlotter:
    def __init__(self):
        self.path_pub = rospy.Publisher('/Vehicle_travelled_path', Path, queue_size=10)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_callback)
        self.prev_x, self.prev_y = 0.0, 0.0
        self.path_msg = Path()

    def odom_callback(self, data):
        pose = PoseStamped()
        pose.header.frame_id = data.header.frame_id
        pose.pose.position.x = float(data.pose.pose.position.x)
        pose.pose.position.y = float(data.pose.pose.position.y)
        pose.pose.orientation = data.pose.pose.orientation

        if self.prev_x != pose.pose.position.x and self.prev_y != pose.pose.position.y:
            pose.header.seq = self.path_msg.header.seq + 1
            self.path_msg.header.frame_id = 'map'
            self.path_msg.header.stamp = rospy.Time.now()
            pose.header.stamp = self.path_msg.header.stamp
            self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)
        print("\rPublished: " + str(self.path_msg.header.seq) + " Points ", end="\t")


if __name__ == '__main__':
    rospy.init_node('odom_path_plotter')
    plotter = OdomPathPlotter()
    rospy.spin()
