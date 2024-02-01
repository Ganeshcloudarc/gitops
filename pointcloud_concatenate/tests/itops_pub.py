#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import std_msgs.msg

def callback(data):
    global target_publisher
    # This function is called whenever a message is received on the source topic
    # Republish the received message on the target topic
    target_publisher.publish(data)

def main():
    global target_publisher
    rospy.init_node('pointcloud_republisher', anonymous=True)  # Initialize the ROS node

    # Get the source and target topic names from command line arguments
    source_topic = "/itops_f25/camera/points" #rospy.get_param('/source_topic', '/source_pointcloud')  # Default to '/source_pointcloud'
    target_topic = "/itops" # rospy.get_param('/target_topic', '/target_pointcloud')  # Default to '/target_pointcloud'
    print(source_topic, target_topic)
    # Create a subscriber to the source topic
    source_subscriber = rospy.Subscriber(source_topic, PointCloud2, callback)

    # Create a publisher for the target topic
    target_publisher = rospy.Publisher(target_topic, PointCloud2, queue_size=10)

    rospy.spin()  # Keep the node running

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
