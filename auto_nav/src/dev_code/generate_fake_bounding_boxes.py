#!/usr/bin/env python3

import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
import numpy as np


class FakeBoundingBoxes:
    def __init__(self):
        self.bounding_box_pub = rospy.Publisher("/bounding_boxes", BoundingBoxArray, queue_size=1)
        rospy.Subscriber('/filtered_detector/jsk_bboxes', BoundingBoxArray, self.bounding_box_callback)
    
    def bounding_box_callback(self, bounding_box_msg):
        '''generate fake bounding boxes in reference to the bounding boxes from obstacle_detector'''
        bounding_box = bounding_box_msg.boxes
        #change data from map frame to rslidar frame
        bounding_box = np.array(list(bounding_box))
        #find the center of each bounding box
        bounding_box_centers = []
        for box in bounding_box:
            bounding_box_centers.append([box.pose.position.x, box.pose.position.y, box.pose.position.z])
        bounding_box_centers = np.array(bounding_box_centers)
        left_side = []
        right_side = []
        for bounding_box_center in bounding_box_centers:
            #if the y value of the center of the bounding box find the closest point to the robot
            if bounding_box_center[1] < 10 and bounding_box_center[1] > 0:
                left_side.append(bounding_box_center)
            elif bounding_box_center[1] > -10 and bounding_box_center[1] < 0:
                right_side.append(bounding_box_center)
        right, left = self.get_fake_bounding_boxes(left_side, right_side)
        #publish the fake bounding boxes to the bounding_boxes topic
        self.publish_fake_bounding_boxes(right, left)

    def publish_fake_bounding_boxes(self, right, left):
        '''
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        jsk_recognition_msgs/BoundingBox[] boxes
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
        geometry_msgs/Pose pose
            geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
            geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
        geometry_msgs/Vector3 dimensions
            float64 x
            float64 y
            float64 z
        float32 value
        uint32 label
        '''
        
        bounding_box_array = BoundingBoxArray()
        bounding_box_array.header.stamp = rospy.Time.now()
        bounding_box_array.header.frame_id = 'rslidar'
        for i in range(len(right)):
            bounding_box = BoundingBox()
            bounding_box.header.stamp = rospy.Time.now()
            bounding_box.header.frame_id = 'rslidar'
            bounding_box.pose.position.x = right[i][0]
            bounding_box.pose.position.y = right[i][1]
            bounding_box.pose.position.z = right[i][2]
            bounding_box.dimensions.x = 1
            bounding_box.dimensions.y = 1
            bounding_box.dimensions.z = 1
            bounding_box_array.boxes.append(bounding_box)
        for i in range(len(left)):
            bounding_box = BoundingBox()
            bounding_box.header.stamp = rospy.Time.now()
            bounding_box.header.frame_id = 'rslidar'
            bounding_box.pose.position.x = left[i][0]
            bounding_box.pose.position.y = left[i][1]
            bounding_box.pose.position.z = left[i][2]
            bounding_box.dimensions.x = 1
            bounding_box.dimensions.y = 1
            bounding_box.dimensions.z = 1
            bounding_box_array.boxes.append(bounding_box)
        self.bounding_box_pub.publish(bounding_box_array)

    def get_fake_bounding_boxes(self, left_side, right_side):
        left_x = np.arange(0, 40, 2)
        right_x = np.arange(0, 40, 2)
        left_row = []
        right_row = []
        for i in left_x:
            left_line = [i,left_side[0][1],left_side[0][2]]
            left_row.append(left_line)
        for i in right_x:
            right_line = [i,right_side[0][1],right_side[0][2]]
            right_row.append(right_line)
        return right_row, left_row
                

if __name__ == '__main__':
    rospy.init_node('fake_bounding_boxes')
    fake_bounding_boxes = FakeBoundingBoxes()
    rospy.spin()