#!/usr/bin/python
# -*- coding: utf-8 -*-

# tf2_ros_example.py: example showing how to use tf2_ros API
# Author: Ravi Joshi
# Date: 2018/12/6

# import modules
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped, Vector3, Vector3Stamped


# print(dir(tf2_geometry_msgs))

def transform_point(transformation, point_wrt_source):
    point_wrt_target = \
        tf2_geometry_msgs.do_transform_point(PointStamped(point=point_wrt_source),
                                             transformation).point
    return [point_wrt_target.x, point_wrt_target.y, point_wrt_target.z]


def transform_point_list(transformation, point_wrt_source):
    point_wrt_target = \
        tf2_geometry_msgs.do_transform_point(PointStamped(point=Point(point_wrt_source[0],point_wrt_source[1],point_wrt_source[2])),
                                             transformation).point
    return [point_wrt_target.x, point_wrt_target.y, point_wrt_target.z]

def get_transformation(source_frame, target_frame,
                       tf_cache_duration=2.0):
    tf_buffer = tf2_ros.Buffer(rospy.Duration(tf_cache_duration))
    tf2_ros.TransformListener(tf_buffer)

    # get the tf at first available time
    try:
        transformation = tf_buffer.lookup_transform(target_frame,
                                                    source_frame, rospy.Time(0), rospy.Duration(0.1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        rospy.logerr('Unable to find the transformation from %s to %s '
                     , source_frame, target_frame)
        return None
    return transformation


def transform_vector(transformation, point_wrt_source):
    print("point_wrt_source",point_wrt_source)
    point_wrt_target = \
        tf2_geometry_msgs.do_transform_vector3(Vector3Stamped(vector=Vector3(point_wrt_source[0], point_wrt_source[1],
                                                                             point_wrt_source[2])), transformation)
    print(point_wrt_target)
    return [point_wrt_target.vector.x, point_wrt_target.vector.y, point_wrt_target.vector.z]


def main():
    # initialize ros node
    rospy.init_node('tf2_ros_example', anonymous=True)

    # define source and target frame
    source_frame = 'base_link'
    target_frame = 'map'

    # define a source point
    point_wrt_source = Point(1.1, 1.2, 2.3)
    p = [34, 2, 3]

    transformation = get_transformation(source_frame, target_frame)
    print(transformation)
    point_wrt_target = transform_point(transformation, point_wrt_source)
    arr = transform_vector(transformation, p)
    b = transform_point_list(transformation, p)
    print(point_wrt_target)
    print(arr)
    print(b)


if __name__ == '__main__':
    main()

'''
https://programtalk.com/python-more-examples/tf2_geometry_msgs.do_transform_vector3.vector/


        # transform to global frame
        if self.global_frame is not None:
            try:
                trans = self.tf_buffer.lookup_transform(self.global_frame, msg.header.frame_id, rclpy.time.Time())
                msg.header.frame_id = self.global_frame
                for i in range(len(detections)):
                    # transform position (point)
                    p = PointStamped()
                    p.point = detections[i].position
                    detections[i].position = do_transform_point(p, trans).point
                    # transform velocity (vector3)
                    v = Vector3Stamped()
                    v.vector = detections[i].velocity
                    detections[i].velocity = do_transform_vector3(v, trans).vector
                    # transform size (vector3)
                    s = Vector3Stamped()
                    s.vector = detections[i].size
                    detections[i].size = do_transform_vector3(s, trans).vector

            except LookupException:
                self.get_logger().info('fail to get tf from {} to {}'.format(msg.header.frame_id, self.global_frame))
                return
                
                
                '''
