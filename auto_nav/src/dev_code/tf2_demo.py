#!/usr/bin/env python
"""Simple tf2 demo.  Print the current distance from the robot to the
origin of the odom frame.

Author: Nathan Sprague
Version: 01/2019
"""

import math
import rospy
import tf2_ros
import tf2_geometry_msgs

class TFDemo(object):
    """ Simple tf demo node. """

    def __init__(self):
        """ Initialize the tf2 demo node. """
        rospy.init_node('tf2_demo')

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

    def connvert_path(self, path_msg):

        

            # Create the point (0,0,0) stamped with the robot's frame...
            point_base = tf2_geometry_msgs.PointStamped()
            point_base.header.frame_id = 'base_link'
            point_base.header.stamp = rospy.get_rostime()

            # Now transform the point into the /odom frame...
            try:
                # The available transforms may be running behind the time
                # stamp on the data.  tf will raise an extrapolation exception
                # if we ask it to transform a point with a "future" time stamp.
                # This call waits (up to 1.0 second) for the necessary
                # transform to become available.

                point_odom = self.tf_buffer.transform(point_base, 'map',
                                                      rospy.Duration(1.0))

                cur_distance = math.sqrt(point_odom.point.x**2 +
                                         point_odom.point.y**2)

                rospy.loginfo("Distance from origin: {}".format(cur_distance))
                rospy.sleep(.5)

            except tf2_ros.TransformException as e:
                print(type(e))
                print("(May not be a big deal.)")

if __name__ == '__main__':
    TFDemo()