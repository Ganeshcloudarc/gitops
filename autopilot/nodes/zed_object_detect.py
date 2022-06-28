#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from zed_interfaces.msg import ObjectsStamped
from ackermann_msgs.msg import AckermannDrive


class ZedObjectDetect:
    def __init__(self):
        cam_name = rospy.get_param("camera_name", "zed2i")

        self.zed_obj_status_pub = rospy.Publisher('zed_obj_status', Bool, queue_size=1)
        '''
        ## zed_obj_status
        True - obstacle prasent in the range
        False - No active obstacle prasent in the range
        '''

        rospy.Subscriber(
            f"{cam_name}/zed_node/obj_det/objects", ObjectsStamped, self.obj_callback)

        self.ack_msg = AckermannDrive()
        self.obstace_stop_distance = rospy.get_param("max_stop_distance_x", 3) # in meters
        self.y_stop = rospy.get_param("max_stop_distance_y", 1)


    def obj_callback(self, data):
        objects = data.objects
        count = 0
        if len(objects) > 0:
            for obj in objects:
                if (obj.position[0] <= self.obstace_stop_distance
                        and abs(obj.position[1]) < self.y_stop
                ):
                    count += 1
            rospy.logdebug("obstacles in range: %s",str(count))
            if count > 0:
                self.object_in_range = True
                self.zed_obj_status_pub.publish(Bool(data=self.object_in_range))
            else:
                self.object_in_range = False
                self.zed_obj_status_pub.publish(Bool(data=self.object_in_range))
        else:
            self.object_in_range = False
            rospy.logdebug("No object detected in the frame")
            self.zed_obj_status_pub.publish(Bool(data=self.object_in_range))

if __name__ == "__main__":
    rospy.init_node("zed_object_detect")
    des = ZedObjectDetect()
    rospy.spin()
