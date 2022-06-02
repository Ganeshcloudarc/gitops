#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from zed_interfaces.msg import ObjectsStamped
from ackermann_msgs.msg import AckermannDrive


class ZedObjectDetect:
    def __init__(self):
        cam_name = rospy.get_param("camera_name", "zed2i")
        cmd_topic_in = "/vehicle/cmd_drive_nosafe" # rospy.get_param("/patrol/control_topic")

        cmd_topic_out = '/vehicle/cmd_drive_safe'
        self.cmd_in_data = AckermannDrive()
        self.time_on_zed = rospy.Time.now().secs
        self.object_in_range = True  # defaluts to True

        self.cmd_pub = rospy.Publisher(cmd_topic_out, AckermannDrive, queue_size=10)

        rospy.Subscriber(cmd_topic_in, AckermannDrive, self.cmd_in_callback)
        rospy.Subscriber(
            f"{cam_name}/zed_node/obj_det/objects", ObjectsStamped, self.obj_callback)

        self.ack_msg = AckermannDrive()
        self.obstace_stop_distance = 3
        self.y_stop = 1

    def cmd_in_callback(self, data):
        tym = rospy.Time.now().secs
        self.cmd_in_data = data
        if self.time_on_zed - 1 <= tym <= self.time_on_zed + 1:
            if self.object_in_range:
                rospy.loginfo("object_in_range  -True")
                self.cmd_in_data.speed = 0
                self.cmd_in_data.jerk = 1
                self.cmd_pub.publish(self.cmd_in_data)
            else:
                rospy.loginfo("object_in_range  -False")
                self.cmd_pub.publish(self.cmd_in_data)
        else:
            diff = tym - self.time_on_zed

            if diff > 5:
                rospy.logwarn("No update on zed object detections from %s secs", diff)
                rospy.logwarn("Stopping vehicle")
                self.cmd_pub.publish(self.ack_msg)
            else:
                rospy.logwarn("No update on zed object detections from %s secs", diff)
                rospy.logwarn("Continuing vehicle")
                if self.object_in_range:
                    rospy.loginfo("object_in_range  -True")

                    self.cmd_in_data.speed = 0
                    self.cmd_in_data.jerk = 1
                    self.cmd_pub.publish(self.cmd_in_data)
                else:
                    rospy.loginfo("object_in_range  -False")
                    self.cmd_pub.publish(self.cmd_in_data)

    def obj_callback(self, data):
        self.time_on_zed = data.header.stamp.secs
        objects = data.objects
        count = 0
        if len(objects) > 0:
            for obj in objects:
                if (obj.position[0] <= self.obstace_stop_distance
                        and abs(obj.position[1]) < self.y_stop
                ):
                    count += 1
            if count > 0:
                self.object_in_range = True
            else:
                self.object_in_range = False
        else:
            self.object_in_range = False
        # print(self.object_in_range)


if __name__ == "__main__":
    rospy.init_node("zed_object_detect")
    des = ZedObjectDetect()
    rospy.spin()
