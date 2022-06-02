#!/usr/bin/env python3
try:
    import rospy
    from mavros_msgs.msg import RCIn
    from ackermann_msgs.msg import AckermannDrive
    import time
    import numpy as np
except Exception as e:
    print('No module named :', str(e))
    exit(e)


class ControlModeSelector:
    def __init__(self):

        self.default_mode = None
        self.rc_data = None
        self.auto_mode = None
        self.manual_mode = None
        self.time_on_cmd_drive = time.time()
        self.cmd_drive = AckermannDrive()
        self.rc_stop = None
        self.wait_time_limit = 1
        rc_enable = rospy.get_param('patrol/rc_control', False)
        if rc_enable:
            self.ack_sub_topic = '/vehicle/cmd_drive_rc'

        od_enable = rospy.get_param('patrol/od_enable', True)
        if od_enable:
            self.ack_pub_topic = '/vehicle/cmd_drive_nosafe'
        else:
            self.ack_pub_topic = '/vehicle/cmd_drive_safe'

        self.ackermann_publisher = rospy.Publisher(self.ack_pub_topic, AckermannDrive, queue_size=10)
        rospy.Subscriber(self.ack_sub_topic, AckermannDrive, self.cmd_drive_callback)
        rospy.Subscriber('/mavros/rc/in', RCIn, self.rc_callback)
        self.main_loop()

    def cmd_drive_callback(self, data):
        self.time_on_cmd_drive = time.time()
        self.cmd_drive = data

    def rc_callback(self, data):
        self.rc_data = data
        if data.channels[5] >= 1010:
            self.rc_stop = 1
        else:
            self.rc_stop = 0
        if 950 < data.channels[4] < 1050:
            self.manual_mode = False
            self.auto_mode = False
            self.default_mode = True
        elif 1450 < data.channels[4] < 1550:
            self.manual_mode = False
            self.auto_mode = True
            self.default_mode = False
        elif 1950 < data.channels[4] < 2050:
            self.manual_mode = True
            self.auto_mode = False
            self.default_mode = False
        # self.manual_control()

    def main_loop(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.default_mode:
                print("waiting for mode select from rc trasmitor")
                self.send_ack_msg(0, 0, 0)
            elif self.manual_mode:
                print('Manual mode')
                self.manual_control()

            elif self.auto_mode:
                now = time.time()
                print("auto mode")
                if now - self.time_on_cmd_drive < self.wait_time_limit:
                    if self.cmd_drive.jerk or self.rc_stop:
                        stop = 1
                        print("stopping")
                    else:
                        print("not stopping")
                        stop = 0

                    self.send_ack_msg(self.cmd_drive.steering_angle, self.cmd_drive.speed, stop)
                else:
                    print('time out from cmd_drive')
                    self.send_ack_msg(0, 0, self.rc_stop)
            else:
                print(" No valid option selected")
                self.send_ack_msg(0, 0, 0)
            r.sleep()

    def manual_control(self):
        throttle = self.rc_data.channels[1]
        steering = self.rc_data.channels[3]
        rc_break = self.rc_data.channels[5]
        throttle = np.interp(throttle, [1000, 2000], [-1, 1])
        steering = np.interp(steering, [1000, 2000], [-30, 30] )

        if rc_break >= 1010:
            rc_break = 1
        else:
            rc_break = 0
        print(throttle, steering, rc_break)
        self.send_ack_msg(steering, throttle, rc_break)

    def send_ack_msg(self, steering_angle, speed, jerk):
        ackermann_msg = AckermannDrive()
        ackermann_msg.steering_angle = steering_angle
        ackermann_msg.speed = speed
        ackermann_msg.jerk = jerk
        self.ackermann_publisher.publish(ackermann_msg)


if __name__ == '__main__':
    rospy.init_node('control_mode_selector')
    cms = ControlModeSelector()
    rospy.spin()
