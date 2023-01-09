#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from can_msgs.msg import Frame
import time
#can_msgs/Frame

class vsCanParser:
    def __init__(self):
        self.battery_soc = None
        self.demanded_steering_angle = None
        self.current_steering_angle = None
        self.log_throttle_period = 0.1 # limit the logs
        self.start_time = None
        self.is_steering_stuck = None
        self.motor_rpm = None
        '''
        Below variables explanation.
        The steering current angle should reach the demanded angle with tolerance of
        +/- steering_diff_th with in steering_stuck_time_th.
        
        This means, within the steering_stuck_time_th threshold, 
        the diff between demand and current angle should be less than steering_diff_th
        '''
        self.steering_diff_th = rospy.get_param('/vehicle_safety/STEER_DIFF_TH',10) # degrees 
        self.steering_stuck_time_th = rospy.get_param('/vehicle_safety/STEER_STUCK_TIME_TH',2) #seconds

        rospy.Subscriber("/received_messages", Frame, self.can_callback)
        
        # Enable below subscriber for pilot status
        
        # rospy.Subscriber("pilot_status",String,self.pilot_cb)
    
    def pilot_cb(self,data):
        '''
        This pilot callback is for debug purpose only
        '''
        data = data.data
        rospy.loginfo(data)
        
    def to_little(self,val):
        """Function used to flip the data to Little Endian
        Arg :
            val(hex value) : hex value to flip to Little Endian
        Return :
            val : Little Endian value
        """

        little_hex = bytearray.fromhex(val)
        little_hex.reverse()
        # print("Byte array format:", little_hex)
        self.str_little = "".join(format(x, "02x") for x in little_hex)
        return self.str_little
    
    def twos_comp(self, val, bits):
        """Generates the twos's complement values for negative values
        Arg :
            val(int) : value from CAN channel
            bits(int): it gives the values to be left shifted to get two's complement.
        return :
            val(int) : negative value.
        """

        if (val & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
            val = val - (1 << bits)  # compute negative value
        return val
    
    def can_callback(self,data):
        if hex(data.id) == "0x183": #Batt soc
            raw_byte = bytearray(data.data)
            hex_value = raw_byte.hex()
            self.battery_soc =(int(self.to_little(hex_value[0:2]), 16) * 0.5)
            rospy.loginfo_throttle(self.log_throttle_period,"Batt soc: {}".format(self.battery_soc))
        
        if hex(data.id) == "0x292": # current steering angle
            raw_byte = bytearray(data.data)
            hex_value = raw_byte.hex()
            self.current_steering_angle = (int(hex_value[2:6], 16)) / 4
            rospy.loginfo_throttle(self.log_throttle_period,"Current Steering Angle: {}".format(self.current_steering_angle))
        
        if hex(data.id) == "0x298": # demanded steering angle
            raw_byte = bytearray(data.data)
            hex_value = raw_byte.hex()
            self.demanded_steering_angle = int(hex_value[2:4], 16)
            rospy.loginfo_throttle(self.log_throttle_period,"Demanded Steering Angle: {}".format(self.demanded_steering_angle))
            if not None in [self.demanded_steering_angle,self.current_steering_angle]:
                # In below logic, If demanded angle is zero, there is something from tracking algorithm but not the steering issue.
                if abs(self.demanded_steering_angle-self.current_steering_angle) > self.steering_diff_th and self.demanded_steering_angle != 0:
                    if self.start_time == None:
                        self.start_time = time.time()
                    if time.time() - self.start_time > self.steering_stuck_time_th:
                        rospy.logerr("STUCK")
                        self.is_steering_stuck = True
                    else:
                        self.is_steering_stuck = False
                else:
                    self.start_time = None

        if hex(data.id) == "0x202":
            raw_byte = bytearray(data.data)
            hex_value = raw_byte.hex()  # converting bytearray to hex format

            self.motor_rpm = self.twos_comp(
                int(self.to_little(hex_value[8:]), 16), 32
            )
            rospy.loginfo_throttle(self.log_throttle_period,"Motor RPM : {} ".format(self.motor_rpm))
        
if __name__ == '__main__':
    rospy.init_node('vsCanParser')
    vscp = vsCanParser()
    rospy.spin()