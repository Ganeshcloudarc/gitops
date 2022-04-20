import rospy
from nav_msgs.msg import Odometry, Path
from rospy_message_converter import message_converter # convert_dictionary_to_ros_message
import sys
import rospkg
import json
from geometry_msgs.msg import Pose, PoseStamped,Quaternion
import time 

class OdomVis:
    def __init__(self,mission_file):
        self.path_pub= rospy.Publisher('/odom_to_path',Odometry,queue_size= 10)
    
        try:
           data = json.load(open(mission_file))
        except Exception as e:
            rospy.logwarn('Error In Reading mission file '+str(e))
            sys.exit('Error In Reading mission file '+str(e))

        if len(data['coordinates']) == 0:
            rospy.logwarn("No way points on mission file "+ str(mission_file))
            sys.exit("No way points on mission file"+ str(mission_file))
        else:
            rospy.loginfo("Success in reading mission file")
            pass
        self.path = Path()

        for i in range(len(data['coordinates'])):
            
            # pose = PoseStamped()
            # print(data['odometry'][i]['pose'])
            self.path_pub.publish(message_converter.convert_dictionary_to_ros_message('nav_msgs/Odometry',data['odometry'][i]))
            time.sleep(0.01)


if __name__ =="__main__":
    rospy.init_node("gps_path_publisher")
    mission_file = rospy.get_param('/patrol/mission_file','default.json')
    if '.json' in mission_file:
        pass
    else:
        mission_file = mission_file+'.json'
        rospy.set_param('/patrol1/mission_file', mission_file)
    try:
        rospack = rospkg.RosPack()
        mission_file_dir = rospack.get_path('autopilot_boson') + "/mission_files/"+str(mission_file)
    except Exception as e:
        rospy.logwarn("Please source autopilot_boson package"+ str(e))
        sys.exit("Please source autopilot_boson package"+ str(e))
    a=OdomVis(mission_file_dir)
    rospy.spin()



