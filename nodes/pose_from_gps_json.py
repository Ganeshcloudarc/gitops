

from geonav_conversions import *
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int32
from tf.transformations import euler_from_quaternion
import json
import time
class PathPubGps:
    def __init__(self):
        
        data,data1 = None, None

        self.xAnt= 0.0
        self.yAnt= 0.0 
        self.count = 0
        
        self.pose_pub = rospy.Publisher('/gps_pose', PoseStamped, queue_size=1,latch=True)
        # self.path = Path()
        self.pose = Pose()
        
        data_file = '/home/boson/exp_ws/src/autopilot_boson/boson_console_ui/test_close.json'
        
        data = json.load(open(data_file))
        home_lat = data['coordinates'][0][1]
        home_long = data['coordinates'][0][0]
        
        for y,x in data['coordinates']:
            x,y = ll2xy(x, y, home_lat, home_long)
            print(x,y)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            # pose.pose.orientation = 
            #Set a atributes of the msg
            pose.header.seq = pose.header.seq + 1
            # self.path.header.frame_id="map"
            # self.path.header.stamp=rospy.Time.
            # now()
            pose.header.stamp = rospy.Time.now()
            # self.path.poses.append(pose)
            self.pose_pub.publish(pose)
            print('pubished')
            time.sleep(0.1)
            






    def pose_callback(self,data):
        self.pose_orientation_data = data.pose.orientation
        # self.heading_angle = euler_from_quaternion(data.orientation)[2]

    
    def gps_callback(self,data):
        latt = data.latitude
        long = data.longitude
        alti = data.altitude

        x,y = ll2xy(latt, long, self.home_gps_lat, self.home_gps_long)
        print(x,y)


        pose = PoseStamped()
        pose.header.frame_id = data.header.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = self.pose_orientation_data


        if (self.xAnt != pose.pose.position.x and self.yAnt != pose.pose.position.y):
                #Set a atributes of the msg
                pose.header.seq = self.path.header.seq + 1
                self.path.header.frame_id="map"
                self.path.header.stamp=rospy.Time.now()
                pose.header.stamp = self.path.header.stamp
                self.path.poses.append(pose)
                #Published the msg

        self.count=self.count+1

        rospy.loginfo("Valor del contador: %i" % self.count)
        # if cont>max_append:
        # 	path.poses.pop(0)

        self.path_pub.publish(self.path)
        self.xAnt=pose.pose.orientation.x
        self.yAnt=pose.pose.position.y
       


if __name__ =="__main__":
    rospy.init_node("gps_path_publisher")
    a=PathPubGps()
    rospy.spin()








