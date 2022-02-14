#!/usr/bin/env python3
from geonav_conversions import *
import rospy
import rospkg
from nav_msgs.msg import  Path
from geometry_msgs.msg import Pose, PoseStamped,Quaternion
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import json
import math
import sys
class PathPubGps:
    def __init__(self,mission_file):      
        self.path_pub = rospy.Publisher('/gps_path', Path, queue_size=1,latch=True)
        self.path = Path()
        
        try:
           data = json.load(open(mission_file))
        except Exception as e:
            rospy.logwarn('Error In Reading mission file '+str(e))
            sys.exit('Error In Reading mission file '+str(e))
        home_lat = data['coordinates'][0][1]
        home_long = data['coordinates'][0][0]

        for i in range(len(data['coordinates'])):
            lon,lat = data['coordinates'][i][0],data['coordinates'][i][1]
            heading = data['heading_list'][i]

            print(lon,lon,math.radians(heading))
            # continue
            x,y = ll2xy(lat, lon, home_lat, home_long)
            # print(x,y)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y #euler_from_quaternion
            # print('heading',heading)
            pose.pose.orientation = Quaternion(*(quaternion_from_euler(0.0,0.0,math.radians(heading)))) # quaternion_from_euler(1.5707, 0, -1.5707)#
            #Set a atributes of the msg
            pose.header.seq = self.path.header.seq + 1
            self.path.header.frame_id="map"
            self.path.header.stamp=rospy.Time.now()
            pose.header.stamp = self.path.header.stamp
            self.path.poses.append(pose)
        self.path_pub.publish(self.path)
        rospy.loginfo('Path of '+str(i)+ ' points are published to the topic /gps_path from file '+str(mission_file))
       


if __name__ =="__main__":
    rospy.init_node("gps_path_publisher")
    mission_file = rospy.get_param('/patrol/mission_file','default.json')
    try:
        rospack = rospkg.RosPack()
        mission_file_dir = rospack.get_path('autopilot_boson') + "/mission_files/"+str(mission_file)
    except Exception as e:
        rospy.logwarn("Please source autopilot_boson package"+ str(e))
        sys.exit("Please source autopilot_boson package"+ str(e))
    a=PathPubGps(mission_file_dir)
    rospy.spin()








