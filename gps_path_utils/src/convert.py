#!/usr/bin/env python3
from math import *
from fastkml import kml 
import rospy 
import rospkg
import json,sys
from fastkml import geometry
# import shapely
# from shapely.geometry import Point,Polygon,LineString
from autopilot_utils.geonav_conversions import ll2xy, xy2ll
from nav_msgs.msg import Odometry
from autopilot_msgs.msg import Trajectory, TrajectoryPoint
from rospy_message_converter import message_converter
from geometry_msgs.msg import Point, PoseArray, Pose, Quaternion
from geojson import LineString
from autopilot_utils.pose_helper import distance_btw_poses, get_yaw, angle_btw_poses, yaw_to_quaternion
import cubic_spline_planner 
import numpy as np
import math 
from scipy.interpolate  import *
from utils.plot import plot_curvature
import matplotlib.pyplot as plt 
from json import dump
file = "ma"

odom_list=[]

class Gps_Waypoints:
    # print('gps_waypoints')

    def __init__(self,mission_file,kml_file):
        # data=json.load(open(kml_file))
        ros_pack= rospkg.RosPack()
        # print('python')
        self.kml_file = rospy.get_param('gps_path_utils/kml_files', str(file)+'.kml')
        self.kml_file_dir = ros_pack.get_path('gps_path_utils') + "/kml_files/" + str(kml_file)
        self.mission_file = rospy.get_param('gps_path_utils/mission_files', str(file)+'.json') 
        self.mission_file_dir = ros_pack.get_path('gps_path_utils') + "/mission_files/" + str(mission_file)
        
        try:
            data = json.load(open(mission_file))
        except Exception as error:
            rospy.logerr('Error in the reading mission file' + str(error)) 
        
        # reading kml file 
        with open(kml_file,'r') as geo_fence :
                doc = geo_fence.read().encode('utf-8') 
                k = kml.KML() 
                k.from_string(doc)
                geo_fence_coordinates=[]  
                for i in k.features():
                        print("{},{}".format(i.name, i.description)) 
                        for j in i.features():
                                if isinstance(j.geometry, geometry.Polygon):
                                        polygon = j.geometry 
                                        for coordinates in polygon.exterior.coords: 
                                                geo_fence_coordinates.append(coordinates)
                                        print(geo_fence_coordinates)
                                        print('length',len(geo_fence_coordinates))

        ax=[] 
        ay=[]
        xx=[] 
        yy=[]
        ll_coordinates=[]
        gps_list=[]

        # latitudes longitudes to xy coordinates
        for i in range(len(geo_fence_coordinates)):
            x,y = ll2xy(geo_fence_coordinates[i][1],geo_fence_coordinates[i][0],geo_fence_coordinates[0][1],geo_fence_coordinates[0][0])
            ax.append(x) 
            ay.append(y) 
            gps_list.append([x,y]) 
        gps_list.append([gps_list[0][0],gps_list[0][1]])
        ax.append(ax[1])
        ay.append(ay[1])

        xx.extend([gps_list[0][0]])
        yy.extend([gps_list[0][1]])

        # # linear interpolation method
        # for i in range(0,len(gps_list)-1):
        #     p1=[ax[i+1-1],ay[i+1-1]]
        #     p2=[ax[i+1],ay[i+1]]
        #     for j in range(10000):
    
        #         distance = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
        #         changex = p2[0] - p1[0] 
        #         changey = p2[1] - p1[1]
        #         d = 5

        #         if distance >= 5:

        #             x1 = p1[0] + (d/distance) * changex
        #             y1 = p1[1] +  (d/distance) * changey
        #             dis = math.sqrt((p2[1]-y1)**2 + (p2[0]-x1)**2)
                    
        #             p1[0] = x1
        #             p1[1] = y1                 xx.append(x1)
        #                 yy.append(y1) 
        #             n = len(xx)+len(yy)
        #             m =[]
        #             m.append(n)
        #             print("n:",n)
                   

        # # cubic_Interpolation_method 
        # cy, cx,cyaw, ck, s = cubic_spline_planner.calc_spline_course(yy,xx,ds=0.1)
        # for x,y,yaw in zip(cy,cx,cyaw):
        #         odom = Odometry()
        #         odom.pose.pose.orientation = yaw_to_quaternion(yaw)
        #         odom.pose.pose.position.x =x
        #         odom.pose.pose.position.y =y 
        #         odom_list.append(message_converter.convert_ros_message_to_dictionary(odom))
        # print("Number of points at every 10 cm in path:",len(odom_list))
        # print("Distance covered by the path:",((len(cy))-1)/10,"meters")
        # print("LOOK AT THE GRAPH FOR THE PLOTTING GRAPH")
        
        # final_points_list=[]
        # for i in range(len(odom_list)-1):
        #         llx,lly = xy2ll(cx[i],cy[i],geo_fence_coordinates[0][1],geo_fence_coordinates[0][0])
        #         final_points_list.append([lly,llx])

        # line_string = LineString(final_points_list) 
        # line_string['odometry'] = odom_list 
        # line_string['gps_coordinates'] = [{'altitude':28}]
        # miss_file =  "/home/suprabha/autopilot_ws/src/autopilot_boson/gps_path_utils/mission_files/"+str(file)+'.json'
        # with open(miss_file, 'w') as f:
        #         dump(line_string, f)
        #         print('json file saved')


        # plt.plot(cx,cy,"xb",label="linear interpolation with 50cm ")
        # plt.text(10,25, 'blue-> json plt after interpolation', fontsize = 15) 
        # plt.text(10,20, 'red-> manual 4 point plt', fontsize = 15)
        # plt.xlabel("X-axis", fontsize = 15)
        # plt.ylabel("Y-axis", fontsize = 15)
        # plt.plot(ax,ay,"r",label="manual plot")
        # plt.show()

        #                 xx.append(x1)
        #                 yy.append(y1) 
        #             n = len(xx)+len(yy)
        #             m =[]
        #             m.append(n)
        #             print("n:",n)
                   

        # # cubic_Interpolation_method 
        # cy, cx,cyaw, ck, s = cubic_spline_planner.calc_spline_course(yy,xx,ds=0.1)
        # for x,y,yaw in zip(cy,cx,cyaw):
        #         odom = Odometry()
        #         odom.pose.pose.orientation = yaw_to_quaternion(yaw)
        #         odom.pose.pose.position.x =x
        #         odom.pose.pose.position.y =y 
        #         odom_list.append(message_converter.convert_ros_message_to_dictionary(odom))
        # print("Number of points at every 10 cm in path:",len(odom_list))
        # print("Distance covered by the path:",((len(cy))-1)/10,"meters")
        # print("LOOK AT THE GRAPH FOR THE PLOTTING GRAPH")
        
        # final_points_list=[]
        # for i in range(len(odom_list)-1):
        #         llx,lly = xy2ll(cx[i],cy[i],geo_fence_coordinates[0][1],geo_fence_coordinates[0][0])
        #         final_points_list.append([lly,llx])

        # line_string = LineString(final_points_list) 
        # line_string['odometry'] = odom_list 
        # line_string['gps_coordinates'] = [{'altitude':28}]
        miss_file =  "/home/suprabha/autopilot_ws/src/autopilot_boson/gps_path_utils/mission_files/"+str(file)+'.json'
        # with open(miss_file, 'w') as f:
        #         dump(line_string, f)
        #         print('json file saved')


        # plt.plot(cx,cy,"xb",label="linear interpolation with 50cm ")
        # plt.text(10,25, 'blue-> json plt after interpolation', fontsize = 15) 
        # plt.text(10,20, 'red-> manual 4 point plt', fontsize = 15)
        # plt.xlabel("X-axis", fontsize = 15)
        # plt.ylabel("Y-axis", fontsize = 15)
        # plt.plot(ax,ay,"r",label="manual plot")
        # plt.show()

        
if __name__ == "__main__":
    rospy.init_node("gps_waypoints")
    mission_file =  str(file)+'.json'
    if '.json' in mission_file:
        pass
    else:
        mission_file = mission_file + '.json'
        rospy.set_param('/gps_path_utils/mission_files', mission_file)
    try:
        ros_pack = rospkg.RosPack()
        mission_file_dir = ros_pack.get_path('gps_path_utils') + "/mission_files/" + str(mission_file)
    except Exception as e:
        rospy.logwarn("Please source autopilot package" + str(e))
        # sys.exit("Please source autopilot package" + str(e))
    kml_file = rospy.get_param('/gps_path_utils/kml_files', str(file)+'.kml')
    if '.kml' in kml_file:
        pass 
    else:
        kml_file = kml_file + '.kml' 
        rospy.set_param('/gps_path_utils/kml_files', kml_file) 
    try: 
        ros_pack = rospkg.RosPack() 
        kml_file_dir =  ros_pack.get_path('gps_path_utils') + "/kml_files/" + str(kml_file) 
    except Exception as e: 
        rospy.logwarn("Please source autopilot package" + str(e)) 
        # sys.exit("Please source autopilot package" + str(e))
    print()
    a = Gps_Waypoints(mission_file_dir, kml_file_dir)
    rospy.spin()