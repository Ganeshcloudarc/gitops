#!/usr/bin/env python3
from math import *
from fastkml import kml 
import rospy 
import rospkg
import json,sys
from fastkml import geometry
import shapely 
from  scipy.interpolate import interp1d
# from shapely.geometry import Point,Polygon,LineString
from autopilot_utils.geonav_conversions import ll2xy, xy2ll
from nav_msgs.msg import Odometry
from autopilot_msgs.msg import Trajectory, TrajectoryPoint
from rospy_message_converter import message_converter
from geometry_msgs.msg import Point, PoseArray, Pose, Quaternion
from geojson import LineString
from autopilot_utils.pose_helper import distance_btw_poses, get_yaw, angle_btw_poses, yaw_to_quaternion, normalize_angle
import cubic_spline_planner 
import numpy as np
import math 
from scipy.interpolate  import *
from utils.plot import plot_curvature
import matplotlib.pyplot as plt 
from json import dump
file = "ma(2)"
import geometry_msgs.msg as gmsg

odom_list=[]
kml_list_x=[] 
kml_list_y=[] 
x_updated=[] 
y_updated=[] 
ax=[]
ay=[]
kml_list=[]
final_points_list=[] 

class KmlToMissionFile:
    # print('gps_waypoints')

    def __init__(self, kml_file):
        # reading kml file 5
        self.linear_sampling_dis = rospy.get_param("linear_sampling_dis", 1)

        self.geo_polygon_pub = rospy.Publisher('/kml_coords', gmsg.PolygonStamped,queue_size=1, latch=True)
        kml_coords=[]  
        try: 
            with open(kml_file,'r') as geo_fence :
                    doc = geo_fence.read().encode('utf-8') 
                    k = kml.KML() 
                    k.from_string(doc)
                    for i in k.features():
                            print("{},{}".format(i.name, i.description)) 
                            for j in i.features():
                                    if isinstance(j.geometry, geometry.Polygon):
                                            polygon = j.geometry 
                                            for coordinates in polygon.exterior.coords: 
                                                        kml_coords.append(coordinates[:-1])                                       
        except Exception as e:
            rospy.logerr('Error in reading %s file ', kml_file)

        
        #print("value",kml_coords)
        # latitudes longitudes to xy coordinates
        kml_coords.reverse()
        kml_coords = kml_coords[1:]
        kml_coords.append(kml_coords[0])

        #print("kml_coords after: ", kml_coords)
        home_lat = kml_coords[0][1]
        home_lon = kml_coords[0][0]
        
        polygon_st = gmsg.PolygonStamped()
        polygon_st.header.frame_id = 'map'
        for lon, lat in kml_coords:
            point = gmsg.Point()
            point.x, point.y = ll2xy(lat, lon, home_lat, home_lon)
            polygon_st.polygon.points.append(point)
            kml_list.append([point.x,point.y])
            kml_list_x.append(point.x) 
            kml_list_y.append(point.y) 
            # kml_list.append([kml_list_x[0],kml_list_y[0]])

        self.geo_polygon_pub.publish(polygon_st) 

        # linear_interpolation_method 
        path_resolution = 5
        x_updated.extend([kml_list[0][0]])
        y_updated.extend([kml_list[0][1]])
        intreploted_x = []
        intreploted_y = []
        intreploted_x.append(kml_list[0][0])
        intreploted_y.append(kml_list[0][1])
        

        for i in range(len(kml_list)-1):
            x,y = kml_list[i][0], kml_list[i][1]

            x1,y1 = kml_list[i+1][0], kml_list[i+1][1]

            changex = x1-x
            changey = y1-y
            dist = math.sqrt(changey**2+changex**2)
            angle = math.atan2(changey,changex)
            number_points_between = dist / self.linear_sampling_dis 
            res_up = (float(number_points_between)-int(number_points_between))/int(number_points_between)
            final__res = res_up + self.linear_sampling_dis

            for _ in range(int(number_points_between)): 
                if j==0:
                    px,py = kml_list[i][0], kml_list[i][0]
                else:
                     px = intreploted_x[-1]
                     py = intreploted_y[-1]
                px_upd = px + final__res * math.cos(angle) 
                # print("intx",intreploted_x)
        # print("inty",intreploted_y)
                py_upd = py + final__res * math.sin(angle)
                intreploted_x.append(px_upd)
                intreploted_y.append(py_upd)
                #print("dtgf",px_upd)
                #print("ay",py_upd)
        intreploted_x.append(intreploted_x[0])
        intreploted_y.append(intreploted_x[0])
        # print("intx",intreploted_x)
        # print("inty",intreploted_y)

        # cubic_interpolation_method                                                             
        cy, cx,cyaw, ck, s = cubic_spline_planner.calc_spline_course(intreploted_y,intreploted_x,ds=0.1)
    
        for x,y,yaw in zip(cy,cx,cyaw):
                odom = Odometry()
                odom.pose.pose.orientation = yaw_to_quaternion(yaw)
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom_list.append(message_converter.convert_ros_message_to_dictionary(odom))
        print("Number of points at every 10 cm in path:",len(odom_list))
        print("Distance covered by the path:",((len(cy))-1)/10,"meters")

        

        #  xy_to_ll 
        for i in range(len(odom_list)-1):
                 llx,lly = xy2ll(cx[i],cy[i],home_lat,home_lon)
                 final_points_list.append([lly,llx])

        #  json_convertion
        line_string = LineString(final_points_list)
        line_string['odometry'] = odom_list 
        line_string['gps_coordinates'] = [{'altitude':28}]
        miss_file =  "/home/suprabha/autopilot_ws/src/autopilot_boson/gps_path_utils/mission_files/"+str(file)+'.json'
        with open(miss_file, 'w') as f:
                dump(line_string, f)
                print('json file saved')

        # graph_plot 
        plt.plot(cx,cy,"b",label="linear interpolation with 50cm ")
        plt.plot(kml_list_x,kml_list_y,"xr",label="manual plot")
        plt.plot(px_upd,py_upd,"r",label="the mannual plot")
        plt.show()

if __name__ == "__main__":
    rospy.init_node("gps_waypoints")
    kml_file =  rospy.get_param('autopilot_boson/gps_path_utils/kml_file/',str(file)+'.kml')
    if '.kml' in kml_file:
        pass
    else:
        kml_file = kml_file + '.kml'
    try:
        ros_pack = rospkg.RosPack()
        mission_file_dir = ros_pack.get_path('gps_path_utils') + "/kml_files/" + str(kml_file)
    except Exception as e:
        # rospy.logwarn("Please source gps_path_utils package: " + str(e))
        sys.exit("Please source autopilot package" + str(e))
    a = KmlToMissionFile(mission_file_dir)
    rospy.spin()
