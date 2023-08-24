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
from autopilot_utils.pose_helper import distance_btw_poses, get_yaw, angle_btw_poses, yaw_to_quaternion, normalize_angle
import cubic_spline_planner 
import numpy as np
import math 
from scipy.interpolate  import *
from utils.plot import plot_curvature
import matplotlib.pyplot as plt 
from json import dump
file = "ma"
import geometry_msgs.msg as gmsg

odom_list=[]
xx = []
yy = []
ax = []
ay = []

class KmlToMissionFile:
    # print('gps_waypoints')

    def __init__(self, kml_file):
        # reading kml file 
        self.linear_sampling_dis = rospy.get_param("linear_sampling_dis", 5.2)

        self.geo_polygon_pub = rospy.Publisher('/geo_fence_polygon', gmsg.PolygonStamped,queue_size=1, latch=True)
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

       
        

        # latitudes longitudes to xy coordinates
        kml_coords.reverse()
        kml_coords = kml_coords[1:]
        kml_coords.append(kml_coords[0])

        print("kml_coords", kml_coords)
        home_lat = kml_coords[0][1]
        home_lon = kml_coords[0][0]
        
        polygon_st = gmsg.PolygonStamped()
        polygon_st.header.frame_id = 'map'
        for lon, lat in kml_coords:
            point = gmsg.Point()
            point.x, point.y = ll2xy(lat, lon, home_lat, home_lon)
            polygon_st.polygon.points.append([point.x,point.y])
            ax.append(point.x)
            ay.append(point.y)
        self.geo_polygon_pub.publish(polygon_st)
        #polygon_st.polygon.points = ([point.x,point.y])
       
        print("ydydygdyud",len(polygon_st.polygon.points))
        
        avg_thr = 5
        path_resolution = 5
        
        for i in range (len(polygon_st.polygon.points)-1):
           #dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2
            print(i)
            distance = math.sqrt((polygon_st.polygon.points[i][0] - polygon_st.polygon.points[i+1][0])**2 + (polygon_st.polygon.points[i][1] - polygon_st.polygon.points[i+1][1])**2 )
            second_x = ax[i+1]
            second_y = ay[i+1]

            print("distance: ",distance)
            ll_coordinates=[]
            if distance > avg_thr :
                rospy.logwarn("path end and start point are  %s meter apart,",str(dist)) 
                n_points =  distance // path_resolution 
                print("value of n_points: ",n_points)
                changex = polygon_st.polygon.points[i+1][0]-polygon_st.polygon.points[i][0] 
                changey = polygon_st.polygon.points[i+1][1]-polygon_st.polygon.points[i][1]
                angle = degrees(math.atan2(changey,changex)) 
                print("angle",angle) 
                for j in range(0,int(n_points)+1): 
                    px = second_x
                    py = second_y
                    px_upd = px + path_resolution * math.cos(angle) 
                    py_upd = py + path_resolution * math.sin(angle)
                    odom = Odometry()
                    odom.pose.pose.position.x =px_upd
                    odom.pose.pose.position.y =py_upd 
                    p_lst = py_upd
                    p_first = px_upd
                    # odom.pose.pose.position.z =rear_axle_center_height_from_ground
                    lat,lng = xy2ll(py_upd,px_upd, home_lat, home_lon)
                    xx.append(lat)
                    yy.append(lng) 
                    i+=1 
                    ll_coordinates.append([lat,lng])
        print("coordinates : ",ll_coordinates)


        # # cubic_Interpolation_method 
        # cy, cx,cyaw, ck, s = cubic_spline_planner.calc_spline_course(yy,xx,ds=0.1)
        for x,y,yaw in zip(cy,cx,cyaw):
                 odom = Odometry()
        #         odom.pose.pose.orientation = yaw_to_quaternion(yaw)
        #         odom.pose.pose.position.x =x
        #         odom.pose.pose.position.y =y 
        #         odom_list.append(message_converter.convert_ros_message_to_dictionary(odom))
        print("Number of points at every 10 cm in path:",len(odom_list))
        #print("Distance covered by the path:",((len(cy))-1)/10,"meters")
        print("LOOK AT THE GRAPH FOR THE PLOTTING GRAPH")
        
        # final_points_list=[]
        # for i in range(len(odom_list)-1):
        #         llx,lly = xy2ll(cx[i],cy[i],home_lat,home_lon)
        #         final_points_list.append([lly,llx])

        # line_string = LineString(final_points_list) 
        # line_string['odometry'] = odom_list 
        # line_string['gps_coordinates'] = [{'altitude':28}]
        # miss_file =  "/home/suprabha/autopilot_ws/src/autopilot_boson/gps_path_utils/mission_files/"+str(file)+'.json'
        # with open(miss_file, 'w') as f:
        #         dump(line_string, f)
        #         print('json file saved')


        #plt.plot(cx,cy,"xb",label="linear interpolation with 50cm ")
        plt.text(10,25, 'blue-> json plt after manipulation', fontsize = 15) 
        plt.text(10,20, 'red-> manual 4 point plt', fontsize = 15)
        plt.xlabel("X-axis", fontsize = 15)
        plt.ylabel("Y-axis", fontsize = 15)
        plt.plot(ax,ay,"r",label="manual plot")
        plt.plot(point.x,point.y,"r",label="manual plot")
        plt.plot()
        plt.show()


        
if __name__ == "__main__":
    rospy.init_node("gps_waypoints")
    kml_file =  rospy.get_param('/gps_path_utils/kml_file', "ma.kml")
    if '.kml' in kml_file:
        pass
    else:
        kml_file = kml_file + '.kml'
        # rospy.set_param('/gps_path_utils/kml_file', kml_file)
    # look for the file inside gps_path_utils package.
    try:
        ros_pack = rospkg.RosPack()
        mission_file_dir = ros_pack.get_path('gps_path_utils') + "/kml_files/" + str(kml_file)
    except Exception as e:
        rospy.logwarn("Please source gps_path_utils package: " + str(e))
        sys.exit("Please source autopilot package" + str(e))

    a = KmlToMissionFile(mission_file_dir)
    rospy.spin()