#!/usr/bin/env python3
from math import *
from fastkml import kml 
import rospy 
import rospkg
import json,sys
from fastkml import geometry
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
import geometry_msgs.msg as gmsg
import logging


def set_rospy_log_lvl(log_level):
    logger = logging.getLogger('rosout')
    logger.setLevel(rospy.impl.rosout._rospy_to_logging_levels[log_level])


class KmlToMissionFile:
    def __init__(self, kml_file_dir, mission_file_dir):

        self.linear_sampling_dis = rospy.get_param("linear_sampling_dis", 6)
        self.start_point_dis = 3

        self.geo_polygon_pub = rospy.Publisher('/kml_coords', gmsg.PolygonStamped, queue_size=1, latch=True)
        self.intropoldated_pub = rospy.Publisher('/intropolated_coords', gmsg.PolygonStamped, queue_size=1, latch=True)

        kml_coords=[]
        rospy.logdebug(f"kml_file: {kml_file_dir}")  
        try: 
            # reading kml file 
            with open(kml_file_dir,'r') as geo_fence :
                    doc = geo_fence.read().encode('utf-8') 
                    k = kml.KML() 
                    k.from_string(doc)
                    for i in k.features():
                            print("{},{}".format(i.name, i.description)) 
                            for j in i.features():
                                    if isinstance(j.geometry, geometry.LineString):
                                            polygon = j.geometry 
                                            print(polygon.coords)
                                            for coordinates in polygon.coords: 
                                                        kml_coords.append(coordinates[:-1])  
                                    else:
                                        rospy.logerr(f"KML file format is not LineString")
                                        sys.exit("KML file format is not LineString")
                                                                
        except Exception as e:
            rospy.logerr('Error in reading %s file ', kml_file)
            sys.exit('Error in reading file '+ str(kml_file))

        rospy.logdebug(f"lenght of kml way points: {len(kml_coords)}")

        # latitudes longitudes to xy coordinates
        # kml_coords.reverse()
        # kml_coords = kml_coords[1:]
        kml_coords.append(kml_coords[0])
        kml_list = []
        kml_list_x = []
        kml_list_y = []
        xx = []
        yy = []
        odom_list =[]
        final_points_list = []
    
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
        xx.extend([kml_list[0][1]])
        yy.extend([kml_list[0][1]])
        intreploted_x = []
        intreploted_y = []
        intreploted_x.append(kml_list[0][1])
        intreploted_y.append(kml_list[0][0])

        x,y = kml_list[0][0], kml_list[0][1]
        x1,y1 = kml_list[1][0], kml_list[1][1]
        changex = x1-x
        changey = y1-y
        dist = math.sqrt(changey**2+changex**2)
        angle = math.atan2(changey,changex)

        first_x = kml_list[0][0] + self.start_point_dis * math.cos(angle) 
        first_y = kml_list[0][1] +  self.start_point_dis * math.sin(angle)


        for i in range(len(kml_list)-1):
            x,y = kml_list[i][0], kml_list[i][1]

            x1,y1 = kml_list[i+1][0], kml_list[i+1][1]

            changex = x1-x
            changey = y1-y
            dist = math.sqrt(changey**2+changex**2)
            angle = math.atan2(changey,changex)


            number_points_between = int(dist / self.linear_sampling_dis)

            print("float number_points_between",float(number_points_between))
            print("integer number_points_between",int(number_points_between))
            res_to_add = (float(number_points_between)- int(number_points_between))/ int(number_points_between)
            print("res_to_add",res_to_add)
            final_res = float(res_to_add) + self.linear_sampling_dis 
            print("res", dist/final_res)


            final_res = dist/number_points_between


            for j in range(int(number_points_between)): 
                if j == 0:
                    px,py = kml_list[i][0], kml_list[i][1]
                else:
                    px = intreploted_x[-1]
                    py = intreploted_y[-1]
                px_upd = px + final_res * math.cos(angle) 
                py_upd = py + final_res * math.sin(angle)
                intreploted_x.append(px_upd)
                intreploted_y.append(py_upd)
        
        intreploted_x.append(first_x)  
        intreploted_y.append(first_y)

        # cubic_interpolation_method                                                               
        cx, cy,cyaw, ck, s = cubic_spline_planner.calc_spline_course(intreploted_x,intreploted_y,ds=0.1)
        polygon_st = gmsg.PolygonStamped()
        polygon_st.header.frame_id = 'map'
        for x,y,yaw in zip(cx,cy,cyaw):
                odom = Odometry()
                odom.pose.pose.orientation = yaw_to_quaternion(yaw)
                odom.pose.pose.position.x =x
                odom.pose.pose.position.y =y 
                point = gmsg.Point()
                point.x = x
                point.y = y
                polygon_st.polygon.points.append(point)
                odom_list.append(message_converter.convert_ros_message_to_dictionary(odom))
        self.intropoldated_pub.publish(polygon_st)
        print("Number of points at every 10 cm in path:",len(odom_list))
        print("Distance covered by the path:",((len(cy))-1)/10,"meters")
        #print("LOOK AT THE GRAPH FOR THE PLOTTING GRAPH")

        # xy_to_ll 
        for i in range(len(odom_list)-1):
                llx,lly = xy2ll(cx[i],cy[i],home_lat,home_lon)
                final_points_list.append([lly,llx])

        # json_convertion
        line_string = LineString(final_points_list)
        line_string['odometry'] = odom_list 
        line_string['gps_coordinates'] = [{'altitude':28}]
        # miss_file =  "/home/suprabha/autopilot_ws/src/autopilot_boson/gps_path_utils/mission_files/"+str(file)+'.json'
        print("mission file ",mission_file_dir )

        with open(mission_file_dir, 'w') as f:
                dump(line_string, f)
                print('json file saved')

        # graph_plot 
        plt.plot(cx,cy,"xb",label="cubic interpolation with 50cm ")
        plt.plot(intreploted_x,intreploted_y,"xr",label="linear intr")
        plt.plot(kml_list_x,kml_list_y,"r",label="manual plot")
        # plt.plot(px_upd,py_upd,"xb",label="the mannual plot")
        # plt.savefig('plot.png')
        plt.show()


        
if __name__ == "__main__":
    rospy.init_node("gps_waypoints")
    set_rospy_log_lvl(rospy.DEBUG)
    kml_file = rospy.get_param("/gps_path_utils/kml_file", "default.kml")
    if '.kml' in kml_file:
        pass
    else:
        kml_file = kml_file + '.kml'
        # rospy.set_param('/gps_path_utils/kml_file', kml_file)
    # look for the file inside gps_path_utils package.
    try:
        ros_pack = rospkg.RosPack() 
        kml_file_dir = ros_pack.get_path('gps_path_utils') + "/kml_files/" + str(kml_file)
        mission_file_dir = ros_pack.get_path('autopilot') + "/mission_files/" + kml_file[:-4] + ".json"
    except Exception as e:
        rospy.logerr("Please source gps_path_utils package: " + str(e))
        sys.exit("Please source autopilot package" + str(e))
    
    rospy.logdebug(f"mission_file_dir: { mission_file_dir}")
    kml_to_mission_file = KmlToMissionFile(kml_file_dir, mission_file_dir)
    rospy.spin()