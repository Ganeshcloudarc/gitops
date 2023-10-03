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
from visualization_msgs.msg import MarkerArray, Marker
from geojson import LineString as ls
from autopilot_utils.pose_helper import distance_btw_poses, get_yaw, angle_btw_poses, yaw_to_quaternion, normalize_angle
import cubic_spline_planner 
import numpy as np
import math 
from utils.plot import plot_curvature
import matplotlib.pyplot as plt 
from json import dump
import geometry_msgs.msg as gmsg
import logging
from jsk_recognition_msgs.msg import PolygonArray
from math import cos, sin
from shapely.geometry import LineString


def set_rospy_log_lvl(log_level):
    logger = logging.getLogger('rosout')
    logger.setLevel(rospy.impl.rosout._rospy_to_logging_levels[log_level])


def to_line(self, width):
        marker_arr = MarkerArray()
        marker = Marker()
        marker.header.frame_id = self._traj_in.header.frame_id
        marker.type = marker.LINE_STRIP
        marker.id = 1
        marker.action = marker.ADD
        marker.scale.x = width
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # marker_arr_msg.header.frame_id = self._traj_in.header.frame_id
        for i in range(self._traj_len - 10):
            pt = Point()
            pt.x = self._traj_in.points[i].pose.position.x
            pt.y = self._traj_in.points[i].pose.position.y
            pt.z = self._traj_in.points[i].pose.position.z
            marker.points.append(pt)
        marker_arr.markers.append(marker)
        marker = Marker()
        marker.header.frame_id = self._traj_in.header.frame_id
        marker.type = marker.LINE_STRIP
        # marker.text = str(round(traj_point.longitudinal_velocity_mps, 2))
        marker.id = 2
        marker.action = marker.ADD
        marker.scale.x = width
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # marker_arr_msg.header.frame_id = self._traj_in.header.frame_id
        for i in range(self._traj_len - 10, self._traj_len):
            pt = Point()
            pt.x = self._traj_in.points[i].pose.position.x
            pt.y = self._traj_in.points[i].pose.position.y
            pt.z = self._traj_in.points[i].pose.position.z
            marker.points.append(pt)
        marker_arr.markers.append(marker)

        return marker_arr

def draw_circle(center, radius):
    x,y = center[0], center[1]
    theta = np.linspace( 0 , 2 * np.pi , 150 )
    # print("theta", theta)
    a = float(x) + radius * np.cos( theta )
    b = float(y) + radius * np.sin( theta )
    points =[ ]

    for i in range(len(a)):
        pt = Point()
        pt.x = a[i]
        pt.y = b[i]
        points.append(pt)
    return points

def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy


def angle_of_vectors(a, b, c, d):
    dotProduct = a * c + b * d
    # for three dimensional simply add dotProduct = a*c + b*d  + e*f
    modOfVector1 = math.sqrt(a * a + b * b) * math.sqrt(c * c + d * d)
    # for three dimensional simply add modOfVector = math.sqrt( a*a + b*b + e*e)*math.sqrt(c*c + d*d +f*f)
    angle = dotProduct / modOfVector1
    print("Cosθ =", angle)
    th = math.acos(angle)
    angleInDegree = math.degrees(th)
    print("θ =", angleInDegree, "°")
    return th

def GetAngle(v1,v2):
    return math.atan2(np.cross(v1,v2), np.dot(v1,v2));

class KmlToMissionFile:
    def __init__(self, kml_file_dir, mission_file_dir): 

        self.direction = rospy.get_param("direction_parallel_line",1) 
        self.no_of_rounds = rospy.get_param("no_of_parallel_line",3)
        # self.length_farm = rospy.get_param("maximum_length of the tada_farm", 136) mango farm = 30 feet (9.1 m)
        self.path_breadth = rospy.get_param("maximum_breath_of_farm",9.1) 
        self.minimum_turning_radius = rospy.get_param("minimum_turning_radius",5)
        self.path_res = rospy.get_param("path_resolution", 0.1) 
        self.start_point_dis = 2
        self.geo_polygon_pub = rospy.Publisher('/kml_coords', gmsg.PolygonStamped,queue_size=1, latch=True)
        self.intropoldated_pub = rospy.Publisher('/intropolated_coords', gmsg.PolygonStamped,queue_size=1, latch=True)
        self.circle_pub = rospy.Publisher('/circles', PolygonArray, queue_size=1, latch=True)
        kml_coords=[]
        rospy.logdebug(f"kml_file: {kml_file_dir}")
        try: 
            # reading kml file 
            with open(kml_file_dir, 'r') as geo_fence:
                    doc = geo_fence.read().encode('utf-8') 
                    k = kml.KML() 
                    k.from_string(doc)
                    for i in k.features():
                        for j in i.features():
                            if isinstance(j.geometry, geometry.LineString):
                                polygon = j.geometry
                                for coordinates in polygon.coords:
                                    # lat, long format
                                    kml_coords.append([coordinates[1], coordinates[0]])
                            else:
                                rospy.logerr(f"KML file format is not LineString")
                                sys.exit("KML file format is not LineString")
                                                                
        except Exception as err:
            rospy.logerr(f"Error in reading: {kml_file_dir}, err : {err}")

        rospy.logdebug(f"lenght of kml way points: {len(kml_coords)}")

        
        
        self.home_lat = kml_coords[0][0]
        self.home_lon = kml_coords[0][1]
        self.kml_list_x = []
        self.kml_list_y = []
        self.kml_list_xy = []
        self.val_x =[]
        self.val_y =[]
        self.final_kml = []
        self.new_kml_x = []
        self.new_kml_y = []
        polygon_st = gmsg.PolygonStamped()
        polygon_st.header.frame_id = 'map'
        self.intreploted_y=[]
        self.intreploted_x=[]
        self.final_points_list=[] 

        # To convert lat,lon to x,y coordinates
        for lat, lon in kml_coords:
            point = gmsg.Point()
            relative_x, relative_y = ll2xy(lat, lon, self.home_lat, self.home_lon)
            point.x = relative_x
            point.y = relative_y
            polygon_st.polygon.points.append(point)
            self.kml_list_xy.append([relative_x, relative_y]) 
            self.val_x.append(relative_x)
            self.val_y.append(relative_y)
        # for debuging 
        self.geo_polygon_pub.publish(polygon_st)
        start_point = self.kml_list_xy[0]
        centers=[]
        polygon_st = gmsg.PolygonStamped()
        polygon_st.header.frame_id = 'map'
        self.intreploted_x.append(self.val_x[0])
        self.intreploted_y.append(self.val_y[0])
        self.kml_list_x.append(self.kml_list_xy[0])
        self.kml_list_y.append(self.kml_list_xy[1])
    

        # para_point(generation)
        for i in range(0,self.no_of_rounds):
            pt_1 = self.kml_list_x[i]
            pt_2 = self.kml_list_y[i]
            # to generate the parallel line upwards or left
            if(self.direction == 1):
                pt_1_2 = LineString([pt_1,pt_2]) 
                left = pt_1_2.parallel_offset(self.path_breadth,'left') 
                right = pt_1_2.parallel_offset(self.path_breadth,'right') 
                c = left.boundary.geoms[1]
                d = right.boundary.geoms[0]
                self.kml_list_y.append([c.x,c.y])
                pt_2_1 = LineString([pt_2,pt_1])
                left = pt_2_1.parallel_offset(self.path_breadth,'left')
                right = pt_2_1.parallel_offset(self.path_breadth,'right')
                c_ = left.boundary.geoms[0]
                d_ = right.boundary.geoms[1] 
                self.kml_list_x.append([d_.x,d_.y])  
            # to generate the parallel line downwards or right
            else:
                pt_1_2 = LineString([pt_1,pt_2]) 
                left = pt_1_2.parallel_offset(self.path_breadth,'left')
                right = pt_1_2.parallel_offset(self.path_breadth,'right')
                c_1 = left.boundary.geoms[0]
                d_1 = right.boundary.geoms[1]
                self.kml_list_y.append([d_1.x,d_1.y])
                pt_2_1 = LineString([pt_2,pt_1])
                left = pt_2_1.parallel_offset(self.path_breadth,'left')
                right = pt_2_1.parallel_offset(self.path_breadth,'right')
                c_2 = left.boundary.geoms[1]
                d_2 = right.boundary.geoms[0]
                self.kml_list_x.append([c_2.x,c_2.y])

        for i in range(0,len(self.kml_list_x)):
            if(i%2 == 0):
                self.final_kml.append(self.kml_list_x[i])
                self.final_kml.append(self.kml_list_y[i])
            else:
                self.final_kml.append(self.kml_list_y[i])
                self.final_kml.append(self.kml_list_x[i])

                # print("kml_list_xy",kml_list_xy)
        self.final_kml.append(self.final_kml[0]) 
        self.final_kml.append(self.final_kml[1])
        new_kml_x, new_kml_y = map(list, zip(*self.final_kml)) 
                
        # circle method interpolation
        for i in range(1, len(new_kml_x)-1):
            print("==========================================================================")
            a = x1,y1 = (new_kml_x[i-1], new_kml_y[i-1])
            b = x2,y2 = (new_kml_x[i], new_kml_y[i])
            c = x3,y3 = (new_kml_x[i+1], new_kml_y[i+1])
            v1 = np.array([x1 - x2, y1 - y2])
            v2 = np.array([x3 - x2, y3 - y2])
            print("v1", v1)
            print("v2", v2)
            angle_between_vectors = GetAngle(v2,v1)

            # to find the circle center
            if  angle_between_vectors < 0:
                angle_between_vectors = GetAngle(v1,v2)
                theta = angle_between_vectors/2
                print("theta", theta)
                OA  = self.minimum_turning_radius/math.sin(theta)
                OB = OA*math.cos(theta)
                rot = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
                v1_init = v1*(1/np.linalg.norm(v1))
                v1_final = v1_init * OA
                print("v1_final",v1_final)
                v_f = np.dot(rot, v1_final)
            else:
                print("angle_between_vectors",angle_between_vectors, math.degrees(angle_between_vectors))
                print("diff", math.pi - angle_between_vectors)
                theta = angle_between_vectors/2
                print("theta", theta)
                OA  = self.minimum_turning_radius/math.sin(theta)
                OB = OA*math.cos(theta) 
                print(f"OA : {OA}, OB: {OB}")
                rot = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
                v2_init = v2*(1/np.linalg.norm(v2))
                v2_final = v2_init * OA
                print("v1_final",v2_final)
                v_f = np.dot(rot, v2_final)

            circle_center = [x2 + v_f[0],y2 + v_f[1]]
            v1_tanget_point = v1*(1/np.linalg.norm(v1)) * OB
            v1_tanget_point = [x2 + v1_tanget_point[0] , y2+ v1_tanget_point[1]]

            v2_tanget_point = v2*(1/np.linalg.norm(v2)) * OB
            v2_tanget_point = [x2 + v2_tanget_point[0] , y2+ v2_tanget_point[1]] 
                
            ## connect start_point to tangent_1.
            changex = v1_tanget_point[0]-start_point[0]
            changey = v1_tanget_point[1]-start_point[1]
            dist_line = math.sqrt(changey**2 + changex**2) 
            angle_line = math.atan2(changey,changex)
            print("distance-----",dist_line)
            print("angle-----",angle_line)
            number_of_pts_line = int(dist_line / self.path_res) 
            print("number of points",number_of_pts_line)
        
            for i in range(int(number_of_pts_line)):
                px = self.intreploted_x[-1]
                py = self.intreploted_y[-1] 
                px_upd = px + self.path_res * math.cos(angle_line)
                py_upd = py + self.path_res * math.sin(angle_line)
                self.intreploted_x.append(px_upd)
                self.intreploted_y.append(py_upd)  
        
            point = gmsg.Point()
            point.x = v1_tanget_point[0]
            point.y = v1_tanget_point[1]
            polygon_st.polygon.points.append(point)
            start_point = v2_tanget_point 

            # to interpolated the waypoints for the circle arc using 2 tangents
            arc_v1 = np.array([v1_tanget_point[0] - circle_center[0], v1_tanget_point[1] - circle_center[1]])
            arc_v2 = np.array([v2_tanget_point[0] - circle_center[0], v2_tanget_point[1] - circle_center[1]])
            print(f"arc_v1 {arc_v1}")
            arc_angle = GetAngle(arc_v1, arc_v2)
            arg_lenght = abs(arc_angle) * self.minimum_turning_radius
            number_of_points = int(arg_lenght/ self.path_res)
            inc_angle = arc_angle/ number_of_points
            theta = 0
            for i in range(number_of_points):
                theta+= inc_angle
                rot = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

                dot_pro = np.dot(rot, arc_v1)
                point = gmsg.Point()
                point.x = dot_pro[0] + circle_center[0]
                point.y = dot_pro[1] + circle_center[1]
                polygon_st.polygon.points.append(point)
                self.intreploted_x.append(point.x)
                self.intreploted_y.append(point.y)
    
            #circle intrepolation  
            point = gmsg.Point() 
            point.x = v2_tanget_point[0]
            point.y = v2_tanget_point[1] 
    
            polygon_st.polygon.points.append(point)
            centers.append([x2+ v_f[0],y2+v_f[1]])

        self.intropoldated_pub.publish(polygon_st)
        
        polygon_st = gmsg.PolygonStamped()
        polygon_st.header.frame_id = 'map'

        plg_arr = PolygonArray()
        plg_arr.header.frame_id = "map" 
        plg_list = [] 
        odom_list=[]
        for i in range(len(centers)):
            polygon_st = gmsg.PolygonStamped()
            polygon_st.header.frame_id = 'map'
            circle_markers = draw_circle(centers[i], self.minimum_turning_radius)
            polygon_st.polygon.points = circle_markers
            plg_list.append(polygon_st)
        plg_arr.polygons = plg_list 
        self.circle_pub.publish(plg_arr) 
        rospy.loginfo("circle published") 
        
        # direction and orientation (odometric_values)
        for  i in range(len(self.intreploted_x)-1): 
                x = self.intreploted_x[i]
                y = self.intreploted_y[i]
                dy = self.intreploted_y[i+1]-self.intreploted_y[i]
                dx = self.intreploted_x[i+1]-self.intreploted_x[i] 
                yaw = math.atan2(dy,dx)
                odom = Odometry() 
                odom.pose.pose.orientation = yaw_to_quaternion(yaw) 
                odom.pose.pose.position.x = x 
                odom.pose.pose.position.y = y  
                point = gmsg.Point() 
                point.x = x 
                point.y = y
                polygon_st.polygon.points.append(point)
                odom_list.append(message_converter.convert_ros_message_to_dictionary(odom))
        odom = Odometry() 
        odom.pose.pose.orientation = yaw_to_quaternion(yaw) 
        odom.pose.pose.position.x = self.intreploted_x[-1]
        odom.pose.pose.position.y = self.intreploted_y[-1]
        odom.pose.pose.orientation = yaw_to_quaternion(yaw) 
        odom_list.append(message_converter.convert_ros_message_to_dictionary(odom))
        
        # xy_to_ll 
        for i in range(len(self.intreploted_y)): 
                llx,lly = xy2ll(self.intreploted_x[i],self.intreploted_y[i],self.home_lat,self.home_lon)
                self.final_points_list.append([lly,llx])

        # json_convertion 
        line_string = ls(self.final_points_list)
        line_string['odometry'] = odom_list 
        line_string['gps_coordinates'] = [{'altitude':28}]
        print("mission file ",mission_file_dir )

        with open(mission_file_dir, 'w') as f:
                dump(line_string, f)
                print('json file saved')   
        rospy.logerr("JSON SAVED. Exiting")
        rospy.signal_shutdown("JSON SAVED. Exiting")

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

    rospy.logdebug(f"kml_file_dir: {kml_file_dir}")
    rospy.logdebug(f"mission_file_dir: {mission_file_dir}") 

    kml_to_mission_file = KmlToMissionFile(kml_file_dir, mission_file_dir)
    rospy.spin()