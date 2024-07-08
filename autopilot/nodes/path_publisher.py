#!/usr/bin/env python3

try : 

    import rospy
    from geopy.distance import geodesic
    import rospkg
    from nav_msgs.msg import Path
    from geojson import LineString as ls 
    from nav_msgs.msg import Path, Odometry 
    from geographic_msgs.msg import GeoPointStamped, GeoPose
    from mavros_msgs.msg import HomePosition
    from geometry_msgs.msg import Pose, PoseStamped, Quaternion, PolygonStamped
    from std_msgs.msg import Float32MultiArray, Int32MultiArray, Bool, Int16, String, Int8, UInt16
    from visualization_msgs.msg import Marker, MarkerArray
    import json, time, math, sys, os
    import numpy as np
    from json import dump 
    from vehicle_common.vehicle_config import vehicle_data
    from autopilot_msgs.msg import Trajectory, TrajectoryPoint
    from autopilot_utils.geonav_conversions import xy2ll, ll2xy
    from autopilot_utils.rdp_helper import rdp_calculate, distance__ 
    from autopilot_utils.pose_helper import distance_btw_poses, get_yaw, angle_btw_poses, yaw_to_quaternion, rotate_yaw_180
    from autopilot_utils.trajectory_helper import trajectory_to_path, trajectory_to_marker
    from autopilot_utils.trajectory_common import TrajectoryManager 
    from autopilot_utils.coordinate_helper import dist_btw_coord , check_direction_dot_product, calculate_initial_bearing, check_direction_cross_product
    from fastkml import kml
    from fastkml import geometry
    import geometry_msgs.msg as gmsg
    from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
    from diagnostic_updater._diagnostic_status_wrapper import DiagnosticStatusWrapper



except Exception as e: 
    import rospy 
    rospy.logerr("module named %s", str(e)) 
    exit() 


def get_tie(theta):
    return np.array([np.cos(theta), np.sin(theta)])
OK = DiagnosticStatus.OK
ERROR = DiagnosticStatus.ERROR
WARN = DiagnosticStatus.WARN
STALE = DiagnosticStatus.STALE


class GlobalGpsPathPub:
    def __init__(self, mission_file_dir):
        self.interploted_x=[] 
        self.interploted_y=[] 
        self.interploted=[]
        self.coordinate =[] 
        self.coordinate_x =[] 
        self.coordinate_y =[] 
        self.path_pub_diagnostics = rospy.Publisher("path_publisher_diagnostics",DiagnosticArray, queue_size=1, latch = True)
        self.diagnostics_status= DiagnosticStatusWrapper() 
        self.diagnostics_array = DiagnosticArray() 
        self.diagnostics_status.name = rospy.get_name()
        self.diagnostics_status.hardware_id = 'zekrom_v1' 
        self.mission_continue_dist_thr = rospy.get_param("path_publisher/mission_continue_dist_thr",4) 
        self.curve_dist = rospy.get_param("path_publisher/curve_dist",10)
        self.smooth_path = rospy.get_param("path_publisher/smooth_path",False)
        self.opposite_path = rospy.get_param("path_publisher/opposite_path_forward_motion",True)
        self.opposite_closest_pt_dis_Thr = rospy.get_param("path_publisher/opposite_closest_pt_dis_Thr",1)
        self.curve_angle = rospy.get_param("path_publisher/rdp_angle",0.09)
        self._traj_manager = TrajectoryManager()
        # parameters for path publisher
        # self.path_res = rospy.get_param("path_publisher/path_resolution",0.1)
        self.debug = rospy.get_param("/patrol/debug",False)
        self.max_forward_speed = rospy.get_param('/patrol/max_forward_speed', 1.5)
        self.min_forward_speed = rospy.get_param("/patrol/min_forward_speed", 0.3)
        self.minimum_data_len = rospy.get_param("path_publisher/minimum_json_length",100) 
        self.distance_to_slowdown_on_ends = rospy.get_param("/path_publisher/distance_to_slowdown_on_ends", 3) 
        self.distance_to_slowstart_on_start = rospy.get_param("/path_publisher/distance_to_slowstart_on_start",3)
        self.mission_continue = rospy.get_param("/mission_continue", True) 
        self.interpolate_with_rtk = rospy.get_param("path_publisher/interpolate_with_rtk",False) 
        self.interpolate_without_rtk = rospy.get_param("path_publisher/interpolate_without_rtk",False)
        self.max_dis_btw_points = rospy.get_param("path_publisher/max_dis_btw_points", 0.5)
        self.path_resolution = rospy.get_param("path_publisher/path_resolution", 0.1)
        self.steering_limits_to_slow_down = rospy.get_param("path_publisher/steering_limits_to_slow_down", 10)
        self.speed_reduce_factor = rospy.get_param("path_publisher/speed_reduce_factor", 0.7)
        self.min_look_ahead = rospy.get_param("pure_pursuit/min_look_ahead_dis", 3)
        self.max_look_ahead = rospy.get_param("pure_pursuit/max_look_ahead_dis", 6)
        self.avg_lhd = (self.min_look_ahead + self.max_look_ahead) / 2
        self.rear_axle_center_height_from_ground = vehicle_data.dimensions.tyre_radius
        self.mission_file_dir = mission_file_dir 
        self.turn_interpolate = None
        self.error_occured =False

        # publishers
        self.global_trajectory_pub = rospy.Publisher('global_gps_trajectory', Trajectory, queue_size=1, latch=True)
        self.gps_path_pub = rospy.Publisher('/global_gps_path', Path, queue_size=1, latch=True)
        # to set mavros home position
        self.starting_point_pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped,
                                                  queue_size=1, latch=True)
        self.home_position_pub = rospy.Publisher('/mavros/global_position/home', HomePosition,
                                                 queue_size=1, latch=True)
        self.trajectory_velocity_marker_pub = rospy.Publisher('/global_gps_trajectory_velocity_marker', MarkerArray,
                                                              queue_size=1, latch=True)

        # debug
        self.polygon_xy_pub = rospy.Publisher('/xy_polygon', PolygonStamped,
                                           queue_size=1, latch=True)
        self.polygon_xy_filled_pub = rospy.Publisher('/polygon_xy_filled_', PolygonStamped,
                                           queue_size=1, latch=True)

    def publisher_diagnostics(self): 
        self.diagnostics_array.status = [] 
        self.diagnostics_array.status.append(self.diagnostics_status)
        self.diagnostics_array.header.stamp = rospy.Time.now()
        self.path_pub_diagnostics.publish(self.diagnostics_array)


    def to_polygon(self, xy_list):
        polygon_st = PolygonStamped()
        polygon_st.header.frame_id = 'map'
        for x, y in xy_list:
            point = gmsg.Point()
            point.x, point.y = x, y
            polygon_st.polygon.points.append(point) 
        return polygon_st 

    def from_json(self):
        data = None 
        json_data = [] 
        try:
           
            data = json.load(open(self.mission_file_dir))

            if (self.smooth_path == True) : 
                rospy.loginfo("smooth path is true")
                # reading the json file 
                for i in data['coordinates']:
                    json_data.append(i) 
                
                #ll2xy 
                for i in range(len(json_data)): 
                    # x, y = ll2xy(long_lat_list[i][1], long_lat_list[i][0], home_lat, home_long)
                    json_data_x,json_data_y = ll2xy (json_data[i][1],json_data[i][0],json_data[0][1],json_data[0][0])
                    self.coordinate.append([json_data_x,json_data_y]) 
                    self.coordinate_x.append([json_data_x]) 
                    self.coordinate_y.append([json_data_y])   
                rdp_thr = np.pi*self.curve_angle
                self.curve_points = rdp_calculate(self.coordinate,rdp_thr) 
                self.curve_points.insert(0,self.coordinate[0])
                self.curve_points.append(self.coordinate[-1])
                for i in range(len(self.curve_points)-1): 
                    dist_val = distance__(self.curve_points[i],self.curve_points[i+1])  
                    if dist_val >= self.curve_dist :  
                        #  linear interpolation 
                        rospy.loginfo("linear interpolation ")
                        a,b = self.curve_points[i][0],self.curve_points[i][1] 
                        a1,b1 = self.curve_points[i+1][0],self.curve_points[i+1][1] 
                        changex = a1-a 
                        changey = b1-b
                        
                        angle_line = math.atan2(changey, changex) 
                        no_of_points = dist_val / self.path_resolution
                        
                        for j in range(int(no_of_points)): 
                            if j == 0: 
                                px_upd = a + self.path_resolution * math.cos(angle_line) 
                                py_upd = b + self.path_resolution * math.sin(angle_line)  
                                self.interploted.append([px_upd,py_upd]) 
                            
                            else: 
                                px = self.interploted[-1][0]
                                py = self.interploted[-1][1]
                                px_upd = px + self.path_resolution * math.cos(angle_line) 
                                py_upd = py + self.path_resolution * math.sin(angle_line)
                                self.interploted.append([px_upd,py_upd])  
                        
                    else: 
                        rospy.loginfo("setting the same turning points ")
                        element_1 = self.curve_points[i] 
                        index1 = self.coordinate.index(element_1)
                        
                        element_2 = self.curve_points[i+1] 
                        index2 = self.coordinate.index(element_2)
                        for i in range(index1,index2+1):  
                            self.interploted.append([self.coordinate[i][0],self.coordinate[i][1]])
                # xy to ll  
                json_coord = []
                json_coord.append(json_data[0])
                for i in range(len(self.interploted)): 
                    llx,lly = xy2ll(self.interploted[i][0],self.interploted[i][1],json_data[0][1],json_data[0][0]) 
                    json_coord.append([lly,llx])

                # json convertion 
                line_string = ls(json_coord)  
                data = line_string
                rospy.loginfo("json is updated") 
                
            else : 
                rospy.loginfo("smooth_path is false")     

        except Exception as error:
            rospy.logerr('Error In Reading mission file ' + str(error))
            if self.debug:
                self.diagnostics_status.summary(ERROR,"Error in Reading Mission file") 
            else: 
                self.diagnostics_status.summary(ERROR,"Invalid mission file")  
            self.diagnostics_status.add("message","Error in Reading Mission file")
            self.diagnostics_status.add("error in reading json",self.mission_file_dir)
            self.diagnostics_status.add("Error Occured",str(error)) 
            self.publisher_diagnostics()  
            return
        odom_key = 'odometry'
        coord_key = 'coordinates'
        feature_key = 'features'

        if coord_key in data.keys() and odom_key in data.keys():
            rospy.loginfo("odometry values found")
            self.from_odometry(data)

        elif coord_key in data.keys():
            long_lat_list = data[coord_key]
            if len(long_lat_list) >= self.minimum_data_len:
                self.publish_path_from_long_lat(long_lat_list) 
            else:
                rospy.logerr("No points available in mission file")
                if self.debug: 
                    self.diagnostics_status.summary(ERROR,"No points available in mission file")
                else:
                    self.diagnostics_status.summary(ERROR,"Invalid mission file") 
                self.diagnostics_status.add("message","No points available in mission file")
                self.diagnostics_status.add("Coordinates Length ",len(long_lat_list)) 
                self.publisher_diagnostics()  
                return
        elif feature_key in data.keys(): 
            feature = data['features'][0] 
            # first feature
            if feature['geometry']['type'] == "LineString": 
                rospy.loginfo("type matched to LineString")  
            else: 
                rospy.logerr(f"geometry does not match, Required LineString, given :{feature['geometry']['type']}") 
                if self.debug: 
                    self.diagnostics_status.summary(ERROR,"Geometry does not match")
                else:
                    self.diagnostics_status.summary(ERROR,"Invalid mission file")   
                self.diagnostics_status.add("message","Geometry does not match")
                self.diagnostics_status.add("LineString",feature['geometry']['type']) 
                self.publisher_diagnostics()  
                return
            long_lat_list = feature['geometry']['coordinates'] 
            if len(long_lat_list) >= 2: 
                self.publish_path_from_long_lat(long_lat_list) 
            else:
                rospy.logerr("No points available in mission file")
                if self.debug:
                    self.diagnostics_status.summary(ERROR,"No points available in mission file") 
                else:
                    self.diagnostics_status.summary(ERROR,"Invalid mission file")  
                self.diagnostics_status.add("message","No points available in mission file")
                self.diagnostics_status.add("Mission_data_Length ",len(long_lat_list))
                self.publisher_diagnostics() 
                return
        else:
            rospy.logerr("could not found proper fields in mission file")
            if self.debug: 
                self.diagnostics_status.summary(ERROR,f"could not find proper field {coord_key},{odom_key} in mission file") 
            else:
                self.diagnostics_status.summary(ERROR,"Invalid mission file")  
            self.diagnostics_status.add("message",f"could not find proper field {coord_key},{odom_key} in mission file")
            self.diagnostics_status.add("coord_key status", coord_key in data.keys()) 
            self.diagnostics_status.add("odom_key status",odom_key in data.keys() )
            self.publisher_diagnostics()  
            return

    def from_kml(self):

        try:
            with open(self.mission_file_dir, 'r') as geo_fence:
                doc = geo_fence.read().encode('utf-8') 
        except Exception as error:
            rospy.logerr('Error In Reading mission file ' + str(error))
            if self.debug: 
                self.diagnostics_status.summary(ERROR,"Error in reading Mission file") 
            else:
                self.diagnostics_status.summary(ERROR,"Invalid mission file")  
            self.diagnostics_status.add("message","Error in reading Mission file")
            self.diagnostics_status.add("error in reading kml",self.mission_file_dir)
            self.diagnostics_status.add("Error Ocurred ",str(error))
            self.publisher_diagnostics() 
            return
        try:         
            k = kml.KML()
            k.from_string(doc)
            long_lat_list = []
            for i in k.features():
                for j in i.features():
                    if isinstance(j.geometry, geometry.LineString):
                        polygon = j.geometry
                        for coordinates in polygon.coords:
                            # lat, long format
                            long_lat_list.append([coordinates[0], coordinates[1]])  

            # to further publish only if they accept and run with try condition
            if len(long_lat_list) >= 2:
                self.publish_path_from_long_lat(long_lat_list) 
                self.diagnostics_status.summary(OK,f"Happily published  {rospy.get_name()}") 
                self.diagnostics_status.add("Length ",len(long_lat_list))  
                self.publisher_diagnostics() 

            else:
                rospy.logerr("No points available in mission file")
                if self.debug:
                    self.diagnostics_status.summary(ERROR,f"only {len(long_lat_list)} points available in mission file") 
                else: 
                    self.diagnostics_status.summary(ERROR,"Invalid mission file") 
                self.diagnostics_status.add("message",f"only {len(long_lat_list)} points available in mission file")
                self.diagnostics_status.add("Length ",len(long_lat_list))
                self.publisher_diagnostics()  
                return

        except Exception as error: 
            rospy.logerr(f"KML file format is not LineString") 
            if self.debug: 
                self.diagnostics_status.summary(ERROR,"KML not in LineString Format") 
            else:
                self.diagnostics_status.summary(ERROR,"Invalid mission file")  
            self.diagnostics_status.add("message","KML not in LineString Format")
            self.diagnostics_status.add("Error Ocurred",str(error))
            self.publisher_diagnostics() 
            return
        
    def set_home_position(self, home_lat, home_long, home_alt=-60):
        geo_point = GeoPointStamped()
        geo_point.position.latitude = home_lat
        geo_point.position.longitude = home_long
        geo_point.position.altitude = home_alt
        home_position_msg = HomePosition()
        home_position_msg.geo = geo_point.position
        self.starting_point_pub.publish(geo_point)
        self.home_position_pub.publish(home_position_msg)
        return geo_point.position

    def publish_path_from_long_lat(self, long_lat_list):
        home_lat = long_lat_list[0][1]
        home_long = long_lat_list[0][0]
        home_position = self.set_home_position(long_lat_list[0][1], long_lat_list[0][0])
        rospy.loginfo('Origin point set')
        time.sleep(0.5)
        xy_list = []
        for i in range(len(long_lat_list)):
            x, y = ll2xy(long_lat_list[i][1], long_lat_list[i][0], home_lat, home_long)
            xy_list.append([x, y])
        self.polygon_xy_pub.publish(self.to_polygon(xy_list))
        if self.mission_continue:
            xy_list.append(xy_list[0])

        x_y_filled = [xy_list[0]]

        for i in range(len(xy_list) - 1):
            dis = math.hypot(xy_list[i + 1][0] - xy_list[i][0], xy_list[i + 1][1] - xy_list[i][1])
            if dis < self.max_dis_btw_points:

                x_y_filled.append(xy_list[i])
            else:
                # x_y_filled.append(xy_list[i])
                number_of_points = int(dis / self.path_resolution)
                angle = math.atan2(xy_list[i + 1][1] - xy_list[i][1], xy_list[i + 1][0] - xy_list[i][0])
                for _ in range(number_of_points):
                    px = x_y_filled[-1][0]
                    py = x_y_filled[-1][1]
                    px_updated = px + self.path_resolution * math.cos(angle)
                    py_updated = py + self.path_resolution * math.sin(angle)
                    x_y_filled.append([px_updated, py_updated])
        self.polygon_xy_filled_pub.publish(self.to_polygon(x_y_filled))

        # yaw calculation and traj pub
        traj_msg = Trajectory()
        traj_msg.header.frame_id = "map"
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.home_position.position = home_position
        dis = 0.0

        for i in range(len(x_y_filled) - 1):
            # print(i, len(x_y_filled))
            trajectory_point = TrajectoryPoint()
            trajectory_point.pose.position.x = x_y_filled[i][0]
            trajectory_point.pose.position.y = x_y_filled[i][1]
            trajectory_point.pose.position.z = self.rear_axle_center_height_from_ground

            if i != len(x_y_filled):
                # print("uygyug",xy_list[i+1][0] - xy_list[i][0], xy_list[i+1][1] - xy_list[i][1])
                yaw = math.atan2(x_y_filled[i + 1][1] - x_y_filled[i][1], x_y_filled[i + 1][0] - x_y_filled[i][0])
                trajectory_point.pose.orientation = yaw_to_quaternion(yaw)
            if i == 0:
                dis = 0.0
            else:
                dis += math.hypot(x_y_filled[i][1] - x_y_filled[i - 1][1],
                                  x_y_filled[i][0] - x_y_filled[i - 1][0])
            trajectory_point.accumulated_distance_m = dis

            # print(x_y_filled[i][0], x_y_filled[i][1], home_lat, home_long)
            lat, lng = xy2ll(x_y_filled[i][0], x_y_filled[i][1], home_lat, home_long)
            trajectory_point.gps_pose.position.latitude = lat
            trajectory_point.gps_pose.position.longitude = lng

            traj_msg.points.append(trajectory_point)

        # velocity profile
        traj_length = len(traj_msg.points)
        for i, traj_point in enumerate(traj_msg.points):

            for lhd_index in range(i, traj_length):
                path_acc_distance = traj_msg.points[lhd_index].accumulated_distance_m - \
                                    traj_point.accumulated_distance_m
                if path_acc_distance > self.avg_lhd:
                    break
            look_ahead = path_acc_distance
            # print("i", i)
            # print("lhd", lhd_index)
            # print("dis", look_ahead)
            slope = angle_btw_poses(traj_msg.points[lhd_index].pose, traj_point.pose)
            alpha = slope - get_yaw(traj_point.pose.orientation)
            delta = math.atan2(2.0 * vehicle_data.dimensions.wheel_base * math.sin(alpha), look_ahead)
            delta_degrees = -math.degrees(delta)
            # print("delta angle",delta_degrees )
            if abs(delta_degrees) <= self.steering_limits_to_slow_down:
                traj_msg.points[i].longitudinal_velocity_mps = self.max_forward_speed
            else:
                traj_msg.points[i].longitudinal_velocity_mps = max(
                    self.speed_reduce_factor * np.interp(abs(delta_degrees),
                                                         [self.steering_limits_to_slow_down,
                                                          vehicle_data.motion_limits.max_steering_angle],
                                                         [self.max_forward_speed,
                                                          self.min_forward_speed]), self.min_forward_speed)
            # print("speed",traj_msg.points[i].longitudinal_velocity_mps )

        self.global_trajectory_pub.publish(traj_msg)
        self._traj_manager.update(traj_msg)
        self.gps_path_pub.publish(self._traj_manager.to_path())
        self.trajectory_velocity_marker_pub.publish(trajectory_to_marker(traj_msg, self.max_forward_speed))
        rospy.loginfo("PUBLISHED GLOBAL GPS PATH")
    


    def from_odometry(self, data): 
        data_keys = data.keys() 
        rtk_status = data['Is_RTK_Good']
        data_len = len(data['coordinates'])  
        odom_key_name = "odometry" 
        gps_key_name = "coordinates" 
        if gps_key_name in data_keys and odom_key_name in data_keys:
            rospy.loginfo("gps coordinates and Odometry fields are  in  mission file")
        else:
            rospy.logwarn("No gps coordinates and Odometry fields are  in  mission file") 
            if self.debug:
                self.diagnostics_status.summary(ERROR,f"No field {odom_key_name} {gps_key_name} in mission file") 
            else: 
                self.diagnostics_status.summary(ERROR,"Invalid mission file") 
            self.diagnostics_status.add("message",f"No field {odom_key_name} {gps_key_name} in mission file")
            self.diagnostics_status.add("odometry field ",gps_key_name in data_keys) 
            self.diagnostics_status.add("coordinates field ",odom_key_name in data_keys) 
            self.publisher_diagnostics() 
            return 

        # Setting home position for mavros node
        home_lat = data['coordinates'][0][1]
        home_long = data['coordinates'][0][0]
        home_alt = -60 # data['gps_coordinates'][0]['altitude'] 
        home_position = self.set_home_position(home_lat, home_long, home_alt)
        rospy.loginfo('Origin point set')
        time.sleep(0.5)

        # Ros messages
        trajectory_msg = Trajectory()
        trajectory_msg.header.frame_id = "map"
        trajectory_msg.header.stamp = rospy.Time.now()
        trajectory_msg.home_position.position = home_position

        # Filling the Trajectory_msg  
        closest_opp_coord_index = 0
        accumulated_distance = 0
        prev_pose = Pose()             
        max_path_resolution = []
        min_close_dis =[]
        reference_yaw = 0
        closest_distance = 0 

        if data_len < self.minimum_data_len: 
            rospy.logerr(f"less than minimum_waypoints {data_len}")
            if self.debug:
                self.diagnostics_status.summary(ERROR,f"CAUTION - {data_len} Number of Waypoints") 
            else: 
                self.diagnostics_status.summary(ERROR,"Invalid mission file") 
            self.diagnostics_status.add("message",f"CAUTION - {data_len} Number of Waypoints") 
            self.diagnostics_status.add("LEN-WAYPOINTS",data_len)
            self.publisher_diagnostics() 
            return
    
        last_index = 0 

        # using rdp find the list of x,y turn coordinates 
        turn_xy_list =[]
        for i in range(len(data['coordinates'])): 
            pose_position = data['odometry'][i]['pose']['pose']['position']
            pose_orientation = data['odometry'][i]['pose']['pose']['orientation']
            pose = Pose() 
            pose.position.x, pose.position.y, pose.position.z = pose_position['x'],pose_position['y'], self.rear_axle_center_height_from_ground 
            pose.orientation = Quaternion(pose_orientation['x'],pose_orientation['y'],pose_orientation['z'],pose_orientation['w'])
            turn_xy_list.append([pose.position.x,pose.position.y])
        
        rdp_thr = np.pi*0.02
        curve_points=[] 
        curve_points = rdp_calculate(turn_xy_list,rdp_thr) 
       

        for i in range(len(data['coordinates'])): 
            # Odom based path 
            lon, lat = data['coordinates'][i][0], data['coordinates'][i][1]
            odom_position = data['odometry'][i]['pose']['pose']['position']
            # print("odom position",odom_position) 
            odom_orientation = data['odometry'][i]['pose']['pose']['orientation']
            
            odom_pose = Pose()
            odom_pose.position.x, odom_pose.position.y, odom_pose.position.z = odom_position['x'], odom_position['y'], \
                                                                            self.rear_axle_center_height_from_ground
            odom_pose.orientation = Quaternion(odom_orientation['x'], odom_orientation['y'], odom_orientation['z'],
                                            odom_orientation['w'])
            if i == 0:
                prev_pose = odom_pose

            dis_btw_2_pose = distance_btw_poses(odom_pose, prev_pose)  
          
            # linear interpolating the highest path_resolution with parameter   
            if dis_btw_2_pose >= self.max_dis_btw_points:   
                
                # check whether the points are found at curve points 
                if not([odom_pose.position.x,odom_pose.position.y] in curve_points or [prev_pose.position.x,prev_pose.position.y] in curve_points): 

                    # if rtk_Status and interplote param is true internal interpolation would occur  
                    if((rtk_status and self.interpolate_with_rtk)or(not rtk_status and self.interpolate_without_rtk)): 
                        
                        rospy.logwarn_once(f"RTK_status- {rtk_status}, Interpolate_with_rtk- {self.interpolate_with_rtk}, Interpolate_without_rtk- {self.interpolate_without_rtk} ")
                        internal_path_res = distance_btw_poses(odom_pose,prev_pose)
                        # rospy.loginfo_once("internal path resolution %s", str(internal_path_res))
                        rospy.logwarn_once("Interpolating the internal path resolution is %s meters", str(internal_path_res))
                        n_points = internal_path_res // self.path_resolution
                        angle = angle_btw_poses(odom_pose,prev_pose)
                        prev_pose_ = prev_pose
                        for j in range(0, int(n_points)-1):
                            # odom path
                            px = trajectory_msg.points[-1].pose.position.x
                            py = trajectory_msg.points[-1].pose.position.y
                            px_updated = px + self.path_resolution * math.cos(angle)
                            py_updated = py + self.path_resolution * math.sin(angle)
                            odom_pose_ = Pose()
                            odom_pose_.position.x, odom_pose_.position.y, odom_pose_.position.z = px_updated, py_updated, \
                                                                                                self.rear_axle_center_height_from_ground
                            odom_pose_.orientation = yaw_to_quaternion(angle)
                            # Trajectory 
                            lat, lng = xy2ll(px_updated, py_updated, home_lat, home_long) 

                            dis_btw_2_pose = distance_btw_poses(odom_pose_, prev_pose_)
                            accumulated_distance = accumulated_distance + dis_btw_2_pose
                            prev_pose_ = odom_pose_

                            # Trajectory msg filling
                            traj_pt_msg = TrajectoryPoint()
                            traj_pt_msg.pose = odom_pose_
                            traj_pt_msg.gps_pose.position.latitude = lat
                            traj_pt_msg.gps_pose.position.longitude = lon
                            # traj_pt_msg.longitudinal_velocity_mps =
                            traj_pt_msg.index = last_index
                            last_index = last_index + 1
                            traj_pt_msg.accumulated_distance_m = accumulated_distance
                            trajectory_msg.points.append(traj_pt_msg)
                                                    
                    else:
                        rospy.logwarn_once(f"RTK_status- {rtk_status}, Interpolate_with_rtk- {self.interpolate_with_rtk}, Interpolate_without_rtk- {self.interpolate_without_rtk} ")
                else: 
                    rospy.logerr_once(f"Points missing at turnings, Distance between points: {dis_btw_2_pose}, cannot interpolate")  
                    self.turn_distance = dis_btw_2_pose
                    self.turn_interpolate = True
            
            accumulated_distance = accumulated_distance + dis_btw_2_pose        
            prev_pose = odom_pose 
            # Trajectory msg filling
            traj_pt_msg = TrajectoryPoint() 
            traj_pt_msg.pose = odom_pose  
            traj_pt_msg.gps_pose.position.latitude = lat
            traj_pt_msg.gps_pose.position.longitude = lon  
            # traj_pt_msg.longitudinal_velocity_mps =
            traj_pt_msg.index = last_index 
            last_index = last_index+1
            traj_pt_msg.accumulated_distance_m = accumulated_distance
            trajectory_msg.points.append(traj_pt_msg)    

        # connecting the first and last way points 
        if self.mission_continue: 
            distance_btw_2end_pts = distance_btw_poses(trajectory_msg.points[-1].pose, trajectory_msg.points[0].pose) 
            rospy.loginfo("distance between first and last way point %s", str(distance_btw_2end_pts)) 
            
            # trajectory pose of last index
            traj_last = trajectory_msg.points[-1]
            traj_last_pose = Pose()
            traj_last_pose.orientation = Quaternion(traj_last.pose.orientation.x, traj_last.pose.orientation.y, traj_last.pose.orientation.z,
                                                    traj_last.pose.orientation.w) 
            # yaw of trajectory last point
            traj_last_yaw = get_yaw(traj_last_pose.orientation)

            # trajectory pose of first index  
            traj_first = trajectory_msg.points[0]
            traj_first_pose = Pose()
            traj_first_pose.orientation = Quaternion(traj_first.pose.orientation.x, traj_first.pose.orientation.y, traj_first.pose.orientation.z,
                                                    traj_first.pose.orientation.w) 
            # yaw of trajectory first point
            traj_first_yaw = get_yaw(traj_first_pose.orientation)   

            # dot product of trajectory first and last yaw 
            dot_product_first_nd_last_point =  math.cos(traj_first_yaw) * math.cos(traj_last_yaw) + math.sin(traj_first_yaw) * math.sin(traj_last_yaw)  
            if dot_product_first_nd_last_point > 0: 
                direction_btw_2end_pts = "forward"
            else: 
                direction_btw_2end_pts = "reversed"

            # checking whether the last point has crossed the starting point
            v1 = get_tie(traj_first_yaw)
            v2 = np.array([trajectory_msg.points[-1].pose.position.x - trajectory_msg.points[0].pose.position.x,
            trajectory_msg.points[-1].pose.position.y - trajectory_msg.points[0].pose.position.y])
            dot_pro = np.dot(v1,v2) 
            last_pt_ahead_start_pt = False

            # when the dot_product is positive to indicate that orientation are faced forward (same direction)
            if distance_btw_2end_pts < self.mission_continue_dist_thr and dot_product_first_nd_last_point > 0:  
                
                #last point is behind start point,hence interpolation them
                if dot_pro < 0: 
                    last_pt_ahead_start_pt = False 
                    rospy.logwarn(f"path's end and start points are {round(distance_btw_2end_pts)} meters apart , {direction_btw_2end_pts} orientation | last_pt_ahead_start_pt:{last_pt_ahead_start_pt}-{round(dot_pro)}, hence Interpolating them ")
                
                #last point is ahead start point,hence interpolation failed
                else:
                    last_pt_ahead_start_pt = True 
                    self.error_occured = True
                    self.diagnostics_status.summary(WARN,f"distance_btw_end_pts: {round(distance_btw_2end_pts)}/{self.mission_continue_dist_thr}-Dist_Th | orientation_btw_end_pts: {round(dot_product_first_nd_last_point)}-{direction_btw_2end_pts} | last_pt_ahead_start_pt:{last_pt_ahead_start_pt} hence interpolation failed") 
                    rospy.logwarn(f"path's end and start points are {round(distance_btw_2end_pts)} meters apart , {direction_btw_2end_pts} orientation | last_pt_ahead_start_pt:{last_pt_ahead_start_pt}-{round(dot_pro)}, hence Interpolation failed ")
            
                if last_pt_ahead_start_pt == False :
                    n_points = distance_btw_2end_pts // self.path_resolution
                    angle = angle_btw_poses(trajectory_msg.points[0].pose, trajectory_msg.points[-1].pose)
                    for j in range(0, int(n_points)):
                        # odom path
                        px = trajectory_msg.points[-1].pose.position.x
                        py = trajectory_msg.points[-1].pose.position.y
                        px_updated = px + self.path_resolution * math.cos(angle)
                        py_updated = py + self.path_resolution * math.sin(angle)
                        odom_pose = Pose()
                        odom_pose.position.x, odom_pose.position.y, odom_pose.position.z = px_updated, py_updated, \
                                                                                            self.rear_axle_center_height_from_ground
                        odom_pose.orientation = yaw_to_quaternion(angle)
                        # Trajectory
                        lat, lng = xy2ll(px_updated, py_updated, home_lat, home_long)

                        dis = distance_btw_poses(odom_pose, prev_pose)
                        accumulated_distance = accumulated_distance + dis
                        prev_pose = odom_pose
                        # Trajectory msg filling
                        traj_pt_msg = TrajectoryPoint()
                        traj_pt_msg.pose = odom_pose

                        traj_pt_msg.gps_pose.position.latitude = lat
                        traj_pt_msg.gps_pose.position.longitude = lng
                        # traj_pt_msg.longitudinal_velocity_mps =
                        traj_pt_msg.index = last_index
                        last_index = last_index +1
                        traj_pt_msg.accumulated_distance_m = accumulated_distance
                        trajectory_msg.points.append(traj_pt_msg) 

            
                
            else:
                rospy.logwarn(f"distance_btw_end_pts: {round(distance_btw_2end_pts)}/{self.mission_continue_dist_thr}-Dist_Th | orientation_btw_end_pts: {round(dot_product_first_nd_last_point)}-{direction_btw_2end_pts} | ends_connected-dot_pro {round(dot_pro)}, interpolation failed") 
                self.error_occured = True
                rospy.set_param("/mission_continue", False)  
                if last_pt_ahead_start_pt:  
                    rospy.logwarn(f"distance_btw_end_pts: {round(distance_btw_2end_pts)}/{self.mission_continue_dist_thr}-Dist_Th | orientation_btw_end_pts: {round(dot_product_first_nd_last_point)}-{direction_btw_2end_pts} | last_pt_ahead_start_pt:{last_pt_ahead_start_pt} interpolation failed") 
                    self.diagnostics_status.summary(WARN,f"distance_btw_end_pts: {round(distance_btw_2end_pts)}/{self.mission_continue_dist_thr}-Dist_Th | orientation_btw_end_pts: {round(dot_product_first_nd_last_point)}-{direction_btw_2end_pts} | last_pt_ahead_start_pt:{last_pt_ahead_start_pt} interpolation failed") 
                else:
                    rospy.logwarn(f"distance_btw_end_pts: {round(distance_btw_2end_pts)}/{self.mission_continue_dist_thr}-Dist_Th | orientation_btw_end_pts: {round(dot_product_first_nd_last_point)}-{direction_btw_2end_pts} interpolation failed") 
                    self.diagnostics_status.summary(WARN,f"distance_btw_end_pts: {round(distance_btw_2end_pts)}/{self.mission_continue_dist_thr}-Dist_Th | orientation_btw_end_pts: {round(dot_product_first_nd_last_point)}-{direction_btw_2end_pts} interpolation failed") 
            self.diagnostics_status.add("mission continue",self.mission_continue)
            self.diagnostics_status.add("mission_continue_distance_Thr ",self.mission_continue_dist_thr)
            self.diagnostics_status.add("distance between 1st and last pt",distance_btw_2end_pts)
            self.publisher_diagnostics()  
                


        else: 
            distance = distance_btw_poses(trajectory_msg.points[-1].pose, trajectory_msg.points[0].pose)
            rospy.logdebug("distance between first and last way point %s", str(distance))
            rospy.logdebug("Mission continue not selected")

            # opposite_path with forward motion is true and mission continue to false  
            # reference pose is considered as last point
            odom_prev = trajectory_msg.points[-1]
            last_reference_pose = Pose()
            last_reference_pose.position.x, last_reference_pose.position.y, last_reference_pose.position.z =  odom_prev.pose.position.x, odom_prev.pose.position.y, \
                                                                            self.rear_axle_center_height_from_ground
            last_reference_pose.orientation = Quaternion(odom_prev.pose.orientation.x, odom_prev.pose.orientation.y, odom_prev.pose.orientation.z,
                                                    odom_prev.pose.orientation.w) 
            reference_yaw = get_yaw(last_reference_pose.orientation) 
          
            # parameter set true for opposite path with forward motion 
            if self.opposite_path == True: 
                rospy.loginfo("Opposite Path forward motion is enabled")  

                # to find the opposite oriented closest point from reverse loops with respect to last trajectory point 
                for i in range(len(trajectory_msg.points),0,-1): 
                    odom_now = trajectory_msg.points[i-1] 
                    current_odom_pose = Pose()
                    current_odom_pose.position.x, current_odom_pose.position.y, current_odom_pose.position.z = odom_now.pose.position.x, odom_now.pose.position.y, \
                                                                                    self.rear_axle_center_height_from_ground
                    current_odom_pose.orientation = Quaternion(odom_now.pose.orientation.x, odom_now.pose.orientation.y, odom_now.pose.orientation.z,
                                                    odom_now.pose.orientation.w) 
                    # calc the yaw for the orientation 
                    check_yaw = get_yaw(current_odom_pose.orientation)   
                    dot_product = math.cos(reference_yaw) * math.cos(check_yaw) + math.sin(reference_yaw) * math.sin(check_yaw) 
                    if dot_product < 0 : #dot product is negative to indicates the orientation are facing reversed 
                        distance = distance_btw_poses(last_reference_pose,current_odom_pose)
                        if closest_distance == 0: 
                            closest_distance = distance 
                        elif distance < closest_distance: 
                            closest_distance = distance
                            closest_opp_coord_index = i - 1  

                if closest_opp_coord_index!=0:
                    if closest_distance <= self.opposite_closest_pt_dis_Thr:
                        # appending the trajectory from closest_opposite_index to start of the path
                        for i in range(closest_opp_coord_index,0,-1): 
                            odom_then = trajectory_msg.points[i] 
                            odom_pose1 = Pose()
                            odom_pose1.position.x, odom_pose1.position.y, odom_pose1.position.z = odom_then.pose.position.x, odom_then.pose.position.y, \
                                                                                            self.rear_axle_center_height_from_ground
                            odom_pose1.orientation = Quaternion(odom_then.pose.orientation.x, odom_then.pose.orientation.y, odom_then.pose.orientation.z,
                                                            odom_now.pose.orientation.w) 
                            odom_now = trajectory_msg.points[i-1] 
                            odom_pose2 = Pose()
                            odom_pose2.position.x, odom_pose2.position.y, odom_pose2.position.z = odom_now.pose.position.x, odom_now.pose.position.y, \
                                                                                            self.rear_axle_center_height_from_ground
                            odom_pose2.orientation = Quaternion(odom_now.pose.orientation.x, odom_now.pose.orientation.y, odom_now.pose.orientation.z,
                                                            odom_now.pose.orientation.w) 
                            
                            close_pt_msg = TrajectoryPoint()
                            close_pt_msg.pose.position = odom_then.pose.position 
                            current_yaw = get_yaw(odom_then.pose.orientation)
                            updated_yaw = rotate_yaw_180(current_yaw)
                            updated_orientation = yaw_to_quaternion(updated_yaw)
                            close_pt_msg.pose.orientation = updated_orientation
                            close_pt_msg.longitudinal_velocity_mps = self.max_forward_speed
                            close_pt_msg.index = len(trajectory_msg.points)+1
                            dis = distance_btw_poses(odom_pose1, odom_pose2)
                            accumulated_distance = accumulated_distance + dis
                            close_pt_msg.accumulated_distance_m = accumulated_distance
                            trajectory_msg.points.append(close_pt_msg)

                        # path_publisher_diagnostics publishing true it opposite path forward motion is set true
                        self.diagnostics_status.summary(OK,f"Activating Opposite Path forward motion : {self.opposite_path}") 
                        self.diagnostics_status.add("distance of closest_opposite_pt",closest_distance)  
                        self.diagnostics_status.add("opposite path forward motion",self.opposite_path)
                        self.diagnostics_status.add("opposite_closest_pt_dis_Thr",self.opposite_closest_pt_dis_Thr)
                        self.publisher_diagnostics()  
                    else: 
                        self.error_occured = True
                        rospy.logerr(f"Cannot Activating Opposite Path forward motion as closest_opposite_pts: {closest_distance}")
                        self.diagnostics_status.summary(ERROR,"Error in Activating opposite path forward motion") 
                        self.diagnostics_status.add("distance of closest_opposite_pt",closest_distance)  
                        self.diagnostics_status.add("opposite path forward motion",self.opposite_path)
                        self.diagnostics_status.add("opposite_closest_pt_dis_Thr",self.opposite_closest_pt_dis_Thr)
                        self.publisher_diagnostics()  
                else: 
                    self.error_occured = True
                    rospy.logerr("Cannot Activate Opposite Path forward motion, As end of save path was not connecting closer with parent-save path")
                    self.diagnostics_status.summary(ERROR,"Error in Activating opposite path forward motion")  
                    self.diagnostics_status.add("closest_opposite_coord_index",closest_opp_coord_index)
                    self.diagnostics_status.add("distance of closest_opposite_pt",closest_distance)  
                    self.diagnostics_status.add("opposite path forward motion",self.opposite_path)
                    self.diagnostics_status.add("opposite_closest_pt_dis_Thr",self.opposite_closest_pt_dis_Thr)
                    self.publisher_diagnostics()  



        traj_length = len(trajectory_msg.points)
        # velocity profile
        for i, traj_point in enumerate(trajectory_msg.points):

            for lhd_index in range(i, traj_length):
                path_acc_distance = trajectory_msg.points[lhd_index].accumulated_distance_m - \
                                    traj_point.accumulated_distance_m
                if path_acc_distance > self.avg_lhd:
                    break
            look_ahead = path_acc_distance
            slope = angle_btw_poses(trajectory_msg.points[lhd_index].pose, traj_point.pose)
            alpha = slope - get_yaw(traj_point.pose.orientation)
            delta = math.atan2(2.0 * vehicle_data.dimensions.wheel_base * math.sin(alpha), look_ahead)
            delta_degrees = -math.degrees(delta)
            if abs(delta_degrees) <= self.steering_limits_to_slow_down:
                trajectory_msg.points[i].longitudinal_velocity_mps = self.max_forward_speed
            else:
                trajectory_msg.points[i].longitudinal_velocity_mps = max(
                    self.speed_reduce_factor * np.interp(abs(delta_degrees),
                                                        [self.steering_limits_to_slow_down,
                                                        vehicle_data.motion_limits.max_steering_angle],
                                                        [self.max_forward_speed,
                                                        self.min_forward_speed]), self.min_forward_speed)
                # trajectory_msg.points[i].longitudinal_velocity_mps =  np.interp(abs(delta_degrees),
                #                                                     [self.steering_limits_to_slow_down,
                #                                                     vehicle_data.motion_limits.max_steering_angle],
                #                                                     [self.max_forward_speed,
                #                                                     self.min_forward_speed])  
        
        # slowing down at end of trajectory
        for slow_end_index in range(traj_length - 1, 0, -1):
            slow_end_acc_distance =  trajectory_msg.points[traj_length-1].accumulated_distance_m - trajectory_msg.points[slow_end_index].accumulated_distance_m
            if (slow_end_acc_distance) > self.distance_to_slowdown_on_ends:   
                break 
        # to find slow start index  at start of trajectory
        for slow_start_index in range(0,traj_length - 1): 
            slow_start_acc_distance =  trajectory_msg.points[slow_start_index].accumulated_distance_m - trajectory_msg.points[0].accumulated_distance_m 
            if (slow_start_acc_distance) > self.distance_to_slowstart_on_start:   
                break 
        #gradually decrease the speed to min at end of traj
        for i in range(traj_length-1,slow_end_index,-1):  
            tmp_dist = abs(trajectory_msg.points[traj_length-1].accumulated_distance_m -
                                            trajectory_msg.points[i].accumulated_distance_m) 
            trajectory_msg.points[i].longitudinal_velocity_mps = np.interp(tmp_dist,[0,self.distance_to_slowdown_on_ends],[self.min_forward_speed,self.max_forward_speed])  
        #gradually increase the speed to max at start of traj
        for i in range(0,slow_start_index):  
            tmp_dist = abs(trajectory_msg.points[slow_start_index].accumulated_distance_m -
                                            trajectory_msg.points[i].accumulated_distance_m) 
            trajectory_msg.points[i].longitudinal_velocity_mps = np.interp(tmp_dist,[0,self.distance_to_slowstart_on_start],[self.max_forward_speed,self.min_forward_speed])  

        marker_arr = trajectory_to_marker(trajectory_msg, self.max_forward_speed)
        self.trajectory_velocity_marker_pub.publish(marker_arr)
        path = trajectory_to_path(trajectory_msg)  
        self.gps_path_pub.publish(path) 
        self.global_trajectory_pub.publish(trajectory_msg) 
        rospy.loginfo("global_trajectory_published") 
        if self.turn_interpolate == True: 
            self.diagnostics_status.summary(ERROR,f"Points missing at turnings, Distance between turn_points: {self.turn_distance}, cautious to interpolate {rospy.get_name()}")  
        elif self.error_occured == False:  
            self.diagnostics_status.summary(OK,f"Happily published  {rospy.get_name()}") 
        self.diagnostics_status.add("Length ",len(data['coordinates']))  
        self.publisher_diagnostics() 

      
    def target_index(self, robot_pose, close_point_ind): 
        """
        search index of target point in the reference path. The following implementation was inspired from
        http://dyros.snu.ac.kr/wp-content/uploads/2021/02/Ahn2021_Article_AccuratePathTrackingByAdjustin-1.pdf
        Args:
            robot_pose:  pose of robot
            close_point_ind : index of close point to the vehicle 
        Returns:
            close_index, target_index, lookahead_distance, cross_track_dis,
        """ 
        lhd = self.compute_lookahead_distance(abs(self.robot_state.twist.twist.linear.x))
        close_dis = self.trajectory_data.points[close_point_ind].accumulated_distance_m
        for ind in range(close_point_ind, len(self.trajectory_data.points)):
            path_acc_distance = self.trajectory_data.points[ind].accumulated_distance_m - close_dis
            if path_acc_distance > lhd:
                return ind, distance_btw_poses(robot_pose, self.trajectory_data.points[ind].pose)
        return ind, distance_btw_poses(robot_pose, self.trajectory_data.points[ind].pose)
   
    
if __name__ == "__main__": 

    rospy.init_node("global_gps_path_publisher")

    mission_file = rospy.get_param('/patrol/mission_file', 'default.json')
    if "/" in mission_file:
        mission_file_dir = mission_file
        rospy.loginfo(f"mission_file_dir : {mission_file_dir}")
    else:
        try:
            ros_pack = rospkg.RosPack()
            mission_file_dir = ros_pack.get_path('autopilot') + "/mission_files/" + str(mission_file)
            rospy.loginfo(f"mission_file_dir : {mission_file_dir}")
        except Exception as e:
            rospy.logerr(f"Could not found autopilot package, consider souring it, ERROR: {e}")
            rospy.signal_shutdown(f"Could not found autopilot package, consider souring it, ERROR: {e}")
    gps_path_pub = GlobalGpsPathPub(mission_file_dir)
    fail_exists = os.path.isfile(mission_file_dir)
    if not fail_exists:
        rospy.logerr(f"mission file does not exist, path: {mission_file_dir}") 
        gps_path_pub.diagnostics_status.summary(ERROR,f"mission file does not exist, path: {mission_file_dir}")
        gps_path_pub.diagnostics_status.add(" mission file dir ",mission_file_dir)
        gps_path_pub.publisher_diagnostics() 
        # rospy.signal_shutdown(f"mission file does not exist, path: {mission_file_dir}") 
    else:
        rospy.loginfo("mission file exists in file directory") 

    if '.json' in mission_file or ".geojson" in mission_file:
        gps_path_pub.from_json()

    elif ".kml" in mission_file:
        gps_path_pub.from_kml()
    else:
        rospy.logerr(f"No proper extension to input mission file, path: {mission_file_dir}")
        rospy.signal_shutdown(f"No proper extension to input mission file, path: {mission_file_dir}")
    rospy.spin()