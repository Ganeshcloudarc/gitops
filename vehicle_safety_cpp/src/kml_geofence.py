#!/usr/bin/env python3
from math import *
import rospy 
import rospkg
from json import dump
from fastkml import geometry, kml 
from shapely.geometry import  Polygon
from vehicle_safety_cpp.srv import *

# import trigger

class KmlToGeofence: 
    def __init__(self,kml_file_dir): 
        # self.cb_clt()
        self.kml_coords =[]  
        self.kml_list=[] 
        self.size_kml = 0 
        self.Area=[]
        self.geofence =[]   
        self.nogozone =[]
        rospy.logdebug(f"kml_file: {kml_file_dir}")
        
        # reading kml file 
        try:
            with open(kml_file_dir,'r')as geo_fence:
                doc = geo_fence.read().encode('utf-8') 
                k = kml.KML() 
                k.from_string(doc)
                # print(features)
                for i in k.features(): 
                    print("{},{}".format(i.name, i.description)) 
                    for j in i.features(): 
                        if isinstance(j.geometry, geometry.MultiPolygon):
                            polygon_ = j.geometry
                            for i in polygon_.geoms:
                                area_polygon = [[i[1],i[0]] for i in list(i.exterior.coords)]
                                self.kml_list.append(area_polygon)

                        elif isinstance(j.geometry, geometry.Polygon): 
                            polygon = j.geometry 
                            for coordinates in polygon.exterior.coords: 
                                self.kml_coords.append([coordinates[1],coordinates[0]])
                                self.size_kml = len(self.kml_coords)

                        else: 
                            rospy.logerr(f"KML file format is not LineString")

                if (self.size_kml != 0):
                    self.geofence = self.kml_coords
                    rospy.set_param('/vehicle_safety/geo_fence_coordinates',self.geofence)
                
                # when multiple polygon of coordinates are selected
                else:
                    for i in range(0,len(self.kml_list)):
                        poly = Polygon(self.kml_list[i]) 
                        Area_ = poly.area 
                        self.Area.append(Area_)
                        max_area = max(self.Area) 
                        index = self.Area.index(max_area) 
                    self.geofence = self.kml_list[index]
                    rospy.set_param('/vehicle_safety/geo_fence_coordinates', self.geofence)
                    self.kml_list.pop(index) 
                    rospy.set_param('/vehicle_safety/no_go_zone_coordinates', self.kml_list)
                    rospy.loginfo("Geofence params set successful from geofence.kml")
                self.cb_clt()
        except FileNotFoundError as err:
            rospy.logwarn(f"Using Default geofence params. geofence.kml not found in {kml_file_dir}")  
            rospy.signal_shutdown("File not found"+ str(err))
        except Exception as err:
            rospy.logwarn(f'Exception in kml_geofence.py: {err}')
            rospy.signal_shutdown("Exception in kml_geofence.py"+ str(err))
    def cb_clt(self):
            # service call for the updating the visuality in rviz
            
            rospy.wait_for_service('no_go_geo_fence_coordinates') 
            try : 
                upd_geo_fence = rospy.ServiceProxy('no_go_geo_fence_coordinates',trigger) 
                rospy.loginfo("ros client is running")
                return upd_geo_fence 
                
            
            except rospy.ServiceException as e: 
                rospy.loginfo("service call failed %s" %e) 
            rospy.loginfo("update with new geofence param")

            

if __name__ == '__main__': 
    rospy.init_node("geofence")
    kml_file = rospy.get_param("/autopilot/mission_file","geofence.kml") 
    if '.kml' in kml_file:
        pass 
    else: 
        kml_file = kml_file + '.kml' 
    try:
        ros_pack = rospkg.RosPack()
        kml_file_dir =ros_pack.get_path('autopilot') + "/mission_files/" +str(kml_file)
    except Exception as e:
        rospy.logerr("Please source vehicle_safety_cpp package: " + str(e)) 


    rospy.logdebug(f"kml_file_dir: {kml_file_dir}")
    kml_to_geofence_coor = KmlToGeofence(kml_file_dir)
    rospy.spin() 
