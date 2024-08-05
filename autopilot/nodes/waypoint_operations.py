#!/usr/bin/env python3
"""
This script implements waypoint operations for a route planner.
"""
import rospy
import rospkg
import json
import time
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geographiclib.geodesic import Geodesic
from pilot_msgs.msg import vehicle_stop_command
import traceback

from diagnostic_updater._diagnostic_status_wrapper import DiagnosticStatusWrapper
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from autopilot_msgs.srv import speedwaypoint, speedwaypointRequest, speedwaypointResponse

# Diagnostic Constants
OK = DiagnosticStatus.OK
ERROR = DiagnosticStatus.ERROR
WARN = DiagnosticStatus.WARN
STALE = DiagnosticStatus.STALE

class WayPointOperation:
    """
    Class to handle waypoint operations.
    """

    def __init__(self):
        
        # Initialize ROS node
        rospy.init_node('route_planner_action')
        
        # Publisher to publish the brake command
        self.brake_cmd_publisher = rospy.Publisher("/vehicle/break_command", vehicle_stop_command, queue_size=1)
        self.brake_command_data = vehicle_stop_command()
        self.brake_command_data.node = rospy.get_name()
        self.brake_command_data.message = "User Specified Waypoint Action"
        
        # Publisher to check the Pause elapsed time in seconds 
        self.pause_time_left_publisher = rospy.Publisher('/waypoint_pause_time_analysis',String,queue_size=1) 
        
        # Publisher for Diagnostics
        self.diagnostic_pub = rospy.Publisher("/waypoint_operation_diagnostics", DiagnosticArray, queue_size=1)
        self.diagnostic_msg = DiagnosticStatusWrapper()
        self.diagnostic_msg.name = rospy.get_name() # To get the node name
        
        # Initialize Variables
        self.current_coordinates = None
        self.pause_start_time = None
        self.pause_duration = None
        self.target_coordinates_list = []
        self.track_visited_target_coordinate = []
        self.properties_to_show_on_hmi = None
        
        # Initialize Ros Parameters
        self.threshold_distance = rospy.get_param("/waypoint_operation_threshold_distance",default=1)
        rospy.loginfo("The Threshold Distance in Waypoint Operations is %s",str(self.threshold_distance))
        
        # Subscribe to GPS topic
        self.gps_sub = rospy.Subscriber("/vehicle/gps", NavSatFix, self.gps_callback)
        
        # Ros Timer
        self.pause_timer = rospy.Timer(rospy.Duration(1), self.pause_timer_callback) # Timer to Publish False on the break command

        # Get mission file path
        ros_pack = rospkg.RosPack()
        self.mission_file_dir = ros_pack.get_path('autopilot') + "/mission_files/"
        
        # Set mission file name (assume no extension)
        self.mission_file = rospy.get_param('/patrol/mission_file')

        # Log GeoJSON file name
        rospy.loginfo("Waypoint Operations Geojson File: %s", self.mission_file)

        # Read mission data
        mission_file_path = self.mission_file_dir + self.mission_file
        
        try:
            self.speed_waypoint_max_value = rospy.get_param("/patrol/max_waypoint_speed",1.5)
            rospy.loginfo("The Max speed for waypoint operation is: %s", str(self.speed_waypoint_max_value))
           
        except KeyError:
            rospy.logwarn("Parameter '/patrol/max_waypoint_speed' not found")
        
        try:
            with open(mission_file_path, 'r', encoding='utf-8') as file:
                # Load GeoJSON data with full precision
                self.mission_data = json.load(file, parse_float=lambda x: round(float(x), 15))
                
                for feature in self.mission_data['features']:
                    if feature.get('properties'):
                        # Convert property keys to lowercase
                        properties_lower = {key.lower(): value for key, value in feature['properties'].items()}
                        self.target_coordinates_list.append((feature['geometry']['coordinates'], properties_lower))
                        
                # If User haven't selected any waypoint actions then killing this node gracefully       
                if len(self.target_coordinates_list) == 0:
                    rospy.logwarn("No Waypoint Operations selected, Shutting down node gracefully")
                    rospy.signal_shutdown("No Waypoint Operations selected, Shutting down node gracefully.")
                    
                rospy.loginfo("File Reading Success")
                rospy.loginfo("The Waypoint Operations: %s",str(self.target_coordinates_list))
                rospy.loginfo("Waiting for /vehicle/gps data")
                print()
        except FileNotFoundError:
            rospy.logerr("File not found: %s", mission_file_path)
        except KeyError as e:
            rospy.logerr("KeyError in waypoint-ops: %s", str(e))
        except Exception as e:
            rospy.logerr("An unexpected error occurred in waypoint-ops: %s", str(e))
            
        self.main_logic()
        
    def publish_diagnostic_msg(self):
        diagnostic_msg_array = DiagnosticArray()
        diagnostic_msg_array.status.append(self.diagnostic_msg)
        diagnostic_msg_array.header.stamp = rospy.Time.now()
        return diagnostic_msg_array
        
    # Callbacks
    def gps_callback(self, msg):
        """Callback function for GPS data."""
        self.current_coordinates = [msg.longitude, msg.latitude]
        rospy.loginfo_once("Data Present in Gps_callback")
       
    def pause_timer_callback(self, event):
        """
        Callback function for pause timer, which will send False to the brake command topic.

        Args:
            event (rospy.TimerEvent): The timer event triggering the callback.
        """
        try:
            self.diagnostic_msg.clearSummary()
            self.diagnostic_msg.values = []
            if self.pause_start_time is not None:
                elapsed_time = (rospy.Time.now() - self.pause_start_time).to_sec()
                if elapsed_time >= self.pause_duration:
                    rospy.loginfo("Publishing False to Brake Command")
                    
                    self.diagnostic_msg.summary(OK, "User Specified Action")
                    self.diagnostic_pub.publish(self.publish_diagnostic_msg())
                    
                    # If the pause time has elapsed, publish False and reset pause variables
                    self.brake_command_data.status = False
                    self.brake_command_data.message = "OK"
                    self.brake_cmd_publisher.publish(self.brake_command_data)
                    self.pause_start_time = None
                    self.pause_duration = None
                else:
                    self.diagnostic_msg.summary(ERROR, "User Specified Action")
                    self.diagnostic_msg.add("Waypoint Operations", str(self.properties_to_show_on_hmi))
                    self.diagnostic_pub.publish(self.publish_diagnostic_msg())
                    self.pause_time_left_publisher.publish(f"Total Pause Time is {self.pause_duration} secs : Current Time is {elapsed_time} secs")
            else:
                # If there's no active pause, do nothing
                pass
        except Exception as e:
            rospy.logerr("Error in pause timer callback: %s", traceback.format_exc())
            

    def pause_at_point(self, pause_time):
        """
        Pause at a waypoint for a specified duration.

        Args:
            pause_time (float): Duration to pause in seconds.
        """
        try:
            rospy.loginfo("The Pause Duration is %s seconds", pause_time)
            self.pause_start_time = rospy.Time.now()
            self.pause_duration = pause_time
            rospy.loginfo("Publishing Brake Command for %s seconds", pause_time)
            self.brake_command_data.status = True
            self.brake_cmd_publisher.publish(self.brake_command_data)
        except Exception as e:
            rospy.logerr("Exception in pause_at_point: %s", str(e))
            rospy.logerr("Error in pause_at_point: %s", traceback.format_exc())
               
    def speed_at_point(self, speed_value):
        """
        Apply speed at a waypoint.

        Args:
            speed_value (float): Speed value to apply.
        """
        # current_value = None
        updated_current_speed = None
        rospy.loginfo("The speed is applied %s", speed_value)

        # try:
            # current_value = rospy.get_param("/patrol/max_forward_speed")
            # rospy.loginfo("The Current Speed Value is: %s", str(current_value))
        # except KeyError:
            # rospy.logwarn("Parameter 'patrol/max_forward_speed' not found")
            # current_value = 3.5

        if speed_value > self.speed_waypoint_max_value:
            rospy.loginfo("The Speed is above the limit so setting it to the system max speed, which is {}".format(self.speed_waypoint_max_value))
            updated_current_speed = self.speed_waypoint_max_value
        elif speed_value > 0 and speed_value <= self.speed_waypoint_max_value:
            updated_current_speed = speed_value
        else:
            rospy.loginfo("Cannot handle the speed value %s", str(speed_value))
            return

        try:
            speed_service = rospy.ServiceProxy('speed_waypoint', speedwaypoint)
            output = speed_service(speed_value)
            rospy.loginfo("The Speed Service Status: %s", output.status)
            
            # Convert the result back to float for logging
            result_speed = float(output.result)
            rospy.loginfo("The Speed Service Value: %f", result_speed)
            
        except rospy.ServiceException as e:
            rospy.loginfo("Speed Service call failed: %s", str(e))

        # Update the parameter with the new speed value
        self.diagnostic_msg.summary(ERROR, "User Specified Action")
        self.diagnostic_msg.add("Waypoint Operations", str(self.properties_to_show_on_hmi))
        self.diagnostic_pub.publish(self.publish_diagnostic_msg())
        
        time.sleep(3)
        
        self.diagnostic_msg.summary(OK, "User Specified Action")
        self.diagnostic_msg.add("Waypoint Operations", str(self.properties_to_show_on_hmi))
        self.diagnostic_pub.publish(self.publish_diagnostic_msg())
        
        # rospy.set_param("patrol/max_forward_speed", updated_current_speed)
        rospy.loginfo("The Updated Speed Value is: %s", str(updated_current_speed))
            
        
    def tipperbed_at_point(self, tipperbed_value):
        """
        Adjust tipperbed at a waypoint.

        Args:
            tipperbed_value (bool): Tipperbed value to adjust.
        """
        rospy.loginfo("The tipperbed value is %s", tipperbed_value)
    
    def are_coordinates_close(self, coord1, coord2, threshold):
        """
        Determine if two geographic coordinates are close based on a given threshold.

        Args:
            coord1 (tuple): A tuple of latitude and longitude (degrees) for the first coordinate.
            coord2 (tuple): A tuple of latitude and longitude (degrees) for the second coordinate.
            threshold (float, optional): The maximum allowed distance in meters between the points.
                                        Defaults to 1 meter.

        Returns:
            bool: True if the distance between the coordinates is less than the threshold, False otherwise.
        """
        try:
            # Create Geodesic object using the WGS84 ellipsoid for best accuracy
            geod = Geodesic.WGS84

            # Convert coordinates from degrees to radians
            longitude1, latitude1 = coord1
            longitude2, latitude2 = coord2

            # Calculate geodesic distance in meters with high precision
            distance_meters = geod.Inverse(latitude1, longitude1, latitude2, longitude2)['s12']
            
            if distance_meters <= threshold:
                rospy.logdebug("The Distance Meters inside if %s",str(distance_meters))
                if coord2 not in self.track_visited_target_coordinate:
                    # Add current coordinate to visited list
                    self.track_visited_target_coordinate.append(coord2)
                    return True
                else:
                    return False
            else:
                rospy.logdebug("Outside The Distance Meters %s",str(distance_meters))
                # Reset the lists as the vehicle has moved away from the waypoint
                self.track_visited_target_coordinate = []
                return False

        except Exception as e:
            rospy.logerr("Error calculating distance: %s", traceback.format_exc())
            return False

    def main_logic(self): 
        """Main logic to handle waypoint operations."""
        while not rospy.is_shutdown():
            try:
                if self.current_coordinates is not None: 
                    closest_target_coordinate = None
                    min_distance = float('inf')
                    for target_coordinates_value, properties in self.target_coordinates_list:
                        # Selecting only one co-ordinate from the list of target_co-ordinates which is close to the current_co-ordinate
                        distance = self.calculate_distance(self.current_coordinates, target_coordinates_value)
                        if distance < min_distance:
                            min_distance = distance
                            closest_target_coordinate = target_coordinates_value

                    if closest_target_coordinate is not None:
                        for target_coordinates_value, properties in self.target_coordinates_list:
                            if target_coordinates_value == closest_target_coordinate:
                                if  self.are_coordinates_close(self.current_coordinates, target_coordinates_value,self.threshold_distance):
                                    print()
                                    rospy.loginfo("Found Close co-ordinates: Current: %s, Target: %s", self.current_coordinates, target_coordinates_value)
                                    rospy.loginfo("Waypoint Action: %s", properties)
                                    self.properties_to_show_on_hmi = properties
                                
                                    if ('pause' in properties) and ('speed' in properties):
                                        rospy.loginfo("Speed & Pause Feature")
                                        pause_time_value = properties['pause']
                                        self.pause_at_point(pause_time_value)
                                        speed_value = properties['speed']
                                        self.speed_at_point(speed_value)
                                        
                                    # Access properties and values here
                                    elif 'speed' in properties:
                                        rospy.loginfo("Speed Feature")
                                        speed_value = properties['speed']
                                        self.speed_at_point(speed_value)
                    
                                    elif 'pause' in properties:
                                        rospy.loginfo("Pause Feature")
                                        pause_time_value = properties['pause']
                                        self.pause_at_point(pause_time_value)
                                        
                                    elif 'tipperbed' in properties:
                                        rospy.loginfo("Tipperbed Feature")
                                        tipperbed_value = properties['tipperbed']
                                        self.tipperbed_at_point(tipperbed_value)
                                    else:
                                        rospy.loginfo("Invalid Selection Made!")          
                                else:
                                    rospy.loginfo_throttle(10,"No Close Points Found for Waypoint Actions")
            except Exception as e:
                rospy.logerr("An error occurred in main logic of waypoint Operation: %s", traceback.format_exc())
            rospy.sleep(0.1)

    def calculate_distance(self, coord1, coord2):
        """
        Calculate distance between two geographic coordinates in meters.

        Args:
            coord1 (tuple): A tuple of latitude and longitude (degrees) for the first coordinate.
            coord2 (tuple): A tuple of latitude and longitude (degrees) for the second coordinate.

        Returns:
            float: The distance between the coordinates in meters.
        """
        try:
            # Create Geodesic object using the WGS84 ellipsoid for best accuracy
            geod = Geodesic.WGS84

            # Convert coordinates from degrees to radians
            longitude1, latitude1 = coord1
            longitude2, latitude2 = coord2

            # Calculate geodesic distance in meters with high precision
            distance_meters = geod.Inverse(latitude1, longitude1, latitude2, longitude2)['s12']
            return distance_meters
        
        except Exception as e:
            rospy.logerr("Error calculating distance: %s", traceback.format_exc())
            return None

if __name__ == "__main__":
    try:
        waypoint_obj = WayPointOperation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node has been terminated in waypoint-ops.")
    except rospy.ROSException as e:
        rospy.logerr("ROS error occurred in waypoint-ops: %s", str(e))
    except Exception as e:
        rospy.logerr("An unexpected error occurred in the main block of waypoint-ops: %s", traceback.format_exc())
