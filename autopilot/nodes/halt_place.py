#!/usr/bin/env python3
import traceback
import rospy
import rospkg
import geojson
from geographiclib.geodesic import Geodesic
from sensor_msgs.msg import NavSatFix
from pilot_msgs.msg import vehicle_stop_command

class HaltArea:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('halt_at_place')

        # Publisher
        self.stop_cmd_publisher = rospy.Publisher("/vehicle/stop_command", vehicle_stop_command, queue_size=1)
        self.stop_command_data = vehicle_stop_command()

        # Initialize current coordinates
        self.current_coordinates = None

        # Get mission file path
        ros_pack = rospkg.RosPack()
        self.mission_file_dir = ros_pack.get_path('autopilot') + "/mission_files/"

        # Subscribe to GPS topic
        self.gps_sub = rospy.Subscriber("/vehicle/gps", NavSatFix, self.gps_callback)

        # Set mission file name (assume no extension)
        self.mission_file = rospy.get_param('/patrol/mission_file')

        # self.mission_file = str(self.mission_file.split(".")[0]) # As the mission has the extension we are removing it

        # # Log GeoJSON file name
        rospy.logerr_once("Geo-JSON file: %s", self.mission_file)

        # # Read mission data
        mission_file_path = self.mission_file_dir + self.mission_file
        # mission_file_path = self.mission_file_dir+"halt.json"

        try:
            with open(mission_file_path, 'r') as file:
                self.mission_data = geojson.load(file)
                rospy.loginfo("File Reading Success")
                rospy.loginfo_once("Waiting for /vehicle/gps data")
        except FileNotFoundError:
            rospy.logerr("File not found: %s", mission_file_path)

    def main(self, current_coordinates):
        dictionary_of_task = {}
        try:
            for feature in self.mission_data['features']:
                if 'name' in feature['properties']:
                    task_name = feature['properties']['name']
                    target_coordinates_list = feature['geometry']['coordinates']  # list of coordinates

                    # Check if the task name already exists in the dictionary
                    if task_name in dictionary_of_task:
                        # If the task name already exists, append the target coordinates list to its list
                        dictionary_of_task[task_name].append(target_coordinates_list)
                    else:
                        # If the task name doesn't exist, create a new list with the target coordinates list
                        dictionary_of_task[task_name] = [target_coordinates_list]
                else:
                    rospy.loginfo_once("Feature does not contain 'name' property")
        except KeyError as e:
            rospy.logerr("Error accessing mission data: %s", e)
            rospy.signal_shutdown("In Pause and Continue KeyError encountered while accessing mission data. Shutting down node gracefully.")

        rospy.loginfo_once("The User Feature Dictionary is %s", dictionary_of_task)
        rospy.loginfo_once("-------------------------------------------------------")

        for key, value in dictionary_of_task.items():
            key = key.lower()
            for target_coordinates in value:  # Iterate over each coordinate list
                if self.are_coordinates_close(current_coordinates, target_coordinates):
                    rospy.loginfo("Found!! Close Point Key is : %s and current_co-ordinate: %s", key, current_coordinates)
                    if key == "pause":
                        try:
                            pause_value_time = feature['properties']['value']
                            self.pause_at_point(pause_value_time)
                            rospy.loginfo_once("The Pause time value is %s ", pause_value_time)
                        except KeyError as e:
                            rospy.logerr("Error accessing pause value: %s", e)
                    else:
                        rospy.loginfo_once("There is a different key %s", key)
                else:
                    rospy.loginfo_throttle(20, "Current coordinates are NOT close to the target coordinates. Current: %s, Target: %s", self.current_coordinates, target_coordinates)


    def pause_at_point(self, pause_time):
        start_time = rospy.Time.now()
        self.stop_command_data.node = rospy.get_name()
        self.stop_command_data.status = True
        self.stop_command_data.message = "Pause Point selected by User"
        while (rospy.Time.now() - start_time).to_sec() < pause_time:
            rospy.loginfo_once("Publishing Stop Command for %s seconds", pause_time)
            self.stop_cmd_publisher.publish(self.stop_command_data)
            rospy.sleep(1)  # Publish every 1 second
        self.stop_command_data.status = False
        self.stop_command_data.message = "OK"
        self.stop_cmd_publisher.publish(self.stop_command_data)

    def are_coordinates_close(self, coord1, coord2, threshold=0.1):
        """
        Determines if two geographic coordinates are considered close based on a given threshold.

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
            latitude1, longitude1 = coord1
            latitude2, longitude2 = coord2

            # Calculate geodesic distance in meters with high precision
            distance_meters = geod.Inverse(latitude1, longitude1, latitude2, longitude2)['s12']
            # rospy.loginfo("The Distance Meter %s",distance_meters)

            # Return True if distance is less than threshold, False otherwise
            return distance_meters < threshold

        except Exception as e:
            rospy.logerr("Error calculating distance: %s", e)
            rospy.logerr(traceback.format_exc())
            return False

    def gps_callback(self, msg):
        self.current_coordinates = [msg.longitude, msg.latitude]
        self.main(self.current_coordinates)

if __name__ == "__main__":
    try:
        halt_place = HaltArea()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node has been terminated.")
