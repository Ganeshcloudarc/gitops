import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import random
from geometry_msgs.msg import Point, PoseArray, Pose, TransformStamped, PoseStamped, PolygonStamped, Point

class DwaPathGenerator:
    init = False

    def __init__(self, wheel_base, steering_angle_max, steering_inc, max_path_length, path_resolution):
        """
        wheel_base
        steering_angle_max : in degrees
        steering_inc : in degrees
        path_resolution : in meters
        """
        self.wheel_base = wheel_base
        self.steering_angle_max = steering_angle_max
        self.steering_inc = steering_inc
        self.max_path_length = max_path_length
        self.path_resolution = path_resolution
        # self.init = False
        self.path_from_origin = self.generate_paths(robot_pose=[0, 0, 0])

    def generate_paths(self, robot_pose=[0, 0, 0]):
        if robot_pose == [0, 0, 0] and DwaPathGenerator.init:
            return self.path_from_origin
        else:
            paths = []
            for i in range(self.steering_angle_max, -self.steering_angle_max - self.steering_inc, -self.steering_inc):
                # print("i", i)
                if i == 0:
                    i = 0.1
                steering_angle = np.radians(i)
                turn_angle = self.max_path_length / self.wheel_base * np.tan(steering_angle)
                turn_radius = self.max_path_length / turn_angle
                # Finding icr
                icr_x = robot_pose[0] - np.sin(robot_pose[2]) * turn_radius
                icr_y = robot_pose[1] + np.cos(robot_pose[2]) * turn_radius
                # Finding arc_length
                arc_length = abs(turn_angle) * turn_radius
                number_of_points_on_arc = np.ceil(arc_length / self.path_resolution)
                # number_of_points_on_arc = arc_length // self.path_resolution
                if number_of_points_on_arc == 0:
                    inc_angle = 0.01
                else:
                    inc_angle = turn_angle / number_of_points_on_arc

                arc_angles_wrt_path_resolution = np.arange(0, turn_angle + np.sign(turn_angle) * inc_angle,
                                                           np.sign(turn_angle) * inc_angle)
                x = icr_x + np.sin(robot_pose[2] + arc_angles_wrt_path_resolution) * turn_radius
                y = icr_y - np.cos(robot_pose[2] + arc_angles_wrt_path_resolution) * turn_radius
                theta = robot_pose[2] + arc_angles_wrt_path_resolution
                array1 = np.column_stack([x, y, theta])
                paths.append(array1)
            DwaPathGenerator.init = True
            return paths

    def get_path_from_origin(self):
        return self.path_from_origin

    def transform_path(self, robot_pose):
        """Transform all the origin points by robot pose"""
        pass


    def get_dwa_paths_marker_array(self, dwa_paths, target_frame="base_link"):
        marker_arr_msg = MarkerArray()
        # marker_arr_msg.header.frame_id = target_frame
        marker = Marker()
        marker.header.frame_id = target_frame
        marker.action = marker.DELETEALL
        marker_arr_msg.markers.append(marker)
        count = 0
        for path in dwa_paths:
            marker = Marker()
            marker.header.frame_id = target_frame
            # marker.header.stamp = rospy.Time.now()
            marker.type = marker.LINE_STRIP
            marker.id = count
            count += 1
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.color.a = 0.5
            marker.color.r = random.random()
            marker.color.g = random.random()
            marker.color.b = random.random()
            for point in path:
                pt = Point()
                pt.x = point[0]
                pt.y = point[1]
                marker.points.append(pt)
            marker_arr_msg.markers.append(marker)
        return marker_arr_msg


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    max_steer = 30
    dwa_path_gen = DwaPathGenerator(2.5, max_steer, 2, 10, 1)

    paths = dwa_path_gen.generate_paths(robot_pose=[0, 0, 0])
    print("len", len(paths))
    # for path in paths:
    path = paths[(len(paths) - 1) // 2]
    plt.plot(path[:, 0], path[:, 1])
    plt.scatter(path[:, 0], path[:, 1])
    plt.xlim(0, 15)
    plt.ylim(-10, 10)
    plt.show()
