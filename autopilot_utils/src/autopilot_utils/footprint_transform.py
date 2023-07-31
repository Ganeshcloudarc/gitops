import numpy as np
from geometry_msgs.msg import Polygon, Point, PolygonStamped, Pose, Polygon
from autopilot_utils.pose_helper import get_yaw
from jsk_recognition_msgs.msg import PolygonArray
from nav_msgs.msg import Path


def transform_footprint(pose, vehicle_data, to_polygon=False):
    """
        Args:
            pose : Pose to which vehicle foot print to be transformed
            vehicle_data : vehicle_data (from vehicle_common.vehicle_config import vehicle_data)
            to_polygon : True return  footprint as polygon msg else: numpy array
        Returns:
            center of circles to
            type: List or PolygonStamped
        """
    vehicle_dim = vehicle_data.dimensions
    print("vehicle_dim", vehicle_dim.rear_overhang)
    foot_print_specs = np.array([
        [-vehicle_dim.rear_overhang, -vehicle_dim.overall_width / 2],
        [vehicle_dim.wheel_base + vehicle_dim.front_overhang, -vehicle_dim.overall_width / 2],
        [vehicle_dim.wheel_base + vehicle_dim.front_overhang, vehicle_dim.overall_width / 2],
        [-vehicle_dim.rear_overhang, vehicle_dim.overall_width / 2],
    ])
    theta = get_yaw(pose.orientation)
    cos_th, sin_th = np.cos(theta), np.sin(theta)
    arr = []
    for foot in foot_print_specs:
        x = pose.position.x + (foot[0] * cos_th - foot[1] * sin_th)
        y = pose.position.y + (foot[0] * sin_th + foot[1] * cos_th)
        arr.append([x, y])
    if to_polygon:
        polygon_st = PolygonStamped()
        polygon_st.header.frame_id = 'map'
        for coors in arr:
            point = Point()
            point.x = coors[0]
            point.y = coors[1]
            polygon_st.polygon.points.append(point)
        return polygon_st

    else:
        return arr

"""
def plot_robot(x, y, yaw, config):  # pragma: no cover
        outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                             (config.robot_length / 2), -config.robot_length / 2,
                             -config.robot_length / 2],
                            [config.robot_width / 2, config.robot_width / 2,
                             - config.robot_width / 2, -config.robot_width / 2,
                             config.robot_width / 2]])
        print(outline)
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), "-k")
"""


def transform_footprint_circles(pose, vehicle_data, width_offset=0.0, length_offset=0.0, to_polygon=False):
    """
    Args:
        pose : Pose to which vehicle foot print to be transformed
        vehicle_data : vehicle_data (from vehicle_common.vehicle_config import vehicle_data)
        to_polygon : True return  footprint as polygon msg else: numpy array
        width_offset : offset to add to width of vehicle
        length_offset : offset to add to lenght of the vehicle.
    Returns:
        center_centers, polygon_msg
    """
    vehicle_length = vehicle_data.dimensions.overall_length + length_offset
    vehicle_width = vehicle_data.dimensions.overall_width + width_offset
    number_of_points = vehicle_length // vehicle_width
    foot_print_forward_lim = vehicle_length - vehicle_data.dimensions.rear_overhang
    ends = [-vehicle_data.dimensions.rear_overhang, foot_print_forward_lim]
    # print("vehicle_data.dimensions.overall_width/2", vehicle_data.dimensions.overall_width / 2)
    circle_offsets = np.arange(-vehicle_data.dimensions.rear_overhang, foot_print_forward_lim,
                               vehicle_width / 2)[1:]
    # print(circle_offsets)

    yaw = get_yaw(pose.orientation)
    cicles_centers = []
    for offset in circle_offsets:
        x_value = pose.position.x + offset * np.cos(yaw)  # + radius * np.cos(theta)
        y_value = pose.position.y + offset * np.sin(yaw)  # + radius * np.sin(theta)
        cicles_centers.append([x_value, y_value])
    # print("cicles_centers", cicles_centers)
    if to_polygon:
        polygon_st = PolygonStamped()
        polygon_st.header.frame_id = 'map'
        theta = np.linspace(0, 2 * np.pi, 100)
        radius = vehicle_width / 2
        for center in cicles_centers:
            x_val, y_val = center[0], center[1]
            circum_points_x = x_val + radius * np.cos(theta)
            circum_points_y = y_val + radius * np.sin(theta)
            for x, y in zip(circum_points_x, circum_points_y):
                point = Point()
                point.x = x
                point.y = y
                polygon_st.polygon.points.append(point)
        return cicles_centers, polygon_st
    else:
        return cicles_centers, None



def path_to_circles(path, vehicle_data, to_polygon=False):
    """
    A function to return circle centers based on vehicle foot print and path.
    Arg:
        path : path
        vehicle_data : vehicle_data (from vehicle_common.vehicle_config import vehicle_data)
         to_polygon : True return  footprint as polygon msg else: list
    Returns:
         list of circle centers along the path with footprint cover
         (list/ PolygonArray)
    """
    if to_polygon:
        polygon_arr = PolygonArray()
        polygon_arr.header.frame_id = "map"
        for pose_st in path.poses:
            pose = pose_st.pose
            circle_polygon = transform_footprint_circles(pose, vehicle_data, to_polygon)

            polygon_arr.polygons.append(circle_polygon)
        return polygon_arr
    else:
        circle_list = []
        for pose_st in path.poses:
            pose = pose_st.pose
            circles = transform_footprint_circles(pose, vehicle_data, to_polygon)
            circle_list.extend(circles)
        return circle_list





def path_callback(msg):
    # polygon_arr = PolygonArray()
    # polygon_arr.header.frame_id = "map"
    # for pose in msg.poses:
    #     pose = pose.pose
    #     polygon = transform_footprint(pose, vehicle_data, to_polygon=True)
    #     circle_polygon = transform_footprint_circles(pose, vehicle_data, to_polygon=True)
    #     polygon_arr.polygons.append(circle_polygon)
    # pub_circles_all.publish(polygon_arr)

    arr_poly = path_to_circles(msg, vehicle_data, to_polygon=True)
    pub_circles_all.publish(arr_poly)


if __name__ == "__main__":
    from vehicle_common.vehicle_config import vehicle_data
    import rospy
    from pose_helper import yaw_to_quaternion
    import math

    rospy.init_node("footprint_node")
    pub = rospy.Publisher("/foot_print_tranform", PolygonStamped, queue_size=1, latch=True)
    pub_circles = rospy.Publisher("/circle_print_tranform", PolygonStamped, queue_size=1, latch=True)
    pub_circles_all = rospy.Publisher("/footprint_and_cicles", PolygonArray, queue_size=1, latch=True)

    rospy.Subscriber("global_gps_path", Path, path_callback)


    print("vehicle_dim", vehicle_data)
    pose = Pose()
    pose.position.x = 0
    pose.position.y = 0
    yaw = math.radians(90)
    pose.orientation = yaw_to_quaternion(yaw)
    polygon = transform_footprint(pose, vehicle_data, to_polygon=True)
    number_of_points = vehicle_data.dimensions.overall_length+1//(vehicle_data.dimensions.overall_width/2)
    foot_print_forward_lim = vehicle_data.dimensions.overall_length+1  - vehicle_data.dimensions.rear_overhang
    ends = [-vehicle_data.dimensions.rear_overhang, foot_print_forward_lim]
    print("vehicle_data.dimensions.overall_width/2", vehicle_data.dimensions.overall_width/2)
    circle_offsets = np.arange(-vehicle_data.dimensions.rear_overhang, foot_print_forward_lim, vehicle_data.dimensions.overall_width/2+0.3)[1:]

    print(circle_offsets)

    theta = np.linspace(0, 2 * np.pi, 100)
    radius = vehicle_data.dimensions.overall_width/2 + 0.3
    # Generating x and y data
    # x = radius * np.cos(theta)
    # y = radius * np.sin(theta)
    polygon_st = PolygonStamped()
    polygon_st.header.frame_id = 'map'
    for offset in circle_offsets:
        x_values = pose.position.x + offset * np.cos(yaw) + radius * np.cos(theta)
        y_values = pose.position.y + offset * np.sin(yaw) + radius * np.sin(theta)
        for x, y in zip(x_values, y_values):
            point = Point()
            point.x = x
            point.y = y
            polygon_st.polygon.points.append(point)

    pub_circles.publish(polygon_st)
    rospy.loginfo("cicles are published")
    print(polygon)
    pub.publish(polygon)
    rospy.loginfo("footprint_published")
    rospy.spin()
