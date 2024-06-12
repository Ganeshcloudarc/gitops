from geometry_msgs.msg import Point, PoseArray, Pose, Quaternion
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

def pol2cart(rho, phi):
    """
    Converts a Polar coordinate point to Cartesian coordinate point.
        Parameters:
            rho(float): distance to the point in Polar coordinate system
            phi(float: angle to the polar point
        Returns:
                x(float): x coordinate in Cartesian system
                y(float): y coordinate in Cartesian system
    """
    x = rho * math.cos(phi)
    y = rho * math.sin(phi)
    return x, y


def distance_btw_poses(pose1, pose2):
    """
    Calculates distance between poses.
        Parameters:
            pose1(geometry_msgs/Pose.msg): pose one.
            pose2(geometry_msgs/Pose.msg): pose two.
        Returns:
            distance(float): distance between the poses
    """
    distance = math.hypot(pose1.position.x - pose2.position.x, pose1.position.y - pose2.position.y)
    return distance


def get_yaw(orientation):
    """
    calculates Yaw from quaternion(rotation wrt Z axis).
        Parameters:
            orientation(geometry_msgs/Quaternion.msg): Orientation in quaternion
        Returns:
            yaw(float): Yaw in euler.
    """
    _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    return yaw


def yaw_to_quaternion(yaw):
    """
    returns queternons of euler yaw
        Pamameters:
            yaw(float): yaw in radians
        Returns:
            quat(geometry_msgs/quaternion) 
    """
    # quat = quaternion_from_euler((0,0, ))
    quat = quaternion_from_euler(0, 0, yaw)
    quat = Quaternion(quat[0], quat[1], quat[2], quat[3])
    return quat

def rotate_yaw_180(yaw):
    # Ensure yaw is between 0 and 2*pi
    yaw = yaw % (2 * math.pi)
    
    # Calculate the new yaw angle by adding pi radians (180 degrees)
    new_yaw = yaw + math.pi
    
    # Ensure the new_yaw is between 0 and 2*pi
    new_yaw = new_yaw % (2 * math.pi)
    
    return new_yaw


def angle_btw_poses(pose1, pose2):
    """
        returns slope of pose1, and pose2
            Parameters:
                pose1(geometry_msgs/Pose.msg): pose one,
                pose2(geometry_msgs/Pose.msg): pose two.
            Returns:
                angle(float): in radians.
        """
    delta_x = pose1.position.x - pose2.position.x
    delta_y = pose1.position.y - pose2.position.y
    angle = math.atan2(delta_y, delta_x)
    return normalize_angle(angle)


def normalize_angle(angle):
    """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def rectangle_form_pose(robot_pose, x_size, y_size):
    trans_list = [
        [-1, -1],
        [-1, 1],
        [1, 1],
        [1, -1]
    ]
    corners_list = []
    yaw_ = get_yaw(robot_pose.pose.orientation)
    c, s = np.cos(yaw_), np.sin(yaw_)
    R = np.array(((c, -s), (s, c)))
    for x_, y_ in trans_list:
        x = robot_pose.pose.position.x + x_ * x_size/2
        y = robot_pose.pose.position.y + y_ * y_size/2
        result = np.dot(R, np.array([x, y]))
        corners_list.append(result)
    return corners_list

def rectangle_form_pose(robo_x,robo_y, x_size, y_size):
    trans_list = [
        [-1, -1],
        [-1, 1],
        [1, 1],
        [1, -1]
    ]
    corners_list = []
#     yaw_ = get_yaw(robot_pose.pose.orientation)
    yaw_ = math.radians(145)
    c, s = np.cos(yaw_), np.sin(yaw_)
    R = np.array(((c, -s), (s, c)))
    for x_, y_ in trans_list:
        x = robo_x + x_ * x_size/2
        y = robo_y + y_ * y_size/2
        result = np.dot(R, np.array([x, y]))
        corners_list.append(result)
    return corners_list



def calc_cte(pose, position):
    """ calulcate perp distance between pose(line ) and positio"""
    yaw = get_yaw(pose.orientation)
    
    target_point = np.array([position[0], position[1]])
    # print(f"yaw: { yaw}")
    # print(" x: {} y: {} ", pose.position.x, pose.position.y)
    # print(" x: {} y: {} ", pose.position.x + math.cos(yaw), pose.position.y + math.sin(yaw))
    point1 = np.array([ pose.position.x, pose.position.y])
    point2 = np.array([pose.position.x + math.cos(yaw), pose.position.y + math.sin(yaw)])

    distance = np.cross(point2 - point1, point1 - target_point) / np.linalg.norm(
                    point2 - point1)
    return distance


if __name__ == "__main__":
    yaw = 3
    quat = yaw_to_quaternion(yaw)
    print(quat)
    yaw_updated = get_yaw(quat)
    print("yaw", yaw)
    print("yaw_updated", yaw_updated)


    # calc_ctc()
