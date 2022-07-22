from geometry_msgs.msg import Point, PoseArray, Pose
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler


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
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
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


def get_poses_slope(pose1, pose2):
    """
        returns slope of pose1, and pose2
            Parameters:
                pose1(geometry_msgs/Pose.msg): pose one.
                pose2(geometry_msgs/Pose.msg): pose two.
            Returns:
                slope(float): slope of two poses.
        """
    delta_x = pose1.position.x - pose2.position.x
    delta_y = pose1.position.y - pose2.position.y
    slope = math.atan2(delta_y, delta_x)
    return slope
