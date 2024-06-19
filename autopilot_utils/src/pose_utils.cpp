#include "autopilot_utils/pose_utils.h"

namespace autopilot_utils {

geometry_msgs::Point pol2cart(float rho, float phi) {
  /**
   * Converts a Polar coordinate point to Cartesian coordinate point.
   * Parameters:
   *      rho(float): distance to the point in Polar coordinate system
   *      phi(float): angle to the polar point
   * Returns:
   *      geometry_msgs::Point: Cartesian coordinate point
   */

  // Create a point message
  geometry_msgs::Point point;

  // Set the x and y coordinates of the point
  point.x = rho * cos(phi);
  point.y = rho * sin(phi);

  // Return the point message
  return point;
}

double distanceBetweenPoses(const geometry_msgs::Pose &pose1,
                            const geometry_msgs::Pose &pose2) {
  /**
   * Calculates the distance between two poses.
   * Parameters:
   *      pose1 (geometry_msgs::Pose): The first pose.
   *      pose2 (geometry_msgs::Pose): The second pose.
   * Returns:
   *      double: The distance between the poses.
   */
  // Calculate the distance between the two poses
  double distance = hypot(pose1.position.x - pose2.position.x,
                          pose1.position.y - pose2.position.y);

  // Return the distance
  return distance;
}

// Calculates distance between poses
double distanceBetweenPoses(const std::vector<double> &p1,
                            const std::vector<double> &p2) {
  // Check if both vectors have at least two elements
  if (p1.size() < 2 || p2.size() < 2) {
    // Handle error condition: insufficient dimensions
    throw std::invalid_argument("Vectors must have at least two elements.");
  }

  // Calculate the distance between the two poses
  return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2));
}

// Converts yaw to quaternion
geometry_msgs::Quaternion get_quaternion_from_yaw(double yaw) {
  // Create a quaternion message
  tf2::Quaternion quaternion;

  // Set the yaw angle of the quaternion
  quaternion.setRPY(0, 0, yaw);

  // Convert the quaternion to a message
  geometry_msgs::Quaternion quaternion_msg;
  quaternion_msg.x = quaternion.x();
  quaternion_msg.y = quaternion.y();
  quaternion_msg.z = quaternion.z();
  quaternion_msg.w = quaternion.w();

  // Return the quaternion message
  return quaternion_msg;
}

geometry_msgs::Quaternion yaw_to_quaternion(double yaw) {
  /**
   * Returns quaternions of euler yaw.
   * Parameters:
   *      yaw (double): Yaw in radians.
   * Returns:
   *      geometry_msgs::Quaternion: Quaternion representation of yaw.
   */

  // Create a quaternion message
  tf::Quaternion quat;

  // Set the yaw angle of the quaternion
  quat.setRPY(0, 0, yaw);

  // Convert the quaternion to a message
  geometry_msgs::Quaternion quaternion;
  quaternion.x = quat.x();
  quaternion.y = quat.y();
  quaternion.z = quat.z();
  quaternion.w = quat.w();

  // Return the quaternion message
  return quaternion;
}


// Calculates squared distance between two 2D points
double calcDistSquared2D(const geometry_msgs::Point &p,
                         const geometry_msgs::Point &q) {
  // Calculate the distance between the two points
  const double dx = p.x - q.x;
  const double dy = p.y - q.y;
  return (dx * dx + dy * dy);
}

double angle_btw_poses(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
  /**
   * Returns the angle between pose1 and pose2.
   * Parameters:
   *      pose1 (const geometry_msgs::Pose&): The first pose.
   *      pose2 (const geometry_msgs::Pose&): The second pose.
   * Returns:
   *      double: Angle in radians.
   */
  double delta_x = pose1.position.x - pose2.position.x;
  double delta_y = pose1.position.y - pose2.position.y;
  double angle = atan2(delta_y, delta_x);
  return normalizeAngle(angle);
}


double normalizeAngle(double euler) {
  /**
   * Normalize an angle to [-pi, pi].
   * Parameters:
   *      angle(double): angle in radians
   * Returns:
   *      double: angle in radians in [-pi, pi]
   */

  double res = euler;
  while (res > M_PI) {
    res -= (2.0 * M_PI);
  }
  while (res < -M_PI) {
    res += 2.0 * M_PI;
  }

  return res;
}
  

Eigen::Vector3d unit_vect(double angle) {
  Eigen::Vector3d v{cos(angle), sin(angle), 0};
  return v;
}
} // End of namespace autopilot_utils
