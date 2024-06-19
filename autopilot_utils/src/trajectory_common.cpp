#include "autopilot_utils/trajectory_common.h"

namespace autopilot_utils{

TrajectoryHelper::TrajectoryHelper() {}

int TrajectoryHelper::getLength() { return traj.points.size(); }

autopilot_msgs::TrajectoryPoint
TrajectoryHelper::get_trajectory_point_by_index(int index) {
  /**
   * Returns the trajectory point at the given index.
   * Parameters:
   *     ind (int): The index of the trajectory point.
   * Returns:
   *     traj_point (autopilot_msgs/TrajectoryPoint): The trajectory point at
   * the given index. Raises: std::out_of_range: If the index is invalid or out
   * of bounds.
   */

  if (index >= 0 && index < traj.points.size())
    return traj.points[index];
  else {
    cerr << "Index out of range" << endl;
    return autopilot_msgs::TrajectoryPoint();
  }
}

double TrajectoryHelper::getCircumRadius(int ind, int index_lim = 10) {
  if ((ind - index_lim > 0) and (ind + index_lim < traj.points.size())) {
    vector<double> p1 = {traj.points[ind - index_lim].pose.position.x,
                         traj.points[ind - index_lim].pose.position.y};
    vector<double> p2 = {traj.points[ind].pose.position.x,
                         traj.points[ind].pose.position.y};
    vector<double> p3 = {traj.points[ind + index_lim].pose.position.x,
                         traj.points[ind + index_lim].pose.position.y};

    // return auto_nav::circumRadius(p1, p2, p3);
    //     den = 2 * ((x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2))
    double den = 2 * ((p2[0] - p1[0]) * (p3[1] - p2[1]) -
                      (p2[1] - p1[1]) * (p3[0] - p2[0]));
    if (den == 0) {
      cerr << "Failed: points are either collinear or not distinct" << endl;
      return 1000;
    }
    // float num = ((((x2 - x1) ** 2) + ((y2 - y1) ** 2)) * (((x3 - x2) ** 2) +
    // ((y3 - y2) ** 2)) * (((x1 - x3) ** 2) + ((y1 - y3) ** 2))) ** (0.5)
    double num = sqrt((pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2)) *
                      (pow(p3[0] - p2[0], 2) + pow(p3[1] - p2[1], 2)) *
                      (pow(p1[0] - p3[0], 2) + pow(p1[1] - p3[1], 2)));

    double cirum_radius = num / den;
    return cirum_radius;
  }

  else {
    cerr << "Index out of bounds" << endl;
    return 0.0;
  }
}

void TrajectoryHelper::setTrajectory(const autopilot_msgs::Trajectory &trajectory) {
  traj = trajectory;
}

std::pair<bool, int32_t> TrajectoryHelper::find_closest_idx_with_dist_ang_thr(
    const geometry_msgs::Pose& robot_pose, double distance_th, double angle_th) {
  // Initialize variables to store the minimum squared distance and the
  // corresponding index
  double dist_squared_min = std::numeric_limits<double>::max();
  int32_t idx_min = -1;

  // Calculate the squared distance threshold
  double square_dist = distance_th * distance_th;

  // Iterate through each point in the trajectory
  for (int32_t i = 0; i < static_cast<int32_t>(traj.points.size()); ++i) {
    // Calculate the squared distance between the robot pose and the current
    // trajectory point
    const double ds =
        calcDistSquared2D(traj.points.at(i).pose.position, robot_pose.position);

    // Check if the squared distance exceeds the distance threshold
    if (ds > square_dist)
      continue; // Skip this point and move to the next one

    // Calculate the yaw angles of the robot pose and the trajectory point
    double yaw_pose = tf::getYaw(robot_pose.orientation);
    double yaw_ps = tf::getYaw(traj.points.at(i).pose.orientation);

    // Calculate the difference in yaw angles, normalizing the result
    double yaw_diff = normalizeAngle(yaw_pose - yaw_ps);

    // Check if the absolute value of the yaw difference exceeds the angle
    // threshold
    if (std::fabs(yaw_diff) > angle_th)
      continue; // Skip this point and move to the next one

    // Update the minimum squared distance and corresponding index if the
    // current point is closer
    if (ds < dist_squared_min) {
      dist_squared_min = ds;
      idx_min = i;
    }
  }

  // Return a pair indicating whether a valid index was found and the index
  // itself
  return (idx_min >= 0) ? std::make_pair(true, idx_min)
                        : std::make_pair(false, idx_min);
}

int TrajectoryHelper::find_close_pose_after_index(const geometry_msgs::Pose& curr_pose,
                                                  int prev_idx,
                                                  double search_distance) {
  /**
   * Returns the closest index from the previous index.
   * Parameters:
   *     curr_pose (geometry_msgs/Pose): The current pose.
   *     prev_idx (int): The index to start the search from.
   *     search_distance (double): The distance to search from the previous
   * index. Returns: close_index (int): The index of the closest pose to the
   * current pose from the previous index.
   */
  double dist_min = std::numeric_limits<double>::max();
  int idx_min = prev_idx;
  if (prev_idx >= traj.points.size()) {
    return traj.points.size() - 1;
  }
  for (int i = prev_idx; i < traj.points.size(); i++) {
    if (abs(traj.points[prev_idx].accumulated_distance_m -
            traj.points[i].accumulated_distance_m) > search_distance)
      break;

    double dist =
        sqrt(pow(curr_pose.position.x - traj.points[i].pose.position.x, 2) +
             pow(curr_pose.position.y - traj.points[i].pose.position.y, 2));
    if (dist < dist_min) {
      dist_min = dist;
      idx_min = i;
    }
  }
  return idx_min;
}

int TrajectoryHelper::next_point_within_dist(int idx, double dist_thr) {
  /**
   * Finds the next index which is dist_thr far from the given index.
   * Parameters:
   *     idx (int): The index to start the search from.
   *     dist_thr (double): The distance threshold.
   * Returns:
   *     next_idx (int): The index of the next point that is dist_thr far from
   * the given index. Raises: std::out_of_range: If the index is invalid or out
   * of bounds.
   */
  // Check if trajectory has points and if the provided index is within bounds
  if (traj.points.size() > 0 && idx < traj.points.size()) {
      // Get the accumulated distance of the closest point to the target distance
      double close_dis = traj.points[idx].accumulated_distance_m;
      
      // Check if the difference between the accumulated distance of the closest point
      // and the accumulated distance of the last point is within the threshold
      if (std::abs(traj.points[idx].accumulated_distance_m -
              traj.points.back().accumulated_distance_m) <= dist_thr)
        // Return the index of the last point if within threshold
        return traj.points.size() - 1;
      else {
        // Iterate through the trajectory points starting from the next point after the provided index
        for (int i = idx + 1; i < traj.points.size(); i++) {
          // Calculate the difference between the accumulated distance of the current point and the closest point
          double path_acc_distance =
              std::abs(traj.points[i].accumulated_distance_m - close_dis);
          // If the difference is greater than the threshold, return the index of the current point
          if (path_acc_distance > dist_thr)
            return i;
        }
      }
    }else
      // If trajectory has no points or provided index is out of bounds, return -1
      return -1;
}

nav_msgs::Path TrajectoryHelper::to_path() {
  /**
   * Returns the path of the trajectory.
   * Returns:
   *     path_msg (nav_msgs/Path): The path of the trajectory.
   */
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = traj.header.frame_id;
  for (int i = 0; i < traj_len; i++) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = traj.header.frame_id;
    pose_stamped.pose = traj.points[i].pose;
    path_msg.poses.push_back(pose_stamped);
  }
  return path_msg;
}

} // End of namespace autopilot_utils
