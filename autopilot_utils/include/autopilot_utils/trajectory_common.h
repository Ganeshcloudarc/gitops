// trajectory_common.h
#ifndef TRAJECTORY_HELPER_H
#define TRAJECTORY_HELPER_H

#include <cmath>
#include <iostream>
#include <vector>

#include "autopilot_msgs/Trajectory.h"
#include "autopilot_msgs/TrajectoryPoint.h"
#include "autopilot_utils/pose_utils.h"
#include "autopilot_utils/tf_utils.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

using namespace std;
namespace autopilot_utils {

class TrajectoryHelper {

public:
  TrajectoryHelper();
  void setTrajectory(const autopilot_msgs::Trajectory &trajectory);

  int getLength();

  int find_close_pose_after_index(const geometry_msgs::Pose& curr_pose, int prev_idx,
                                  double search_distance);

  int next_point_within_dist(int idx, double dist_thr);

  double getCircumRadius(int, int);

  autopilot_msgs::TrajectoryPoint get_trajectory_point_by_index(int idx);

  pair<bool, int32_t>
  find_closest_idx_with_dist_ang_thr(const geometry_msgs::Pose& robot_pose,
                                     double distance_th, double angle_th);

  nav_msgs::Path to_path();

private:
  autopilot_msgs::Trajectory traj;
  autopilot_msgs::Trajectory traj_in;
  int traj_len;
};

} // namespace autopilot_utils
#endif
