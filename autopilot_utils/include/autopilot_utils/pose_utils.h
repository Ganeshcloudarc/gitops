#ifndef POSE_UTILS_H
#define POSE_UTILS_H

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

namespace autopilot_utils {
double distanceBetweenPoses(const geometry_msgs::Pose &p1,
                            const geometry_msgs::Pose &p2);
double distanceBetweenPoses(const std::vector<double> &,
                            const std::vector<double> &);
geometry_msgs::Quaternion get_quaternion_from_yaw(double yaw);
double normalizeAngle(double angle);
Eigen::Vector3d unit_vect(double angle);
double calcDistSquared2D(const geometry_msgs::Point &,
                         const geometry_msgs::Point &);
} // namespace autopilot_utils

#endif