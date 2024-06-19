#ifndef POSE_UTILS_H
#define POSE_UTILS_H

// Import necessary libraries
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>

// Import ROS libraries
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>

// Import tf2 libraries
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
using namespace std;

namespace autopilot_utils {
geometry_msgs::Point pol2cart(float rho, float phi);
double distanceBetweenPoses(const geometry_msgs::Pose &p1,const geometry_msgs::Pose &p2);
double distanceBetweenPoses(const std::vector<double> &p1,const std::vector<double> &p2);
geometry_msgs::Quaternion get_quaternion_from_yaw(double yaw);
double normalizeAngle(double angle);
geometry_msgs::Quaternion yaw_to_quaternion(float yaw);
Eigen::Vector3d unit_vect(double angle);
double angle_btw_poses(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);
double calcDistSquared2D(const geometry_msgs::Point &,
                         const geometry_msgs::Point &);
} // namespace autopilot_utils

#endif