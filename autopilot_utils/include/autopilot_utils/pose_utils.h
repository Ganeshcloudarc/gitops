#ifndef POSE_UTILS_H
#define POSE_UTILS_H

#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include<vector>
#include <Eigen/Dense>

namespace autopilot_utils
{
    double distanceBetweenPoses(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
    double distanceBetweenPoses(std::vector<double> , std::vector<double>);
    geometry_msgs::Quaternion get_quaternion_from_yaw(double yaw);
    double normalizeAngle(double angle);
    Eigen::Vector3d unit_vect(double angle);
    double calcDistSquared2D(const geometry_msgs::Point& , const geometry_msgs::Point&);
}

#endif  