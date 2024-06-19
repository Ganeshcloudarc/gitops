#ifndef TF_UTILS_H
#define TF_UTILS_H

// Import ROS libraries
#include <list>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tuple>

// Import geometry message types
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

// Import sensor message types
#include <sensor_msgs/PointCloud2.h>

// Import custom message types
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <zed_interfaces/ObjectsStamped.h>

using namespace std;

namespace autopilot_utils {
vector<double> quaternion_to_list(geometry_msgs::Quaternion quaternion);
geometry_msgs::Quaternion list_to_quaternion(const vector<double> &l);
geometry_msgs::Pose convert_pose(const geometry_msgs::Pose &pose,
                                 const std::string &from_frame,
                                 const std::string &to_frame);
geometry_msgs::Point convert_point(geometry_msgs::Point point,
                                   string from_frame, string to_frame);
geometry_msgs::Pose current_robot_pose(std::string reference_frame = "map",
                                       std::string base_frame = "base_link");
geometry_msgs::Pose
align_pose_orientation_to_frame(geometry_msgs::Pose from_pose,
                                std::string from_reference_frame,
                                std::string to_reference_frame);
double get_yaw(geometry_msgs::Quaternion orientation);
sensor_msgs::PointCloud2 transformCloud(const sensor_msgs::PointCloud2 &pc2_in,
                                        const std::string &target_frame);
geometry_msgs::Point
convert_point_by_transform(const geometry_msgs::Point &point,
                           const geometry_msgs::TransformStamped &trans);
geometry_msgs::Point
convert_point_by_transform(const std::vector<double> &point,
                           const geometry_msgs::TransformStamped &trans);
geometry_msgs::Pose
convert_pose_by_transform(const geometry_msgs::Pose &pose,
                          const geometry_msgs::Transform &trans);
zed_interfaces::ObjectsStamped
transform_zed_objects(zed_interfaces::ObjectsStamped object_data,
                      string to_frame);
jsk_recognition_msgs::BoundingBoxArray
transform_lidar_objects(jsk_recognition_msgs::BoundingBoxArray bbox_arr_data,
                        std::string to_frame);
vector<vector<double>> bbox_to_corners(jsk_recognition_msgs::BoundingBox bbox);
geometry_msgs::PoseArray convert_path(const geometry_msgs::PoseArray &path,
                                      const std::string &to_frame);
} // namespace autopilot_utils

#endif