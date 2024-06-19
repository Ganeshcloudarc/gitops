#ifndef OBSTACLE_STOP_PLANNER_H
#define OBSTACLE_STOP_PLANNER_H

// Basic Imports
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

// Ros Related Imports
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>

// Navigation Stack
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// JSK Recognition Messages
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

// ZED Camera Interfaces
#include <zed_interfaces/Object.h>
#include <zed_interfaces/ObjectsStamped.h>

// Visualization Messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Sensor Messages
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

// Geometry Messages
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

// Laser Geometry
#include <laser_geometry/laser_geometry.h>

// Point Cloud Library (PCL)
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// Transform Listener
#include <tf/transform_listener.h>

// Autopilot Related Imports

#include "autopilot_utils/pose_utils.h"
#include "autopilot_utils/tf_utils.h"
#include <autopilot_msgs/Trajectory.h>
#include <autopilot_utils/footprint_transform.h>
#include <autopilot_utils/trajectory_common.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <vehicle_common/config_vehicle.h>

using namespace std;
using namespace autopilot_utils;

namespace obstacle_stop_planner {

class ObstacleStopPlanner {
private:
  ros::NodeHandle *nh;
  // Publisher Initialization
  ros::Publisher laser_np_2d_pub;
  ros::Publisher bbox_pub;
  ros::Publisher publisher;
  ros::Publisher slow_pose_pub;
  ros::Publisher box_corner_pub;
  ros::Publisher slow_start_pub;
  ros::Publisher close_pose_pub;
  ros::Publisher front_pose_pub;
  ros::Publisher mission_count_pub;
  ros::Publisher circles_polygon_pub;
  ros::Publisher local_traj_publisher;
  ros::Publisher path_percent_publisher;
  ros::Publisher collision_points_polygon;
  ros::Publisher velocity_marker_publisher;
  ros::Publisher collision_points_publisher;
  ros::Publisher transformed_zed_objects_publisher;

  // Subscriber Initialization
  ros::Subscriber scan_subscriber;
  ros::Subscriber odom_subscriber;
  ros::Subscriber bbox_topic_subscriber;
  ros::Subscriber global_traj_subscriber;
  ros::Subscriber zed_objects_topic_subscriber;

  // Message Data-type Initialization
  geometry_msgs::Pose robot_pose;
  autopilot_msgs::Trajectory _traj_in;
  geometry_msgs::Pose path_pose;
  geometry_msgs::Pose robot_head_pose;
  geometry_msgs::PolygonStamped circles_polygon;
  autopilot_msgs::Trajectory trajectory_msg;
  autopilot_msgs::TrajectoryPoint traj_point;
  jsk_recognition_msgs::BoundingBoxArray bboxes;
  zed_interfaces::ObjectsStamped::ConstPtr zed_objects;

  // Diagnostics Initialization
  int OK = diagnostic_msgs::DiagnosticStatus::OK;
  int WARN = diagnostic_msgs::DiagnosticStatus::WARN;
  int ERROR = diagnostic_msgs::DiagnosticStatus::ERROR;
  int STALE = diagnostic_msgs::DiagnosticStatus::STALE;
  diagnostic_updater::DiagnosticStatusWrapper diagnostics_publisher;

  // String Initialization
  string scan_topic;
  string odom_topic;
  string bbox_topic;
  string _robot_base_frame;
  string local_traj_in_topic;
  string global_traj_topic;
  string zed_objects_topic;
  string reason_sensor_msg;
  string obs_found_sensor;
  string obstacle_found_sensor_main_logic;

  // Integer Initialization
  int sigma;
  int end_idx;
  int close_bbx_id;
  int kernal_size;
  int front_tip_idx;
  int objects_number;
  int traj_end_index;
  int collision_index;
  int _traj_end_index;
  int _close_idx = -1;
  std_msgs::Int32 count_mission_repeat;
  int _TIME_OUT_FROM_ODOM = 2;
  int _TIME_OUT_FROM_ZED = 2;
  int _TIME_OUT_FROM_LASER = 2; // In Seconds
  int time_to_wait_at_ends;
  int stop_threshold_time_for_obs;

  // Float Initialization
  float slow_line_buffer;
  float _stop_line_buffer;
  float robot_min_speed_th;
  float robot_max_speed_th;
  float _min_look_ahead_dis;
  float _max_look_ahead_dis;
  float _trajectory_resolution;
  float _lookup_collision_distance;
  float radial_off_set_to_vehicle_width;

  // Double Initialization
  double close_dis;
  double robot_speed;
  double path_percent;
  double by_pass_dist;
  double width_offset;
  double length_offset;
  double _base_to_front;
  double zed_data_in_time;
  double odom_data_in_time;
  double laser_data_in_time;
  double close_obj_from_zed;
  double prev_by_pass_dist;
  double path_acc_distance;
  double _radius_to_search;
  double radius_to_search_near_robot;
  double loop_start_time;
  double vehicle_stop_init_time_for_obs = -1;

  // Boolean Initialization
  bool use_obs_v1;
  bool to_polygon;
  bool use_pcl_boxes;
  bool obstacle_found;
  bool mission_continue;
  bool use_zed_detections;
  bool scan_data_received;
  bool _vis_trajectory_rviz;
  bool _vis_collision_points;
  bool obstacle_encountered;
  bool obs_found_robot_collision;
  bool velocity_speed_profile_enable;
  bool robot_level_collision_check_enable;

  // Class Initialization
  laser_geometry::LaserProjection laser_geo_obj;
  autopilot_utils::TrajectoryHelper traj_helper_obj;
  sensor_msgs::PointCloud2 laser_np_2d;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  vector<vector<double>> circles_array;
  std::vector<int> collision_points;
  std::vector<float> distances;
  std::vector<int> new_collision_points;

  // class Initialization
  DataVehicle data_vehicle_obj;
  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener;
  pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud;

public:
  ObstacleStopPlanner(ros::NodeHandle *nh1)
      : t_cloud(new pcl::PointCloud<pcl::PointXYZ>), tf2_buffer(),
        tf2_listener(tf2_buffer), odom_data_in_time(ros::Time::now().toSec()) {
    nh = nh1;

    // Parameters
    ObstacleStopPlanner::loadParams();

    publisher = nh->advertise<diagnostic_msgs::DiagnosticArray>(
        "/obstacle_stop_planner_diagnostics", 1, true);
    diagnostics_publisher.name = ros::this_node::getName();
    diagnostics_publisher.hardware_id = "zekrom_v1";
    diagnostics_publisher.values.clear();
    laser_np_2d.header.frame_id = "map";

    // Subscribers
    // ROS_ERROR("The SCAN TOPIC %s",scan_topic.c_str());
    // ROS_ERROR("The ODOM TOPIC %s",odom_topic.c_str());
    // ROS_ERROR("The GLOBAL TRAJ TOPIC %s",global_traj_topic.c_str());
    scan_subscriber = nh->subscribe(scan_topic, 1,
                                    &ObstacleStopPlanner::scan_callback, this);
    odom_subscriber = nh->subscribe(odom_topic, 1,
                                    &ObstacleStopPlanner::odom_callback, this);
    global_traj_subscriber =
        nh->subscribe(global_traj_topic, 1,
                      &ObstacleStopPlanner::global_traj_callback, this);

    // Publishers
    laser_np_2d_pub =
        nh->advertise<sensor_msgs::PointCloud2>("laser_np_2d_topic", 1);
    mission_count_pub = nh->advertise<std_msgs::Int32>("mission_count", 2);
    path_percent_publisher =
        nh->advertise<std_msgs::Float32>("/osp_path_percentage", 1, true);
    local_traj_publisher =
        nh->advertise<autopilot_msgs::Trajectory>(local_traj_in_topic, 10);
    slow_start_pub = nh->advertise<geometry_msgs::PoseStamped>(
        "obstacle_stop_planner/slow_start", 1);
    slow_pose_pub = nh->advertise<geometry_msgs::PoseStamped>(
        "obstacle_stop_planner/slow_point", 1);
    close_pose_pub = nh->advertise<geometry_msgs::PoseStamped>(
        "obstacle_stop_planner/close_point", 1);
    front_pose_pub = nh->advertise<geometry_msgs::PoseStamped>(
        "obstacle_stop_planner/front_point", 1);
    collision_points_publisher = nh->advertise<sensor_msgs::PointCloud2>(
        "obstacle_stop_planner/collision_points", 10);

    circles_polygon_pub = nh->advertise<geometry_msgs::PolygonStamped>(
        "/obstacle_stop_planner/collision_checking_circles", 2, true);
    velocity_marker_publisher = nh->advertise<visualization_msgs::MarkerArray>(
        "obstacle_stop_planner/collision_velocity_marker", 10);

    _radius_to_search =
        data_vehicle_obj.overall_width / 2 + radial_off_set_to_vehicle_width;
    _base_to_front =
        data_vehicle_obj.wheel_base + data_vehicle_obj.front_overhang;

    if (use_pcl_boxes) {
      // Subscribe to PCL bounding box topic
      bbox_topic_subscriber = nh->subscribe(
          bbox_topic, 1, &ObstacleStopPlanner::pcl_bboxes_callback, this);
      bbox_pub =
          nh->advertise<jsk_recognition_msgs::BoundingBox>("collision_bbox", 1);
      collision_points_polygon = nh->advertise<geometry_msgs::PolygonStamped>(
          "collision_points_polygon", 1);
    }

    if (use_zed_detections) {
      ROS_INFO_ONCE("RUNNING ZED OBS !!!");
      zed_objects_topic_subscriber =
          nh->subscribe(zed_objects_topic, 1,
                        &ObstacleStopPlanner::zed_objects_callback, this);
      transformed_zed_objects_publisher =
          nh->advertise<zed_interfaces::ObjectsStamped>(
              "/obstacle_stop_planner/transformed_zed_obj", 1);
    }
    count_mission_repeat.data = 0;
    main_loop();
  }

  void loadParams() {

    ROS_INFO("Parameter Function Called!! ");
    nh->param("/gaussian_velocity_filter/sigma", sigma, 1);
    nh->param("/gaussian_velocity_filter/kernal_size", kernal_size, 11);
    nh->param("patrol/wait_time_on_mission_complete", time_to_wait_at_ends, 2);
    nh->param("/obstacle_stop_planner/stop_threshold_time_for_obs",
              stop_threshold_time_for_obs, 3);

    nh->param("/obstacle_stop_planner/by_pass_dist", by_pass_dist, 0.0);
    nh->param("obstacle_stop_planner/footprint/offset_to_width", width_offset,
              0.3);
    nh->param("obstacle_stop_planner/footprint/offset_to_forward_length",
              length_offset, 1.0);

    nh->param("patrol/min_forward_speed", robot_min_speed_th, 0.8f);
    nh->param("patrol/max_forward_speed", robot_max_speed_th, 1.0f);
    nh->param("/pure_pursuit/min_look_ahead_dis", _min_look_ahead_dis, 3.0f);
    nh->param("/pure_pursuit/max_look_ahead_dis", _max_look_ahead_dis, 6.0f);
    nh->param("obstacle_stop_planner/slow_line_buffer", slow_line_buffer, 2.0f);
    nh->param("obstacle_stop_planner/stop_line_buffer", _stop_line_buffer,
              3.0f);
    nh->param("obstacle_stop_planner/trajectory_resolution",
              _trajectory_resolution, 0.5f);
    nh->param("obstacle_stop_planner/lookup_collision_distance",
              _lookup_collision_distance, 20.0f);
    nh->param("obstacle_stop_planner/radial_off_set_to_vehicle_width",
              radial_off_set_to_vehicle_width, 0.5f);

    nh->param<std::string>("patrol/odom_topic", odom_topic, "vehicle/odom");
    nh->param<std::string>("robot_base_frame", _robot_base_frame, "base_link");
    nh->param<std::string>("obstacle_stop_planner/scan_in", scan_topic,
                           "laser_scan");
    nh->param<std::string>("/pcl_bbox_topic", bbox_topic,
                           "/obstacle_detector/jsk_bboxes");
    nh->param<std::string>("obstacle_stop_planner/traj_in", global_traj_topic,
                           "global_gps_trajectory");
    nh->param<std::string>("obstacle_stop_planner/traj_out",
                           local_traj_in_topic, "local_gps_trajectory");
    nh->param<std::string>("obstacle_stop_planner/zed_object_topic",
                           zed_objects_topic,
                           "/zed2i/zed_node/obj_det/objects");

    nh->param("use_pcl_boxes", use_pcl_boxes, true);
    nh->param("/patrol/enable_obs_v1", use_obs_v1, true);
    nh->param("/obstacle_stop_planner/mission_continue", mission_continue,
              true);
    nh->param("/obstacle_stop_planner/vis_trajectory_rviz",
              _vis_trajectory_rviz, true);
    nh->param("/obstacle_stop_planner/vis_collision_points",
              _vis_collision_points, true);
    nh->param("obstacle_stop_planner/use_zed_object_detection",
              use_zed_detections, false);
    nh->param("obstacle_stop_planner/velocity_speed_profile_enable",
              velocity_speed_profile_enable, true);
    nh->param("obstacle_stop_planner/robot_level_collision_check_enable",
              robot_level_collision_check_enable, true);
  }

  double interpolate(double x, double x0, double x1, double y0, double y1);
  void publish(const diagnostic_msgs::DiagnosticStatus &diagnostics_publisher);
  pair<bool, std::string> do_initial_sensor_check();
  void main_loop();

  void global_traj_callback(const autopilot_msgs::Trajectory::ConstPtr &data) {
    // Process global trajectory message
    _traj_in = *data; // Dereference the pointer to get the actual object
    traj_helper_obj.setTrajectory(
        *data); // Dereference the pointer when passing it to update
    _traj_end_index =
        traj_helper_obj.getLength(); // Get the length of the trajectory and
                                     // assign it to _traj_end_index
  }

  void scan_callback(const sensor_msgs::LaserScan::ConstPtr &data) {
    // Get the current time
    ros::Time start = ros::Time::now();

    // Get the laser data in time
    laser_data_in_time = ros::Time::now().toSec();

    try {
      laser_geo_obj.transformLaserScanToPointCloud("map", *data, laser_np_2d,
                                                   tf2_buffer);

    } catch (tf2::TransformException &ex) {
      // Handle exception
      ROS_ERROR("The error in scan_callback %s", ex.what());
    }
    scan_data_received = true;
    laser_np_2d_pub.publish(laser_np_2d);

    // Print the time taken for the laser scan callback
    ROS_DEBUG_STREAM("time taken for laser scan callback: "
                     << (ros::Time::now() - start).toSec());
  }

  void odom_callback(const nav_msgs::Odometry::ConstPtr &data) {
    // Update robot pose and speed
    robot_pose = data->pose.pose;
    robot_speed = sqrt(pow(data->twist.twist.linear.x, 2) +
                       pow(data->twist.twist.linear.y, 2));

    // Get current time in seconds
    // odom_data_in_time = ros::Time::now().toSec();

    // Use the timestamp from the odom message
    odom_data_in_time = data->header.stamp.toSec();
    ROS_INFO("Updated odom_data_in_time: %f", odom_data_in_time);

    // Calculate pose heading
    double pose_heading = get_yaw((data->pose.pose.orientation));

    // Calculate robot head pose
    robot_head_pose.position.x =
        data->pose.pose.position.x + _base_to_front * cos(pose_heading);
    robot_head_pose.position.y =
        data->pose.pose.position.y + _base_to_front * sin(pose_heading);
    robot_head_pose.position.z = data->pose.pose.position.z;
    robot_head_pose.orientation = data->pose.pose.orientation;
  }

  double find_close_object_zed(zed_interfaces::ObjectsStamped::ConstPtr objects,
                               vector<double> point);
  void pcl_bboxes_callback(jsk_recognition_msgs::BoundingBoxArray data) {
    if (data.header.frame_id.compare("map") == 0) {
      bboxes = data;
    } else {
      /*
          Transforms jsk_recognition_msgs/BoundingBoxArray objects data to the
         target frame.

          Args:
              bbox_arr_data: jsk_recognition_msgs/BoundingBoxArray object data
         to transform. to_frame: Frame to which you want to transform the data.

          Returns:
              - Transformed object data (jsk_recognition_msgs/BoundingBoxArray)
         in the target frame.
              - Corners list (list of geometry_msgs/Point).
      */
      try {
        // Check for an empty bounding box array
        if (data.boxes.empty()) {
          ROS_WARN("A blank bounding box array was received");
          return;
        }
        // Lookup the transform from "map" to "data.header.frame_id"
        geometry_msgs::TransformStamped trans = tf2_buffer.lookupTransform(
            "map", data.header.frame_id, ros::Time(0));

        // Transform each pose in the bounding box array
        for (int i = 0; i < data.boxes.size(); i++) {
          // Apply the transform to the pose
          tf2::doTransform(data.boxes[i].pose, data.boxes[i].pose, trans);

          // Update the frame ID of each bounding box
          data.boxes[i].header.frame_id = "map";
        }
        // Update the frame ID and timestamp of the bounding box array
        data.header.frame_id = "map";
        data.header.stamp = ros::Time::now();
        bboxes = data;

      } catch (tf2::TransformException &e) {
        // If the transform lookup fails, log an error and optionally propagate
        // the exception
        ROS_ERROR("Pcl_bboxes_callback Error, failed to get transform from %s "
                  "to %s: %s",
                  "map", data.header.frame_id.c_str(), e.what());
      }
    }
  }
  void
  zed_objects_callback(const zed_interfaces::ObjectsStamped::ConstPtr &data);
  pair<int, double>
  find_close_object(jsk_recognition_msgs::BoundingBoxArray bboxes,
                    const pair<double, double> &point);

  void publish_bbox(jsk_recognition_msgs::BoundingBox bbox) {
    // Create a PolygonStamped object and set the frame_id to "map"
    geometry_msgs::PolygonStamped polygon_obj;
    polygon_obj.header.frame_id = "map";

    // Iterate through each bounding box
    for (const auto &bbox : bboxes.boxes) {
      // Convert the bounding box to a list of corners
      auto box_list = bbox_to_corners(bbox);

      // Iterate through each corner point
      for (const auto &point : box_list) {
        // Create a Point32 object and set its x and y coordinates
        geometry_msgs::Point32 pt_obj;
        pt_obj.x = point[0];
        pt_obj.y = point[1];

        // Add the point to the polygon
        polygon_obj.polygon.points.push_back(pt_obj);
      }

      // Add the first point again to close the polygon
      geometry_msgs::Point32 pt_obj;
      pt_obj.x = box_list[0][0];
      pt_obj.y = box_list[0][1];
      polygon_obj.polygon.points.push_back(pt_obj);
    }

    // Publish the polygon object
    box_corner_pub.publish(polygon_obj);
  }

  std::tuple<int, float, bool> find_close_point(geometry_msgs::Pose robot_pose,
                                                int old_close_index) {
    bool heading_ok;
    float close_dis =
        distanceBetweenPoses(robot_pose, _traj_in.points[old_close_index].pose);

    try {
      // Iterate over trajectory points starting from old_close_index + 1
      for (int ind = old_close_index + 1; ind < traj_end_index; ind++) {
        float dis = distanceBetweenPoses(robot_pose, _traj_in.points[ind].pose);

        // Check if the distance to the current point is smaller than the
        // closest distance so far
        if (close_dis >= dis) {
          close_dis = dis;
        } else {
          // Calculate headings and check if they are within a certain threshold
          double robot_heading = get_yaw(robot_pose.orientation);
          double path_heading = get_yaw(_traj_in.points[ind].pose.orientation);
          double degrees = 90.0;
          double radians = degrees * (M_PI / 180.0);

          if (std::abs(std::abs(robot_heading) - std::abs(path_heading)) >
              radians) {
            ROS_INFO("Headings are %f apart",
                     std::abs(robot_heading - path_heading));
            heading_ok = false;
          } else {
            heading_ok = true;
          }

          // Return the index of the previous point, closest distance, and
          // heading check result
          return {ind - 1, close_dis, heading_ok};
        }
      }

    } catch (const std::exception &e) {
      // Handle the exception
      ROS_ERROR("Exception caught in find_close_point: %s", e.what());
    }

    // Return default values if an exception occurs or the loop completes
    return {traj_end_index, 0, true};
  }

  tuple<int, double, bool> calc_nearest_ind(geometry_msgs::Pose robot_pose) {
    bool heading_ok;
    vector<double> distance_list;

    // Iterate through all the points in traj_in.
    for (size_t ind = 0; ind < _traj_in.points.size(); ++ind) {
      // Calculate the distance between the robot_pose and the current point.
      double distance =
          distanceBetweenPoses(robot_pose, _traj_in.points[ind].pose);
      // Add the distance to the distance_list.
      distance_list.push_back(distance);
    }

    // Get the index of the minimum distance in the distance_list.
    int ind = std::distance(
        distance_list.begin(),
        std::min_element(distance_list.begin(), distance_list.end()));

    // Get the heading of the robot and the path.
    double robot_heading = get_yaw(robot_pose.orientation);
    double path_heading = get_yaw(_traj_in.points[ind].pose.orientation);

    // Set the threshold for acceptable heading difference.
    double degrees = 90.0;
    double radians = degrees * (M_PI / 180.0);

    // Check if the difference between the headings is within the threshold.
    if (abs(abs(robot_heading) - abs(path_heading)) > radians) {
      ROS_INFO("Headings are %f apart", abs(robot_heading - path_heading));
      heading_ok = false;
    } else {
      heading_ok = true;
    }

    // Get the distance at the nearest index.
    int dis = distance_list[ind];
    // Return the nearest index, distance and heading_ok as a tuple.
    return {ind, dis, heading_ok};
  }

  void publish_points(const std::vector<int> &collisionPointsIndex);
  void publish_velocity_marker(const autopilot_msgs::Trajectory &trajectory);
};
} // namespace obstacle_stop_planner

#endif // OBSTACLE_STOP_PLANNER_H