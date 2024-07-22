#include "obstacle_stop_planner/obstacle_stop_planner.h"

namespace obstacle_stop_planner {

// Linearly interpolate a value for a given input within a given range.
double ObstacleStopPlanner::interpolate(double x, double x0, double x1,
                                        double y0, double y1) {
  // Calculate the normalized parameter t in the range [0, 1]
  double normalized_parameter = (x - x0) / (x1 - x0);

  // Ensure the normalized parameter is within the valid range [0, 1]
  normalized_parameter = std::max(0.0, std::min(1.0, normalized_parameter));

  // Perform linear interpolation and return the result
  return y0 + normalized_parameter * (y1 - y0);
}

void ObstacleStopPlanner::publish(
    const diagnostic_msgs::DiagnosticStatus &diagnostics_publisher) {
  diagnostic_msgs::DiagnosticArray diagnostics;
  diagnostics.status.push_back(diagnostics_publisher);
  diagnostics.header.stamp = ros::Time::now();
  publisher.publish(diagnostics);
}

bool isValidPose(const geometry_msgs::Pose &pose) {
  return !std::isnan(pose.position.x) && !std::isnan(pose.position.y) &&
         !std::isnan(pose.position.z);
}

std::pair<bool, std::string> ObstacleStopPlanner::do_initial_sensor_check() {
  bool is_lidar_sensor_healthy = true;
  bool is_zed_sensor_healthy = true;
  bool is_global_path_ok = false;
  std::string reason_sensor_msg = "Waiting for";

  if (traj_helper_obj.getLength() > 0 && isValidPose(robot_pose)) {
    ROS_INFO("Global Path & Robot_Pose Received");
    is_global_path_ok = true;

    if ((use_obs_v1 || use_pcl_boxes) && !scan_data_received) {
      ROS_WARN("Waiting for Data scan");
      is_lidar_sensor_healthy = false;
      reason_sensor_msg += " lidar data";
    }

    if (use_pcl_boxes && bboxes.boxes.empty()) {
      ROS_WARN("Waiting for Bounding Boxes");
      is_lidar_sensor_healthy = false;
      reason_sensor_msg += " lidar data";
    }

    if (use_zed_detections && !zed_objects) {
      ROS_WARN("Waiting for ZED Data");
      is_zed_sensor_healthy = false;
      if (debug_enable) {
        reason_sensor_msg += " zed data";
      } else {
        reason_sensor_msg += " camera data";
      }
    } else if (use_zed_detections) {
      ROS_INFO("ZED has been acknowledged");
    }
  } else {
    ROS_WARN("Waiting for global traj: %d, odom: [x: %f, y: %f, z: %f]",
             traj_helper_obj.getLength(), robot_pose.position.x,
             robot_pose.position.y, robot_pose.position.z);

    if (debug_enable) {
      reason_sensor_msg += " global-trajectory";
    } else {
      reason_sensor_msg += " path";
    }
  }

  bool sensor_status =
      is_lidar_sensor_healthy && is_zed_sensor_healthy && is_global_path_ok;
  return {sensor_status, reason_sensor_msg};
}

void ObstacleStopPlanner::main_loop() {

  ros::Rate rate(1);
  while (!ros::isShuttingDown()) {
    diagnostics_publisher.clearSummary();
    diagnostics_publisher.values.clear();
    bool sensor_status;
    std::string sensor_msg;
    std::tie(sensor_status, sensor_msg) = do_initial_sensor_check();

    if (sensor_status) {
      diagnostics_publisher.summary(OK, "Received data");
      diagnostics_publisher.add("Sensor_Status ", sensor_status);
      publish(diagnostics_publisher);
      break;
    } else {
      diagnostics_publisher.summary(ERROR, sensor_msg);
      diagnostics_publisher.add("Sensor_Status ", sensor_status);
      publish(diagnostics_publisher);
    }
    rate.sleep();
    ros::spinOnce();
  }
  rate = ros::Rate(10);

  int start_by_pass_index = -1, end_by_pass_index = -1, prev_by_pass_dist = -1;
  while (!ros::isShuttingDown()) {
    diagnostics_publisher.clearSummary();
    diagnostics_publisher.values.clear();

    loop_start_time = ros::Time::now().toSec();

    // Odom Related timing check
    odom_loop_start_time = ros::Time::now();
    odom_time_difference = odom_loop_start_time - odom_data_in_time;

    // Debug logging
    ROS_INFO("Loop start time: %f", odom_loop_start_time.toSec());
    ROS_INFO("Last odom update time: %f", odom_data_in_time.toSec());
    ROS_INFO("Time difference: %f", odom_time_difference.toSec());

    if (odom_time_difference > _TIME_OUT_FROM_ODOM) {
      ROS_WARN("No Update on Odom (Robot Position)");
      if (debug_enable) {
        diagnostics_publisher.summary(ERROR,
                                      "No update on odom (robot position)");
      } else {
        diagnostics_publisher.summary(ERROR, "No update on gps");
      }
      diagnostics_publisher.add("Loop start time", odom_loop_start_time);
      diagnostics_publisher.add("Last odom update time", odom_data_in_time);
      diagnostics_publisher.add("Odom Time difference", odom_time_difference);
      publish(diagnostics_publisher);
      rate.sleep();
      ros::spinOnce();
      continue;
    }

    // Check if using obs_v1 or pcl_boxes
    if (use_obs_v1 || use_pcl_boxes) {
      // Check for a timeout on laser data updates
      if (loop_start_time - laser_data_in_time > _TIME_OUT_FROM_LASER) {
        ROS_WARN_THROTTLE(2, "No update on laser data from last %f",
                          loop_start_time - laser_data_in_time);

        // Publish diagnostics message for laser data timeout
        if (debug_enable) {
          diagnostics_publisher.summary(ERROR, "No update on laser data");
        } else {
          diagnostics_publisher.summary(ERROR, "No update on lidar");
        }
        diagnostics_publisher.add("Last laser time",
                                  (loop_start_time - laser_data_in_time));
        publish(diagnostics_publisher);

        // Sleep and continue to the next iteration
        rate.sleep();
        ros::spinOnce();
        continue;
      }
    }
    // Check if using zed_detections
    if (use_zed_detections) {
      // Check for a timeout on ZED data updates
      if (loop_start_time - zed_data_in_time > _TIME_OUT_FROM_ZED) {
        ROS_WARN_THROTTLE(2, "No update on ZED data from last %f",
                          loop_start_time - zed_data_in_time);

        // Publish diagnostics message for ZED data timeout
        if (debug_enable) {
          diagnostics_publisher.summary(ERROR, "No update on ZED data");
        } else {
          diagnostics_publisher.summary(ERROR, "No update on camera");
        }
        diagnostics_publisher.add("Last ZED time",
                                  (loop_start_time - zed_data_in_time));
        publish(diagnostics_publisher);

        // Sleep and continue to the next iteration
        rate.sleep();
        ros::spinOnce();
        continue;
      }
    }
    // Check if _close_idx is not assigned
    if (_close_idx == -1) {
      double angle_th = 90;
      bool found;
      int32_t index;

      // Find the closest index with distance and angle threshold
      tie(found, index) = traj_helper_obj.find_closest_idx_with_dist_ang_thr(
          robot_pose, _max_look_ahead_dis, angle_th);
      if (found) {
        // Assign the found index as the close index
        _close_idx = index;
        // ROS_INFO( "Close Index IF: %d", _close_idx);
      } else {
        // Log a warning and publish diagnostics for no close point found
        ROS_WARN("No close point found dist_thr: %f, angle_thr: %f",
                 _max_look_ahead_dis, angle_th);
        diagnostics_publisher.summary(ERROR, "No close point found");
        diagnostics_publisher.add("Distance Threshold", _max_look_ahead_dis);
        diagnostics_publisher.add("Angle Threshold", angle_th);
        publish(diagnostics_publisher);

        // Sleep and continue to the next iteration
        rate.sleep();
        ros::spinOnce();
        continue;
      }
    } else {
      // Update the close index based on the current robot pose and the existing
      // close index
      _close_idx = traj_helper_obj.find_close_pose_after_index(robot_pose,
                                                               _close_idx, 10);
      // ROS_INFO( "Close Index ELSE: %d", _close_idx);
    }
    // Create a PoseStamped message and publish it
    geometry_msgs::PoseStamped poseStamped_obj;
    poseStamped_obj.header.frame_id = "map";
    poseStamped_obj.pose =
        traj_helper_obj.get_trajectory_point_by_index(_close_idx).pose;

    close_pose_pub.publish(poseStamped_obj);
    front_tip_idx =
        traj_helper_obj.next_point_within_dist(_close_idx, _base_to_front);
    geometry_msgs::PoseStamped poseStamped_object;
    poseStamped_object.header.frame_id = "map";
    poseStamped_object.pose =
        traj_helper_obj.get_trajectory_point_by_index(front_tip_idx).pose;
    float path_percent =
        (traj_helper_obj.get_trajectory_point_by_index(_close_idx)
             .accumulated_distance_m /
         _traj_in.points.back().accumulated_distance_m) *
        100;
    // ROS_INFO("Calculated Path percent: %f", path_percent);
    // Check if path_percent is within the valid range
    if (std::isfinite(path_percent)) {
      // Create a Float32 message
      std_msgs::Float32 percent_msg;
      percent_msg.data = path_percent;

      // Publish the message
      path_percent_publisher.publish(percent_msg);
    } else {
      ROS_ERROR("Invalid path_percent value: %.2f", path_percent);
    }

    if (path_percent > 95.0 &&
        (distanceBetweenPoses(robot_pose, _traj_in.points.back().pose) <=
         _min_look_ahead_dis)) {
      count_mission_repeat.data += 1;
      ROS_INFO("Mission Count %d", (count_mission_repeat.data));
      if (mission_continue) {
        _close_idx = 1;
        diagnostics_publisher.summary(OK, "Mission Complete ");
        diagnostics_publisher.add("Mission Repeat ",
                                  (count_mission_repeat.data));
        publish(diagnostics_publisher);
        // Time Conversion
        std::chrono::milliseconds time_to_wait_ms(time_to_wait_at_ends);
        std::this_thread::sleep_for(time_to_wait_ms);
        continue;
      } else {
        rate.sleep();
        break;
      }
    }
    // Bypass Mode
    prev_processed_ind = _close_idx;
    obstacle_found = false;

    autopilot_msgs::Trajectory trajectory_msg;
    trajectory_msg.header.frame_id = "map";
    trajectory_msg.home_position = _traj_in.home_position;
    ros::param::get("/obstacle_stop_planner/by_pass_dist", by_pass_dist);
    ROS_INFO_THROTTLE(5, "The Bypass Distance %f", by_pass_dist);

    if (by_pass_dist > 0) {
      if (prev_by_pass_dist != -1 && prev_by_pass_dist != 0) {
        prev_by_pass_dist = by_pass_dist;
      }

      if (end_by_pass_index == -1 || prev_by_pass_dist != by_pass_dist) {
        ROS_INFO_THROTTLE(10, "Vehicle is Bypassing!");
        start_by_pass_index = _close_idx;
        int ind;
        for (ind = _close_idx; ind < _traj_end_index; ind++) {
          path_acc_distance =
              _traj_in.points[ind].accumulated_distance_m -
              _traj_in.points[_close_idx].accumulated_distance_m;

          if (path_acc_distance > (by_pass_dist + _base_to_front)) {
            break;
          }
          trajectory_msg.points.push_back(_traj_in.points[ind]);
        }
        end_by_pass_index = ind;
      } // End of if

      else {
        // Fill Traj msg till end_bypass_index
        for (int ind = start_by_pass_index; ind < end_by_pass_index + 1;
             ind++) {
          traj_point = _traj_in.points[ind];
          traj_point.longitudinal_velocity_mps = robot_min_speed_th;
          trajectory_msg.points.push_back(traj_point);
          start_by_pass_index = _close_idx;
        }
        for (int ind = end_by_pass_index; ind < _traj_end_index - 1; ind++) {
          path_acc_distance =
              _traj_in.points[ind].accumulated_distance_m -
              _traj_in.points[_close_idx].accumulated_distance_m;
          if (path_acc_distance > _lookup_collision_distance) {
            break;
          }

          traj_point = _traj_in.points[ind];
          traj_point.longitudinal_velocity_mps = 0.0;
          trajectory_msg.points.push_back(traj_point);
        }
      }

      ROS_INFO("Start_by_pass_index is %d and end_by_pass_index %d ",
               start_by_pass_index, end_by_pass_index);
      local_traj_publisher.publish(trajectory_msg);
      publish_velocity_marker(trajectory_msg);

      diagnostics_publisher.summary(ERROR, "Bypass collision");
      diagnostics_publisher.add("Bypassing the obstacle of Meters ",
                                by_pass_dist);
      publish(diagnostics_publisher);

      // Resetting the By-pass Parameters
      if (_close_idx > end_by_pass_index) {
        ros::param::set("/obstacle_stop_planner/by_pass_dist", 0);
        end_by_pass_index = -1;
        start_by_pass_index = -1;
        prev_by_pass_dist = -1;
      }
      prev_by_pass_dist = by_pass_dist;
      rate.sleep();
      ros::spinOnce();
      continue;
    }
    if (use_obs_v1) {
      ROS_INFO_ONCE("V1 Kd-Tree");
      try {
        // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
        pcl::fromROSMsg(laser_np_2d, *t_cloud);
        // Ensure that the point cloud is not empty
        if (t_cloud->points.empty()) {
          ROS_WARN_THROTTLE(5, "Input point cloud is empty");
          continue;
        }

        /*
        The Else block is created because the z values also matters in the
        c++ version. The Python version takes the 2d data as its library takes
        care of it whereas c++ version library takes 3d data so from the map
        frame if a obstacle is at the edges the kd-tree query is ignoring the
        obstacles as it has the z data.
        */
        else {
          // Iterate through each point and set z-coordinate to 0
          for (auto &point : t_cloud->points) {
            point.z = 0.0;
          }
        }

        // ----------------- To check Pointcloud values
        // --------------------------- for
        // (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it !=
        // cloud->end(); ++it) {
        //     float x = it->x;  // Access x coordinate
        //     float y = it->y;  // Access y coordinate
        //     float z = it->z;  // Access z coordinate

        //     // Now you can use x, y, and z for further processing or store
        //     them in some data structure.
        //     // For example, you might want to print the coordinates:
        //     std::cout << "Point: (" << x << ", " << y << ", " << z << ")" <<
        //     std::endl;
        // }

        // ROS_INFO("The cloud size %zu", t_cloud->size());
        kdtree.setInputCloud(t_cloud);
      } catch (const std::exception &e) {
        ROS_ERROR("Failed to fill the KD-tree structure: %s", e.what());
        rate.sleep();
        ros::spinOnce();
        continue;
      }
    }

    // ------- Start of Robot Level Collision --------

    if (robot_level_collision_check_enable) {
      // Activate collision check if enabled
      ROS_INFO_ONCE("Robot Level Collision Check Activated");
      obs_found_robot_collision = false;
      obs_found_sensor = "";
      zed_found_object = "";
      radius_to_search_near_robot =
          (data_vehicle_obj.overall_width + width_offset) / 2;

      std::tie(circles_array, circles_polygon) = transform_footprint_circles(
          robot_pose, data_vehicle_obj, width_offset = width_offset,
          length_offset = length_offset, to_polygon = true);
      circles_polygon_pub.publish(circles_polygon);

      if (use_obs_v1) {
        // Run obstacle check using OBS V1 if enabled
        ROS_INFO_ONCE("Running with OBS V1");
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        for (const auto &circle : circles_array) {
          pcl::PointXYZ query_point;
          query_point.x = circle[0];
          query_point.y = circle[1];
          // query_point.z = circle[2];

          // Check whether the points have NAN or inf because kd-tree will crash
          // if these values are passed so skipping it
          if (!std::isfinite(query_point.x) || !std::isfinite(query_point.y)) {
            ROS_ERROR("Circle query points (%f, %f) have non-finite values. "
                      "Skipping it.",
                      query_point.x, query_point.y);
            continue;
          }

          try {
            // Search for obstacles using radius search
            if (kdtree.radiusSearch(query_point, radius_to_search_near_robot,
                                    pointIdxRadiusSearch,
                                    pointRadiusSquaredDistance) > 0) {
              obs_found_robot_collision = true;
              obs_found_sensor = "V1 (RLC)"; // Robot Level Collision
              ROS_WARN("RLC: Identified obstacle (V1)");
              publish_points(pointIdxRadiusSearch);
              break;
            }
          } catch (const std::exception &e) {
            ROS_ERROR("Exception caught in use_obs_v1 circle array logic: %s",
                      e.what());
          }
        }
      }

      // If any obstacle found in V1 then this v2 won't get executed (Achieved
      // by the flag variable obs_found_robot_collision)
      if (use_pcl_boxes && !obs_found_robot_collision &&
          !bboxes.boxes.empty()) {
        // Run obstacle check using OBS V2 if enabled
        ROS_INFO_ONCE("Running with OBS V2");

        for (const auto &circle : circles_array) {
          if (circle.size() >= 2) {
            std::pair<double, double> center(circle[0], circle[1]);

            try {
              // Find the closest object from bounding boxes
              std::pair<int, double> result = find_close_object(bboxes, center);
              // close_box_id - first and close_dis - second
              if (result.second < radius_to_search_near_robot) {
                ROS_WARN_THROTTLE(3, "Identified Obstacle (V2)");
                bbox_pub.publish(bboxes.boxes[result.first]);
                obs_found_robot_collision = true;
                obs_found_sensor = "V2 (RLC)";
                break;
              }
            } catch (const std::exception &e) {
              ROS_ERROR("Error in find_close_object: %s", e.what());
            }
          }
        }
      }

      // If any obstacle found in V1 or V2, then this won't get executed
      // (Achieved by the flag variable obs_found_robot_collision)
      try {
        if (use_zed_detections && !obs_found_robot_collision) {
          ROS_INFO_ONCE("Running Zed");

          for (const auto &circle : circles_array) {
            std::pair<float, int> result =
                find_close_object_zed(zed_objects, circle);
            close_obj_from_zed = result.first;
            close_obj_zed_idx = result.second;

            if (close_obj_from_zed < radius_to_search_near_robot) {
              ROS_WARN_THROTTLE(3, "Identified Obstacle (ZED)");
              obs_found_robot_collision = true;
              obs_found_sensor = "ZED (RLC)";

              if (close_obj_zed_idx == -1 || close_obj_from_zed <= 0) {
                zed_found_object = "Undefined Object";
                ROS_DEBUG("The Distance of object (RLC): %f",
                          close_obj_from_zed);
                ROS_DEBUG("The Index of object (RLC): %d", close_obj_zed_idx);
              } else {
                // Ensure close_obj_zed_idx is within bounds or else creates
                // segmentation fault
                if (close_obj_zed_idx >= 0 &&
                    close_obj_zed_idx < zed_objects->objects.size()) {
                  zed_found_object =
                      zed_objects->objects[close_obj_zed_idx].label;

                  if (zed_found_object.empty()) {
                    zed_found_object = "Unlabeled Object (RLC)";
                  } else {
                    // check for utf-8' encoding type chars
                    for (char &c : zed_found_object) {
                      if (!isprint(c)) {
                        zed_found_object = "Invalid Characters (RLC)";
                        break;
                      }
                    }
                  }
                } else {
                  zed_found_object = "Object Index Invalid (RLC)";
                  ROS_WARN("Invalid Zed object index: %d", close_obj_zed_idx);
                }
              }
              break;
            }
          }
        }
      } catch (const std::bad_alloc &e) {
        ROS_ERROR(
            "Memory allocation failed during RLC ZED obstacle detection: %s",
            e.what());
      } catch (const std::exception &e) {
        ROS_ERROR("An unexpected exception occurred during RLC ZED obstacle "
                  "detection: %s",
                  e.what());
      } catch (...) {
        ROS_ERROR(
            "An unknown exception occurred during RLC ZED obstacle detection");
      }

      if (obs_found_robot_collision) {
        end_idx = std::min(_close_idx + 100, _traj_end_index);
        trajectory_msg.points.assign(_traj_in.points.begin() + _close_idx,
                                     _traj_in.points.begin() + end_idx);

        for (auto &point : trajectory_msg.points) {
          point.longitudinal_velocity_mps = 0.0;
        }

        local_traj_publisher.publish(trajectory_msg);
        publish_velocity_marker(trajectory_msg);

        ROS_WARN(
            "RLC : Immediate stop as an obstacle is found near the vehicle");
        ROS_INFO_THROTTLE(10, "Collision Points Published");
        if (debug_enable) {
          diagnostics_publisher.summary(ERROR, "Obstacle in front");
        } else {
          diagnostics_publisher.summary(ERROR, "Obstacle Found");
        }
        diagnostics_publisher.add("Obstacle Detected length offset",
                                  length_offset);
        diagnostics_publisher.add("Obstacle Detected Sensor", obs_found_sensor);
        if (obs_found_sensor == "ZED (RLC)") {
          diagnostics_publisher.add("Zed Detected Object", zed_found_object);
        }
        publish(diagnostics_publisher);
        rate.sleep();
        ros::spinOnce();
        continue;
      } else {
        ROS_INFO(
            "RLC : No Obstructions present in the proximity of the vehicle\n");
      }
    }

    // ------End of Robot Level Collision ---------

    int global_index = _close_idx;
    obstacle_encountered = false; // Flag to indicate if obstacle is encountered
    obstacle_found_sensor_main_logic = "";
    zed_obstacle_found_main_logic = "";
    for (; global_index < _traj_end_index; global_index++) {
      path_acc_distance = _traj_in.points[global_index].accumulated_distance_m -
                          _traj_in.points[_close_idx].accumulated_distance_m;

      if (path_acc_distance > _lookup_collision_distance) {
        break;
      }

      vector<double> collision_points;

      if (_traj_in.points[global_index].accumulated_distance_m -
              _traj_in.points[prev_processed_ind].accumulated_distance_m >
          _radius_to_search / 2) {
        path_pose = _traj_in.points[global_index].pose;

        try {
          if (use_zed_detections && !obstacle_encountered) {
            ROS_INFO_ONCE("Running ZED!!!");

            if (zed_objects != NULL) {
              std::vector<double> point_obj = {path_pose.position.x,
                                               path_pose.position.y};
              std::pair<float, int> result =
                  find_close_object_zed(zed_objects, point_obj);
              close_obj_from_zed = result.first;
              close_obj_zed_idx = result.second;

              if (close_obj_from_zed < _radius_to_search) {
                obstacle_found = true;
                obstacle_encountered = true;
                obstacle_found_sensor_main_logic = "Zed Camera";
                ROS_DEBUG("The Distance of object: %f", close_obj_from_zed);
                ROS_DEBUG("The Index of object: %d", close_obj_zed_idx);

                if (close_obj_zed_idx == -1 || close_obj_from_zed <= 0) {
                  ROS_DEBUG("Not a valid object");
                  zed_obstacle_found_main_logic = "Undefined Object";
                } else {
                  // Ensure close_obj_zed_idx is within bounds
                  if (close_obj_zed_idx >= 0 &&
                      close_obj_zed_idx < zed_objects->objects.size()) {
                    zed_obstacle_found_main_logic =
                        zed_objects->objects[close_obj_zed_idx].label;

                    if (zed_obstacle_found_main_logic.empty()) {
                      zed_obstacle_found_main_logic = "Object Unlabeled";
                    } else {
                      // Sanitize the label to avoid invalid characters
                      for (char &c : zed_obstacle_found_main_logic) {
                        if (!isprint(c)) {
                          zed_obstacle_found_main_logic =
                              "Invalid Characters Found";
                          break;
                        }
                      }
                    }
                  } else {
                    zed_obstacle_found_main_logic = "Invalid Object Index";
                    ROS_ERROR("Invalid Zed object index: %d",
                              close_obj_zed_idx);
                  }
                }
                break;
              }
            } else {
              ROS_ERROR_THROTTLE(2, "Zed Data is Empty");
            }
          }
        } catch (const std::bad_alloc &e) {
          ROS_ERROR("Memory allocation failed during main loop of ZED obstacle "
                    "detection: %s",
                    e.what());
        } catch (const std::exception &e) {
          ROS_ERROR("An unexpected exception occurred during main loop of ZED "
                    "obstacle detection: %s",
                    e.what());
        } catch (...) {
          ROS_ERROR("An unknown exception occurred during main loop of ZED "
                    "obstacle detection");
        }

        if (use_obs_v1 && !obstacle_encountered) {
          ROS_INFO_ONCE("OBS-V1 Running");

          pcl::PointXYZ search_point;
          search_point.x = path_pose.position.x;
          search_point.y = path_pose.position.y;
          // search_point.z = 0.7;

          // Check whether the points have NAN or inf because kd-tree will crash
          // it these values are passed so skipping it
          if (std::isnan(search_point.x) || std::isnan(search_point.y) ||
              std::isinf(search_point.x) || std::isinf(search_point.y)) {
            ROS_DEBUG(
                "Points at (%f, %f) has Non-finite values, Skipping it ohoo!",
                search_point.x, search_point.y);
          } else {
            try {
              std::vector<int> pointIdxRadiusSearch;
              std::vector<float> pointRadiusSquaredDistance;
              // ROS_INFO(" The Radius Value %f",_radius_to_search);
              if (kdtree.radiusSearch(search_point, _radius_to_search,
                                      pointIdxRadiusSearch,
                                      pointRadiusSquaredDistance) > 0) {
                new_collision_points =
                    pointIdxRadiusSearch; // For Publishing the collision points
                obstacle_found = true;
                obstacle_encountered = true;
                obstacle_found_sensor_main_logic = "V1 Lidar";
                break;
              } else {
                new_collision_points.clear();
              }
              prev_processed_ind = global_index;
            } catch (const std::exception &e) {
              ROS_ERROR("Could Not Query Kd-Tree: %s", e.what());
            }
          } // End of else-block
        }   // end if use_ob_v1

        if (use_pcl_boxes && !bboxes.boxes.empty() && !obstacle_encountered) {
          ROS_INFO_ONCE("OBS-V2 Running");
          try {
            tie(close_bbx_id, close_dis) = find_close_object(
                bboxes, {path_pose.position.x, path_pose.position.y});
            if (close_dis < _radius_to_search) {
              obstacle_found = true;
              obstacle_encountered = true;
              obstacle_found_sensor_main_logic = "V2 Lidar";
              bbox_pub.publish(bboxes.boxes[close_bbx_id]);
              break;
            }
          } catch (const std::exception &err) {
            ROS_ERROR("Error in find_close_object %s", err.what());
          }

        } // end if(use_pcl_boxes)
      }
    } // end for loop

    // ROS_ERROR("The Global Index is %d ",global_index);
    collision_index = global_index;
    // if (!collision_points.empty()) {
    //   new_collision_points.push_back(collision_points[0]);
    // }
    // ROS_INFO("The Old After Loop %d",_close_idx);

    if (obstacle_encountered) {
      double dis_to_obstacle =
          abs(_traj_in.points[collision_index].accumulated_distance_m -
              _traj_in.points[_close_idx].accumulated_distance_m);
      ROS_WARN("Obstacle Found in %f Mtrs", dis_to_obstacle);

      if (dis_to_obstacle < (_stop_line_buffer + _base_to_front)) {
        ROS_INFO("The sum of stop line buffer + base to front %f",
                 (_stop_line_buffer + _base_to_front));
        ROS_WARN("Applying brakes due to a nearby obstacle\n");

        if (debug_enable) {
          diagnostics_publisher.summary(ERROR, "Obstacle is Very Close");
        } else {
          diagnostics_publisher.summary(ERROR, "Obstacle Found");
        }

        diagnostics_publisher.add("Applying Brake", true);
        diagnostics_publisher.add("Obstacle Found at ", dis_to_obstacle);
        diagnostics_publisher.add("Obstacle Detected Sensor ",
                                  obstacle_found_sensor_main_logic);
        if (obstacle_found_sensor_main_logic == "Zed Camera") {
          diagnostics_publisher.add("Zed Detected Obstacle ",
                                    zed_obstacle_found_main_logic);
          // diagnostics_publisher.add("Zed Detected Obstacle distance
          // ",close_obj_from_zed);
        }
        publish(diagnostics_publisher);

        vehicle_stop_init_time_for_obs = ros::Time::now().toSec();
        for (int index = _close_idx; index < collision_index + 1; index++) {
          traj_point = _traj_in.points[index];
          traj_point.longitudinal_velocity_mps = 0.0;
          trajectory_msg.points.push_back(traj_point);
        }
      }

      else {
        ROS_INFO("The obstacle is at a distance greater than the stopping "
                 "distance\n");
        double stop_index = collision_index;
        double temp_dist = 0.0;

        while ((temp_dist < (_stop_line_buffer + _base_to_front)) &&
               (stop_index > _close_idx)) {
          temp_dist =
              abs(_traj_in.points[collision_index].accumulated_distance_m -
                  _traj_in.points[stop_index].accumulated_distance_m);
          stop_index -= 1;
        }
        if (velocity_speed_profile_enable) {
          try {
            double slow_stop_index = stop_index;
            double tmp_dist = 0.0;

            // Find the slow-stop index based on accumulated distance
            while ((tmp_dist < slow_line_buffer) &&
                   slow_stop_index > _close_idx) {
              tmp_dist =
                  abs(_traj_in.points[stop_index].accumulated_distance_m -
                      _traj_in.points[slow_stop_index].accumulated_distance_m);
              slow_stop_index -= 1;
            }

            // Speed sent to zero to the trajectory from collision index to stop
            // index
            for (int index = collision_index; index > stop_index; --index) {
              traj_point = _traj_in.points[index];
              traj_point.longitudinal_velocity_mps = 0.0;
              trajectory_msg.points.push_back(traj_point);
            }

            // Interpolate speeds and update trajectory from stop_index to
            // slow_stop_index
            for (int index = stop_index + 1; index > slow_stop_index; --index) {
              tmp_dist =
                  abs(_traj_in.points[stop_index].accumulated_distance_m -
                      _traj_in.points[index].accumulated_distance_m);
              double updt_spd =
                  interpolate(tmp_dist, 0, slow_line_buffer, robot_min_speed_th,
                              robot_max_speed_th);
              traj_point = _traj_in.points[index];
              traj_point.longitudinal_velocity_mps = min(
                  updt_spd, _traj_in.points[index].longitudinal_velocity_mps);
              trajectory_msg.points.push_back(traj_point);
              ROS_INFO("Minimum of Interpolated value %f & "
                       "longitudinal_velocity_mps value %f",
                       updt_spd, traj_point.longitudinal_velocity_mps);
              ROS_WARN(
                  "Obstacle in sight,the vehicle is programmed to slow down\n");
            }

            // Append the remaining trajectory from slow_stop_index to
            // _close_idx
            for (int index = slow_stop_index; index > _close_idx; --index) {
              traj_point = _traj_in.points[index];
              trajectory_msg.points.push_back(traj_point);
            }

            // Publish the pose of the slow-down point
            geometry_msgs::PoseStamped stamped_obj;
            stamped_obj.header.frame_id = "map";
            stamped_obj.pose =
                traj_helper_obj.get_trajectory_point_by_index(slow_stop_index)
                    .pose;
            slow_pose_pub.publish(stamped_obj);

            // Reverse the trajectory for proper ordering
            std::reverse(trajectory_msg.points.begin(),
                         trajectory_msg.points.end());

            // Publish diagnostics information
            diagnostics_publisher.summary(WARN, "Vehicle will slow down");
            diagnostics_publisher.add("Obstacle Found at ", dis_to_obstacle);
            publish(diagnostics_publisher);

          } catch (const std::exception &e) {
            // Handle the exception
            ROS_ERROR("Exception caught in velocity speed profile: %s",
                      e.what());
          }

        } else {
          // Speed Profile Disabled
          for (int index = _close_idx; index < stop_index + 1; ++index) {
            trajectory_msg.points.push_back(_traj_in.points[index]);
          }

          // Stop the trajectory from stop_index to collision_index
          for (int index = stop_index; index < collision_index; ++index) {
            traj_point = _traj_in.points[index];
            traj_point.longitudinal_velocity_mps = 0.0;
            trajectory_msg.points.push_back(traj_point);
          }
        }
      }
    } // End of if(obstacle_found)

    // Check if no obstacle is found
    else {

      // Check if the vehicle stop initialization time for obstacle exists
      if (vehicle_stop_init_time_for_obs != -1) {
        double current_time = ros::Time::now().toSec();
        // Check if the time since the vehicle stop initialization exceeds the
        // threshold
        if ((current_time - vehicle_stop_init_time_for_obs) >
            stop_threshold_time_for_obs) {
          ROS_INFO("The way is clear!");

          // Publish diagnostics message for no obstacle found
          diagnostics_publisher.summary(OK, "Obstacle Free");
          diagnostics_publisher.add("Vehicle Moving", true);
          publish(diagnostics_publisher);

          // Populate the trajectory message with points from _close_idx to
          // collision_index
          for (int index = _close_idx; index < collision_index; ++index) {
            trajectory_msg.points.push_back(_traj_in.points[index]);
          }
        } else {
          // Calculate the remaining time for the vehicle to resume movement
          double elapsed_time =
              ros::Time::now().toSec() - vehicle_stop_init_time_for_obs;
          double remaining_time = stop_threshold_time_for_obs - elapsed_time;

          // Log a warning about the obstacle wait time
          ROS_INFO("OBS Time Limit not crossed %lf", remaining_time);

          // Publish diagnostics message for obstacle wait time
          if (debug_enable) {
            diagnostics_publisher.summary(ERROR, "Obstacle Wait Time");
          } else {
            diagnostics_publisher.summary(ERROR, "Obstacle Found");
          }
          diagnostics_publisher.add("Vehicle Will Move in", remaining_time);
          publish(diagnostics_publisher);

          // Populate the trajectory message with points, setting longitudinal
          // velocity to 0.0
          for (int index = _close_idx; index < collision_index; ++index) {
            traj_point = _traj_in.points[index];
            traj_point.longitudinal_velocity_mps = 0.0;
            trajectory_msg.points.push_back(traj_point);
          }
        }
      } else {
        diagnostics_publisher.summary(OK, "Obstacle Free");
        diagnostics_publisher.add("Vehicle Moving!!!", true);
        publish(diagnostics_publisher);
        // Populate the trajectory message with points
        for (int index = _close_idx; index < collision_index; ++index) {
          traj_point = _traj_in.points[index];
          trajectory_msg.points.push_back(traj_point);
        }
      }
    } // End of else (obstacle_found)

    // Publish local trajectory
    local_traj_publisher.publish(trajectory_msg);

    // Publish collision points
    publish_points(new_collision_points);
    // ROS_INFO("THe Collision Index %d",collision_index);
    // Publish velocity marker for visualization
    publish_velocity_marker(trajectory_msg);

    // Sleep based on the specified rate
    rate.sleep();
    ros::spinOnce();
  } // End of While Loop

} // End Main_loop Function

// Find the minimum distance from a given point to objects detected by ZED
// camera

std::pair<float, int> ObstacleStopPlanner::find_close_object_zed(
    zed_interfaces::ObjectsStamped::ConstPtr objects,
    std::vector<double> point) {
  // Store the distances to ZED-detected objects
  vector<double> zed_obs_dis_data;

  // Check if the objects pointer is null
  if (!objects) {
    ROS_ERROR("Null pointer received for objects");
    return std::make_pair(std::numeric_limits<float>::infinity(), -1);
  }

  /*
  The axes are defined according to the ROS standard:
  X Forward, Y LEFT, Z UP. The distance of an object from the camera is on the X
  axis. The position of the object is the centroid of the positions of all the
  3D points that compose the object itself. In case of partial object detection,
  the centroid is calculated based on visible data. If tracking is active and
  the partial object matches a previously seen object, the centroid position is
  "smoothed."
  */

  // Loop through detected objects
  for (const auto &object : objects->objects) {
    // Check if the object position has at least 2 coordinates
    if (object.position.size() < 2) {
      ROS_WARN_THROTTLE(5, "Object position size less than 2");
      continue;
    }

    // Calculate the distance from the center point (xyz) to the object
    distance_cal_zed = std::hypot(point[0] - object.position[0],
                                  point[1] - object.position[1]);

    // If the distance is less than the threshold, directly pass the distance;
    // otherwise, check for corners too to avoid redundancy in object detection.
    if (distance_cal_zed < _radius_to_search) {
      zed_obs_dis_data.push_back(distance_cal_zed);
    } else {
      // Check distances to corners for redundancy check
      for (const auto &corner : object.bounding_box_3d.corners) {
        // Check if the corner keypoint has at least 2 coordinates
        if (corner.kp.size() < 2) {
          ROS_WARN_THROTTLE(5, "Corner keypoint size less than 2");
          continue;
        }
        distance_cal_zed =
            std::hypot(point[0] - corner.kp[0], point[1] - corner.kp[1]);
        zed_obs_dis_data.push_back(distance_cal_zed);
      }
    }
  }

  try {
    if (!zed_obs_dis_data.empty()) {
      // Find the minimum distance index
      auto min_it =
          std::min_element(zed_obs_dis_data.begin(), zed_obs_dis_data.end());
      zed_obs_dis_data_min = std::distance(zed_obs_dis_data.begin(), min_it);
      // ROS_WARN("The distance: %f", zed_obs_dis_data[zed_obs_dis_data_min]);
      // Return the Minimum distance & Minimum distance index
      return std::make_pair(
          static_cast<float>(zed_obs_dis_data[zed_obs_dis_data_min]),
          zed_obs_dis_data_min);
    }
  } catch (const std::bad_alloc &e) {
    ROS_ERROR_STREAM(
        "Memory allocation failed in find_close_object_zed: " << e.what());
    return std::make_pair(std::numeric_limits<float>::infinity(), -1);
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("Exception from find_close_object_zed: " << e.what());
    return std::make_pair(std::numeric_limits<float>::infinity(), -1);
  }

  // Return -1 and infinity if no valid distances found
  return std::make_pair(std::numeric_limits<float>::infinity(), -1);
}

void ObstacleStopPlanner::zed_objects_callback(
    const zed_interfaces::ObjectsStamped::ConstPtr &data) {

  // Record the start time
  ros::Time start = ros::Time::now();

  // Initialize a pointer to store the transformed data in the map frame
  zed_data_in_time = ros::Time::now().toSec();

  auto object_data = *data;
  std::string to_frame = "map";

  try {
    geometry_msgs::TransformStamped trans =
        tf2_buffer.lookupTransform(to_frame, object_data.header.frame_id,
                                   ros::Time(0), ros::Duration(1.0));

    // Transform the position of each object
    for (int i = 0; i < object_data.objects.size(); i++) {
      // Convert boost::array<float, 3UL> to geometry_msgs::Point
      geometry_msgs::Point position;
      position.x = object_data.objects[i].position[0];
      position.y = object_data.objects[i].position[1];
      position.z = object_data.objects[i].position[2];

      // Update boost::array<float, 3UL> with the converted values
      auto converted_position = convert_point_by_transform(position, trans);
      object_data.objects[i].position = {
          static_cast<float>(converted_position.x),
          static_cast<float>(converted_position.y),
          static_cast<float>(converted_position.z)};
    }

    // Transform the corners of the bounding box of each object
    for (int i = 0; i < object_data.objects.size(); i++) {
      for (int j = 0; j < object_data.objects[i].bounding_box_3d.corners.size();
           j++) {
        // Convert std::vector<double> to geometry_msgs::Point
        geometry_msgs::Point newPoint;
        newPoint.x = object_data.objects[i].bounding_box_3d.corners[j].kp[0];
        newPoint.y = object_data.objects[i].bounding_box_3d.corners[j].kp[1];
        newPoint.z = object_data.objects[i].bounding_box_3d.corners[j].kp[2];

        // Update boost::array<float, 3UL> with the converted values
        auto convertedPoint = convert_point_by_transform(newPoint, trans);
        object_data.objects[i].bounding_box_3d.corners[j].kp = {
            static_cast<float>(convertedPoint.x),
            static_cast<float>(convertedPoint.y),
            static_cast<float>(convertedPoint.z)};
      }
    }
    object_data.header.frame_id = to_frame;
    zed_objects =
        boost::make_shared<const zed_interfaces::ObjectsStamped>(object_data);
    transformed_zed_objects_publisher.publish(zed_objects);
  }
  // If transform lookup fails, log the error and return
  catch (tf2::TransformException &e) {
    ROS_ERROR("In transform_zed_objects function failed to get transform "
              "from %s to %s: %s",
              to_frame.c_str(), object_data.header.frame_id.c_str(), e.what());
    // return zed_interfaces::ObjectsStamped();
    return;
  } catch (const std::length_error &e) {
    ROS_ERROR_STREAM("Length error in zed_objects_callback: " << e.what());
  } catch (const std::bad_alloc &e) {
    ROS_ERROR_STREAM(
        "Memory allocation failed in zed_objects_callback: " << e.what());
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("Exception in zed_objects_callback: " << e.what());
  }
  // Calculate the duration of the callback
  ros::Duration duration = ros::Time::now() - start;

  // Print the duration for debugging purposes
  ROS_DEBUG_STREAM("Time taken for zed objects callback: " << duration.toSec()
                                                           << " seconds");
}
pair<int, double> ObstacleStopPlanner::find_close_object(
    jsk_recognition_msgs::BoundingBoxArray bboxes,
    const pair<double, double> &point) {
  // Create a vector to store the distances
  vector<double> distance_list;

  // Iterate through each bounding box using iterators
  for (const auto &box : bboxes.boxes) {
    // Calculate the distance between the point and the bounding box
    double distance = hypot(point.first - box.pose.position.x,
                            point.second - box.pose.position.y);

    // Add the distance to the list
    distance_list.push_back(distance);
  }

  // Find the index of the minimum distance
  int close_bbox_id = std::distance(
      distance_list.begin(),
      std::min_element(distance_list.begin(), distance_list.end()));

  // Return the index and the corresponding distance
  return {close_bbox_id, distance_list[close_bbox_id]};
}

void ObstacleStopPlanner::publish_points(
    const std::vector<int> &collisionPointsIndex) {
  try {
    // Convert laser_np_2d to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(laser_np_2d, pcl_cloud);

    // Set the width and height of the PCL point cloud
    pcl_cloud.height = 1;
    pcl_cloud.width = collisionPointsIndex.size();
    pcl_cloud.points.resize(pcl_cloud.width * pcl_cloud.height);

    // Populate the PCL point cloud with collision points
    for (int curr_index = 0; curr_index < collisionPointsIndex.size() &&
                             curr_index < t_cloud->points.size();
         curr_index++) {
      pcl_cloud.points[curr_index].x =
          (*t_cloud)[collisionPointsIndex[curr_index]].x;
      pcl_cloud.points[curr_index].y =
          (*t_cloud)[collisionPointsIndex[curr_index]].y;
    }
    // Convert PCL point cloud to ROS sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 collision_points_pc2;
    pcl::toROSMsg(pcl_cloud, collision_points_pc2);

    // Set the frame ID for the collision_points_pc2
    collision_points_pc2.header.frame_id = "map";

    // Publish the collision points
    collision_points_publisher.publish(collision_points_pc2);
    ROS_DEBUG_STREAM("Joyfully Publishing Sample Pointcloud!");
  } catch (const std::exception &error) {
    // Handle exception if unable to publish collision points
    ROS_ERROR("Not able to Publish Collision Points: %s", error.what());
  }
}

void ObstacleStopPlanner::publish_velocity_marker(
    const autopilot_msgs::Trajectory &trajectory) {
  try {
    // Create a MarkerArray message to hold the markers
    visualization_msgs::MarkerArray marker_arr_msg;

    // Create a Marker object for deletion
    visualization_msgs::Marker marker_obj;
    int counter = 0;

    // Set the frame ID and action of the marker_obj
    marker_obj.header.frame_id = trajectory.header.frame_id;
    marker_obj.action = marker_obj.DELETEALL;

    // Add the marker_obj to the marker_arr_msg
    marker_arr_msg.markers.push_back(marker_obj);

    // Iterate through each trajectory point
    for (const auto &traj_point : trajectory.points) {
      // Create a new Marker object for each point
      visualization_msgs::Marker marker_obj;
      marker_obj.header.frame_id = trajectory.header.frame_id;
      marker_obj.type =
          visualization_msgs::Marker::TEXT_VIEW_FACING; // Use fully qualified
                                                        // name

      // Set the text of the marker_obj to the rounded longitudinal velocity
      marker_obj.text = std::to_string(
          std::round(traj_point.longitudinal_velocity_mps * 100) / 100);

      // Set the ID and action of the marker_obj
      marker_obj.id = counter;
      counter += 1;
      marker_obj.action =
          visualization_msgs::Marker::ADD; // Use fully qualified name

      // Set the scale, color, and pose of the marker_obj based on the
      // longitudinal velocity
      if (traj_point.longitudinal_velocity_mps == 0) {
        marker_obj.scale.x = marker_obj.scale.y = marker_obj.scale.z = 0.4;
        marker_obj.color.a = 1.0;
        marker_obj.color.r = 1.0;
        marker_obj.color.g = 0.0;
        marker_obj.color.b = 0.0;

      } else if (traj_point.longitudinal_velocity_mps < robot_max_speed_th) {
        marker_obj.scale.x = marker_obj.scale.y = marker_obj.scale.z = 0.2;
        marker_obj.color.a = 1.0;
        marker_obj.color.r = 0.0;
        marker_obj.color.g = 0.0;
        marker_obj.color.b = 1.0;
      } else {
        marker_obj.scale.x = marker_obj.scale.y = marker_obj.scale.z = 0.2;
        marker_obj.color.a = 1.0;
        marker_obj.color.r = 0.0;
        marker_obj.color.g = 1.0;
        marker_obj.color.b = 0.0;
      }

      // Set the pose of the marker_obj to the trajectory point pose
      marker_obj.pose = traj_point.pose;

      // Add the marker_obj to the marker_arr_msg
      marker_arr_msg.markers.push_back(marker_obj);
    }
    // Publish the marker_arr_msg
    velocity_marker_publisher.publish(marker_arr_msg);

    // Log a message indicating that the marker has been published
    ROS_INFO_THROTTLE(25, "Velocity Marker Published");
  } catch (const std::exception &e) {
    // Handle the exception
    ROS_ERROR("Exception caught in publish_velocity_marker: %s", e.what());
  }
}
} // namespace obstacle_stop_planner

int main(int argc, char **argv) {
  ros::init(argc, argv, "obstacle_stop_planner_node");
  ros::Time::init();
  ros::NodeHandle nh;
  obstacle_stop_planner::ObstacleStopPlanner planner(&nh);
  return 0;
}