#ifndef FOOTPRINT_TRANSFORM_H
#define FOOTPRINT_TRANSFORM_H

#include <cmath>
#include <iostream>

#include <jsk_recognition_msgs/PolygonArray.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>

#include "autopilot_utils/pose_utils.h"
#include "autopilot_utils/tf_utils.h"
#include "vehicle_common/config_vehicle.h"

using namespace std;

namespace autopilot_utils {

vector<vector<double>> transform_footprint(geometry_msgs::Pose pose,
                                           DataVehicle data_vehicle_obj,
                                           bool to_polygon = false);
std::pair<vector<vector<double>>, geometry_msgs::PolygonStamped>
transform_footprint_circles(geometry_msgs::Pose pose,
                            DataVehicle data_vehicle_obj,
                            double width_offset = 0.0,
                            double length_offset = 0.0,
                            bool to_polygon = false);
std::pair<jsk_recognition_msgs::PolygonArray, std::vector<std::vector<double>>>
path_to_circles(nav_msgs::Path path, DataVehicle data_vehicle_obj,
                bool to_polygon = false);

} // namespace autopilot_utils

#endif