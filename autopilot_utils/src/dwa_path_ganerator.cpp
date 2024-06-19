#include "autopilot_utils/dwa_path_ganerator.h"

namespace autopilot_utils {
DwaPathGenerator::DwaPathGenerator() {
  // std::cout << "Default constructor called." << std::endl;
  // You can initialize attributes here if needed
}
DwaPathGenerator::DwaPathGenerator(double wheel_base, double steering_angle_max,
                                   double steering_inc, double max_path_length,
                                   double path_resolution) {
  this->wheel_base = wheel_base;
  this->steering_angle_max = steering_angle_max;
  this->steering_inc = steering_inc;
  this->max_path_length = max_path_length;
  this->path_resolution = path_resolution;
  this->init = false;
  // this->path_from_origin = generate_paths({0, 0, 0});
}

void DwaPathGenerator::update_params(double wheel_base,
                                     double steering_angle_max,
                                     double steering_inc,
                                     double max_path_length,
                                     double path_resolution) {
  this->wheel_base = wheel_base;
  this->steering_angle_max = steering_angle_max;
  this->steering_inc = steering_inc;
  this->max_path_length = max_path_length;
  this->path_resolution = path_resolution;
  this->init = false;
  // this->path_from_origin = generate_paths({0, 0, 0});
}

void DwaPathGenerator::generate_paths(
    const std::vector<double> &robot_pose,
    std::vector<std::vector<std::vector<double>>> &paths) {

  // std::vector<std::vector<std::vector<double>>> paths;
  for (double i = steering_angle_max; i > -steering_angle_max - steering_inc;
       i -= steering_inc) {
    if (i == 0) {
      i = 0.01;
    }
    double steering_angle = i * M_PI / 180.0;
    double turn_angle = max_path_length / wheel_base * tan(steering_angle);
    double turn_radius = max_path_length / turn_angle;
    double icr_x = robot_pose[0] - sin(robot_pose[2]) * turn_radius;
    double icr_y = robot_pose[1] + cos(robot_pose[2]) * turn_radius;
    double arc_length = abs(turn_angle) * turn_radius;
    double number_of_points_on_arc = abs(ceil(arc_length / path_resolution));
    double inc_angle;
    if (number_of_points_on_arc == 0) {
      inc_angle = 0.01;
    } else {
      inc_angle = turn_angle / abs(number_of_points_on_arc);
    }
    vector<vector<double>> array1;
    for (int j = 0; j < number_of_points_on_arc; j++) {
      double angle = j * inc_angle;
      double x = icr_x + sin(robot_pose[2] + angle) * turn_radius;
      double y = icr_y - cos(robot_pose[2] + angle) * turn_radius;
      double theta = robot_pose[2] + angle;
      array1.push_back({x, y, theta});
    }
    paths.push_back(array1);
  }
}

std::vector<std::vector<std::vector<double>>>
DwaPathGenerator::get_path_from_origin() {
  return path_from_origin;
}

void DwaPathGenerator::transform_path(const std::vector<double> &robot_pose) {
  // Transform all the origin points by robot pose
}

void DwaPathGenerator::get_dwa_paths_marker_array(
    const vector<vector<vector<double>>> &dwa_paths,
    visualization_msgs::MarkerArray &marker_arr_msg, std::string target_frame) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = target_frame;
  marker.action = marker.DELETEALL;
  marker_arr_msg.markers.push_back(marker);
  int count = 0;

  for (std::vector<vector<double>> path : dwa_paths) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_frame;
    marker.type = marker.LINE_STRIP;
    marker.id = count;
    count++;
    marker.action = marker.ADD;
    marker.scale.x = 0.1;
    marker.pose.orientation.w = 1;
    marker.color.a = 0.3;
    marker.color.r = static_cast<double>(rand()) / RAND_MAX;
    marker.color.g = static_cast<double>(rand()) / RAND_MAX;
    marker.color.b = static_cast<double>(rand()) / RAND_MAX;
    for (std::vector<double> point : path) {
      geometry_msgs::Point pt;
      pt.x = point[0];
      pt.y = point[1];
      marker.points.push_back(pt);
    }
    marker_arr_msg.markers.push_back(marker);
  }
}
void get_dwa_paths_marker_array(
    const std::vector<std::vector<double>> &dwa_paths,
    visualization_msgs::MarkerArray &marker_arr_msg, std::string target_frame) {
  visualization_msgs::Marker marker_;
  marker_.header.frame_id = target_frame;
  marker_.action = marker_.DELETEALL;
  marker_arr_msg.markers.push_back(marker_);
  int count = 0;

  // for (std::vector<vector<double>> path : dwa_paths) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = target_frame;
  marker.type = marker.LINE_STRIP;
  marker.id = count;
  count++;
  marker.action = marker.ADD;
  marker.scale.x = 0.3;
  marker.pose.orientation.w = 1;
  marker.color.a = 1;
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 1;
  for (std::vector<double> point : dwa_paths) {
    geometry_msgs::Point pt;
    pt.x = point[0];
    pt.y = point[1];
    marker.points.push_back(pt);
  }
  marker_arr_msg.markers.push_back(marker);
  // }
}

}; // namespace autopilot_utils

// int main() {
//     double max_steer = 30;
//     DwaPathGenerator dwa_path_gen(2.5, max_steer, 2, 10, 1);

//     std::vector<std::vector<std::vector<double>>> paths =
//     dwa_path_gen.generate_paths({0, 0, 0}); cout<< "path len" <<paths.size();
//     // std::vector<double> path = paths[(paths.size() - 1) / 2];
//     // for (std::vector<double> point : path) {
//     //     std::cout << point[0] << " " << point[1] << std::endl;
//     // }
//     return 0;
// }