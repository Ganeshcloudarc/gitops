#include "autopilot_utils/footprint_transform.h"

namespace autopilot_utils {

vector<vector<double>> transform_footprint(geometry_msgs::Pose pose,
                                           DataVehicle data_vehicle_obj,
                                           bool to_polygon) {
  /*
  Args:
      pose: Pose to which vehicle footprint is to be transformed
      vehicle_data: Vehicle data (from vehicle_common.vehicle_config import
  vehicle_data) to_polygon: True returns footprint as PolygonStamped, else
  returns a numpy array Returns: center of circles type: List or PolygonStamped
  */

  // Get vehicle dimensions
  double rear_overhang = data_vehicle_obj.rear_overhang;
  double overall_width = data_vehicle_obj.overall_width;
  double wheel_base = data_vehicle_obj.wheel_base;
  double front_overhang = data_vehicle_obj.front_overhang;

  // Print vehicle dimensions
  cout << "The Vehicle Dimension Rear Overhang: " << rear_overhang << endl;

  // Define vehicle footprint coordinates
  vector<vector<double>> foot_print_specs = {
      {-rear_overhang, -overall_width / 2},
      {wheel_base + front_overhang, -overall_width / 2},
      {wheel_base + front_overhang, overall_width / 2},
      {-rear_overhang, overall_width / 2}};

  // Get vehicle orientation
  double theta = get_yaw(pose.orientation);

  // Calculate cos and sin of orientation
  double cos_th = cos(theta);
  double sin_th = sin(theta);

  // Define array to store transformed coordinates
  vector<vector<double>> arr;

  // Transform each coordinate and add to array
  for (auto foot : foot_print_specs) {
    double x = pose.position.x + (foot[0] * cos_th - foot[1] * sin_th);
    double y = pose.position.y + (foot[0] * sin_th + foot[1] * cos_th);
    arr.push_back({x, y});
  }

  // If to_polygon is true, return polygon
  if (to_polygon) {
    geometry_msgs::PolygonStamped polygon_st;
    polygon_st.header.frame_id = "map";
    for (auto coors : arr) {
      geometry_msgs::Point32 point;
      point.x = coors[0];
      point.y = coors[1];
      polygon_st.polygon.points.push_back(point);
    }
    return arr;
  }
  // Otherwise, return vector of vectors
  else {
    return arr;
  }
}

std::pair<vector<vector<double>>, geometry_msgs::PolygonStamped>
transform_footprint_circles(geometry_msgs::Pose pose,
                            DataVehicle data_vehicle_obj, double width_offset,
                            double length_offset, bool to_polygon) {
  /*
  Args:
      pose: Pose to which the vehicle footprint is to be transformed.
      vehicle_data: Vehicle data (from vehicle_common.vehicle_config import
  vehicle_data). to_polygon: If true, returns the footprint as a PolygonMsg;
  otherwise, returns a numpy array. width_offset: Offset to add to the width of
  the vehicle. length_offset: Offset to add to the length of the vehicle.

  Returns:
      center_centers: List of center coordinates.
      polygon_msg: Message representing the footprint (PolygonMsg).
  */

  double vehicle_length = data_vehicle_obj.overall_length + length_offset;
  double vehicle_width = data_vehicle_obj.overall_width + width_offset;

  // Calculate the forward limit of the footprint
  double foot_print_forward_lim =
      vehicle_length - data_vehicle_obj.rear_overhang;
  // Calculate the offsets for each circle
  vector<double> circle_offsets;
  for (double offset = -data_vehicle_obj.rear_overhang;
       offset < foot_print_forward_lim; offset += vehicle_width / 2) {
    circle_offsets.push_back(offset);
  }

  // Exclude the first element as in Python
  circle_offsets.erase(circle_offsets.begin());

  /*
 Example: Suppose the values are like np.arange(-0.633, 4.883, 2.025/2)[1:]
 python Answer [0.3795 1.392  2.4045 3.417  4.4295]
 cpp Answer    0.3795 1.392 2.4045 3.417 4.4295
 */

  // Calculate the yaw angle of the pose
  double yaw = get_yaw(pose.orientation);

  // Calculate the centers of the circles
  vector<vector<double>> circles_centers;
  for (auto offset : circle_offsets) {
    double x_value = pose.position.x + offset * cos(yaw);
    double y_value = pose.position.y + offset * sin(yaw);
    circles_centers.push_back({x_value, y_value});
  }

  if (to_polygon) {
    // Create a polygon message
    geometry_msgs::PolygonStamped polygon_st;
    polygon_st.header.frame_id = "map";

    // Calculate angles for the circumference points
    std::vector<double> theta(100);
    double step = 2 * M_PI / (theta.size() - 1);
    for (int i = 0; i < theta.size(); ++i) {
      theta[i] = i * step;
    }

    // Calculate radius of the circles
    double radius = vehicle_width / 2;

    // Generate the circumference points for each circle
    for (const auto &center : circles_centers) {
      double x_val = center[0];
      double y_val = center[1];

      // Calculate the circumference points
      vector<double> circum_points_x;
      vector<double> circum_points_y;
      for (double angle : theta) {
        circum_points_x.push_back(x_val + radius * cos(angle));
        circum_points_y.push_back(y_val + radius * sin(angle));
      }

      // Add the circumference points to the polygon message
      for (int i = 0; i < circum_points_x.size(); i++) {
        geometry_msgs::Point32 point;
        point.x = circum_points_x[i];
        point.y = circum_points_y[i];
        polygon_st.polygon.points.push_back(point);
      }
    }

    return std::make_pair(circles_centers, polygon_st);
  } else {
    return std::make_pair(circles_centers, geometry_msgs::PolygonStamped());
  }
}

std::pair<jsk_recognition_msgs::PolygonArray, std::vector<std::vector<double>>>
path_to_circles(nav_msgs::Path path, DataVehicle data_vehicle_obj,
                bool to_polygon) {
  /*
      Function: Return circle centers based on the vehicle footprint and path.
      Args:
          path: Path of the vehicle.
          vehicle_data: Vehicle data (from vehicle_common.vehicle_config import
     vehicle_data). to_polygon: If true, returns the footprint as a
     PolygonArray; otherwise, returns a list. Returns: List of circle centers
     along the path with footprint coverage. (List/ PolygonArray)
  */

  // Create a PolygonArray object
  jsk_recognition_msgs::PolygonArray polygon_arr;
  polygon_arr.header.frame_id = "map";

  // Create a list of circle centers
  vector<vector<double>> circle_list;

  // Iterate over each pose in the path
  for (auto pose_st : path.poses) {
    // Get the pose
    geometry_msgs::Pose pose = pose_st.pose;

    // Transform the footprint circles
    std::pair<std::vector<std::vector<double>>, geometry_msgs::PolygonStamped>
        circles =
            transform_footprint_circles(pose, data_vehicle_obj, to_polygon);

    if (to_polygon) {
      // Add the transformed circles to the PolygonArray
      // circles.second is used as the return type is a pair and second is of
      // type polygon
      polygon_arr.polygons.push_back(circles.second);
    } else {
      // Add the transformed circles to the list
      circle_list.insert(circle_list.end(), circles.first.begin(),
                         circles.first.end());
    }
  }

  // Return either the PolygonArray or the list of circle centers based on
  // to_polygon
  return {polygon_arr, circle_list};
}

} // End of namespace autopilot_utils
