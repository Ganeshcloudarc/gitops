#include "autopilot_utils/autonav_utils.h"

namespace autopilot_utils {

geometry_msgs::Point VectorToPoint(const Eigen::Vector3d &v) {
  geometry_msgs::Point point;
  point.x = v.x();
  point.y = v.y();
  point.z = v.z();
  return point;
}

geometry_msgs::Point VectorToPoint(const double v[]) {
  geometry_msgs::Point point;
  point.x = v[0];
  point.y = v[1];
  // point.z = v[2];
  return point;
}

double circumRadius(const vector<double> &p1, const vector<double> &p2,
                    const vector<double> &p3) {

  //     den = 2 * ((x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2))
  double den = 2 * ((p2[0] - p1[0]) * (p3[1] - p2[1]) -
                    (p2[1] - p1[1]) * (p3[0] - p2[0]));
  if (den == 0) {
    cerr << "Failed: points are either collinear or not distinct" << endl;
    return 1000;
  }
  // float num = ((((x2 - x1) ** 2) + ((y2 - y1) ** 2)) * (((x3 - x2) ** 2) +
  // ((y3 - y2) ** 2)) * (((x1 - x3) ** 2) + ((y1 - y3) ** 2))) ** (0.5)
  double num = sqrt((pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2)) *
                    (pow(p3[0] - p2[0], 2) + pow(p3[1] - p2[1], 2)) *
                    (pow(p1[0] - p3[0], 2) + pow(p1[1] - p3[1], 2)));
  double cirum_radius = num / den;
  return cirum_radius;
}

vector<int> distancesToLine(const vector<Vector3d> &points, double offset,
                            const Vector3d &L1, const Vector3d &L2) {
  Vector3d lineDir = L1 - L2;
  double lineMagnitude = lineDir.norm();

  if (lineMagnitude < 1e-9) {
    cerr << "Failed: Line direction magnitude is too small" << endl;
    return vector<int>();
  }

  vector<int> inliers_inds;
  for (int i = 0; i < points.size(); ++i) {
    const Vector3d &point = points[i];
    Vector3d v1 = point - L1;
    double distance = (v1.cross(lineDir)).norm() / lineMagnitude;
    // cout<<"distance :" <<distance;
    if (distance <= offset)
      inliers_inds.push_back(i);
  }

  return inliers_inds;
}

double distanceToLine(const Vector3d &P0, const Vector3d &P1,
                      const Vector3d &P2) {
  // Calculate the direction vector of the line
  Vector3d lineDir = (P2 - P1);

  // Calculate the vector from P1 to P0
  Vector3d v1 = (P0 - P1);

  // Calculate the cross product of v1 and lineDir
  Vector3d crossProduct = v1.cross(lineDir);
  double dot_product = lineDir.x() * v1.y() - lineDir.y() * v1.x();

  // Calculate the magnitude (norm) of the cross product
  double distance = crossProduct.norm();

  // Calculate the magnitude of the line direction
  double lineMagnitude = lineDir.norm();

  // Ensure the denominator is not zero
  if (lineMagnitude < 1e-9) {
    return v1.norm(); // P1 and P2 are very close, so return the distance to P1
  }

  // Calculate the final distance
  distance /= lineMagnitude;
  if (dot_product >= 0) {
    return distance;
  } else {
    return -distance;
  }

  // return dot_product
}

std::pair<int, int> ganerateRandomPair(int max_limit) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dist(0, max_limit - 1);
  return std::make_pair(dist(gen), dist(gen));
}

std::pair<double, double> leastSquareMethod(const vector<Vector3d> &points)
// void leastSquareMethod(vector<Vector3d> points)

{
  // MatrixXd A(points.size(), 3);
  // MatrixXd b(points.size(), 1);
  // Create a matrix for the design matrix X
  MatrixXd X(points.size(), 2);
  X.col(0) = VectorXd::Ones(points.size()); // Intercept term
  VectorXd y(points.size());
  // X.col(1) = x;

  for (int i = 0; i < points.size(); i++) {
    X(i, 1) = points[i].x();
    y(i) = points[i].y();
  }
  // Perform linear regression using the normal equation
  VectorXd theta = (X.transpose() * X).ldlt().solve(X.transpose() * y);
  // The coefficients of the linear regression model
  double intercept = theta(0);
  double slope = theta(1);

  // std::cout << "Intercept: " << intercept << std::endl;
  // std::cout << "Slope: " << slope << std::endl;
  return std::make_pair(slope, intercept);
}

// Line two_points_to_line(std::vector<double> p1, std::vector<double> p2) {
//     double m = (p2[1] - p1[1]) / (p2[0] - p1[0]);
//     double c = p2[1] - m * p2[0];
//     return Line(m, c);
// }

visualization_msgs::Marker
eigenToMarker(const vector<Eigen::Vector3d> &points_vect, string frame_id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.type = marker.LINE_STRIP;
  marker.scale.x = 0.1;
  marker.pose.orientation.w = 1;
  marker.color.a = 1;
  marker.color.r = static_cast<double>(rand()) / RAND_MAX;
  marker.color.g = static_cast<double>(rand()) / RAND_MAX;
  marker.color.b = static_cast<double>(rand()) / RAND_MAX;
  for (int i = 0; i < points_vect.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = points_vect[i][0];
    pt.y = points_vect[i][1];
    marker.points.push_back(pt);
  }
  return marker;
}

visualization_msgs::Marker xy_to_marker(double x, double y, int id,
                                        string frame_id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.type = marker.CUBE;
  marker.id = id;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.orientation.w = 1;
  marker.color.a = 1;
  marker.color.r = static_cast<double>(rand()) / RAND_MAX;
  marker.color.g = static_cast<double>(rand()) / RAND_MAX;
  marker.color.b = static_cast<double>(rand()) / RAND_MAX;

  return marker;
}

autopilot_msgs::Trajectory
twoPointsToTrajectory(const vector<double> &p1, const vector<double> &p2,
                      double res, std::string frame_id, double speed) {
  autopilot_msgs::Trajectory line_traj;
  line_traj.header.frame_id = frame_id;
  autopilot_msgs::TrajectoryPoint traj_point;
  // traj_point.header.frame_id = traj_point;

  double line_heading = atan2(p2[1] - p1[1], p2[0] - p1[0]);
  double dis = sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2));

  double itr = 0;
  // cout<<"dis : "<<dis<<endl;
  while (itr < dis) {

    traj_point.pose.position.x = p1[0] + cos(line_heading) * itr;
    traj_point.pose.position.y = p1[1] + sin(line_heading) * itr;
    traj_point.pose.orientation = get_quaternion_from_yaw(line_heading);
    traj_point.longitudinal_velocity_mps = speed;
    traj_point.accumulated_distance_m = itr;
    line_traj.points.push_back(traj_point);

    itr = itr + res;
  }
  return line_traj;
}

nav_msgs::Path trajectoryToPath(const autopilot_msgs::Trajectory &traj) {
  nav_msgs::Path path;
  path.header.frame_id = traj.header.frame_id;
  geometry_msgs::PoseStamped pst;
  pst.header.frame_id = traj.header.frame_id;
  for (int i = 0; i < traj.points.size(); i++) {
    pst.pose = traj.points[i].pose;
    path.poses.push_back(pst);
  }
  return path;
}

autopilot_msgs::TrajectoryPoint
DubinsPointToTrajectoryPoint(double q[3], double dis, double speed) {
  autopilot_msgs::TrajectoryPoint traj_point;
  traj_point.pose.position.x = q[0];
  traj_point.pose.position.y = q[1];
  traj_point.pose.orientation = get_quaternion_from_yaw(q[2]);
  traj_point.accumulated_distance_m = dis;
  traj_point.longitudinal_velocity_mps = speed;
  return traj_point;
}

} // namespace autopilot_utils
