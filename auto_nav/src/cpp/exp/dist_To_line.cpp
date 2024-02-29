#include <Eigen/Dense>
#include <iostream>

int main() {
  // Define the point
  Eigen::Vector3d point(-1.0, -2.0, 3.0);

  // Define two points on the line
  Eigen::Vector3d linePoint1(2.0, 2.0, 0.0);
  Eigen::Vector3d linePoint2(3.0, 3.0, 0.0);

  // Calculate the vector from linePoint1 to the point
  Eigen::Vector3d lineToP = point - linePoint1;

  // Calculate the direction vector of the line
  Eigen::Vector3d lineDirection = (linePoint2 - linePoint1).normalized();

  // Calculate the distance from the point to the line
  double distance =
      (lineToP - lineToP.dot(lineDirection) * lineDirection).norm();

  // Output the distance
  std::cout << "Distance from the point to the line: " << distance << std::endl;

  return 0;
}

// In this code, we define the point and two points on the line. We calculate
// the vector from linePoint1 to the point and the direction vector of the line.
// The distance is then calculated by projecting the vector from the point to
// the line onto the direction vector of the line and finding the norm of the
// resulting vector.
