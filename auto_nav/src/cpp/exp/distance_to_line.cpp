#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

double distanceToLine(const Vector3d &P0, const Vector3d &L1,
                      const Vector3d &L2) {
  // Calculate the direction vector of the line
  Vector3d lineDir = (L2 - L1);

  // Calculate the vector from L1 to P0
  Vector3d v1 = (P0 - L1);

  // Calculate the cross product of v1 and lineDir
  Vector3d crossProduct = v1.cross(lineDir);

  // Calculate the magnitude (norm) of the cross product
  double distance = crossProduct.norm();

  // Calculate the magnitude of the line direction
  double lineMagnitude = lineDir.norm();

  // Ensure the denominator is not zero
  if (lineMagnitude < 1e-9) {
    return v1.norm(); // L1 and L2 are very close, so return the distance to L1
  }

  // Calculate the final distance
  distance /= lineMagnitude;

  return distance;
}

int main() {
  Eigen::Vector3d L1(0.0, 0.0, 0.0);
  Eigen::Vector3d P(0.0, 2.0, 0.0);
  Eigen::Vector3d p(1.0, 1.0, 0.0);

  double dist = distanceToLine(P0, L1, L2);

  std::cout << "Distance from point to line: " << dist << std::endl;

  return 0;
}
