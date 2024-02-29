#include <Eigen/Dense>
#include <iostream>

int main() {
  // Define a 3x3 matrix
  Eigen::MatrixXd matrix(30, 30, 3);
  // matrix << 1, 2, 3,
  //           4, 5, 6,
  //           7, 8, 9;
  // matrix(0,0,0) = 1;
  std::cout << matrix;

  // Compute eigenvalues and eigenvectors
  Eigen::EigenSolver<Eigen::MatrixXd> solver(matrix);

  // Get the eigenvalues
  Eigen::VectorXd eigenvalues = solver.eigenvalues().real();

  // Get the eigenvectors
  Eigen::MatrixXd eigenvectors = solver.eigenvectors().real();

  // Print the eigenvalues and eigenvectors
  std::cout << "Eigenvalues:\n" << eigenvalues << std::endl;
  std::cout << "Eigenvectors:\n" << eigenvectors << std::endl;

  return 0;
}
