#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

int main() {
    // Sample data (x and y values)
    VectorXd x(5);
    VectorXd y(5);
    x << 1, 2, 3, 4, 5;
    y << 1, 2, 3, 4, 5;

    // Create a matrix for the design matrix X
    MatrixXd X(5, 2);
    X.col(0) = VectorXd::Ones(5); // Intercept term
    X.col(1) = x;

    // Perform linear regression using the normal equation
    VectorXd theta = (X.transpose() * X).ldlt().solve(X.transpose() * y);

    // The coefficients of the linear regression model
    double intercept = theta(0);
    double slope = theta(1);

    std::cout << "Intercept: " << intercept << std::endl;
    std::cout << "Slope: " << slope << std::endl;

    return 0;
}
