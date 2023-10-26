#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
using namespace std;
using namespace Eigen;


int main() {
    Eigen::Matrix2d matrix;
    Eigen::Vector3d line_point1 = {0,0,0};
    Eigen::Vector3d line_point2 = {10,10,0};
    double theta = M_PI/2;

    
    auto v1 = line_point2 - line_point1;
    cout<<v1<<endl;
    auto v2 = matrix.dot(v1);
    cout<<v2<<endl;
  
    
}
