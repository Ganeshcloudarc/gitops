// #include <Sophus/se3.h>
#include <Eigen/Core>
#include<vector>
#include <iostream>
#include<algorithm>

using namespace std;
// Eigen::Vector3d original_point(1.0, 0.0, 0.0);
std::vector<int> input = {1, 2, 3, 4, 5};
std::vector<int> output(input.size());
Eigen::Vector3d transformed_point;
Sophus::SE3d se3_transform(Sophus::SO3d::rotX(1.0), Eigen::Vector3d(1.0, 2.0, 3.0));
Eigen::Vector3d original_point(1.0, 0.0, 0.0);
Eigen::Vector3d transformed_point = se3_transform * original_point;
 
int main()
{
std::transform(input.begin(), input.end(),output.begin(), [](int x) { return x * x;});
for (const int& value: output)
{
    std::cout<<value<<" ";
}
}