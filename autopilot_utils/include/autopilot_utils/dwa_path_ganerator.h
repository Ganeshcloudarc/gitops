#ifndef DWA_PATH_GANERATOR_H
#define DWA_PATH_GANERATOR_H
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include<geometry_msgs/Point.h>

int sign(double x)
{
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);

}


namespace autopilot_utils
{   using namespace std;
    class DwaPathGenerator {
            private:
                bool init;
                double wheel_base;
                double steering_angle_max;
                double steering_inc;
                double max_path_length;
                double path_resolution;
                std::vector<std::vector<std::vector<double>>> path_from_origin;

            public:
                DwaPathGenerator() ;
                DwaPathGenerator(double wheel_base, double steering_angle_max, double steering_inc, double max_path_length, double path_resolution);
                void update_params(double wheel_base, double steering_angle_max, double steering_inc, double max_path_length, double path_resolution);
                std::vector<std::vector<std::vector<double>>> generate_paths(std::vector<double> robot_pose);
                std::vector<std::vector<std::vector<double>>> get_path_from_origin();
                void transform_path(std::vector<double> robot_pose);
                visualization_msgs::MarkerArray get_dwa_paths_marker_array(std::vector<std::vector<std::vector<double>>> dwa_paths, std::string target_frame = "base_link") ;
                visualization_msgs::MarkerArray get_dwa_paths_marker_array(std::vector<std::vector<double>> dwa_paths, std::string target_frame = "base_link");


              
    };
}


#endif  