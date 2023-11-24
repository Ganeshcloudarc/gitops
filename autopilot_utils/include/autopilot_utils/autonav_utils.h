#ifndef AUTONAV_UTILS_H
#define AUTONAV_UTILS_H
#include <iostream>
#include <vector>
#include<cmath>
#include<random>
#include<visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include<geometry_msgs/Point.h>
#include <autopilot_msgs/Trajectory.h>
#include <autopilot_msgs/TrajectoryPoint.h>
#include <geometry_msgs/Pose.h>
#include<nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include<autopilot_utils/pose_utils.h>

using namespace std;
using namespace Eigen;
namespace autopilot_utils
{
    geometry_msgs::Point VectorToPoint(Eigen::Vector3d &v);
    geometry_msgs::Point VectorToPoint(double v[]);

    double circumRadius(vector<double> p1 , vector<double>p2, vector<double> p3);
    vector<int> distancesToLine( std::vector<Eigen::Vector3d> points, double offset, const Vector3d& L1, const Vector3d& L2);
    double distanceToLine(const Vector3d& P0, const Vector3d& P1, const Vector3d& P2);
    std::pair<int, int> ganerateRandomPair(int max_limit);
    std::pair<double, double> leastSquareMethod(vector<Vector3d> points);
    visualization_msgs::Marker eigenToMarker(vector<Eigen::Vector3d> points_vect, string frame_id);
    visualization_msgs::Marker xy_to_marker(double x, double y,int id, string frame_id="map" );
    autopilot_msgs::Trajectory twoPointsToTrajectory(vector<double> p1, vector<double> p2, double res, std::string frame_id, double speed);
    nav_msgs::Path trajectoryToPath(autopilot_msgs::Trajectory & traj);
    autopilot_msgs::TrajectoryPoint DubinsPointToTrajectoryPoint(double q[3], double dis, double speed);

class Line {
private:
    double m;
    double c;
    double heading_angle;

public:
    Line(){}
    Line(double slope, double constant) {
        m = slope;
        c = constant;
    }
    bool valid()
    {
        if(m and c and heading_angle)
        {
            return true;
        }
        else return false;
    }
    void updateHeading(double h)
        {
        heading_angle = h;
        }
    double getHeadingAngle(){ return heading_angle;}

    double getSlope(){return m;}
    double getConstant(){return c;}

    void update_line(double slope, double constant)
    {
        m = slope;
        c = constant;
    }

    std::string toString() {
        return "y = " + std::to_string(m) + "*x + " + std::to_string(c);
    }

    std::pair<double, double> intersect(Line l2) {
        if (m == l2.m) {
            return std::make_pair(INFINITY, INFINITY);
        }
        double x = (c - l2.c) / (l2.m - m);
        double y = m * x + c;
        return std::make_pair(x, y);
    }

    geometry_msgs::Point intersct_point_to_line( geometry_msgs::Point point) {
        double perp_slope = -1 / m;
        double perp_c = point.y - perp_slope * point.x;
        geometry_msgs::Point point_out;
        point_out.x = (c - perp_c) / (perp_slope - m);
        point_out.y = m *  point_out.x + c;
        return point_out;
    }

      std::pair<double, double> intersct_point_to_line(double x,double y ) {
        double perp_slope = -1 / m;
        double perp_c = y - perp_slope * x;
        double x1 = (c - perp_c) / (perp_slope - m);
        double y1 = m * x1 + c;
        return std::make_pair(x1, y1);
    }
    vector<double> intersct_point_to_line(std::vector<double> v ) {
        
        double perp_slope = -1 / m;
        double perp_c = v[1] - perp_slope * v[0];
        double x1 = (c - perp_c) / (perp_slope - m);
        double y1 = m * x1 + c;
        return std::vector<double> {x1,y1};
    }


    double distance_to_point(std::vector<double> point) {
        // cout<< point[0]<< " " <<point[1]<<endl;
        double dino = sqrt(m * m + 1);
        double perp_dis = (m * point[0] - point[1] + c) / dino;
        return perp_dis;
    }


    double heading() {
        std::vector<double> p1 = {0, m * 0 + c};
        std::vector<double> p2 = {1, m * 1 + c};
        double y_diff = p2[1] - p1[1];
        double x_diff = p2[0] - p1[0];
        return atan2(y_diff, x_diff);
    }

    Line shift_line(double distance) {
        double c2 = c + distance * sqrt(1 + m * m);
        return Line(m, c2);
    }
};

}
#endif  