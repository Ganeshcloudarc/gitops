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
using namespace std;
using namespace Eigen;

namespace auto_nav
{  

    

    double distanceBetweenPoses(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
    {
         return sqrt(pow(p1.position.x - p2.position.x, 2) + 
                        pow(p1.position.y - p2.position.y,2));
    }
    double distanceBetweenPoses(vector<double> p1, vector<double> p2)
    {
         return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2));
    }

    geometry_msgs::Quaternion get_quaternion_from_yaw(double yaw)
    {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);
    geometry_msgs::Quaternion quaternion_msg;
    quaternion_msg.x = quaternion.x();
    quaternion_msg.y = quaternion.y();
    quaternion_msg.z = quaternion.z();
    quaternion_msg.w = quaternion.w();
    return quaternion_msg;
    }

    Eigen::Vector3d unit_vect(double angle)
    {
        Eigen::Vector3d v{cos(angle), sin(angle), 0};
        return v;

    }

    // shift_vect(Eigen::Vector3d &v,double dis, double angle)
    // {
        
    // }
    geometry_msgs::Point VectorToPoint(Eigen::Vector3d &v)
    { 
        geometry_msgs::Point point;
        point.x = v.x();
        point.y = v.y();
        point.z = v.z();
        return point;

    }

     geometry_msgs::Point VectorToPoint(double v[])
    { 
        geometry_msgs::Point point;
        point.x = v[0];
        point.y = v[1];
        // point.z = v[2];
        return point;

    }




    double circumRadius(vector<double> p1 , vector<double>p2, vector<double> p3)
    {

        //     den = 2 * ((x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2))
        double den = 2* ((p2[0]-p1[0]) * (p3[1] - p2[1]) - (p2[1] -p1[1]) * (p3[0] - p2[0]));
        if(den == 0)
        { 
            cerr<<"Failed: points are either collinear or not distinct"<<endl; 
            return 1000;
        }
        // float num = ((((x2 - x1) ** 2) + ((y2 - y1) ** 2)) * (((x3 - x2) ** 2) + ((y3 - y2) ** 2)) * (((x1 - x3) ** 2) + ((y1 - y3) ** 2))) ** (0.5)
        double num = sqrt((pow(p2[0] - p1[0],2) + pow(p2[1] - p1[1], 2)) * (pow(p3[0] - p2[0], 2) + pow(p3[1] - p2[1], 2)) * (pow(p1[0] - p3[0], 2) + pow(p1[1] - p3[1], 2)));

        double cirum_radius = num / den;
        return cirum_radius;

    }

    vector<int> distancesToLine( std::vector<Eigen::Vector3d> points, double offset, const Vector3d& L1, const Vector3d& L2)
    {
        Vector3d lineDir = (L1 - L2);
        vector<int> inliers_inds;
        for (int i = 0; i < points.size();i ++)
        {
            auto point = points[i];
            Vector3d v1 = (point - L1);
            Vector3d crossProduct = v1.cross(lineDir);
            double distance = crossProduct.norm();
            double lineMagnitude = lineDir.norm();
            if (lineMagnitude < 1e-9) {
                distance = v1.norm(); // P1 and P2 are very close, so return the distance to P1
            }
            distance /= lineMagnitude;
            if (abs(distance) <= offset)
                inliers_inds.push_back(i);
        }
        return inliers_inds;

    }

     double distanceToLine(const Vector3d& P0, const Vector3d& P1, const Vector3d& P2) {
        // Calculate the direction vector of the line
        Vector3d lineDir = (P2 - P1);


        // Calculate the vector from P1 to P0
        Vector3d v1 = (P0 - P1);

        // Calculate the cross product of v1 and lineDir
        Vector3d crossProduct = v1.cross(lineDir);
        double dot_product  = lineDir.x() * v1.y() - lineDir.y() * v1.x();

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
        if (dot_product >= 0)
        {
            return distance;
        }
        else
        {
            return -distance;
        }

        // return dot_product 
}

std::pair<int, int> ganerateRandomPair(int max_limit)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(0, max_limit - 1);
    return std::make_pair(dist(gen), dist(gen));    
}

std::pair<double, double> leastSquareMethod(vector<Vector3d> points)
// void leastSquareMethod(vector<Vector3d> points)

{
    // MatrixXd A(points.size(), 3);
    // MatrixXd b(points.size(), 1);
    // Create a matrix for the design matrix X
    MatrixXd X(points.size(), 2);
    X.col(0) = VectorXd::Ones(points.size()); // Intercept term
    VectorXd y(points.size());
    // X.col(1) = x;


    for (int i = 0; i < points.size(); i++)
    {
        X(i,1) = points[i].x();
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

    // std::pair<double, double> intersct_point_to_line(std::vector<double> pose2d) {
    //     double perp_slope = -1 / m;
    //     double perp_c = pose2d[1] - perp_slope * pose2d[0];
    //     double x = (c - perp_c) / (perp_slope - m);
    //     double y = m * x + c;
    //     return std::make_pair(x, y);
    // }

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

// Line two_points_to_line(std::vector<double> p1, std::vector<double> p2) {
//     double m = (p2[1] - p1[1]) / (p2[0] - p1[0]);
//     double c = p2[1] - m * p2[0];
//     return Line(m, c);
// }


visualization_msgs::Marker eigenToMarker(vector<Eigen::Vector3d> points_vect, string frame_id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.type = marker.LINE_STRIP;
    marker.scale.x = 0.1;
    marker.pose.orientation.w = 1;
    marker.color.a = 1;
    marker.color.r = static_cast<double>(rand()) / RAND_MAX;
    marker.color.g = static_cast<double>(rand()) / RAND_MAX;
    marker.color.b = static_cast<double>(rand()) / RAND_MAX;
    for(int i =0; i<points_vect.size();i++)
    {
        geometry_msgs::Point pt;
        pt.x = points_vect[i][0];
        pt.y = points_vect[i][1];
        marker.points.push_back(pt);
           
    }
    return marker;
}

visualization_msgs::Marker xy_to_marker(double x, double y,int id, string frame_id="map" )
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.type = marker.CUBE;
    marker.id = id;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.pose.position.x = x;
    marker.pose.position.y =y;
    marker.pose.orientation.w = 1;
    marker.color.a = 1;
    marker.color.r = static_cast<double>(rand()) / RAND_MAX;
    marker.color.g = static_cast<double>(rand()) / RAND_MAX;
    marker.color.b = static_cast<double>(rand()) / RAND_MAX;
    
    return marker;
}

autopilot_msgs::Trajectory twoPointsToTrajectory(vector<double> p1, vector<double> p2, double res, std::string frame_id, double speed)
    {
    autopilot_msgs::Trajectory line_traj;
    line_traj.header.frame_id = frame_id;
    autopilot_msgs::TrajectoryPoint traj_point;
    // traj_point.header.frame_id = traj_point;
    
    double line_heading = atan2(p2[1] - p1[1] , p2[0] - p1[0]);
    double dis = sqrt(pow(p1[0] - p2[0], 2) + 
                        pow(p1[1] - p2[1],2));

    
    double itr  = 0;
    // cout<<"dis : "<<dis<<endl;
    while (itr<dis)
    {
    
    traj_point.pose.position.x =  p1[0] + cos(line_heading)*itr;
    traj_point.pose.position.y =  p1[1] + sin(line_heading)*itr;
    traj_point.pose.orientation = get_quaternion_from_yaw(line_heading);
    traj_point.longitudinal_velocity_mps = speed;
    traj_point.accumulated_distance_m = itr;
    line_traj.points.push_back(traj_point);
    
    itr = itr + res;
   
    }
    return line_traj;
}


nav_msgs::Path trajectoryToPath(autopilot_msgs::Trajectory & traj)
{
    nav_msgs::Path path;
    path.header.frame_id = traj.header.frame_id;
    geometry_msgs::PoseStamped pst;
    pst.header.frame_id = traj.header.frame_id;
    for (int i = 0; i< traj.points.size(); i++)
    {  pst.pose = traj.points[i].pose;
       path.poses.push_back(pst);
    }
    return path;
}

autopilot_msgs::TrajectoryPoint DubinsPointToTrajectoryPoint(double q[3], double dis, double speed)
{
 autopilot_msgs::TrajectoryPoint traj_point;
 traj_point.pose.position.x = q[0];
 traj_point.pose.position.y = q[1];
 traj_point.pose.orientation = get_quaternion_from_yaw(q[2]);
 traj_point.accumulated_distance_m = dis;
 traj_point.longitudinal_velocity_mps = speed;
 return traj_point;

}


}





