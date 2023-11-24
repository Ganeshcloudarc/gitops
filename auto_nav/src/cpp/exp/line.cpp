#include <iostream>
#include <cmath>
#include <vector>
#include<string>
#include <sstream>
#include<algorithm>
#include <functional> // std::minus, std::divides
#include <numeric> // std::inner_product

using namespace std;
/*
class Pose2D {
public:
    double x;
    double y;
    double yaw;
    double circum_radius;
    int turn_info;

public:
    Pose2D(double x, double y, double yaw=0, double circum_radius=std::numeric_limits<double>::infinity(), int turn_info=0) {
        this->x = x;
        this->y = y;
        this->yaw = yaw;
        this->circum_radius = circum_radius;
        this->turn_info = turn_info;
    }

    std::string toString() {
        std::ostringstream oss;
        oss << "x: " << x << " y: " << y << " yaw: " << yaw * 180 / M_PI << " circum_radius: " << circum_radius << " turn_info: " << turn_info;
        return oss.str();
    }

    Pose2D operator+(const Pose2D& other) {
        double newX = x + other.x;
        double newY = y + other.y;
        return Pose2D(newX, newY);
    }

    // geometry_msgs::PoseStamped toPoseStamped(const std::string& target_frame) {
    //     geometry_msgs::PoseStamped pose;
    //     pose.header.frame_id = target_frame;
    //     pose.pose.position.x = x;
    //     pose.pose.position.y = y;
    //     pose.pose.orientation = yawToQuaternion(yaw);
    //     return pose;
    // }

    double distance(const Pose2D& other) {
        return hypot(x - other.x, y - other.y);
    }

    void updateCircumRadius(double circum_radius) {
        this->circum_radius = circum_radius;
    }

    void updateTurnInfo(int turn_info) {
        // +1 => right turn
        // -1 negative turn
        // 0 No turn
        this->turn_info = turn_info;
    }

    std::vector<double> toVector() {
        return {x, y};
    }

    std::vector<double> toList() {
        return {x, y, yaw};
    }

    bool dirCheck(const Pose2D& other) {
        // Return true if both poses lie on the same side
        std::vector<double> v1 = {cos(yaw), sin(yaw)};
        std::vector<double> v2 = {other.x - x, other.y - y};
        double dotProduct = std::inner_product(v1.begin(), v1.end(), v2.begin(), 0.0);
        if (dotProduct >= 0) {
            return true;
        } else {
            return false;
        }
    }

    bool headingCheck(const Pose2D& other, double th=M_PI/2) {
        // Dot product between the yaw
        std::vector<double> v1 = {cos(yaw), sin(yaw)};
        std::vector<double> v2 = {cos(other.yaw), sin(other.yaw)};
        // double dotProduct = std::inner_product(v1.begin(), v1.end(), v2.begin(), 0.0);
        double angle = getAngle(v1, v2);
        if (abs(angle) < M_PI/2) {
            return true;
        }
        return false;
    }

private:
    double getAngle(const std::vector<double>& v1, const std::vector<double>& v2) {
        double dotProduct = std::inner_product(v1.begin(), v1.end(), v2.begin(), 0.0);
        double magnitudeV1 = sqrt(std::inner_product(v1.begin(), v1.end(), v1.begin(), 0.0));
        double magnitudeV2 = sqrt(std::inner_product(v2.begin(), v2.end(), v2.begin(), 0.0));
        double cosTheta = dotProduct / (magnitudeV1 * magnitudeV2);
        return acos(cosTheta);
    }

    // geometry_msgs::Quaternion yawToQuaternion(double yaw) {
    //     geometry_msgs::Quaternion quaternion;
    //     quaternion.x = 0;
    //     quaternion.y = 0;
    //     quaternion.z = sin(yaw / 2);
    //     quaternion.w = cos(yaw / 2);
    //     return quaternion;
    // }
};
*/


class Line {
private:
    double m;
    double c;

public:
    Line(double slope, double constant) {
        m = slope;
        c = constant;
    }

    double getSlope(){return m;}
    double getConstant(){return c;}

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

    std::pair<double, double> intersct_point_to_line(std::vector<double> pose2d) {
        double perp_slope = -1 / m;
        double perp_c = pose2d[1] - perp_slope * pose2d[0];
        double x = (c - perp_c) / (perp_slope - m);
        double y = m * x + c;
        return std::make_pair(x, y);
    }

    double distance_to_point(std::vector<double> point) {
        cout<< point[0]<< " " <<point[1]<<endl;
        double dino = sqrt(m * m + 1);
        double perp_dis = (m * point[0] - point[1] + c) / dino;
        return perp_dis;

        //  dino = pow(self.m * self.m + (-1 * -1), 0.5)
        //     perp_dis = (self.m * (point[0]) - point[1] + self.c) / dino
        //     return perp_dis
    }
    std::pair<double, double> intersct_point_to_line(double x,double y ) {
        double perp_slope = -1 / m;
        double perp_c = y - perp_slope * x;
        double x1 = (c - perp_c) / (perp_slope - m);
        double y1 = m * x1 + c;
        return std::make_pair(x1, y1);
    }


    // double distance_to_point(Pose2D point) {
    //     double dino = sqrt(m * m + (-1 * -1));
    //     double perp_dis = (m * point.x - point.y + c) / dino;
    //     return perp_dis;
    // }

    double heading() {
        std::vector<double> p1 = {0, m * 0 + c};
        std::vector<double> p2 = {1, m * 1 + c};
        double y_diff = p2[1] - p1[1];
        double x_diff = p2[0] - p1[0];
        return atan2(y_diff, x_diff);
    }

    // std::vector<int> inlier(std::vector<std::vector<double>> points, double offset) {
    //     double dino = sqrt(m * m + (-1 * -1));
    //     std::vector<int> left_points_ind;
    //     for (int i = 0; i < points.size(); i++) {
    //         double numerator = (m * points[i][0] - points[i][1] + c);
    //         double dir_perpendicular_dis = numerator / dino;
    //         if (abs(dir_perpendicular_dis) <= offset) {
    //             left_points_ind.push_back(i);
    //         }
    //     }
    //     return left_points_ind;
    // }

    // std::vector<int> inliers_cross_product(std::vector<std::vector<double>> cloud_points, double offset) {
    //     std::vector<double> p1 = {0, m * 0 + c};
    //     std::vector<double> p2 = {1, m * 1 + c};
    //     std::vector<int> points_ind;
    //     for (int i = 0; i < cloud_points.size(); i++) {
    //         std::vector<double> p = cloud_points[i];
    //         double dir_perpendicular_dis = (p2[0] - p1[0]) * (p1[1] - p[1]) - (p1[0] - p[0]) * (p2[1] - p1[1]);
    //         dir_perpendicular_dis /= sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2));
    //         if (abs(dir_perpendicular_dis) <= offset) {
    //             points_ind.push_back(i);
    //         }
    //     }
    //     return points_ind;
    // }

    Line shift_line(double distance) {
        double c2 = c + distance * sqrt(1 + m * m);
        return Line(m, c2);
    }
};

Line two_points_to_line(std::vector<double> p1, std::vector<double> p2) {
    double m = (p2[1] - p1[1]) / (p2[0] - p1[0]);
    double c = p2[1] - m * p2[0];
    return Line(m, c);
}


int main()
{

    Line line(1,0);
    //  Line line1(0,3);
    // cout<<line.toString()<<endl;
    // cout<<line.intersect(line1).first<<endl;
    // cout<<line.intersect(line1).second<<endl;
    vector<double> b;
    b = {-1,3};
    cout<<"b val : "<<b[0]<< " sr : "<<b[1]<<endl;
    auto c = line.intersct_point_to_line(b);
    cout<<"c :"<<c.first<<" " <<c.second<<endl;

    auto a = line.intersct_point_to_line(-1,3);
    cout<<"a :"<<a.first<<" " <<a.second<<endl;

}