#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/console.h>
#include <diagnostic_updater/publisher.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <pilot_msgs/vehicle_stop_command.h>
#include <ctime>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include "DouglasPeucker.h"
#include <iostream>

using namespace std;
class CurveDetectorCPP
{
	ros::NodeHandle cd;

private:
	ros::Publisher is_curve_publisher;
	ros::Subscriber gps_path_sub, vehicle_odom_sub;
	std::vector<RDP::Point2d> points;

public:
	double min_angle = M_PI * 0.05;
	int dist_from_curve_point_th = 5;
	float rdp_tolerance = 2.0;

	std::vector<std::vector<double>> directions;
	std::vector<std::vector<double>> curve_points;

	CurveDetectorCPP() // constructor
	{
		is_curve_publisher = cd.advertise<std_msgs::Bool>("/global_gps_path/is_curve", 1, this);
		gps_path_sub = cd.subscribe("/global_gps_path", 1, &CurveDetectorCPP::gps_path_cb, this);
		vehicle_odom_sub = cd.subscribe("/vehicle/odom", 1, &CurveDetectorCPP::odom_cb, this);
	}

	void gps_path_cb(const nav_msgs::Path &data)
	{
		for (const auto &pose : data.poses)
		{
			// std::string str_value = std::to_string(pose.pose.position.x);
			// int decimal_digits = str_value.size() - str_value.find('.') - 1;
			// std::cout << "Number of decimal digits pose.x: " << decimal_digits << std::endl;
			points.push_back({pose.pose.position.x, pose.pose.position.y});
		}
		calculate_curve_points(points);
	}

	int calculate_curve_points(std::vector<RDP::Point2d> vertices1)
	{

		std::vector<RDP::Point2d> simplified = calculate_rdp_line(vertices1);

		// To access rdp_line
		// std::cout.precision(15);

		// for (auto p : simplified)
		// {

		// 	std::cout << p.x_ << " " << p.y_ << "\n";
		// }

		for (int i = 0; i < simplified.size() - 1; i++)
		{
			std::vector<double> direction;
			direction.push_back(simplified[i + 1].x_ - simplified[i].x_);
			direction.push_back(simplified[i + 1].y_ - simplified[i].y_);
			directions.push_back(direction);
		}
		std::vector<double> theta = angle(directions);
		std::cout << "Theta size: 	" << theta.size() << "\n";
		// Select the index of the points with the greatest theta
		// Large theta is associated with greatest change in direction.
		std::vector<int> idx;
		std::cout << "min angle " << min_angle << "\n";
		for (int i = 0; i < theta.size(); i++)
		{
			// std::cout << "printing theta " << theta[i] << "\n";
			if (theta[i] > min_angle)
				idx.push_back(i + 1);
		}

		std::cout << "idx size: 	" << idx.size() << "\n";

		// std::cout << "\nSize of Simplified vector: " << simplified.size();
		// std::cout << "\nIndices of turning points: " << idx.size();

		// std::cout << "\nTurning Points : ";

		for (int s = 0; s < idx.size(); s++)
		{
			// std::cout <<"IN the loop" << "\n";
			std::cout << simplified[idx[s]].x_ << "," << simplified[idx[s]].x_ << "\n";
			curve_points.push_back({simplified[idx[s]].x_, simplified[idx[s]].y_});
			// curve_points.push_back(simplified[idx[s]].y_);
			// std::cout << "[" << simplified[idx[s]].x_ << "," << simplified[idx[s]].y_ << "],";
		}

		// 	for (const auto& point : curve_points) {
		//     std::cout << "(" << point[0] << ", " << point[1] << ")" << std::endl;
		// }

		return 0;
	}

	std::pair<std::vector<double>, double> findClosestPoint(const std::vector<std::vector<double>> &points, const std::vector<double> &target)
	{
		double minDistance = std::numeric_limits<double>::max();
		std::vector<double> closestPoint;
		for (const auto &point : points)
		{
			double distance = std::sqrt(std::pow(target[0] - point[0], 2) + std::pow(target[1] - point[1], 2));
			if (distance < minDistance)
			{
				minDistance = distance;
				closestPoint = point;
			}
		}
		return std::make_pair(closestPoint, minDistance);
	}

	void odom_cb(const nav_msgs::Odometry &data)
	{
		double vehicle_x = data.pose.pose.position.x;
		double vehicle_y = data.pose.pose.position.y;

		std::vector<double> vehicle_xy = {vehicle_x, vehicle_y};
		auto [closest, distance] = findClosestPoint(curve_points, vehicle_xy);
		// std::cout << "Closest point is: (" << closest[0] << ", " << closest[1] << ") with distance "<< distance << std::endl;
		std_msgs::Bool msg;

		if (distance <= dist_from_curve_point_th)
		{
			ROS_INFO_ONCE("CURVE");
			msg.data = true;

			is_curve_publisher.publish(msg);
		}
		else
		{
			ROS_INFO_ONCE("Straigt line");
			msg.data = false;
			is_curve_publisher.publish(msg);
		}

		// cout << vehicle_x <<"," << vehicle_y << "\n";
	}

	std::vector<RDP::Point2d> calculate_rdp_line(std::vector<RDP::Point2d> vertices1)
	{

		std::vector<RDP::Point2d> simplified = RDP::DouglasPeucker::Simplify(vertices1, rdp_tolerance);

		// for (auto p:simplified){
		// 	std::cout << p.x_ << " " << p.y_ << "\n";

		// std::string str_value = std::to_string(p.x_);
		// 	int decimal_digits = str_value.size() - str_value.find('.') - 1;
		// 	std::cout << "Number of decimal digits: " << decimal_digits << std::endl;}

		return simplified;
	}

	// below is the 3d vector working code.
	// 	std::vector<double> angle(std::vector<std::vector<double>> &dir)
	// {
	//     int N = dir.size();
	//     std::vector<double> theta;
	//     for (int i = 0; i < N - 1; i++)
	//     {
	//         double dot = 0;
	//         std::vector<double> cross(dir[i].size());
	//         for (int j = 0; j < dir[i].size(); j++)
	//         {
	//             dot += dir[i][j] * dir[i + 1][j];
	//             cross[j] = dir[i][(j + 1) % dir[i].size()] * dir[i + 1][(j + 2) % dir[i].size()] - dir[i][(j + 2) % dir[i].size()] * dir[i + 1][(j + 1) % dir[i].size()];
	//         }
	//         theta.push_back(atan2(sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]), dot));
	//     }
	//     return theta;
	// }

	// 2d vectors as input - ours is 2d vectors.

	std::vector<double> angle(std::vector<std::vector<double>> &dir)
	{
		int N = dir.size();
		std::vector<double> theta;
		for (int i = 0; i < N - 1; i++)
		{
			double dot = 0;
			double norm1 = 0;
			double norm2 = 0;
			for (int j = 0; j < dir[i].size(); j++)
			{
				dot += dir[i][j] * dir[i + 1][j];
				norm1 += dir[i][j] * dir[i][j];
				norm2 += dir[i + 1][j] * dir[i + 1][j];
			}
			norm1 = sqrt(norm1);
			norm2 = sqrt(norm2);
			theta.push_back(acos(dot / (norm1 * norm2)));
		}
		return theta;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "curve_detector_cpp");
	ros::NodeHandle cd;
	CurveDetectorCPP CurveDetectorCPP;
	// while(ros::ok())
	// {
	//     sleep(0.1);
	//     ros::spinOnce();
	// }
	ros::spin(); // To reduce cpu load. Instead of using while loop. Spin will do the job
	return 0;
}