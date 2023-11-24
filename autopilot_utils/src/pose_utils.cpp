#include "autopilot_utils/pose_utils.h"
namespace autopilot_utils
{

    double distanceBetweenPoses(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
    {
         return sqrt(pow(p1.position.x - p2.position.x, 2) + 
                        pow(p1.position.y - p2.position.y,2));
    }
    double distanceBetweenPoses(std::vector<double> p1, std::vector<double> p2)
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

    double calcDistSquared2D(const geometry_msgs::Point &p, const geometry_msgs::Point &q)
        {
        const double dx = p.x - q.x;
        const double dy = p.y - q.y;
        return (dx * dx + dy * dy);
        }


    double normalizeAngle(double euler)
        {
        double res = euler;
        while (res > M_PI)
        {
            res -= (2.0 * M_PI);
        }
        while (res < -M_PI)
        {
            res += 2.0 * M_PI;
        }

        return res;
        }

Eigen::Vector3d unit_vect(double angle)
    {
        Eigen::Vector3d v{cos(angle), sin(angle), 0};
        return v;

    }
}


    

