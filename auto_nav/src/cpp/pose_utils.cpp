#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>

using namespace std;
namespace autopilot_utils
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
}


    

