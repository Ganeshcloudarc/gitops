#include <autopilot_msgs/Trajectory.h>
#include <autopilot_msgs/TrajectoryPoint.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <tf/tf.h>
// #include "autonav_utils.hpp"

using namespace std;
namespace autopilot_utils{


class TrajectoryHelper 
{
    public:
        TrajectoryHelper();
        // ~TrajectoryHelper();
        void setTrajectory(autopilot_msgs::Trajectory trajectory);
        int getLength();
        std::tuple<bool, int>  find_closest_idx_with_dist_ang_thr(geometry_msgs::Pose robot_pose, double dist_thr, double  angle_thr);
        int find_close_pose_after_index(geometry_msgs::Pose curr_pose, int prev_idx, double search_distance);
        int  next_point_within_dist(int idx, double dist_thr);
        autopilot_msgs::TrajectoryPoint get_trajectory_point_by_index(int idx);
        double getCircumRadius(int, int);

        // autopilot_msgs::Trajectory getTrajectory();
    //      get_traj_point()
    //        def next_point_within_dist(self, idx, dist_thr):
    //     def find_closest_idx_with_dist_ang_thr(self, curr_pose, dist_thr, angle_thr):

    // def find_close_pose_after_index(self, curr_pose, prev_idx, search_distance=10):
    // def to_path(self):

    // def to_marker(self):
        autopilot_msgs::Trajectory traj;

    
    private:
        


};

TrajectoryHelper::TrajectoryHelper()
{
    
}

int TrajectoryHelper::getLength()
{
    return traj.points.size();
}
autopilot_msgs::TrajectoryPoint TrajectoryHelper::get_trajectory_point_by_index(int index)
{   
    if (index >= 0 && index < traj.points.size())
        return traj.points[index];
    else
    {
        cerr<<"Index out of range"<<endl;
        return autopilot_msgs::TrajectoryPoint();
    }
        

}

double TrajectoryHelper::getCircumRadius(int ind, int index_lim = 10)
{       
    if ((ind - index_lim > 0) and (ind + index_lim  < traj.points.size()))
        {   
            vector<double> p1 = {traj.points[ind - index_lim].pose.position.x, traj.points[ind - index_lim].pose.position.y};
            vector<double> p2 = {traj.points[ind].pose.position.x, traj.points[ind].pose.position.y};
            vector<double> p3 = {traj.points[ind + index_lim].pose.position.x, traj.points[ind + index_lim].pose.position.y};

            // return auto_nav::circumRadius(p1, p2, p3);
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
            
    else
        {
            cerr<<"Index out of bounds"<<endl;
            return 0.0;
        }

}

    

void TrajectoryHelper::setTrajectory(autopilot_msgs::Trajectory trajectory)
{
     traj = trajectory;
}

 std::tuple<bool, int> TrajectoryHelper::find_closest_idx_with_dist_ang_thr(geometry_msgs::Pose robot_pose, double distance_th, double angle_th)
{
    double dist_min = 1000000000;
    int idx_min = -1;

    // double yaw = tf2_ros::getYaw(robot_pose.orientation)
    // tf2_geometry_msgs::Quaternion q;
    tf2::Quaternion tf_quaternion(
        robot_pose.orientation.x,
        robot_pose.orientation.y,
        robot_pose.orientation.z,
        robot_pose.orientation.w
    );
    // Convert the quaternion to Euler angles
    double roll, pitch, robot_yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, robot_yaw);


    for(int i = 0; i < traj.points.size(); i++)
    {
        double dis = sqrt(pow(robot_pose.position.x - traj.points[i].pose.position.x, 2) + 
                        pow(robot_pose.position.y - traj.points[i].pose.position.y,2));
        tf2::Quaternion tf_quaternion1(
        traj.points[i].pose.orientation.x,
        traj.points[i].pose.orientation.y,
        traj.points[i].pose.orientation.z,
        traj.points[i].pose.orientation.w
         );
        double roll, pitch, curr_yaw;
        tf2::Matrix3x3(tf_quaternion1).getRPY(roll, pitch, curr_yaw);
        if (dis <distance_th  and abs(curr_yaw - robot_yaw) < angle_th)
        {
            return std::make_tuple(true, i);  
        }   
        
    }
    return std::make_tuple(false, -1);

}

int  TrajectoryHelper::find_close_pose_after_index(geometry_msgs::Pose curr_pose, int prev_idx, double search_distance)
{       
        double dist_min = 1000000000;
        int idx_min = prev_idx;
        if (prev_idx >= traj.points.size())
        {
            return traj.points.size() -1;
        }
        for (int i = prev_idx; i< traj.points.size();i++)
        {
            if (abs(traj.points[prev_idx].accumulated_distance_m -
                   traj.points[i].accumulated_distance_m) > search_distance)
                   break;
            double dist = sqrt(pow(curr_pose.position.x - traj.points[i].pose.position.x, 2) + 
                        pow(curr_pose.position.y - traj.points[i].pose.position.y,2));
             if (dist < dist_min)
             {
                dist_min = dist;
                idx_min = i;
             }    
        }
        return idx_min;

}

int  TrajectoryHelper::next_point_within_dist(int idx, double dist_thr)
{
    if (traj.points.size() > 0 and idx < traj.points.size())
    {
        double close_dis = traj.points[idx].accumulated_distance_m;
        if ( abs(traj.points[idx].accumulated_distance_m - \
                   traj.points.back().accumulated_distance_m) <= dist_thr)
                return traj.points.size() - 1;
        else
        {
            for (int i = idx +1; i < traj.points.size(); i++)
            {
                double path_acc_distance = abs(traj.points[i].accumulated_distance_m - close_dis);
                if (path_acc_distance > dist_thr)
                        return i;
            }
        }
    }
    else
        return -1;

}




}
