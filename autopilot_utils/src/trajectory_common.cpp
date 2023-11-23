#include "autopilot_utils/trajectory_common.h"

namespace autopilot_utils{

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
    double robot_yaw = tf::getYaw(robot_pose.orientation);
    

    for(int i = 0; i < traj.points.size(); i++)
    {
        double dis = sqrt(pow(robot_pose.position.x - traj.points[i].pose.position.x, 2) + 
                        pow(robot_pose.position.y - traj.points[i].pose.position.y,2));
        
        double  curr_yaw = tf::getYaw( traj.points[i].pose.orientation);
        if (dis <distance_th  and abs(normalizeAngle(curr_yaw) - normalizeAngle(robot_yaw)) < angle_th)
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
