#ifndef TRAJECTORY_COMMON_H
#define TRAJECTORY_COMMON_H

#include <autopilot_msgs/Trajectory.h>
#include <autopilot_msgs/TrajectoryPoint.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <tf/tf.h>
#include"autopilot_utils/pose_utils.h"

using namespace std;
namespace autopilot_utils{




class TrajectoryHelper 
{
    public:
        TrajectoryHelper();
        // ~TrajectoryHelper();
        void setTrajectory(autopilot_msgs::Trajectory trajectory);
        int getLength();
        std::pair<bool, int32_t> find_closest_idx_with_dist_ang_thr(geometry_msgs::Pose robot_pose, double distance_th, double angle_th);
        int find_close_pose_after_index(geometry_msgs::Pose curr_pose, int prev_idx, double search_distance);
        int  next_point_within_dist(int idx, double dist_thr);
        autopilot_msgs::TrajectoryPoint get_trajectory_point_by_index(int idx);
        double getCircumRadius(int, int);
        
    private:
    autopilot_msgs::Trajectory traj;
        


};

}
#endif  
