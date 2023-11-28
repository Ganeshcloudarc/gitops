#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include<autopilot_msgs/Trajectory.h>
#include<autopilot_msgs/TrajectoryPoint.h>
#include <std_msgs/Float32.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <sstream>


#include "autopilot_utils/trajectory_common.h"
#include "autopilot_utils/pose_utils.h"
#include "autopilot_utils/autonav_utils.h"
#include <laser_geometry/laser_geometry.h>
#include "autopilot_utils/costmap_manager.h"
#include "autopilot_utils/tf_utils.h"
#include "autopilot_utils/dwa_path_ganerator.h"
#include "autopilot_utils/dubins.h"

#include "Utils.hpp"

#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>

#include <tf2/LinearMath/Quaternion.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include<diagnostic_updater/DiagnosticStatusWrapper.h>




using namespace kiss_icp_ros::utils;
using namespace Eigen;
using namespace autopilot_utils;
// using namespace  std;

namespace auto_nav
{
class AutoNavCore 
{
    public:
        AutoNavCore();
        virtual ~AutoNavCore() {};
        ros::NodeHandle nh;
        tf2_ros::Buffer tf2_buffer;
        tf2_ros::TransformListener tf2_listener;
        
        autopilot_utils::TrajectoryHelper traj_helper;
        int OK = diagnostic_msgs::DiagnosticStatus::OK;
        int ERROR = diagnostic_msgs::DiagnosticStatus::ERROR;
        int WARN = diagnostic_msgs::DiagnosticStatus::WARN;
        int STALE = diagnostic_msgs::DiagnosticStatus::STALE;

    private:
        void localScanCallback(const sensor_msgs::LaserScan::ConstPtr& local_scan);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& local_scan);
        void trajectoryCallback(const autopilot_msgs::Trajectory::ConstPtr& trajectory);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
        void main_loop(ros::NodeHandle);
        void loadParams(ros::NodeHandle);
        ros::Subscriber scan_sub, local_scan_sub, trajectory_sub, odometry_sub;
        // ros::publisher local_path_pub;
        ros::Publisher local_traj_pub,local_path_pub, center_line_pub, left_line_pub, right_line_pub, path_percent_publisher, front_pose_pub;
        ros::Publisher map_point_cloud_pub, left_liners_cloud_pub,right_inliers_cloud_pub;
        ros::Publisher lanes_marker_pub, dwa_marker_pub, dwa_collsion_free_marker_pub, ransac_samples_pub;
        ros::Publisher vibration_path_pub, dwa_best_traj_marker_pub;
        ros::Publisher diagnostis_pub;
        nav_msgs::Odometry::ConstPtr curr_odom;
        autopilot_msgs::Trajectory local_traj, global_traj;
        sensor_msgs::LaserScan::ConstPtr curr_scan, curr_local_scan;
        geometry_msgs::Pose curr_robot_pose;
        nav_msgs::Path vibration_path;
        bool odom_data_received, global_traj_data_received, curr_scan_data_received, curr_local_scan_data_receiced;
        inline void publish_diagnostics();
        laser_geometry::LaserProjection scan_projector;
        laser_geometry::LaserProjection local_scan_projector;
         costmap_2d::Costmap2DROS* costmap_ros_;

         costmap_2d::Costmap2D* costmap_;
        // OccupencyGridManager* occ_manager;
        vector<double> slope_list;
        vector<double> intercept_list;
        // bool in_turn_status;
        
        // Parameters
        bool use_dwa, enable_moving_avg_filter, pub_debug_topics, use_previous_line, mission_continue;
        int moving_avg_filter_window_size, ransac_max_iterations, loop_frequency;
        float row_spacing, tree_width, tree_width_tolerance;
        float radius_to_check_turn, minimum_turn_radius, forward_point_dis;
        string costmap_topic;
        float dwa_constant_speed, dwa_steering_agle_lim,dwa_steering_angle_increment,
        dwa_path_len,dwa_path_resolution, dwa_wheel_base, local_traj_length;
        // DwaPathGenerator dwa_path_gen;
        DwaPathGenerator dwa_path_gen;
       


        Line center_line;
        Line moving_avg_center_line;
        double center_line_heading, moving_avg_center_line_heading;
        vector<double> row_end_point, left_row_end_point, right_row_end_point;
        bool row_end_point_found;
        vector<double> next_row_start_point, next_left_row_start_point, next_right_row_start_point;

        bool valid_turn_found,global_turn_detected; 
        bool reset_costmap = true;
        int close_index = -1;
        int turn_start_index = -1;
        int turn_end_index = -1;
        std_msgs::Float32 path_percent_val;

        autopilot_msgs::Trajectory local_trajectory_msg;

        #define THETA M_PI/2;
        #define DUBINS_LEFT 3;
        #define DUBINS_RIGHT 6;

        double speed  =1;
        visualization_msgs::MarkerArray marker_arr;
        bool in_turn_status = false;
        diagnostic_updater::DiagnosticStatusWrapper diag_status;
};
}