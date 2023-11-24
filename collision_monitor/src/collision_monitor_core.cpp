#include"ollision_monitor/collision_monitor_core.hpp"
// collision_monitor/include/collision_monitor/collision_monitor_core.hpp

using namespace std;

namespace collsion_monitor

 collsion_monitor::collsion_monitor(): m_is_laser_received{false}, 
    m_is_trajectory_received{false}, 
    m_is_odom_received{false},
    tf2_listener(tf2_buffer)
    {
        ros::NodeHandle nh;

        //Parameters
        loadParams(nh);

        // ROS publishers
        pub_collision_points = nh.advertise<sensor_msgs::PointCloud2>("/collision_points", 10);
        pub_trajectory = nh.advertise<autopilot_msgs::Trajectory>("/local_gps_trajectory", 10);
        pub_traj_vis = nh.advertise<visualization_msgs::Marker>("/collision_velocity_marker", 10);
        pub_close_point = nh.advertise<geometry_msgs::PoseStamped>("/close_point", 10);
        pub_trans_cloud_points = nh.advertise<sensor_msgs::PointCloud2>("/trans_cloud", 10);

        // ROS subscribers
        sub_laserscan = nh.subscribe("/laser_scan",1, &ObstacleStopPlanner::scanCallback, this);
        sub_traj = nh.subscribe("/global_gps_trajectory",1, &ObstacleStopPlanner::globalTrajectoryCallback, this);
        sub_odom = nh.subscribe("/vehicle/odom", 10, &ObstacleStopPlanner::odomCallback, this);

    

    }


