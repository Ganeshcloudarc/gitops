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
#include <costmap_2d/costmap_2d_ros.h>

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

    private:
        void localScanCallback(const sensor_msgs::LaserScan::ConstPtr& local_scan);
        void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& local_scan);
        void TrajectoryCallback(const geometry_msgs::PoseStamped::ConstPtr& trajectory);
        ros::Subscriber scan_sub, local_scan_sub, trajectory_sub, odometry_sub;
        costmap_2d::Costmap2DROS costmap;



};

}

