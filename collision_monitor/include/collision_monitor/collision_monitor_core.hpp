// #ifndef COLLISION_MONITOR_CORE_H
// #define COLLISION_MONITOR_CORE_H

#include<ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include<autopilot_msgs/Trajectory.h>
#include <visualization_msgs/Marker.h>
#include<nav_msgs/Odometry.h>
// #include<nav_msgs/Path.h/>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/TransformStamped.h>
#include<zed_interfaces/ObjectsStamped.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
// #include <zed_wrapper/object_stamped.h>


#include <cmath>
#include "pcl/point_cloud.h"
#include "std_msgs/Bool.h"


#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include "tf/message_filter.h"
#include<tf2_ros/transform_listener.h>
#include "tf2_eigen/tf2_eigen.h"
#include<tf/tf.h>
#include "message_filters/subscriber.h"
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sstream>
#include <ctime>
#include <bits/stdc++.h>
#include <chrono>

// /home/boson/obs_cpp_ws/src/autopilot_boson/depend/libwaypoint_follower
// /home/boson/obs_cpp_ws/src/autopilot_boson/depend/libwaypoint_follower/include/libwaypoint_follower/libwaypoint_follower.h
using namespace std::chrono;
using std::tuple;
using std::get;   
namespace collsion_monitor{


    class CollisionMonitor{

        public:
            CollisionMonitor();
            void mainLoop();
            
        
        private:

            std::string robot_base_frame_;
            std::string lidar_frame_;
    
            float robot_level_collision_check_enable;//  rospy.get_param("obstacle_stop_planner"
                                                                  //"/robot_level_collision_check_enable", True)
            float width_offset;// = rospy.get_param("obstacle_stop_planner/footprint/offset_to_width", 0.3)
            float  length_offset;// = rospy.get_param("obstacle_stop_planner/footprint/offset_to_forward_length", 1)
        
            float vehicle_width;
            float vehicle_lenght; 
            geometry_msgs::Pose robot_pose;


            void scanCallback(const sensor_msgs::LaserScan &scan);
            void odomCallback(const nav_msgs::Odometry &data);
            void zedCallback(const zed_interfaces::ObjectsStamped &data);
            void BboxesCallback(const jsk_recognition_msgs::BoundingBoxArray &data);
            void loadParams(ros::NodeHandle nh);
            
            // bool areDataReceived(const bool &m_is_laser_received, const bool &m_is_trajectory_received, const bool &m_is_odom_received);
            bool areDataReceived();


            laser_geometry::LaserProjection projector;
            tf::TransformListener tfListener;

            ros::Publisher pub_collision_points, pub_trajectory, pub_traj_vis, pub_close_point,
            pub_trans_cloud_points;
            ros::Subscriber sub_laserscan,sub_traj, sub_odom, sub_zed_detections, sub_lidar_bboxes;
           

            //! Flag for receiving laser
            bool m_is_laser_received{};

            //! Flag for receiving global trajectory
            bool m_is_trajectory_received{};
            
            //! Flag for receiving Odom
            bool m_is_odom_received{};


            float distance_between_pose;


    public:
    void polTocart(double rho, double phi, float &x, float &y)
    {
        x = rho * cos(phi);
        y = rho * sin(phi);
    }

     pcl::PointXYZ polTocart(double rho, double phi)
     {
        pcl::PointXYZ point;
        point.x = rho * cos(phi);
        point.y = rho * sin(phi);
        point.z = 0;
        return point;
     }


//      tuple<bool, geometry_msgs::TransformStamped>
// maybe_get_transform(string const &parent_frame, string const &child_frame,
//                     const tf2_ros::Buffer &buffer) {
//   bool got_transform = false;
//   geometry_msgs::TransformStamped current_transformation;
//   try {
//     current_transformation = buffer.lookupTransform(parent_frame,
//                                                     child_frame,
//                                                     ros::Time::now(),
//                                                     ros::Duration(1.0));
//     got_transform = true;
//   }
//   catch (tf2::TransformException &ex) {
//     ROS_WARN("error: %s", ex.what());
//   }
//   return tuple<bool, geometry_msgs::TransformStamped>(got_transform,
//                                                       current_transformation);
// }

    
    sensor_msgs::PointCloud2 transform_cloud(sensor_msgs::PointCloud2 pc2_in, std::string target_frame)
    {
    if (pc2_in.header.frame_id == target_frame)
    {
        return pc2_in;
    }

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::TransformStamped transform;
    // transform = tf_buffer.lookupTransform(pc2_in.header.frame_id,target_frame, ros::Time(0));
 
    try{
         transform = tf_buffer.lookupTransform(pc2_in.header.frame_id,target_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        // ros::Duration(1.0).sleep();
        sensor_msgs::PointCloud2 empty_cloudl;
        return empty_cloudl;
    }
    // std::cout <<transform<<"\n";
    sensor_msgs::PointCloud2 pc2_out;
    tf2::doTransform(pc2_in, pc2_out, transform);
    // pc2_out.header.frame_id = "map";
    // ROS_INFO("Transformed to target frame");

    // std::cout << transform << std::endl;
    return pc2_out;

}


    sensor_msgs::PointCloud2 transformCloudFromTranform(sensor_msgs::PointCloud2 pc2_in, geometry_msgs::TransformStamped transform)
    {

    sensor_msgs::PointCloud2 pc2_out;
    tf2::doTransform(pc2_in, pc2_out, transform);
    // pc2_out.header.frame_id = "map";
    // ROS_INFO("Transformed to target frame");

    // std::cout << transform << std::endl;
    return pc2_out;

}


geometry_msgs::TransformStamped toTransformStamped(geometry_msgs::PoseStamped pose, std::string child_frame)
{
/*
Header: 
  seq: 0
  stamp: 980.132651366
  frame_id: map
child_frame_id: ego_vehicle
transform: 
  translation: 
    x: 21.5305
    y: -24.6795
    z: 0.00167938
  rotation: 
    x: 1.10352e-07
    y: 5.38034e-07
    z: 0.013958
    w: 0.999903

*/
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = pose.header.frame_id;
    transform.header.stamp = ros::Time::now();
    transform.child_frame_id = child_frame;
    transform.transform.translation.x  = pose.pose.position.x;
    transform.transform.translation.y =  pose.pose.position.y;
    transform.transform.translation.z = pose.pose.position.z;
    transform.transform.rotation = pose.pose.orientation;
    return transform; 
}




      
    };
}