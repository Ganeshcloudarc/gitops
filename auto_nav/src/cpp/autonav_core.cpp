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
#include "trajectory_common.cpp"
#include "pose_utils.cpp"
#include "autonav_utils.hpp"
#include <laser_geometry/laser_geometry.h>
#include "tf_utils.hpp"
#include "Utils.hpp"
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include "dwa_path_ganerator.cpp"
#include <tf2/LinearMath/Quaternion.h>
#include "costmap_manager.hpp"
using namespace kiss_icp_ros::utils;

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
        ros::Publisher vibration_path_pub;
        nav_msgs::Odometry::ConstPtr curr_odom;
        autopilot_msgs::Trajectory local_traj, global_traj;
        sensor_msgs::LaserScan::ConstPtr curr_scan, curr_local_scan;
        geometry_msgs::Pose curr_robot_pose;
        nav_msgs::Path vibration_path;
        bool odom_data_received, global_traj_data_received, curr_scan_data_received, curr_local_scan_data_receiced;
        laser_geometry::LaserProjection scan_projector;
        laser_geometry::LaserProjection local_scan_projector;
        //  costmap_2d::Costmap2DROS* costmap_ros;

        //  costmap_2d::Costmap2D costmap_;
        OccupencyGridManager* occ_manager;
        vector<double> slope_list;
        vector<double> intercept_list;
        bool in_turn_status;
        
        // Parameters
        bool use_dwa, enable_moving_avg_filter, pub_debug_topics, use_previous_line, mission_continue;
        int moving_avg_filter_window_size, ransac_max_iterations, loop_frequency;
        float row_spacing, tree_width, tree_width_tolerance;
        float radius_to_check_turn, mininum_turn_radius, forward_point_dis;
        string costmap_topic;
        float dwa_constant_speed, dwa_steering_agle_lim,dwa_steering_angle_increment,
        dwa_path_len,dwa_path_resolution, dwa_wheel_base, local_traj_length;
        // DwaPathGenerator dwa_path_gen;

        

        

};

AutoNavCore::AutoNavCore() : tf2_listener(tf2_buffer),
odom_data_received(false), global_traj_data_received(false), curr_scan_data_received(false), curr_local_scan_data_receiced(false), in_turn_status(false)

{
  ros::NodeHandle private_nh("~");
  loadParams(private_nh);
  // costmap_ros = new costmap_2d::Costmap2DROS("/costmap_node/costmap/costmap", tf2_buffer);
  // ROS_DEBUG_STREAM(costmap_ros->getGlobalFrameID());
  scan_sub = private_nh.subscribe<sensor_msgs::LaserScan>("/laser_scan", 1, &AutoNavCore::scanCallback, this);
  local_scan_sub = private_nh.subscribe<sensor_msgs::LaserScan>("/local_cloud_laser_scan", 1, &AutoNavCore::localScanCallback, this);
  trajectory_sub = private_nh.subscribe<autopilot_msgs::Trajectory>("/global_gps_trajectory", 1, &AutoNavCore::trajectoryCallback, this);
  odometry_sub = private_nh.subscribe<nav_msgs::Odometry>("/vehicle/odom", 1, &AutoNavCore::odomCallback, this);
  local_traj_pub = private_nh.advertise<autopilot_msgs::Trajectory>("/local_gps_trajectory", 1);
  local_path_pub = private_nh.advertise<nav_msgs::Path>("/local_trajectory", 1);
  path_percent_publisher =  private_nh.advertise<std_msgs::Float32>("/osp_path_percentage", 10);
  lanes_marker_pub = private_nh.advertise<visualization_msgs::MarkerArray>("/lanes", 1,true);


  if (pub_debug_topics)
  {
  center_line_pub = private_nh.advertise<nav_msgs::Path>("center_line", 1);
  left_line_pub = private_nh.advertise<nav_msgs::Path>("left_line", 1);
  right_line_pub = private_nh.advertise<nav_msgs::Path>("right_line", 1);
  front_pose_pub = private_nh.advertise<geometry_msgs::PoseStamped>("front_pose", 1);
  map_point_cloud_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/map_point_cloud", 1);
  left_liners_cloud_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/left_inliers", 1);
  right_inliers_cloud_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/right_inliers", 1);
  dwa_marker_pub = private_nh.advertise<visualization_msgs::MarkerArray>("/dwa_paths", 1,true);
  dwa_collsion_free_marker_pub = private_nh.advertise<visualization_msgs::MarkerArray>("/dwa_paths_collsion_free", 1,true);
  ransac_samples_pub = private_nh.advertise<visualization_msgs::Marker>("/ransac_samples", 1,true);
  vibration_path_pub =  private_nh.advertise<nav_msgs::Path>("vibration_path", 1);
  }

  // double max_steer = 30;
  // DwaPathGenerator dwa_path_gen(2.5, max_steer, 1, 10, 0.1);
  // std::vector<std::vector<std::vector<double>>> paths = dwa_path_gen.generate_paths({0, 0, 0});
  
  // lanes_marker_pub.publish(dwa_path_gen.get_dwa_paths_marker_array(paths));
  vibration_path.header.frame_id = "map";
  main_loop(private_nh);

}

void AutoNavCore::loadParams(ros::NodeHandle private_nh)
{

  private_nh.param<int>("loop_frequency", loop_frequency,10);
  private_nh.param<bool>("mission_continue", mission_continue,true);


  private_nh.param<bool>("enable_moving_avg_filter", enable_moving_avg_filter,true);
  private_nh.param<bool>("pub_debug_topics", pub_debug_topics,true);
  private_nh.param<bool>("use_previous_line", use_previous_line,true);

  private_nh.param<int>("moving_avg_filter_window_size", moving_avg_filter_window_size,10);
  private_nh.param<int>("ransac/max_iterations", ransac_max_iterations,50);

  private_nh.param<float>("orchard_details/row_spacing", row_spacing, 6.2);
  private_nh.param<float>("orchard_details/tree_width", tree_width,2.3);
  private_nh.param<float>("orchard_details/tree_width_tolerace", tree_width_tolerance,0.3);


  private_nh.param<float>("turnings/radius_to_check_turn", radius_to_check_turn, 15);
  private_nh.param<float>("turnings/mininum_turn_radius", radius_to_check_turn, 7);
  private_nh.param<float>("turnings/forward_point_dis", forward_point_dis, 3);
  private_nh.param<float>("local_traj_length", local_traj_length, 8);



  // DWA related
  private_nh.param<bool>("dwa/enable", use_dwa,true);
  private_nh.param<string>("dwa/costmap_topic", costmap_topic,"/costmap_node/costmap/costmap");
  private_nh.param<float>("dwa/constant_speed", dwa_constant_speed,1);
  private_nh.param<float>("dwa/wheel_base", dwa_wheel_base, 2.5);

  private_nh.param<float>("dwa/steering_agle_lim", dwa_steering_agle_lim,30);
  private_nh.param<float>("dwa/steering_angle_increment", dwa_steering_angle_increment,1);
  private_nh.param<float>("dwa/path_len", dwa_path_len,6);
  private_nh.param<float>("dwa/path_resolution", dwa_path_resolution, 0.1);

}

void AutoNavCore::main_loop(ros::NodeHandle private_nh)
{   
    if(use_dwa)
      {
        occ_manager = new OccupencyGridManager(private_nh, costmap_topic, true);
      }
    //intial sensor check
    ros::Rate rate(5);
    while(ros::ok())
    {
    if (curr_local_scan_data_receiced and curr_scan_data_received and global_traj_data_received and odom_data_received)
    {   ROS_INFO("data received on all sensors");
        break;
    }
    else
    {
      ROS_WARN_STREAM("NO data received on all sensors");
    //   ROS_WARN_STREAM("curr_local_scan_data_receiced" + curr_local_scan_data_receiced );//"  and curr_scan_data_received and global_traj_data_received and odom_data_received
      rate.sleep();
      ros::spinOnce();
    }
    
    }

    ros::Rate loop_rate(loop_frequency);
    int close_index = -1;
    
    DwaPathGenerator dwa_path_gen(dwa_wheel_base, dwa_steering_agle_lim, dwa_constant_speed, dwa_path_len, dwa_path_resolution);
      if (pub_debug_topics)
      {
        std::vector<std::vector<std::vector<double>>> paths = dwa_path_gen.generate_paths({0, 0, 0});

        lanes_marker_pub.publish(dwa_path_gen.get_dwa_paths_marker_array(paths));
      }
    
    while(ros::ok())
    {
      
        tuple<bool, int> val;
        if (close_index == -1)
        {
          val = traj_helper.find_closest_idx_with_dist_ang_thr(curr_robot_pose, row_spacing, M_PI/2);
          if (std::get<0>(val))
          {
            close_index = std::get<1>(val);
          }
          else
          {
            ROS_WARN_STREAM("No close point found");
            loop_rate.sleep();
            ros::spinOnce();
            continue;
          }}
        else
        { 
          close_index = traj_helper.find_close_pose_after_index(curr_robot_pose, close_index, 10);
        }

        // cout<<std::get<0>(val)<< std::get<1>(val);
        //  std::stringstream ss;
        // ss << "My integer value is: " << close_index;
        ROS_DEBUG_STREAM("close_index : " << close_index);

        // path percentage
        float path_percent = (global_traj.points[close_index].accumulated_distance_m /
                            global_traj.points.back().accumulated_distance_m) * 100;
        std_msgs::Float32 path_percent_val;
        path_percent_val.data = path_percent;
        path_percent_publisher.publish(path_percent_val);
        if (path_percent > 95.0 and autopilot_utils::distanceBetweenPoses(curr_robot_pose, global_traj.points.back().pose) < row_spacing)
        {
          if (mission_continue)
          {
            ROS_INFO("mission completed");
            ROS_INFO("Restarting the mission");
            close_index = 1;
            loop_rate.sleep();
            ros::spinOnce();
            continue;
            
          }
              ROS_ERROR("mission completed");
              return ;
        }
        int forward_path_idx = traj_helper.next_point_within_dist(close_index, row_spacing);
        ROS_DEBUG_STREAM("forward_path_idx : "<<forward_path_idx);
        if (pub_debug_topics)
        {
        geometry_msgs::PoseStamped pst;
        pst.header.frame_id = "map"; 
        pst.pose = traj_helper.get_trajectory_point_by_index(forward_path_idx).pose;
        front_pose_pub.publish(pst);
        ROS_DEBUG("front pose is publshed");
        }
   
        // Checking for Turning point whose every point is 2 meters apart
        int p2_index = traj_helper.next_point_within_dist(close_index, forward_point_dis);
        int p3_index = traj_helper.next_point_within_dist(p2_index, forward_point_dis);
        vector<double> p1 = {traj_helper.get_trajectory_point_by_index(close_index).pose.position.x, traj_helper.get_trajectory_point_by_index(close_index).pose.position.y};
        vector<double> p2 = {traj_helper.get_trajectory_point_by_index(p2_index).pose.position.x, traj_helper.get_trajectory_point_by_index(p2_index).pose.position.y};
        vector<double> p3 = {traj_helper.get_trajectory_point_by_index(p3_index).pose.position.x, traj_helper.get_trajectory_point_by_index(p3_index).pose.position.y};
        float circum_radius = auto_nav::circumRadius(p1,p2,p3);
        ROS_DEBUG_STREAM("circum_radius : "<<circum_radius);

        if (abs(circum_radius) < radius_to_check_turn or in_turn_status == true)
        { 
          
          
          ROS_WARN("Turn detected");
          autopilot_msgs::Trajectory turn_traj;
          turn_traj.header.frame_id = "map";
          for(int i = close_index; i <traj_helper.getLength(); i++)
          { 

            if (abs(traj_helper.get_trajectory_point_by_index(i).accumulated_distance_m - traj_helper.get_trajectory_point_by_index(close_index).accumulated_distance_m) > local_traj_length)
            {
              break;
            }

            turn_traj.points.push_back(traj_helper.get_trajectory_point_by_index(i));
          }
          local_traj_pub.publish(turn_traj);
          ROS_INFO("Turn trajectory published");
          slope_list.clear();
          intercept_list.clear();
        }
        else
        { 
          ROS_INFO("No turn detected");
          
          // Finding  close cloud points
          sensor_msgs::PointCloud2  cloud;
          scan_projector.transformLaserScanToPointCloud("map",*curr_local_scan, cloud, tf2_buffer);


          std::vector<Eigen::Vector3d> local_map_points;
          local_map_points = kiss_icp_ros::utils::PointCloud2ToEigen(cloud);
          Header header;
          header.frame_id = "map";
          // if (pub_debug_topics)
          //  {
          //   const auto eigen_cloud = EigenToPointCloud2(local_map_points, header);
          //   map_point_cloud_pub.publish(*std::move(eigen_cloud));}
          
          Eigen::Vector3d L1(traj_helper.get_trajectory_point_by_index(close_index).pose.position.x, traj_helper.get_trajectory_point_by_index(close_index).pose.position.y, 0.0);
          Eigen::Vector3d L2(traj_helper.get_trajectory_point_by_index(forward_path_idx).pose.position.x, traj_helper.get_trajectory_point_by_index(forward_path_idx).pose.position.y, 0.0);
          std::vector<Eigen::Vector3d> left_close_points, right_close_points, all_close_points;
          for (size_t i = 0; i < local_map_points.size(); i++) 
            { Eigen::Vector3d point = local_map_points[i];
              point[2] = 0.0;
              double distance_to_line = distanceToLine(point,L1,L2);
              if (abs(distance_to_line) < 6)
              {
                all_close_points.push_back(point);
                if(distance_to_line >0)
                  left_close_points.push_back(point);
                else
                right_close_points.push_back(point);
              }
            }
          if(pub_debug_topics){
          const auto eigen_cloud = EigenToPointCloud2(all_close_points, header);
          map_point_cloud_pub.publish(*std::move(eigen_cloud));}
          if (left_close_points.size() == 0 or right_close_points.size() == 0)
          {
            ROS_ERROR("No Left points and right points");
            loop_rate.sleep();
            ros::spinOnce();
            continue;
          }
          // int ransac_max_iterations = 50;
          // double row_spacing = 6.2;
          int max_inlilers_left, max_inlilers_right;
          max_inlilers_left = 0;
          max_inlilers_right = 0;
          double offset = tree_width + tree_width_tolerance;
          vector<int> inliers_left_final, inliers_right_final;
          for(int i = 0; i < ransac_max_iterations ; i++)
          { //left points
            
            auto pair = ganerateRandomPair(left_close_points.size());
            // cout<<pair.first<<" "<<pair.second<<endl;
            Eigen::Vector3d line_point1 = left_close_points[pair.first];
            Eigen::Vector3d line_point2 = left_close_points[pair.second];
            vector<int> inliers_left;
            inliers_left = distancesToLine(left_close_points, offset, line_point1, line_point2);
            // inlilers_left_count = inliers_left.size();
            // cout<<"inliers_left  :"<<inliers_left.size()<<endl;
            float theta = -M_PI/2;
            Eigen::Vector3d v1 = line_point2 - line_point1;
            auto vect =  v1.normalized() * row_spacing;
            double new_x = vect.x() * cos(theta) - vect.y() * sin(theta);
            double new_y = vect.x() * sin(theta) + vect.y() * cos(theta);
            Eigen::Vector3d line_point3(line_point1.x() + new_x, line_point1.y() + new_y, 0.0  );
            Eigen::Vector3d line_point4(line_point2.x() + new_x, line_point2.y() + new_y, 0.0  );
            vector<int> inliers_right;
            inliers_right = distancesToLine(right_close_points, offset, line_point3, line_point4);
            // inlilers_right_count = inliers_right.size();           
            
            if (inliers_left.size() >max_inlilers_left and  inliers_right.size()> max_inlilers_right)
              { 
                max_inlilers_left = inliers_left.size();
                max_inlilers_right = inliers_right.size();
                inliers_left_final = inliers_left;
                inliers_right_final = inliers_right;
                if(pub_debug_topics){
                std::vector<Eigen::Vector3d> points_vect;
                points_vect.push_back(line_point1);
                points_vect.push_back(line_point2);
                points_vect.push_back(line_point3);
                points_vect.push_back(line_point4);
                points_vect.push_back(line_point1);
                ransac_samples_pub.publish(eigenToMarker(points_vect, "map"));}

              }
            



            // on right cloud
            auto pair_ = ganerateRandomPair(right_close_points.size());
            // cout<<pair_.first<<" "<<pair_.second<<endl;
            Eigen::Vector3d line_point1_ = right_close_points[pair_.first];
            Eigen::Vector3d line_point2_ = right_close_points[pair_.second];
            // Eigen::Vector3d line_point1(0,-3, 0);
            // Eigen::Vector3d line_point2(6,-3,0);
            vector<int> inliers_right_;
            inliers_right_ = distancesToLine(right_close_points, offset, line_point1_, line_point2_);
            // inlilers_left_count = inliers_left.size();
            // cout<<"inliers_right_  :"<<inliers_right_.size()<<endl;
            float theta_ = M_PI/2;
            Eigen::Vector3d v1_ = line_point2_ - line_point1_;
            auto vect_ =  v1_.normalized() * row_spacing;
            double new_x_ = vect_.x() * cos(theta_) - vect_.y() * sin(theta_);
            double new_y_ = vect_.x() * sin(theta_) + vect_.y() * cos(theta_);
            Eigen::Vector3d line_point3_(line_point1_.x() + new_x_, line_point1_.y() + new_y_, 0.0  );
            Eigen::Vector3d line_point4_(line_point2_.x() + new_x_, line_point2_.y() + new_y_, 0.0  );
            vector<int> inliers_left_;
            inliers_left_ = distancesToLine(left_close_points, offset, line_point3_, line_point4_);
            // inlilers_right_count = inliers_right.size();
            // cout<<"inliers_left_  :"<<inliers_left_.size()<<endl;

            
            if (inliers_left_.size() >max_inlilers_left and  inliers_right_.size()> max_inlilers_right)
              { 
                max_inlilers_left = inliers_left_.size();
                max_inlilers_right = inliers_right_.size();
                inliers_left_final = inliers_left_;
                inliers_right_final = inliers_right_;
                 if(pub_debug_topics){ 
                std::vector<Eigen::Vector3d> points_vect_;
                points_vect_.push_back(line_point1_);
                points_vect_.push_back(line_point2_);
                points_vect_.push_back(line_point3_);
                points_vect_.push_back(line_point4_);
                points_vect_.push_back(line_point1_);
                ransac_samples_pub.publish(eigenToMarker(points_vect_, "map"));}

              }
          }
          ROS_DEBUG_STREAM("inliers_left_final : "<<inliers_left_final.size());
          ROS_DEBUG_STREAM("inliers_right_final : "<<inliers_right_final.size());

          std::vector<Eigen::Vector3d> left_inliers_final, right_inliers_final;
          for (const int index : inliers_left_final) {
            left_inliers_final.push_back(left_close_points[index]);
          }
          for (const int index : inliers_right_final) {
            right_inliers_final.push_back(right_close_points[index]);
          }
          
          const auto left_eigen_cloud = EigenToPointCloud2(left_inliers_final, header);
          if(pub_debug_topics)
            left_liners_cloud_pub.publish(*std::move(left_eigen_cloud));

          const auto right_eigen_cloud = EigenToPointCloud2(right_inliers_final, header);
          if(pub_debug_topics)
            right_inliers_cloud_pub.publish(*std::move(right_eigen_cloud));
          sensor_msgs::PointCloud2  right_eigen_cloud_base_link, left_eigen_cloud_base_link;
          geometry_msgs::TransformStamped transform_to_local_frame; 
            try
          {
            transform_to_local_frame = tf2_buffer.lookupTransform("base_link", right_eigen_cloud->header.frame_id, ros::Time(0));
            tf2::doTransform(*right_eigen_cloud,right_eigen_cloud_base_link, transform_to_local_frame);
            tf2::doTransform(*left_eigen_cloud,left_eigen_cloud_base_link, transform_to_local_frame);


          }
          catch (tf2::TransformException& ex)
          {
            ROS_WARN("%s", ex.what());
            ROS_ERROR("Could not transform");
            loop_rate.sleep();
            ros::spinOnce();
            continue;
            
          }
          
          std::vector<Eigen::Vector3d> right_eigen_base_link;
          right_eigen_base_link = kiss_icp_ros::utils::PointCloud2ToEigen(right_eigen_cloud_base_link);
         
          pair<double, double> right_coeffs = leastSquareMethod(right_eigen_base_link);
          ROS_WARN_STREAM("right params - slope : "<<right_coeffs.first <<", intercept : "<<right_coeffs.second );
          std::vector<Eigen::Vector3d> left_eigen_base_link;
          left_eigen_base_link = kiss_icp_ros::utils::PointCloud2ToEigen(left_eigen_cloud_base_link);
          pair<double, double> left_coeffs = leastSquareMethod(left_eigen_base_link);
          ROS_WARN_STREAM("left params - slope : "<<left_coeffs.first <<", intercept : "<<left_coeffs.second );

          Line left_lane((left_coeffs.first+right_coeffs.first)/2, left_coeffs.second);
          Line right_lane((left_coeffs.first+right_coeffs.first)/2, right_coeffs.second);
          Line center_lane((left_coeffs.first+right_coeffs.first)/2 , (left_coeffs.second+right_coeffs.second)/2);
          ROS_INFO_STREAM("left_lane : "<<left_lane.toString());
          ROS_INFO_STREAM("right line : "<< right_lane.toString());
          ROS_INFO_STREAM("center_lane : "<<center_lane.toString());


          // Marker to set left and right lines in base_link
          double val = local_traj_length;
          visualization_msgs::MarkerArray marker_arr;
          // left line
          visualization_msgs::Marker marker_left;
          marker_left.header.frame_id = "base_link";
          marker_left.type =  marker_left.LINE_STRIP;
          marker_left.id = 0;
          marker_left.color.g = 1;
           marker_left.color.a = 0.5;
          marker_left.scale.x = offset;
          // marker_left.scale.y = 1;
          // marker_left.scale.z = 1;
          marker_left.pose.position.x = 1;
          marker_left.pose.orientation.w = 1;
          geometry_msgs::Point point1_left;
          point1_left.x = -val;
          point1_left.y = left_lane.getSlope() * -val + left_lane.getConstant();

          marker_left.points.push_back(point1_left);
          geometry_msgs::Point point2_left;
          // left_lane(0,0);
          auto p2_left  = left_lane.intersct_point_to_line(10,0);
          // cout<<"left p2"<<p2_left.first<<"  " <<p2_left.second<<endl;
          point2_left.x = val;
           point2_left.y = left_lane.getSlope() * val + left_lane.getConstant();

          marker_left.points.push_back(point2_left);
          marker_arr.markers.push_back(marker_left);

          //Right line
          visualization_msgs::Marker marker_right;
          marker_right.header.frame_id = "base_link";
          marker_right.type =  marker_right.LINE_STRIP;
          marker_right.id = 1;
          marker_right.color.r = 1;
          marker_right.color.a = 0.5;
          marker_right.scale.x = offset;
          // marker_right.scale.y = 1;
          // marker_right.scale.z = 1;
          marker_right.pose.orientation.w = 1;
          geometry_msgs::Point point1_right;
          point1_right.x = -val;
          point1_right.y = right_lane.getSlope() * -val + right_lane.getConstant();
          marker_right.points.push_back(point1_right);
          geometry_msgs::Point point2_right;
          point2_right.x = val;
          point2_right.y = right_lane.getSlope() * val + right_lane.getConstant(); 
          marker_right.points.push_back(point2_right);
          marker_arr.markers.push_back(marker_right);


           //Center line
          visualization_msgs::Marker marker_center;
          marker_center.header.frame_id = "base_link";
          marker_center.type =  marker_right.LINE_STRIP;
          marker_center.id = 2;
          // marker_center.color.r = 1;
          // marker_center.color.g = 1;
          marker_center.color.b = 1;
          marker_center.color.a = 1;
          marker_center.scale.x = 0.1;
          // marker_right.scale.y = 1;
          // marker_right.scale.z = 1;
          marker_center.pose.orientation.w = 1;
          geometry_msgs::Point point1_center;
          // point1_center.x = (point1_right_map.x + point1_left_map.x)/2;
          // point1_center.y = (point1_right_map.y + point1_left_map.y)/2;
          point1_center.x = -val;
          point1_center.y = center_lane.getSlope() * -val + center_lane.getConstant();
          marker_center.points.push_back(point1_center);
          geometry_msgs::Point point2_center;
          // point2_center.x = (point2_right_map.x + point2_left_map.x)/2;
          // point2_center.y = (point2_right_map.y + point2_left_map.y)/2;
          point2_center.x = val;
          point2_center.y = center_lane.getSlope() * val + center_lane.getConstant();
          marker_center.points.push_back(point2_center);
          marker_arr.markers.push_back(marker_center);
          

          // Moving avarage filter on center line
          geometry_msgs::TransformStamped transform_to_base_link; 
          geometry_msgs::Point point1_center_map, point2_center_map;
          try
          {
            transform_to_base_link = tf2_buffer.lookupTransform("map", "base_link", ros::Time(0));
            tf2::doTransform(point1_center, point1_center_map, transform_to_base_link);
            tf2::doTransform(point2_center, point2_center_map, transform_to_base_link);
          }
          catch (tf2::TransformException& ex)
          {
            ROS_WARN("%s", ex.what());
            ROS_ERROR("Could not transform");
               loop_rate.sleep();
                ros::spinOnce();
                continue;
            // return ;
          }
          double slope = (point2_center_map.y - point1_center_map.y)/(point2_center_map.x -point1_center_map.x);
          double intercept = point1_center_map.y - slope * point1_center_map.x;
          slope_list.push_back(slope);
          intercept_list.push_back(intercept);
          if(slope_list.size()> moving_avg_filter_window_size and intercept_list.size()> moving_avg_filter_window_size)
          {
            // slope_list.pop_back();
            slope_list.erase(slope_list.begin());
            intercept_list.erase(intercept_list.begin());
            // intercept_list.pop_back();
          }
          cout<<"lenght of slope list : " <<slope_list.size();
          double slope_sum = 0;
          double intercept_sum = 0;
          for(double s : slope_list)
            slope_sum += s;
          for(double interc : intercept_list)
              intercept_sum += interc;
          cout<<"slope sum : "<< slope_sum <<" intercept sum :" << intercept_sum;
          double moving_avg_slope, moving_avg_intercept;
          moving_avg_slope = slope_sum /slope_list.size();
          moving_avg_intercept = intercept_sum / intercept_list.size();

          Line moving_avg_center_lane(moving_avg_slope, moving_avg_intercept);
          geometry_msgs::Point robot_point, robot_forward_point,start_point_center_lane, forward_point_center_lane ;
          robot_point.x = curr_robot_pose.position.x;
          robot_point.y = curr_robot_pose.position.y;
          robot_forward_point.x = curr_robot_pose.position.x + cos(tf::getYaw(curr_robot_pose.orientation)) * local_traj_length;
          robot_forward_point.y = curr_robot_pose.position.y + sin(tf::getYaw(curr_robot_pose.orientation)) * local_traj_length;
          start_point_center_lane = moving_avg_center_lane.intersct_point_to_line(robot_point);
          forward_point_center_lane = moving_avg_center_lane.intersct_point_to_line(robot_forward_point);
          // Moving avg center line
          visualization_msgs::Marker marker_center_avg_filter;
          marker_center_avg_filter.header.frame_id = "map";
          marker_center_avg_filter.type =  marker_right.LINE_STRIP;
          marker_center_avg_filter.id = 3;
           marker_center_avg_filter.color.r = 0.5;
          marker_center_avg_filter.color.g = 1;
          marker_center_avg_filter.color.b = 0.5;
          marker_center_avg_filter.color.a = 1;
          marker_center_avg_filter.scale.x = 0.3;
          marker_center_avg_filter.pose.orientation.w = 1;
          marker_center_avg_filter.points.push_back(start_point_center_lane);
          marker_center_avg_filter.points.push_back(forward_point_center_lane);
          marker_arr.markers.push_back(marker_center_avg_filter);
          // filtering and pushing.
          double line_heading = atan2(forward_point_center_lane.y - start_point_center_lane.y , forward_point_center_lane.x - start_point_center_lane.x);

          lanes_marker_pub.publish(marker_arr);
          ROS_DEBUG("lane markers puslished");
          if (pub_debug_topics){
          geometry_msgs::PoseStamped ps;
          ps.header.frame_id = "map";
          ps.pose.position.x = start_point_center_lane.x;
          ps.pose.position.y = start_point_center_lane.y;
          ps.pose.orientation = autopilot_utils::get_quaternion_from_yaw(line_heading);
          vibration_path.poses.push_back(ps);
          vibration_path_pub.publish(vibration_path);
          ROS_DEBUG("vibration path is puslished");
          }
          if(use_dwa)
          {

          // Checking collsions on center line on costmap
          pair<bool, unsigned char> center_collision = occ_manager->get_line_cost_world(start_point_center_lane.x, start_point_center_lane.y, forward_point_center_lane.x, forward_point_center_lane.y);
          if (center_collision.first)
          {
            if (center_collision.second == 0)
            {
              ROS_INFO("NO collions found on moving_avg_center_lane");
            }
            else
            {
              ROS_ERROR("collions found on moving_avg_center_lane");
            }

          }
          else{
            ROS_ERROR("Could not find cost to line");
          }
          std::vector<std::vector<std::vector<double>>> dwa_paths = dwa_path_gen.generate_paths({curr_robot_pose.position.x, curr_robot_pose.position.y,tf::getYaw(curr_robot_pose.orientation)});

          if(pub_debug_topics){
            dwa_marker_pub.publish(dwa_path_gen.get_dwa_paths_marker_array(dwa_paths, "map"));
          }

          // scoring of Dwa paths
          vector<int> collision_free_path_ids;
          std::vector<std::vector<std::vector<double>>> collision_free_paths;


          
          for(int i = 0; i< dwa_paths.size(); i++)
          { bool collision_found = false;
            for (int j=0; j<dwa_paths[i].size();j++)
            { unsigned int mx,my;
              // ROS_DEBUG_STREAM("world to map :"<<"x : "<<dwa_paths[i][j][0] <<"y :" <<dwa_paths[i][j][1]<<"cost_val :" <<occ_manager->costmap_2d.worldToMap(dwa_paths[i][j][0], dwa_paths[i][j][1], mx, my));
                
              if (occ_manager->costmap_2d.worldToMap(dwa_paths[i][j][0], dwa_paths[i][j][1], mx, my))
              { 
                
                // ROS_DEBUG_STREAM("cell cost : "<<static_cast<unsigned int>(occ_manager->costmap_2d.getCost(mx, my)));
                if (occ_manager->costmap_2d.getCost(mx, my) != 0)
                {
                  collision_found  = true;
                  break;
                }
              }
              else
               ROS_WARN("conversion error");
            }
              
    
            if (collision_found == false)
            {
              collision_free_path_ids.push_back(i);
              collision_free_paths.push_back(dwa_paths[i]);
            }
 
          }
          

          ROS_DEBUG_STREAM("collision free patths len: "<<collision_free_path_ids.size());
          if (collision_free_path_ids.size() == 0)
          {
            ROS_ERROR("Could not found collision free  published");
            loop_rate.sleep();
            ros::spinOnce();
            continue;

          }
          
          // dwa_collsion_free_marker_pub.publish(dwa_path_gen.get_dwa_paths_marker_array(collision_free_paths, "map"));
          ROS_DEBUG_STREAM("collision_free_paths published");
          // selecting the final path, closest to center line.
          // geometry_msgs::Point robot_point, robot_forward_point,start_point_center_lane, forward_point_center_lane ;
          // double moving_avg_center_lane_heading = atan(forward_point_center_lane.y - start_point_center_lane.y / forward_point_center_lane.x - start_point_center_lane.x);
          vector<double> dist_list, angle_diff_list, dist_list_norm, angle_diff_norm, total_norm;
          double dist_sum, angle_diff_sum;
          dist_sum = 0;
          angle_diff_sum = 0;
          int min_cost_index = 0;
          double min_dis = 1000;
          for(int i = 0; i< collision_free_paths.size(); i++) 
          {
            vector<double> point = collision_free_paths[i].back();
            double dis = moving_avg_center_lane.distance_to_point(point);
            double angle_diff = line_heading - point[2];
            dist_list.push_back(dis);
            angle_diff_list.push_back(angle_diff);
            dist_sum += dis;
            angle_diff_sum += angle_diff;
            if(abs(dis) < min_dis)
            {
              min_cost_index  = i;
              min_dis = abs(dis);
            }



          }
          // int min_cost_index;
          for(int i = 0; i < dist_list.size(); i++)
          {
            dist_list_norm.push_back(dist_list[i]/dist_sum);
            angle_diff_norm.push_back(angle_diff_list[i]/angle_diff_sum);
            total_norm.push_back(dist_list[i]/dist_sum + angle_diff_list[i]/angle_diff_sum);
          }
          if(pub_debug_topics)
          {
            dwa_collsion_free_marker_pub.publish(dwa_path_gen.get_dwa_paths_marker_array(collision_free_paths[min_cost_index], "map"));
            ROS_DEBUG_STREAM("collision_free_paths published");
          }



          autopilot_msgs::Trajectory dwa_traj;
          dwa_traj.header.frame_id = "map";
          nav_msgs::Path dwa_path;
          dwa_path.header.frame_id = "map";
          for(int i = 0;  i< collision_free_paths[min_cost_index].size(); i++)
          {
            autopilot_msgs::TrajectoryPoint traj_point;
            traj_point.pose.position.x = collision_free_paths[min_cost_index][i][0];
            traj_point.pose.position.y = collision_free_paths[min_cost_index][i][1];
            traj_point.pose.orientation = autopilot_utils::get_quaternion_from_yaw(collision_free_paths[min_cost_index][i][2]);
            traj_point.longitudinal_velocity_mps = 1;
            if (i == 0)
            traj_point.accumulated_distance_m = 0;
            else
             traj_point.accumulated_distance_m =  dwa_traj.points.back().accumulated_distance_m + autopilot_utils::distanceBetweenPoses(dwa_traj.points.back().pose, traj_point.pose);
            dwa_traj.points.push_back(traj_point);
            geometry_msgs::PoseStamped pst;
            pst.header.frame_id = "map";
            pst.pose = traj_point.pose;
            dwa_path.poses.push_back(pst);
          }
          local_traj_pub.publish(dwa_traj);
          local_path_pub.publish(dwa_path);
          ROS_INFO("dwa_traj and dwa_path are published");
          }
          
        else{
          // publish the line
          // double line_heading = atan( start_point_center_lane.y-forward_point_center_lane.y /start_point_center_lane.x - forward_point_center_lane.x);
          // double line_heading = atan2(forward_point_center_lane.y - start_point_center_lane.y , forward_point_center_lane.x - start_point_center_lane.x);
          
          autopilot_msgs::Trajectory line_traj;
          line_traj.header.frame_id = "map";
          nav_msgs::Path line_path;
          line_path.header.frame_id = "map";
          for(int i = 0; i<  local_traj_length/dwa_path_resolution; i++ )
          {
            autopilot_msgs::TrajectoryPoint traj_point;
            traj_point.pose.position.x =  start_point_center_lane.x + cos(line_heading)*i*dwa_path_resolution;
            traj_point.pose.position.y =  start_point_center_lane.y + sin(line_heading)*i*dwa_path_resolution;
            traj_point.pose.orientation = autopilot_utils::get_quaternion_from_yaw(line_heading);
            traj_point.longitudinal_velocity_mps = 1;
            traj_point.accumulated_distance_m = i*dwa_path_resolution;
            line_traj.points.push_back(traj_point);
            geometry_msgs::PoseStamped pst;
            pst.header.frame_id = "map";
            pst.pose = traj_point.pose;
            line_path.poses.push_back(pst);
          }
          local_traj_pub.publish(line_traj);
          local_path_pub.publish(line_path);
          ROS_INFO("line_traj and line_path are published");
        }

        }
        loop_rate.sleep();
        ros::spinOnce();
        
    }



}


void AutoNavCore::localScanCallback(const sensor_msgs::LaserScan::ConstPtr& local_scan)
{
    curr_local_scan = local_scan;
    curr_local_scan_data_receiced = true;
}

void AutoNavCore::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    curr_scan = scan;
    curr_scan_data_received = true;
}

void AutoNavCore::trajectoryCallback(const autopilot_msgs::Trajectory::ConstPtr& trajectory)
{
    global_traj = *trajectory;
    global_traj_data_received = true;
    traj_helper.setTrajectory(global_traj);
}

void AutoNavCore::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    curr_odom = odom;
    odom_data_received = true;
    curr_robot_pose = odom->pose.pose;
}


}

int main(int argc, char** argv)
{
  // ros::init(argc, argv, "autonav_core_node", ros::init_options::AnonymousName);
  // ros::init(argc, argv, "autonav_core_node", ros::init_options::AnonymousName);
  // ros::init(argc, argv, "autonav_core_node", ros::init_options::AnonymousName);
  ros::init(argc, argv,"autonav_core_node" );

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   ros::console::notifyLoggerLevelsChanged();
}
  auto_nav::AutoNavCore autonav_core;
 ros::spin();
  
  return 0;
}

