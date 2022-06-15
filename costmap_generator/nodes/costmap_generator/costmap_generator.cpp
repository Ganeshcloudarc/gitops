/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************/

// headers in local directory
// #include "object_map/object_map_utils.hpp"
#include "costmap_generator/costmap_generator.h"

// Constructor
CostmapGenerator::CostmapGenerator()
  : private_nh_("~")
  , has_subscribed_wayarea_(false)
  , OBJECTS_BOX_COSTMAP_LAYER_("objects_box")
  , OBJECTS_CONVEX_HULL_COSTMAP_LAYER_("objects_convex_hull")
  , SENSOR_POINTS_COSTMAP_LAYER_("sensor_points")
  , VECTORMAP_COSTMAP_LAYER_("vectormap")
  , COMBINED_COSTMAP_LAYER_("costmap")
{
}

CostmapGenerator::~CostmapGenerator()
{
}

void CostmapGenerator::init()
{
  private_nh_.param<std::string>("lidar_frame", lidar_frame_, "velodyne");
  private_nh_.param<std::string>("map_frame", map_frame_, "map");
  private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
  private_nh_.param<double>("grid_min_value", grid_min_value_, 0.0);
  private_nh_.param<double>("grid_max_value", grid_max_value_, 1.0);
  private_nh_.param<double>("grid_resolution", grid_resolution_, 0.2);
  private_nh_.param<double>("grid_length_x", grid_length_x_, 50);
  private_nh_.param<double>("grid_length_y", grid_length_y_, 30);
  private_nh_.param<double>("grid_position_x", grid_position_x_, 20);
  private_nh_.param<double>("grid_position_y", grid_position_y_, 0);
  private_nh_.param<double>("maximum_lidar_height_thres", maximum_lidar_height_thres_, 0.3);
  private_nh_.param<double>("minimum_lidar_height_thres", minimum_lidar_height_thres_, -2.2);
  private_nh_.param<bool>("use_objects_box", use_objects_box_, false);
  private_nh_.param<bool>("use_objects_convex_hull", use_objects_convex_hull_, true);
  private_nh_.param<bool>("use_points", use_points_, true);
  private_nh_.param<bool>("use_wayarea", use_wayarea_, true);
  private_nh_.param<double>("expand_polygon_size", expand_polygon_size_, 1.0);
  private_nh_.param<int>("size_of_expansion_kernel", size_of_expansion_kernel_, 9);

  initGridmap();
}

void CostmapGenerator::run()
{
  pub_costmap_ = nh_.advertise<grid_map_msgs::GridMap>("/semantics/costmap", 1);
  pub_occupancy_grid_ = nh_.advertise<nav_msgs::OccupancyGrid>("/semantics/costmap_generator/occupancy_grid", 1);


  if (use_points_)
  {
    sub_points_ = nh_.subscribe("/points_no_ground", 1, &CostmapGenerator::sensorPointsCallback, this);
  }
}


void CostmapGenerator::sensorPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& in_sensor_points_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_sensor_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*in_sensor_points_msg, *in_sensor_points);
  tf::TransformListener listener;
  tf::StampedTransform transform;
  // tf::TransformListener listener;
//  ros::Time now = ros::Time::now();

//  camTfListener->waitForTransform(REF_FRAME, SOURCE_FRAME, ros::Time(0), ros::Duration(10.0),
//                                    ros::Duration(0.01), &error_msg);

  try{
        listener.waitForTransform(base_frame_, lidar_frame_, ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform(base_frame_, lidar_frame_, ros::Time(0), transform);
        }
  catch (tf::TransformException ex){
      ROS_WARN("%s",ex.what());}
  
  // std::cout<<transform;


  // sensor_msgs::PointCloud2 local_pointcloud;

  // try
  // {
  //   pcl_ros::transformPointCloud("/ego_vehicle", in_sensor_points_msg, local_pointcloud, listener);
  // }
  // catch(tf::TransformException& ex)
  // {
  //   ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
  //   return;
  // }
  // try
  //   {
  //     transform_stamped = tf_buffer_1.lookupTransform("base_link",
  //                                                     in_sensor_points_msg->header.frame_id,
  //                                                     ros::Time(0),
  //                                                     ros::Duration(2));
  //   }
  // catch (tf2::TransformException &ex)
  //   {
  //     ROS_ERROR("%s",ex.what());
  //     return;
  //   }

    // Eigen::Affine3d pc_to_base_link = tf2::transformToEigen(transform_stamped);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    // PointCloud::Ptr fpTransformed(new PointCloud);
    // PointCloud2::Ptr cloud_transformed (new PointCloud2 ());

  //   PointCloud2::Ptr in1_t (new PointCloud2 ());

  //   pcl_ros::transformPointCloud (in_sensor_points_msg, in1, *in1_t, tf_);
  //   if (output_frame_ != in1.header.frame_id)
  //   pcl_ros::transformPointCloud (output_frame_, in1, *in1_t, tf_);
  // else

    pcl_ros::transformPointCloud( *in_sensor_points, *cloud_transformed,transform);



  // Eigen::Affine3d pc_to_base_link = tf2::transformToEigen(transform_stamped);
  // pcl::transformPointCloud(*input_cloud, *input_cloud, pc_to_base_link);


  costmap_[SENSOR_POINTS_COSTMAP_LAYER_] = generateSensorPointsCostmap(cloud_transformed);
  // costmap_[VECTORMAP_COSTMAP_LAYER_] = generateVectormapCostmap();
  // costmap_[COMBINED_COSTMAP_LAYER_] = generateCombinedCostmap();
  // in_sensor_points_msg->header->frame_id = "base_link";
  std_msgs::Header in_header;
  // = in_sensor_points_msg->header;
  in_header.frame_id ="ego_vehicle";
  in_header.stamp = ros::Time::now();
  // in_header->header -
  publishRosMsg(costmap_, in_header);
}

void CostmapGenerator::initGridmap()
{
  costmap_.setFrameId(lidar_frame_);
  costmap_.setGeometry(grid_map::Length(grid_length_x_, grid_length_y_), grid_resolution_,
                       grid_map::Position(grid_position_x_, grid_position_y_));

  costmap_.add(SENSOR_POINTS_COSTMAP_LAYER_, grid_min_value_);
  // costmap_.add(OBJECTS_BOX_COSTMAP_LAYER_, grid_min_value_);
  // costmap_.add(OBJECTS_CONVEX_HULL_COSTMAP_LAYER_, grid_min_value_);
  // costmap_.add(VECTORMAP_COSTMAP_LAYER_, grid_min_value_);
  // costmap_.add(COMBINED_COSTMAP_LAYER_, grid_min_value_);
}

grid_map::Matrix
CostmapGenerator::generateSensorPointsCostmap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_sensor_points)
{
  grid_map::Matrix sensor_points_costmap = points2costmap_.makeCostmapFromSensorPoints(
      maximum_lidar_height_thres_, minimum_lidar_height_thres_, grid_min_value_, grid_max_value_, costmap_,
      SENSOR_POINTS_COSTMAP_LAYER_, in_sensor_points);
  return sensor_points_costmap;
}






void CostmapGenerator::publishRosMsg(const grid_map::GridMap& costmap, const std_msgs::Header& in_header)
{
  nav_msgs::OccupancyGrid out_occupancy_grid;
  grid_map::GridMapRosConverter::toOccupancyGrid(costmap, SENSOR_POINTS_COSTMAP_LAYER_, grid_min_value_, grid_max_value_,
                                                 out_occupancy_grid);
  out_occupancy_grid.header = in_header;
  pub_occupancy_grid_.publish(out_occupancy_grid);

  grid_map_msgs::GridMap out_gridmap_msg;
  grid_map::GridMapRosConverter::toMessage(costmap, out_gridmap_msg);
  out_gridmap_msg.info.header = in_header;
  pub_costmap_.publish(out_gridmap_msg);
}
