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

#ifndef COSTMAP_GENERATOR_H
#define COSTMAP_GENERATOR_H

// headers in ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
// #include <tf2_ros.h>
// #include <buffer.h>
// #include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include<pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/io.h>
#include <tf2_eigen/tf2_eigen.h>
// #include <tf/transform_listener.h>
// headers in local directory
// #include "vector_map/vector_map.h"
// #include "autoware_msgs/DetectedObjectArray.h"
#include "points_to_costmap.h"
// #include "objects_to_costmap.h"
// #include <pcl_ros/transforms.h>


// headers in STL
#include <memory>

class LocalPlanner
{
public:
  LocalPlanner();
  ~LocalPlanner();

   void init();
   void run();
  private:
   ros::NodeHandle nh_;
   ros::NodeHandle private_nh_;


    void globalPathCallback(const nav_msgs::Path::ConstPtr& in_sensor_points);

    void
#endif  // COSTMAP_GENERATOR_H
