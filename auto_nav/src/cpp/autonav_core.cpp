#include<auto_nav/autonav_core.h>



AutoNavCore::AutoNavCore() : tf2_listener(tf2_buffer),
odom_data_received(false), global_traj_data_received(false), curr_scan_data_received(false), curr_local_scan_data_receiced(false), in_turn_status(false)

{
  ros::NodeHandle private_nh("~");
  loadParams(private_nh);
  dwa_path_gen.update_params(dwa_wheel_base, dwa_steering_agle_lim, dwa_constant_speed, dwa_path_len, dwa_path_resolution);

  if(use_dwa)
    costmap_ros_ = new costmap_2d::Costmap2DROS("/costmap", tf2_buffer);
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
  dwa_best_traj_marker_pub = private_nh.advertise<visualization_msgs::MarkerArray>("/dwa_best_traj", 1,true);

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
  private_nh.param<float>("turnings/minimum_turn_radius", minimum_turn_radius, 7);
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
  // IMP lines for dubins path 
  // https://github.com/karlkurzer/path_planner/blob/887996746c275e2c8335ae77ea096969079e688f/src/algorithm.cpp#L463

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
          // ROS_WARN_STREAM("curr_local_scan_data_receiced" + curr_local_scan_data_receiced );//"  and curr_scan_data_received and global_traj_data_received and odom_data_received
          rate.sleep();
          ros::spinOnce();
        }
    }

    ros::Rate loop_rate(loop_frequency);
    
   
    
    
    while(ros::ok())
    { 
      marker_arr.markers.clear();
      
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
        path_percent_val.data = (global_traj.points[close_index].accumulated_distance_m /
                            global_traj.points.back().accumulated_distance_m) * 100;
        path_percent_publisher.publish(path_percent_val);
        if ( path_percent_val.data > 95.0 and distanceBetweenPoses(curr_robot_pose, global_traj.points.back().pose) < row_spacing)
        {
          if (mission_continue)
          {
            ROS_INFO("mission completed");
            ROS_INFO("Restarting the mission");
            close_index = 1;
            loop_rate.sleep();
            ros::spinOnce();
            in_turn_status = false;
            turn_start_index = -1;
            turn_end_index = -1;
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
        int p1_index = traj_helper.next_point_within_dist(close_index, forward_point_dis);

        int p2_index = traj_helper.next_point_within_dist(p1_index, forward_point_dis);
        
        int p3_index = traj_helper.next_point_within_dist(p2_index, forward_point_dis);
        vector<double> p1 = {traj_helper.get_trajectory_point_by_index(p1_index).pose.position.x, traj_helper.get_trajectory_point_by_index(p1_index).pose.position.y};
        vector<double> p2 = {traj_helper.get_trajectory_point_by_index(p2_index).pose.position.x, traj_helper.get_trajectory_point_by_index(p2_index).pose.position.y};
        vector<double> p3 = {traj_helper.get_trajectory_point_by_index(p3_index).pose.position.x, traj_helper.get_trajectory_point_by_index(p3_index).pose.position.y};
        float circum_radius = circumRadius(p1,p2,p3);
        ROS_DEBUG_STREAM("circum_radius : "<<circum_radius);
        auto close_pose = traj_helper.get_trajectory_point_by_index(close_index).pose;
        Vector3d start_point_v(cos(tf::getYaw(close_pose.orientation)), sin(tf::getYaw(close_pose.orientation)), 0);
        /* Auto Turnings 
        if (abs(circum_radius) < radius_to_check_turn)
        { 
          global_turn_detected = true;
          ROS_WARN("Turn detected");
        }
        else{
          global_turn_detected = false;
        }

        Auto Turnings end*/ 

        if (abs(circum_radius) < radius_to_check_turn or in_turn_status == true )
        { 
        
          ROS_WARN("Turn detected");
          if (turn_start_index == -1 and turn_end_index == -1)
          {
            turn_start_index = close_index;
            for(int i = turn_start_index; i <traj_helper.getLength(); i++)
            { 
              auto traj_pose= traj_helper.get_trajectory_point_by_index(i).pose;
              Vector3d end_point_v(traj_pose.position.x- close_pose.position.x, traj_pose.position.y- close_pose.position.y, 0);
              float dot_pro = start_point_v.dot(end_point_v);
              ROS_DEBUG_STREAM("dot pro : " <<dot_pro);
              if (dot_pro < 0)
              {
                
                break;
              }
              
              turn_end_index = i;
              in_turn_status = true;
          }}
          if (close_index > turn_end_index)
            {in_turn_status = false;
            turn_start_index = -1;
             turn_end_index = -1;

            }
          autopilot_msgs::Trajectory turn_traj;
          nav_msgs::Path turn_path;
          turn_traj.header.frame_id = "map";
          turn_path.header.frame_id = "map";
          int loop_end_index;
          if(turn_end_index+60 < traj_helper.getLength())
              loop_end_index = turn_end_index+60;
          else
              loop_end_index = traj_helper.getLength();
          for(int i = close_index; i <loop_end_index; i++)
          { 
            ROS_DEBUG_STREAM("i :" << i);
            ROS_DEBUG_STREAM("lenght :" <<traj_helper.getLength());
            turn_traj.points.push_back(traj_helper.get_trajectory_point_by_index(i));
            geometry_msgs::PoseStamped pst;
            pst.header.frame_id = "map";
            pst.pose = traj_helper.get_trajectory_point_by_index(i).pose;
            turn_path.poses.push_back(pst);
          }
          local_traj_pub.publish(turn_traj);
          local_path_pub.publish(turn_path);

          ROS_INFO("Turn trajectory published");
          slope_list.clear();
          intercept_list.clear();
        }
        

      
        /* Auto Turnings start
       if ((global_turn_detected or  in_turn_status) and !row_end_point.empty())
        {
            // check to initiate dubins curve or path till the row_end is reached
            ROS_ERROR("inside the if condition of turn");
            in_turn_status = true;
            if (moving_avg_center_line.valid())
            {
              // checking whether vehicle crossed the row_end_point
              Eigen::Vector3d end_point_to_robot {row_end_point[0]-curr_robot_pose.position.x, row_end_point[1]-curr_robot_pose.position.y , 0};
              Eigen::Vector3d  unit_vector = unit_vect(moving_avg_center_line.getHeadingAngle());
              double dot_pro = end_point_to_robot.dot(unit_vector);
              cout<<"dot_pro"<<dot_pro<<endl;
              if (dot_pro > 0)
              {
                // Both opposuite direction , vehicle is still in the row send the avg_line points
              

              vector<double> start_point_center_lane = moving_avg_center_line.intersct_point_to_line(std::vector<double>{curr_robot_pose.position.x, curr_robot_pose.position.y});
              vector<double> forward_point_center_lane = moving_avg_center_line.intersct_point_to_line(std::vector<double>{row_end_point[0], row_end_point[1]});
              cout<<"before"<<endl;
              local_trajectory_msg = twoPointsToTrajectory(start_point_center_lane, forward_point_center_lane, 0.1,"map", speed); // speed 1
              cout<<"afer"<<endl;
              local_traj_pub.publish(local_trajectory_msg);
              local_path_pub.publish(trajectoryToPath(local_trajectory_msg));
              ROS_DEBUG_STREAM("traj and path are published till the end of row");


              }
              else
              {
                
                //vehicle crossed the row
                // Start Dubins path from the start point to next_row_start_point;

                // Dubins curves connecting mid points
          double q0[] = {row_end_point[0], row_end_point[1], row_end_point[2]};
          Eigen::Vector3d dir_vect =  {p1[0]- row_end_point[0], p1[1]- row_end_point[1], 0};

          
          double crossProduct = unit_vector.x() * dir_vect.y() - unit_vector.y() * dir_vect.x();

          double theta;
          if(crossProduct<0)
            theta = -M_PI/2;
          else 
            theta = M_PI/2;
          
          cout<<"cross_pro :"<< crossProduct<<endl;
          cout<<"theta :"<< theta<<endl;
          
          
          Eigen::Vector3d vect =  unit_vector * (2*row_spacing);
          double x_scaled = vect.x() * cos(theta) - vect.y() * sin(theta);
          double y_scaled = vect.x() * sin(theta) + vect.y() * cos(theta);
          double q1[] = {row_end_point[0]+x_scaled, row_end_point[1]+y_scaled, row_end_point[2]+ M_PI };

          
          DubinsPath path;
          visualization_msgs::Marker DubinsMarker;
          DubinsMarker.header.frame_id = "map";
          DubinsMarker.type =  DubinsMarker.LINE_STRIP;
          DubinsMarker.id = 15;
          
          DubinsMarker.color.g = 1.0;
          DubinsMarker.color.b = 0;
          DubinsMarker.color.a = 1;
          DubinsMarker.scale.x = 0.3;
          DubinsMarker.pose.orientation.w = 1;

          local_trajectory_msg.points.clear();

          // calculate the path
          dubins_init(q0, q1, minimum_turn_radius, &path);
          int i = 0;
          float x = 0.f;
          float length = dubins_path_length(&path);
          double dubinsStepSize  = 0.1;
          x+= dubinsStepSize;
          unsigned int mx,my;
          bool collision_found = false;
          int cost; 
          double min_dist = 1000000;
          double mix_x = 0;

          while (x <  length) {
          double q[3];
          dubins_path_sample(&path, x, q);
          double dist = sqrt(pow(curr_robot_pose.position.x - q[0], 2) + 
                          pow(curr_robot_pose.position.y - q[1],2));
          if (dist < min_dist)
            {
              min_dist = dist;
              mix_x = x;
            }
            x+= dubinsStepSize;
          }
          cout<<"mix_x : "<<mix_x<<endl;
          while (mix_x <  length) {
            cout<<"mix_x" <<mix_x<<endl;
            double q[3];
            dubins_path_sample(&path, mix_x, q);
            // end_point_to_robot = {q[0]-curr_robot_pose.position.x, q[1]-curr_robot_pose.position.y , 0};
            // double dist = sqrt(pow(curr_robot_pose.position.x - q[0], 2) + 
            //               pow(curr_robot_pose.position.y - q[1],2));
            // Eigen::Vector3d  v_dubins = unit_vect(q[2]);

            // Eigen::Vector3d  v_robot = unit_vect(tf::getYaw(curr_robot_pose.orientation));
            // dot_pro = end_point_to_robot.dot(v_dubins);
            // double robot_dot = v_dubins.dot(v_robot);


            // if (dot_pro > 0 and robot_dot > 0)
            //   { 
                costmap_->worldToMap(q[0], q[1], mx, my);
                cost = costmap_->getCost(mx, my);
                // cout<<"cost"<<static_cast<unsigned int>(cost)<<endl;
                if (cost != 0 ){
                
                collision_found  = true;
                }

                // }
                 DubinsMarker.points.push_back(VectorToPoint(q));
            local_trajectory_msg.points.push_back(DubinsPointToTrajectoryPoint(q, mix_x, speed));


              


           

            mix_x+= dubinsStepSize;
          }
          if (collision_found)
            {
              ROS_ERROR("collision found on dubins path");
              ROS_ERROR("stopping the vehicle");
              for(int i=0; i<local_trajectory_msg.points.size();i++)
                local_trajectory_msg.points[i].longitudinal_velocity_mps = 0;
        
              DubinsMarker.color.r = 1;
              DubinsMarker.color.g = 0.0; 
            }

          end_point_to_robot = {local_trajectory_msg.points.back().pose.position.x-curr_robot_pose.position.x, local_trajectory_msg.points.back().pose.position.y-curr_robot_pose.position.y , 0};
          double dis = distanceBetweenPoses(curr_robot_pose,local_trajectory_msg.points.back().pose );
          
          Eigen::Vector3d  unit_vector = unit_vect(tf::getYaw(local_trajectory_msg.points.back().pose.orientation));
          dot_pro = end_point_to_robot.dot(unit_vector);
          if (dot_pro > 0 and dis < 1)
            {
              ROS_INFO("Completed Dubins path");
              in_turn_status = false;
              row_end_point.clear();
              slope_list.clear();
              intercept_list.clear();


            }
            

          marker_arr.markers.push_back(DubinsMarker);

          local_traj_pub.publish(local_trajectory_msg);
          local_path_pub.publish(trajectoryToPath(local_trajectory_msg));
          ROS_DEBUG_STREAM("Dubins path published ");

              }
            }
            else{
              ROS_INFO("No valid center line found");
            }
        }
         Auto Turnings end*/
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
              if (abs(distance_to_line) < row_spacing)
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
          geometry_msgs::Point point1_center_map, point2_center_map, point1_left_map, point2_left_map, point1_right_map, point2_right_map;
          try
          {
            transform_to_base_link = tf2_buffer.lookupTransform("map", "base_link", ros::Time(0));
            tf2::doTransform(point1_center, point1_center_map, transform_to_base_link);
            tf2::doTransform(point2_center, point2_center_map, transform_to_base_link);
            tf2::doTransform(point1_left, point1_left_map, transform_to_base_link);
            tf2::doTransform(point2_left, point2_left_map, transform_to_base_link);
            tf2::doTransform(point1_right, point1_right_map, transform_to_base_link);
            tf2::doTransform(point2_right, point2_right_map, transform_to_base_link);
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
          robot_forward_point.x = curr_robot_pose.position.x + cos(tf::getYaw(curr_robot_pose.orientation)) * (local_traj_length);
          robot_forward_point.y = curr_robot_pose.position.y + sin(tf::getYaw(curr_robot_pose.orientation)) * (local_traj_length);
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
          moving_avg_center_line.update_line(moving_avg_slope, moving_avg_intercept);
          moving_avg_center_line.updateHeading(line_heading);
          
          if (pub_debug_topics){
          geometry_msgs::PoseStamped ps;
          ps.header.frame_id = "map";
          ps.pose.position.x = start_point_center_lane.x;
          ps.pose.position.y = start_point_center_lane.y;
          ps.pose.orientation = get_quaternion_from_yaw(line_heading);
          vibration_path.poses.push_back(ps);
          vibration_path_pub.publish(vibration_path);
          ROS_DEBUG("vibration path is puslished");
          }


          
          if(use_dwa)
          {

            //initialize the copy of the costmap the controller will use
            costmap_ = costmap_ros_->getCostmap();
            // costmap_->raytraceLine(start_point_center_lane.x, start_point_center_lane.y, forward_point_center_lane.x, forward_point_center_lane.y);

          // Checking collsions on center line on costmap
          pair<bool, unsigned char> center_collision = raytraceLineCost(*costmap_, start_point_center_lane.x, start_point_center_lane.y, forward_point_center_lane.x, forward_point_center_lane.y);
          ROS_DEBUG_STREAM("first" <<center_collision.first <<" second : " <<static_cast<unsigned int>(center_collision.second));
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

          // Finding the row end detection using left lane and right lanes
          Eigen::Vector3d c1(start_point_center_lane.x, start_point_center_lane.y, 0);
          Eigen::Vector3d c2(forward_point_center_lane.x, forward_point_center_lane.y, 0);
          double x_scaled, y_scaled;
          Eigen::Vector3d vect;
          vect = c2 - c1;
          vect =  vect.normalized() * row_spacing/2;
          x_scaled = vect.x() * cos(M_PI/2) - vect.y() * sin(M_PI/2);
          y_scaled = vect.x() * sin(M_PI/2) + vect.y() * cos(M_PI/2);
          Eigen::Vector3d l1(c1.x() + x_scaled, c1.y() + y_scaled, 0.0  );
          Eigen::Vector3d l2(c2.x() + x_scaled, c2.y() + y_scaled, 0.0  ); 
        
          x_scaled = vect.x() * cos(-M_PI/2) - vect.y() * sin(-M_PI/2);
          y_scaled = vect.x() * sin(-M_PI/2) + vect.y() * cos(-M_PI/2);

          Eigen::Vector3d r1(c1.x() + x_scaled, c1.y() + y_scaled, 0.0  );
          Eigen::Vector3d r2(c2.x() + x_scaled, c2.y() + y_scaled, 0.0  );

          // Calculate next rows aswell, ll1, ll2, rr1, rr2
          vect =  vect.normalized() * (row_spacing/2+row_spacing);
          x_scaled = vect.x() * cos(M_PI/2) - vect.y() * sin(M_PI/2);
          y_scaled = vect.x() * sin(M_PI/2) + vect.y() * cos(M_PI/2);
          Eigen::Vector3d ll1(c1.x() + x_scaled, c1.y() + y_scaled, 0.0  );
          Eigen::Vector3d ll2(c2.x() + x_scaled, c2.y() + y_scaled, 0.0  ); 
          x_scaled = vect.x() * cos(-M_PI/2) - vect.y() * sin(-M_PI/2);
          y_scaled = vect.x() * sin(-M_PI/2) + vect.y() * cos(-M_PI/2);
          Eigen::Vector3d rr1(c1.x() + x_scaled, c1.y() + y_scaled, 0.0  );
          Eigen::Vector3d rr2(c2.x() + x_scaled, c2.y() + y_scaled, 0.0  );

          


          cout<<"l1:  "<< l1 << " l2 :"<< l2<<endl;
          cout<<"r1:  "<< r1 << " r2 :"<< r2<<endl;
          // Moving avg center line
          visualization_msgs::Marker marker_sides_avg_filter;
          marker_sides_avg_filter.header.frame_id = "map";
          marker_sides_avg_filter.type =  marker_right.LINE_LIST;
          marker_sides_avg_filter.id = 4;
           marker_sides_avg_filter.color.r = 0.5;
          marker_sides_avg_filter.color.g = 1;
          marker_sides_avg_filter.color.b = 0.5;
          marker_sides_avg_filter.color.a = 1;
          marker_sides_avg_filter.scale.x = 0.3;
          marker_sides_avg_filter.pose.orientation.w = 1;
          marker_sides_avg_filter.points.push_back(VectorToPoint(l1));
          marker_sides_avg_filter.points.push_back(VectorToPoint(l2));
           marker_sides_avg_filter.points.push_back(VectorToPoint(ll1));
          marker_sides_avg_filter.points.push_back(VectorToPoint(ll2));

          marker_sides_avg_filter.points.push_back(VectorToPoint(r1));
          marker_sides_avg_filter.points.push_back(VectorToPoint(r2));
          marker_sides_avg_filter.points.push_back(VectorToPoint(rr1));
          marker_sides_avg_filter.points.push_back(VectorToPoint(rr2));

          marker_arr.markers.push_back(marker_sides_avg_filter);
          
          // ray trace and find the end of row 
          // unsigned int mx1,my1,mx2,my2;
          // if (costmap.worldToMap(x1, y1, mx1, my1) and costmap.worldToMap(x2, y2, mx2, my2))
          // {
            
          // }
          double lx_free, ly_free,llx_free,lly_free, rx_free, ry_free,rrx_free,rry_free;
          
          if (raytraceLineLookGap(*costmap_, l1.x(), l1.y(), l2.x(), l2.y(), lx_free, ly_free, 1))
          {
            ROS_ERROR_STREAM("Free left end points detected x :" <<lx_free <<" y:"<< ly_free );
          }
          else{
            ROS_INFO_STREAM("No left free end points detected");
          }
          marker_arr.markers.push_back(xy_to_marker(lx_free,ly_free, 10));
          if (raytraceLineLookGap(*costmap_, ll1.x(), ll1.y(), ll2.x(), ll2.y(), llx_free, lly_free, 1))
                  {
                    ROS_ERROR_STREAM("Free left end points detected x :" <<llx_free <<" y:"<< lly_free );
                  }
          else{
            ROS_INFO_STREAM("No left free end points detected");
          }
          marker_arr.markers.push_back(xy_to_marker(llx_free,lly_free, 11));


           if (raytraceLineLookGap(*costmap_, r1.x(), r1.y(), r2.x(), r2.y(), rx_free, ry_free,1))
          {
            ROS_ERROR_STREAM("Free right end points detected x :" <<rx_free <<" y:"<< ry_free );
          }
          else{
            ROS_INFO_STREAM("No right free end points detected");
          }
          marker_arr.markers.push_back(xy_to_marker(rx_free,ry_free,12));

          if (raytraceLineLookGap(*costmap_, rr1.x(), rr1.y(), rr2.x(), rr2.y(), rrx_free, rry_free,1))
          {
            ROS_ERROR_STREAM("Free right end points detected x :" <<rrx_free <<" y:"<< rry_free );
          }
          else{
            ROS_INFO_STREAM("No right free end points detected");
          }
          marker_arr.markers.push_back(xy_to_marker(rrx_free,rry_free,13));

          // Dubins curves connecting mid points
          double q0[] = {(lx_free+ rx_free)/2, (ly_free+ry_free)/2, line_heading};
          vect =  vect.normalized() * (2*row_spacing);
          x_scaled = vect.x() * cos(-M_PI/2) - vect.y() * sin(-M_PI/2);
          y_scaled = vect.x() * sin(-M_PI/2) + vect.y() * cos(-M_PI/2);
          double q1[] = {(lx_free+ rx_free)/2 +x_scaled, (ly_free+ry_free)/2+ y_scaled, line_heading+ M_PI };

          row_end_point  = {(lx_free+ rx_free)/2, (ly_free+ry_free)/2, line_heading};
          
          DubinsPath path;
          visualization_msgs::Marker DubinsMarker;
          DubinsMarker.header.frame_id = "map";
          DubinsMarker.type =  marker_right.LINE_STRIP;
          DubinsMarker.id = 15;
          
          DubinsMarker.color.g = 1.0;
          DubinsMarker.color.b = 0;
          DubinsMarker.color.a = 1;
          DubinsMarker.scale.x = 0.3;
          DubinsMarker.pose.orientation.w = 1;
          
          // calculate the path
          dubins_init(q0, q1, minimum_turn_radius, &path);
          int i = 0;
          float x = 0.f;
          float length = dubins_path_length(&path);
          double dubinsStepSize  = 0.1;
          x+= dubinsStepSize;
          unsigned int mx,my;
          while (x <  length) {
            double q[3];
            dubins_path_sample(&path, x, q);
            costmap_->worldToMap(q[0], q[1], mx, my);
            int cost = costmap_->getCost(mx, my);
            // cout<<"cost"<<static_cast<unsigned int>(cost)<<endl;
            if (cost != 0 ){
              ROS_ERROR("COLLISION FOUND ON DUBINS CURVES");
              DubinsMarker.color.r = 1;
              DubinsMarker.color.g = 0.0;
              break;
            }

            DubinsMarker.points.push_back(VectorToPoint(q));
            x+= dubinsStepSize;

          

          }
          marker_arr.markers.push_back(DubinsMarker);







        

        
          ROS_DEBUG("lane markers puslished");


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
                
              if (costmap_->worldToMap(dwa_paths[i][j][0], dwa_paths[i][j][1], mx, my))
              { 
                
                // ROS_DEBUG_STREAM("cell cost : "<<static_cast<unsigned int>(occ_manager->costmap_2d.getCost(mx, my)));
                if (costmap_->getCost(mx, my) != 0)
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
          ROS_DEBUG_STREAM("costmap reset status : "<< reset_costmap );
          if (collision_free_path_ids.size() == 0)
          {
            ROS_ERROR("Could not found collision free  published");
            // costmap_ = costmap_ros_->getCostmap();
            if (reset_costmap == true)
             { costmap_ros_->resetLayers();
               reset_costmap = false;
             }
            loop_rate.sleep();
            ros::spinOnce();
            continue;

          }
          reset_costmap = true;

          if(pub_debug_topics){
          dwa_collsion_free_marker_pub.publish(dwa_path_gen.get_dwa_paths_marker_array(collision_free_paths, "map"));
          ROS_DEBUG_STREAM("collision_free_paths published");
          }
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
            dwa_best_traj_marker_pub.publish(dwa_path_gen.get_dwa_paths_marker_array(collision_free_paths[min_cost_index], "map"));
            ROS_DEBUG_STREAM("DWA_best_traj published");
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
            traj_point.pose.orientation = get_quaternion_from_yaw(collision_free_paths[min_cost_index][i][2]);
            traj_point.longitudinal_velocity_mps = 1;
            if (i == 0)
            traj_point.accumulated_distance_m = 0;
            else
             traj_point.accumulated_distance_m =  dwa_traj.points.back().accumulated_distance_m + distanceBetweenPoses(dwa_traj.points.back().pose, traj_point.pose);
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
            traj_point.pose.orientation = get_quaternion_from_yaw(line_heading);
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
        lanes_marker_pub.publish(marker_arr);
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

