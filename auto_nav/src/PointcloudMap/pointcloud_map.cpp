// #include"lidar_obstacle_detector/obstacle_detector.hpp"
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// #include<pcl_ros/transforms.hpp>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <ros/console.h>
#include<cmath>
namespace LocalPointCloudMap
{
  
class LocalPointCloudMapNode
{   public:
        LocalPointCloudMapNode();
        virtual ~LocalPointCloudMapNode() {};
        private:    
        ros::NodeHandle nh;
        tf2_ros::Buffer tf2_buffer;
        tf2_ros::TransformListener tf2_listener;
        ros::Subscriber sub_lidar_points, sub_odom;
        ros::Publisher pub_local_cloud_map,pub_global_cloud_map;
        void lidarPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_points);
        void odomCallback(const nav_msgs::Odometry::ConstPtr & odom);
        void loadParams();
        nav_msgs::Odometry curr_odom;
        bool odom_recrived = false; 
        sensor_msgs::PointCloud2 raw_cloud, cloud_in_base_frame;
        pcl::PointCloud<pcl::PointXYZ> final_cloud;
        sensor_msgs::PointCloud2 final_ros;


        bool enable_global_cloud_map,enable_local_cloud_map;
        std::string local_cloud_map_topic, global_cloud_map_topic,odometry_topic;
        std::string local_frame, global_frame;

        float voxel_size;
        float local_map_size, min_height_local_frame, max_height_local_frame;
        float local_map_size_x, local_map_size_y;
        // float roi_max_x,roi_max_y, roi_max_z,roi_min_x,roi_min_y,roi_min_z;

};

void LocalPointCloudMapNode::loadParams()
    {
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("odom_topic", odometry_topic, "vehicle/odom");
    private_nh.param<std::string>("global_frame", global_frame, "map");
    private_nh.param<std::string>("local_frame", local_frame, "base_link");
    private_nh.param<bool>("enable_global_cloud_map", enable_global_cloud_map, true);///zed2i/zed_node/point_cloud/cloud_registered
    private_nh.param<bool>("enable_local_cloud_map", enable_local_cloud_map, true);///zed2i/zed_node/point_cloud/cloud_registered
    private_nh.param<std::string>("global_cloud_map_topic", global_cloud_map_topic, "global_cloud_map");

    private_nh.param<float>("voxel_size", voxel_size, 0.1);///zed2i/zed_node/point_cloud/cloud_registered
    private_nh.param<std::string>("local_cloud_map_topic", local_cloud_map_topic, "local_cloud_map");
//    private_nh.param<float>("local_map_size", local_map_size, 30.0);
    private_nh.param<float>("local_map_size_x", local_map_size_x, 30.0);
    private_nh.param<float>("local_map_size_y", local_map_size_y, 30.0);


    private_nh.param<float>("min_height_local_frame", min_height_local_frame,0.0);
    private_nh.param<float>("max_height_local_frame", max_height_local_frame, 2);



    }


LocalPointCloudMapNode::LocalPointCloudMapNode() : tf2_listener(tf2_buffer)
{
  ros::NodeHandle private_nh("~");
  loadParams();
  std::string input_cloud;
  private_nh.param<std::string>("input_cloud", input_cloud, "rslidar_points");///zed2i/zed_node/point_cloud/cloud_registered

  sub_lidar_points = nh.subscribe(input_cloud, 1, &LocalPointCloudMapNode::lidarPointsCallback, this);
  sub_odom = nh.subscribe(odometry_topic, 1, &LocalPointCloudMapNode::odomCallback, this);

  pub_local_cloud_map = nh.advertise<sensor_msgs::PointCloud2>(local_cloud_map_topic, 1); 
  if (enable_global_cloud_map)
    pub_global_cloud_map = nh.advertise<sensor_msgs::PointCloud2>(global_cloud_map_topic, 1);  

  // pub_base_cloud = nh.advertise<sensor_msgs::PointCloud2>("cloud_in_base_link", 1);  

  pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // float roi_max_x,roi_max_y, roi_max_z,roi_min_x,roi_min_y,roi_min_z;
}

void LocalPointCloudMapNode::lidarPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_points)
{

    ROS_DEBUG_ONCE("lidar callback called");
    raw_cloud = *lidar_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*lidar_points, *points_raw);

    // pub_local_cloud_map.publish(lidar_points);
    if (odom_recrived==false)
    {
      ROS_WARN("WAITING FOR ODOM DATA");
      return ;
    }   
    // Step 2: Downsample the point cloud to sparse point cloud

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZ>);


    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(points_raw);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size); // Adjust the leaf size as per requirement
    sor.filter(*downsampledCloud);
    sensor_msgs::PointCloud2 voxel_filterd_cloud, cloud_in_global_frame;

    pcl::toROSMsg(*downsampledCloud, voxel_filterd_cloud);
    voxel_filterd_cloud.header = lidar_points->header;
  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf2_buffer.lookupTransform(global_frame, lidar_points->header.frame_id, ros::Time(0));
    tf2::doTransform(voxel_filterd_cloud,cloud_in_global_frame, transform_stamped);

  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return ;
  }
  cloud_in_global_frame.header.frame_id = global_frame;
  // pub_base_cloud.publish(cloud_in_global_frame);

  pcl::PointCloud<pcl::PointXYZ> cloud_in_global_frame_pcl;

  pcl::fromROSMsg(cloud_in_global_frame, cloud_in_global_frame_pcl);

  final_cloud += cloud_in_global_frame_pcl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_in(new pcl::PointCloud<pcl::PointXYZ>);
    *voxel_in = final_cloud;
    pcl::VoxelGrid<pcl::PointXYZ> sor1;
    sor.setInputCloud(voxel_in);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size); // Adjust the leaf size as per requirement
    sor.filter(final_cloud);
    if (enable_global_cloud_map)
    {
    pcl::toROSMsg(final_cloud, final_ros);
    final_ros.header.frame_id =global_frame;
    pub_global_cloud_map.publish(final_ros);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_roi = final_cloud;
    pcl::CropBox<pcl::PointXYZ> region(false);
    region.setMin(Eigen::Vector4f(curr_odom.pose.pose.position.x-local_map_size_x, curr_odom.pose.pose.position.y-local_map_size_y, min_height_local_frame, 1));
    region.setMax(Eigen::Vector4f(curr_odom.pose.pose.position.x+local_map_size_x, curr_odom.pose.pose.position.y+local_map_size_y, max_height_local_frame, 1));
    region.setInputCloud(cloud_roi);
    region.filter(*cloud_roi);
    sensor_msgs::PointCloud2 local_cloud_ros;
    pcl::toROSMsg(*cloud_roi, local_cloud_ros);
    if (!enable_global_cloud_map)
    final_cloud = *cloud_roi;
    
    local_cloud_ros.header.frame_id = global_frame;
    geometry_msgs::TransformStamped transform_to_local_frame;
    sensor_msgs::PointCloud2  cloud_in_local_frame;

    try
    {
      transform_to_local_frame = tf2_buffer.lookupTransform(local_frame, local_cloud_ros.header.frame_id, ros::Time(0));
      tf2::doTransform(local_cloud_ros,cloud_in_local_frame, transform_to_local_frame);

    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return ;
    }
    cloud_in_local_frame.header.frame_id = local_frame;
      pub_local_cloud_map.publish(cloud_in_local_frame);
        // ROS_DEBUG_STREAM("final_cloud after pass thorough "<< final_cloud.size());
    //  pcl::fromROSMsg(cloud_in_local_frame, final_cloud);
 
}

void LocalPointCloudMapNode::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    ROS_DEBUG_ONCE("odom callback called");
    odom_recrived = true;
    curr_odom = *odom_msg;
  
}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_pointcloud_map_node", ros::init_options::AnonymousName);
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   ros::console::notifyLoggerLevelsChanged();
}
  LocalPointCloudMap::LocalPointCloudMapNode local_point_cloud_map_node;
 ros::spin();
  
  return 0;
}