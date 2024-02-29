#include "lidar_obstacle_detector/obstacle_detector.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
// #include<pcl_ros/transforms.hpp>
#include <cmath>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <mavros_msgs/HomePosition.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <ros/console.h>
#include <ros/package.h>

// #include<lidar_obstacle_detector/obstacle_detector.h>
using namespace lidar_obstacle_detector;
using namespace std;

namespace LocalPointCloudMap {
class LocalPointCloudMapNode {
public:
  LocalPointCloudMapNode();
  virtual ~LocalPointCloudMapNode(){};

private:
  ros::NodeHandle nh;
  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener;
  ros::Subscriber sub_lidar_points, sub_odom, sub_gps_home_position;
  ros::Publisher pub_local_cloud_map, pub_global_cloud_map, local_bboxes_pub,
      tree_cloud_pub;
  void
  lidarPointsCallback(const sensor_msgs::PointCloud2::ConstPtr &lidar_points);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom);
  void gpsHomepositionCallback(
      const mavros_msgs::HomePosition::ConstPtr &home_position);
  void loadParams();
  bool savePclfile(string path, pcl::PointCloud<pcl::PointXYZ> tree_cloud);

  nav_msgs::Odometry curr_odom;
  mavros_msgs::HomePosition gps_home_position_data;
  bool odom_received = false;
  bool gps_home_position_received = false;
  sensor_msgs::PointCloud2 raw_cloud, cloud_in_base_frame;
  pcl::PointCloud<pcl::PointXYZ> final_cloud;
  // pcl::PointCloud<pcl::PointXYZ> tree_cloud;
  pcl::PointCloud<pcl::PointXYZ> tree_cloud;

  sensor_msgs::PointCloud2 final_ros;

  bool enable_global_cloud_map, enable_local_cloud_map, enable_tree_mapping;
  std::string input_cloud_topic, local_cloud_map_topic, global_cloud_map_topic,
      odometry_topic, tree_mapping_topic, gps_home_position_topic;
  std::string local_frame, global_frame;

  float voxel_size;
  float local_map_size, min_height_local_frame, max_height_local_frame;
  float local_map_size_x, local_map_size_y;
  // float roi_max_x,roi_max_y, roi_max_z,roi_min_x,roi_min_y,roi_min_z;
  std::shared_ptr<ObstacleDetector<pcl::PointXYZ>> obstacle_detector;
  std::vector<Box> prev_boxes_, curr_boxes_;
  size_t obstacle_id_;
  float VOXEL_GRID_SIZE;
  Eigen::Vector4f ROI_MAX_POINT, ROI_MIN_POINT;
  Eigen::Vector4f CROP_BOX_MAX_POINT, CROP_BOX_MIN_POINT;
  int NEIGHOBORS;
  float STANDARD_DEVIATION;
  float cluster_threshold, cluster_max_size, cluster_min_size;
  float row_width, row_lenght, tree_height;
  string tree_mapping_file_name;
};

void LocalPointCloudMapNode::loadParams() {
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>(
      "input_cloud", input_cloud_topic,
      "rslidar_points"); /// zed2i/zed_node/point_cloud/cloud_registered

  private_nh.param<std::string>("odom_topic", odometry_topic, "vehicle/odom");
  private_nh.param<std::string>("global_frame", global_frame, "map");
  private_nh.param<std::string>("local_frame", local_frame, "base_link");
  private_nh.param<std::string>("gps_home_position_topic",
                                gps_home_position_topic,
                                "/mavros/global_position/home");
  private_nh.param<bool>("enable_global_cloud_map", enable_global_cloud_map,
                         true); /// zed2i/zed_node/point_cloud/cloud_registered
  private_nh.param<bool>("enable_local_cloud_map", enable_local_cloud_map,
                         true); /// zed2i/zed_node/point_cloud/cloud_registered
  private_nh.param<bool>("enable_tree_mapping", enable_tree_mapping,
                         true); /// zed2i/zed_node/point_cloud/cloud_registered

  private_nh.param<std::string>("global_cloud_map_topic",
                                global_cloud_map_topic, "global_cloud_map");
  private_nh.param<std::string>("tree_mapping_topic", tree_mapping_topic,
                                "tree_cloud_map");
  private_nh.param<std::string>("tree_mapping_file_name",
                                tree_mapping_file_name, "");

  private_nh.param<float>("voxel_size", voxel_size,
                          0.1); /// zed2i/zed_node/point_cloud/cloud_registered
  private_nh.param<std::string>("local_cloud_map_topic", local_cloud_map_topic,
                                "local_cloud_map");
  //    private_nh.param<float>("local_map_size", local_map_size, 30.0);
  private_nh.param<float>("local_map_size_x", local_map_size_x, 30.0);
  private_nh.param<float>("local_map_size_y", local_map_size_y, 30.0);

  private_nh.param<float>("min_height_local_frame", min_height_local_frame,
                          0.0);
  private_nh.param<float>("max_height_local_frame", max_height_local_frame, 2);

  float crop_box_max_x, crop_box_max_y, crop_box_max_z;
  float crop_box_min_x, crop_box_min_y, crop_box_min_z;

  private_nh.param<float>("crop_box_max_x", crop_box_max_x, 10);
  private_nh.param<float>("crop_box_max_y", crop_box_max_y, 10);
  private_nh.param<float>("crop_box_max_z", crop_box_max_z, 10);

  private_nh.param<float>("crop_box_min_x", crop_box_min_x, -10);
  private_nh.param<float>("crop_box_min_y", crop_box_min_y, -10);
  private_nh.param<float>("crop_box_min_z", crop_box_min_z, -10);

  CROP_BOX_MIN_POINT =
      Eigen::Vector4f(crop_box_min_x, crop_box_min_y, crop_box_min_z, 1);
  CROP_BOX_MAX_POINT =
      Eigen::Vector4f(crop_box_max_x, crop_box_max_y, crop_box_max_z, 1);

  // cluster related
  private_nh.param<float>("cluster_threshold", cluster_threshold, 0.7);
  private_nh.param<float>("cluster_max_size", cluster_max_size, 300);
  private_nh.param<float>("cluster_min_size", cluster_min_size, 10);

  // tree mapping parameters
  private_nh.param<float>("row_width", row_width, 6);
  private_nh.param<float>("row_lenght", row_lenght, 4.5);
  private_nh.param<float>("tree_height", tree_height, 3);
}

LocalPointCloudMapNode::LocalPointCloudMapNode() : tf2_listener(tf2_buffer) {
  ros::NodeHandle private_nh("~");
  loadParams();

  sub_lidar_points = nh.subscribe(
      input_cloud_topic, 1, &LocalPointCloudMapNode::lidarPointsCallback, this);
  sub_odom = nh.subscribe(odometry_topic, 1,
                          &LocalPointCloudMapNode::odomCallback, this);
  sub_gps_home_position =
      nh.subscribe(gps_home_position_topic, 1,
                   &LocalPointCloudMapNode::gpsHomepositionCallback, this);

  pub_local_cloud_map =
      nh.advertise<sensor_msgs::PointCloud2>(local_cloud_map_topic, 1);
  if (enable_global_cloud_map)
    pub_global_cloud_map =
        nh.advertise<sensor_msgs::PointCloud2>(global_cloud_map_topic, 1);
  if (enable_tree_mapping)
    tree_cloud_pub =
        nh.advertise<sensor_msgs::PointCloud2>(tree_mapping_topic, 1);

  // pub_base_cloud =
  // nh.advertise<sensor_msgs::PointCloud2>("cloud_in_base_link", 1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr tree_cloud(new
  // pcl::PointCloud<pcl::PointXYZ>); tree_cloud->is_dense = false;

  // float roi_max_x,roi_max_y, roi_max_z,roi_min_x,roi_min_y,roi_min_z;
  obstacle_detector = std::make_shared<ObstacleDetector<pcl::PointXYZ>>();
  obstacle_id_ = 0;
  local_bboxes_pub =
      nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("local_bboxes", 1);
}

void LocalPointCloudMapNode::lidarPointsCallback(
    const sensor_msgs::PointCloud2::ConstPtr &lidar_points) {

  ROS_DEBUG_ONCE("lidar callback called");
  raw_cloud = *lidar_points;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_raw(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*lidar_points, *points_raw);

  // pub_local_cloud_map.publish(lidar_points);
  if (odom_received == false) {
    ROS_WARN("WAITING FOR ODOM DATA");
    return;
  }
  // Step 2: Downsample the point cloud to sparse point cloud

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(points_raw);
  sor.setLeafSize(voxel_size, voxel_size,
                  voxel_size); // Adjust the leaf size as per requirement
  sor.filter(*downsampledCloud);

  // Removing the car roof region
  std::vector<int> indices;
  // pcl::CropBox<pcl::PointXYZ> roof(false);
  // ROS_INFO("Roof");
  // roof.setMin(Eigen::Vector4f(-3.0,-3.0,-1.0,1.0));
  // roof.setMax(Eigen::Vector4f(3.0,3.0,1.0,1.0));
  // roof.setInputCloud(downsampledCloud);
  // roof.filter(*downsampledCloud);

  pcl::CropBox<pcl::PointXYZ> roof(true);
  // roof.setMin(Eigen::Vector4f(-10, -10, -10, 1));
  // std::cout<<CROP_BOX_MIN_POINT;
  // std::cout<<CROP_BOX_MAX_POINT;
  roof.setMin(Eigen::Vector4f(CROP_BOX_MIN_POINT));

  // roof.setMax(Eigen::Vector4f(10, 10, 10, 1));
  roof.setMax(CROP_BOX_MAX_POINT);
  roof.setInputCloud(downsampledCloud);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (auto &point : indices)
    inliers->indices.push_back(point);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(downsampledCloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*downsampledCloud);
  // sensor_msgs::PointCloud2 bbox;
  // pcl::toROSMsg(*downsampledCloud, bbox);
  // bbox.header = lidar_points->header;
  // pub_local_cloud_map.publish(bbox);
  // return ;

  sensor_msgs::PointCloud2 voxel_filterd_cloud, cloud_in_global_frame;

  pcl::toROSMsg(*downsampledCloud, voxel_filterd_cloud);
  voxel_filterd_cloud.header = lidar_points->header;
  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf2_buffer.lookupTransform(
        global_frame, lidar_points->header.frame_id, ros::Time(0));
    tf2::doTransform(voxel_filterd_cloud, cloud_in_global_frame,
                     transform_stamped);

  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  cloud_in_global_frame.header.frame_id = global_frame;
  // pub_base_cloud.publish(cloud_in_global_frame);

  pcl::PointCloud<pcl::PointXYZ> cloud_in_global_frame_pcl;

  pcl::fromROSMsg(cloud_in_global_frame, cloud_in_global_frame_pcl);

  final_cloud += cloud_in_global_frame_pcl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_in(
      new pcl::PointCloud<pcl::PointXYZ>);
  *voxel_in = final_cloud;
  pcl::VoxelGrid<pcl::PointXYZ> sor1;
  sor.setInputCloud(voxel_in);
  sor.setLeafSize(voxel_size, voxel_size,
                  voxel_size); // Adjust the leaf size as per requirement
  sor.filter(final_cloud);
  if (enable_global_cloud_map) {
    pcl::toROSMsg(final_cloud, final_ros);
    final_ros.header.frame_id = global_frame;
    pub_global_cloud_map.publish(final_ros);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(
      new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_roi = final_cloud;
  pcl::CropBox<pcl::PointXYZ> region(false);

  // proper map on both x,y coordinates
  float min_x, min_y, max_x, max_y;
  float yaw = tf::getYaw(curr_odom.pose.pose.orientation);

  max_x = curr_odom.pose.pose.position.x + local_map_size_x;
  max_y = curr_odom.pose.pose.position.y + local_map_size_y;
  min_x = curr_odom.pose.pose.position.x + (-local_map_size_x);
  min_y = curr_odom.pose.pose.position.y + (-local_map_size_y);
  region.setMin(Eigen::Vector4f(min_x, min_y, min_height_local_frame, 1));
  region.setMax(Eigen::Vector4f(max_x, max_y, max_height_local_frame, 1));
  region.setInputCloud(cloud_roi);
  region.filter(*cloud_roi);
  sensor_msgs::PointCloud2 local_cloud_ros;
  pcl::toROSMsg(*cloud_roi, local_cloud_ros);
  if (!enable_global_cloud_map)
    final_cloud = *cloud_roi;

  local_cloud_ros.header.frame_id = global_frame;
  geometry_msgs::TransformStamped transform_to_local_frame;
  sensor_msgs::PointCloud2 cloud_in_local_frame;

  try {
    transform_to_local_frame = tf2_buffer.lookupTransform(
        local_frame, local_cloud_ros.header.frame_id, ros::Time(0));
    tf2::doTransform(local_cloud_ros, cloud_in_local_frame,
                     transform_to_local_frame);

  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  cloud_in_local_frame.header.frame_id = local_frame;
  pub_local_cloud_map.publish(cloud_in_local_frame);

  // pcl::PointCloud<pcl::PointXYZ> cloud_in_local_frame_pcl;
  // pcl::PointXYZ cloud_in_local_frame_pcl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_local_frame_pcl(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(cloud_in_local_frame, *cloud_in_local_frame_pcl);

  // Boxes

  auto cloud_clusters =
      obstacle_detector->clustering(cloud_in_local_frame_pcl, cluster_threshold,
                                    cluster_min_size, cluster_max_size);
  for (auto &cluster : cloud_clusters) {

    Box box = obstacle_detector->axisAlignedBoundingBox(cluster, obstacle_id_);

    obstacle_id_ = (obstacle_id_ < SIZE_MAX) ? ++obstacle_id_ : 0;
    curr_boxes_.emplace_back(box);
  }

  // Construct Bounding Boxes from the clusters
  jsk_recognition_msgs::BoundingBoxArray jsk_bboxes;
  jsk_bboxes.header.frame_id = local_frame;
  geometry_msgs::TransformStamped transform_to_map_frame;

  try {
    transform_to_map_frame =
        tf2_buffer.lookupTransform(global_frame, local_frame, ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  pcl::PointXYZ point;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr
  // ROS_DEBUG_STREAM("before pusing back");
  for (auto &box : curr_boxes_) {
    geometry_msgs::Pose pose, pose_transformed;
    jsk_recognition_msgs::BoundingBox jsk_bbox;

    pose.position.x = box.position(0);
    pose.position.y = box.position(1);
    pose.position.z = box.position(2);
    pose.orientation.w = box.quaternion.w();
    pose.orientation.x = box.quaternion.x();
    pose.orientation.y = box.quaternion.y();
    pose.orientation.z = box.quaternion.z();
    jsk_bbox.header.frame_id = local_frame;
    jsk_bbox.pose = pose;
    jsk_bbox.dimensions.x = box.dimension(0);
    jsk_bbox.dimensions.y = box.dimension(1);
    jsk_bbox.dimensions.z = box.dimension(2);
    jsk_bbox.value = 1.0f;
    jsk_bbox.label = box.id;

    jsk_bboxes.boxes.emplace_back(std::move(jsk_bbox));
    tf2::doTransform(pose, pose_transformed, transform_to_map_frame);
    point.x = pose_transformed.position.x;
    point.y = pose_transformed.position.y;
    point.z = pose_transformed.position.z;
    tree_cloud.push_back(point);
  }
  // ROS_DEBUG_STREAM("after pusing back");
  local_bboxes_pub.publish(std::move(jsk_bboxes));
  curr_boxes_.clear();

  // Publishing Bounding boxes as point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr tree_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  *tree_cloud_ptr = tree_cloud;

  // pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZ>> voxel;
  sor.setInputCloud(tree_cloud_ptr);
  sor.setLeafSize(row_width, row_lenght,
                  tree_height); // Adjust the leaf size as per requirement
  sor.filter(tree_cloud);

  sensor_msgs::PointCloud2 tree_cloud_ros;
  pcl::toROSMsg(tree_cloud, tree_cloud_ros);
  tree_cloud_ros.header.frame_id = global_frame;
  tree_cloud_pub.publish(tree_cloud_ros);

  if (ros::param::has("save_tree_mapping")) {
    bool save_tree_mapping;
    ros::param::get("/save_tree_mapping", save_tree_mapping);
    if (save_tree_mapping) {
      LocalPointCloudMapNode::savePclfile("", tree_cloud);
    }
  }
}

bool LocalPointCloudMapNode::savePclfile(
    string path, pcl::PointCloud<pcl::PointXYZ> tree_cloud) {
  string package_path = ros::package::getPath("auto_nav");
  string output_json_file_dir =
      package_path + "/maps/" + tree_mapping_file_name + "_map_config.yaml";

  string pcd_file_name =
      package_path + "/maps/" + tree_mapping_file_name + "_tree_map.pcd";
  fstream FileName;

  FileName.open(output_json_file_dir, ios::out);
  if (!FileName) {
    ROS_ERROR("Error while creating config file");
    return false;
  } else {
    stringstream json_string;
    json_string << "map_origin: \n";
    json_string << "    latitude: " << fixed << setprecision(8)
                << gps_home_position_data.geo.latitude << "\n";
    json_string << "    longitude: " << fixed << setprecision(8)
                << gps_home_position_data.geo.longitude << "\n";
    json_string << "    altitude: " << fixed << setprecision(8)
                << gps_home_position_data.geo.altitude << "\n";
    json_string << "    roll: " << 0.0 << "\n";
    json_string << "    pitch: " << 0.0 << "\n";
    json_string << "    yaw: " << 0.0 << "\n";
    json_string << "pcd_file_name: "
                << tree_mapping_file_name + "_tree_map.pcd";

    FileName << json_string.str();
    FileName.close();
    pcl::io::savePCDFileASCII(pcd_file_name, tree_cloud);
    ROS_INFO_STREAM("Saved " << tree_cloud.size() << " data points to "
                             << tree_mapping_file_name + "_tree_map.pcd");
    ros::param::set("/save_tree_mapping", false);
    return true;
  }
}

void LocalPointCloudMapNode::odomCallback(
    const nav_msgs::Odometry::ConstPtr &odom_msg) {
  ROS_DEBUG_ONCE("odom callback called");
  odom_received = true;
  curr_odom = *odom_msg;
}

void LocalPointCloudMapNode::gpsHomepositionCallback(
    const mavros_msgs::HomePosition::ConstPtr &home_position) {
  ROS_DEBUG_ONCE("GPS home position callback");
  gps_home_position_received = true;
  gps_home_position_data = *home_position;
}
} // namespace LocalPointCloudMap

int main(int argc, char **argv) {
  ros::init(argc, argv, "local_pointcloud_map_node",
            ros::init_options::AnonymousName);
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  LocalPointCloudMap::LocalPointCloudMapNode local_point_cloud_map_node;
  ros::spin();

  return 0;
}