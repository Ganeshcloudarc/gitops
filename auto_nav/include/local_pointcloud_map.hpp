#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>



namespace LocalPointCloudMap
{
   
template <typename PointT>
class PointCloudMap
{
    public:
        PointCloudMap();
        virtual ~PointCloudMap();
        typename pcl::PointCloud<PointT>::Ptr filterCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const float filter_res, const Eigen::Vector4f& min_pt, const Eigen::Vector4f& max_pt, const Eigen::Vector4f& crop_box_min_pt, const Eigen::Vector4f& crop_box_max_pt, int NEIGHOBORS, float STANDARD_DEVIATION);


};
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr PointCloudMap<PointT>::filterCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const float filter_res, const Eigen::Vector4f& min_pt, const Eigen::Vector4f& max_pt, const Eigen::Vector4f& crop_box_min_pt, const Eigen::Vector4f& crop_box_max_pt, int neighbors, float standard_deviation)
{
  // Time segmentation process
  // const auto start_time = std::chrono::steady_clock::now();

  // Create the filtering object: downsample the dataset using a leaf size
  pcl::VoxelGrid<PointT> vg;
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  vg.setInputCloud(cloud);
  vg.setLeafSize(filter_res, filter_res, filter_res);
  vg.filter(*cloud_filtered);

  // Cropping the ROI
  typename pcl::PointCloud<PointT>::Ptr cloud_roi(new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> region(true);
  region.setMin(min_pt);
  region.setMax(max_pt);
  region.setInputCloud(cloud_filtered);
  region.filter(*cloud_roi);

  // Removing the car roof region
  std::vector<int> indices;
  pcl::CropBox<PointT> roof(true);
  //following are the provided by this pacakge.

  // roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  // roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  //changed to suit for our vehicle, when kept on top.
  //TODO keep these on param file. 
  // roof.setMin(Eigen::Vector4f(-1.5, -1, -1, 1));
  // roof.setMax(Eigen::Vector4f(0.3, 1, 1, 1));
  roof.setMin(crop_box_min_pt);
  roof.setMax(crop_box_max_pt);
  roof.setInputCloud(cloud_roi);
  roof.filter(indices);

  //changed to suit for our vehicle, when kept on top.


  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (auto& point : indices)
    inliers->indices.push_back(point);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_roi);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_roi);




  // statistical radial removal
   pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_roi);
  sor.setMeanK (neighbors);
  sor.setStddevMulThresh (standard_deviation);
  sor.filter (*cloud_roi);

  // const auto end_time = std::chrono::steady_clock::now();
  // const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  // std::cout << "filtering took " << elapsed_time.count() << " milliseconds" << std::endl;

  return cloud_roi;
}

}