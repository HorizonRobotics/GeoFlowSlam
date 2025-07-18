// Author of SSL_SLAM2: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LASER_PROCESSING_CLASS_H_
#define _LASER_PROCESSING_CLASS_H_

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Lidar.h"
// points covariance class
typedef pcl::PointXYZRGBA PointType;
namespace ORB_SLAM3 {
class Double2d {
 public:
  int id;
  double value;
  Double2d(int id_in, double value_in);
};

// points info class
class PointsInfo {
 public:
  int layer;
  double time;
  PointsInfo(int layer_in, double time_in);
};

class LaserProcessingClass {
 public:
  LaserProcessingClass();
  void init(std::string& file_path);
  void featureExtraction(const pcl::PointCloud<PointType>::Ptr& pc_in,
                         pcl::PointCloud<PointType>::Ptr& pc_out_edge,
                         pcl::PointCloud<PointType>::Ptr& pc_out_surf);
  void featureExtractionFromSector(
      const pcl::PointCloud<PointType>::Ptr& pc_in,
      std::vector<Double2d>& cloudCurvature,
      pcl::PointCloud<PointType>::Ptr& pc_out_edge,
      pcl::PointCloud<PointType>::Ptr& pc_out_surf);

 private:
  LidarParam lidar_param;
  pcl::VoxelGrid<PointType> edge_downsize_filter;
  pcl::VoxelGrid<PointType> surf_downsize_filter;
  pcl::RadiusOutlierRemoval<PointType> edge_noise_filter;
  pcl::RadiusOutlierRemoval<PointType> surf_noise_filter;
};
}  // namespace ORB_SLAM3
#endif  // _LASER_PROCESSING_CLASS_H_