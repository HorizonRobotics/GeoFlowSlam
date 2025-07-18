// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT

/// @brief Basic point cloud registration example with small_gicp::align()
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include <Eigen/Core>
#include <Thirdparty/small_gicp/include/small_gicp/benchmark/read_points.hpp>
#include <Thirdparty/small_gicp/include/small_gicp/registration/registration_helper.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>
using namespace small_gicp;

class RegistrationGICP {
 private:
  /* data */
 public:
  RegistrationGICP(/* args */);
  ~RegistrationGICP();
  RegistrationResult RegisterPointClouds(
      const std::vector<Eigen::Vector4f>& target_points,
      const std::vector<Eigen::Vector4f>& source_points,
      const Eigen::Isometry3d& init_T_target_source);
  bool NDTRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
                       Eigen::Matrix4f& final_transform);
};
