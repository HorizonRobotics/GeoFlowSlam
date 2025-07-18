#include "RegistrationGICP.h"
RegistrationGICP::RegistrationGICP(/* args */) {}
RegistrationGICP::~RegistrationGICP() {}

RegistrationResult RegistrationGICP::RegisterPointClouds(
    const std::vector<Eigen::Vector4f>& target_points,
    const std::vector<Eigen::Vector4f>& source_points,
    const Eigen::Isometry3d& init_T_target_source) {
  RegistrationSetting setting;
  setting.num_threads = 4;                 // Number of threads to be used
  setting.downsampling_resolution = 0.02;  // Downsampling resolution
  setting.max_correspondence_distance =
      0.1;  // Maximum correspondence distance between points (e.g., triming
            // threshold)
  setting.type = setting.RegistrationType::GICP;
  Eigen::Isometry3d init_T_target = init_T_target_source;
  RegistrationResult result =
      align(target_points, source_points, init_T_target, setting);
  return result;
}
bool RegistrationGICP::NDTRegistration(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
    Eigen::Matrix4f& final_transform) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_source_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setLeafSize(0.05, 0.05, 0.05);
  voxel_grid.setInputCloud(source_cloud);
  voxel_grid.filter(*filtered_source_cloud);

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setTransformationEpsilon(0.01);
  ndt.setStepSize(0.05);
  ndt.setResolution(0.05);
  ndt.setMaximumIterations(35);
  ndt.setInputSource(filtered_source_cloud);
  ndt.setInputTarget(target_cloud);

  Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
  ndt.align(*output_cloud, initial_guess);

  if (ndt.hasConverged()) {
    final_transform = ndt.getFinalTransformation();
    std::cout << "NDT has converged, score: " << ndt.getFitnessScore()
              << std::endl;
    std::cout << "Transformation matrix: \n" << final_transform << std::endl;
    return true;
  } else {
    std::cout << "NDT did not converge." << std::endl;
    return false;
  }
}