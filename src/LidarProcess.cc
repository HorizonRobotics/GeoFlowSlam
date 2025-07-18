
#include "LidarProcess.h"

namespace ORB_SLAM3 {

void LaserProcessingClass::init(std::string& file_path) {
  lidar_param.loadParam(file_path);
  double map_resolution = lidar_param.getLocalMapResolution();
  edge_downsize_filter.setLeafSize(map_resolution / 4.0, map_resolution / 4.0,
                                   map_resolution / 4.0);
  surf_downsize_filter.setLeafSize(map_resolution / 2.0, map_resolution / 2.0,
                                   map_resolution / 2.0);

  edge_noise_filter.setRadiusSearch(map_resolution);
  edge_noise_filter.setMinNeighborsInRadius(3);
  surf_noise_filter.setRadiusSearch(map_resolution);
  surf_noise_filter.setMinNeighborsInRadius(14);
}

void LaserProcessingClass::featureExtraction(
    const pcl::PointCloud<PointType>::Ptr& pc_in,
    pcl::PointCloud<PointType>::Ptr& pc_out_edge,
    pcl::PointCloud<PointType>::Ptr& pc_out_surf) {
  std::vector<int> indices;
  pcl::PointCloud<PointType>::Ptr pc_filtered(new pcl::PointCloud<PointType>());
  pcl::removeNaNFromPointCloud(*pc_in, *pc_filtered, indices);
  // std::cout << "Start feature extraction" << endl;

  std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudScans;

  double last_angle =
      atan2(pc_filtered->points[0].y, pc_filtered->points[0].z) * 180 / M_PI;
  int count = 0;
  int point_size = pc_filtered->points.size() - 1;

  for (int i = 0; i < (int)pc_filtered->points.size(); i++) {
    double angle =
        atan2(pc_filtered->points[i].y, pc_filtered->points[i].z) * 180 / M_PI;

    if (fabs(angle - last_angle) > 0.05) {
      if (count > 20) {
        pcl::PointCloud<PointType>::Ptr pc_temp(
            new pcl::PointCloud<PointType>());

        // check the starting angle, if it is not border, set max distance
        double start_angle = atan2(pc_filtered->points[i - count].x,
                                   pc_filtered->points[i - count].z) *
                             180 / M_PI;
        if (start_angle > -lidar_param.getHorizontalAngle() / 2.0 + 5.0) {
          for (int k = 0; k < 5; k++) {
            PointType point_temp;
            point_temp.x = pc_filtered->points[i - count].x;
            point_temp.y = pc_filtered->points[i - count].y;
            point_temp.z = lidar_param.getMaxDistance();
            pc_temp->push_back(point_temp);
          }
        }

        for (int k = 0; k < count; k++) {
          pc_temp->push_back(pc_filtered->points[i - count + k]);
        }

        double end_angle =
            atan2(pc_filtered->points[i - 1].x, pc_filtered->points[i - 1].z) *
            180 / M_PI;
        if (end_angle < lidar_param.getHorizontalAngle() / 2.0 - 5.0) {
          for (int k = 0; k < 5; k++) {
            PointType point_temp;
            point_temp.x = pc_filtered->points[i - 1].x;
            point_temp.y = pc_filtered->points[i - 1].y;
            point_temp.z = lidar_param.getMaxDistance();
            pc_temp->push_back(point_temp);
          }
        }
        laserCloudScans.push_back(pc_temp);
      }
      count = 0;
      last_angle = angle;
    }
    count++;
  }

  // ROS_INFO_ONCE("total points array %d", laserCloudScans.size());
  // std::cout << "total points array " << laserCloudScans.size() << endl;

  pcl::PointCloud<PointType>::Ptr edge_raw(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr surf_raw(new pcl::PointCloud<PointType>());

  for (int i = 0; i < laserCloudScans.size(); i++) {
    std::vector<Double2d> cloudCurvature;
    int total_points = laserCloudScans[i]->points.size() - 10;
    for (int j = 5; j < (int)laserCloudScans[i]->points.size() - 5; j++) {
      double point_distance =
          laserCloudScans[i]->points[j].x * laserCloudScans[i]->points[j].x +
          laserCloudScans[i]->points[j].y * laserCloudScans[i]->points[j].y +
          laserCloudScans[i]->points[j].z * laserCloudScans[i]->points[j].z;
      double diffX = laserCloudScans[i]->points[j - 5].x +
                     laserCloudScans[i]->points[j - 4].x +
                     laserCloudScans[i]->points[j - 3].x +
                     laserCloudScans[i]->points[j - 2].x +
                     laserCloudScans[i]->points[j - 1].x -
                     10 * laserCloudScans[i]->points[j].x +
                     laserCloudScans[i]->points[j + 1].x +
                     laserCloudScans[i]->points[j + 2].x +
                     laserCloudScans[i]->points[j + 3].x +
                     laserCloudScans[i]->points[j + 4].x +
                     laserCloudScans[i]->points[j + 5].x;
      double diffY = laserCloudScans[i]->points[j - 5].y +
                     laserCloudScans[i]->points[j - 4].y +
                     laserCloudScans[i]->points[j - 3].y +
                     laserCloudScans[i]->points[j - 2].y +
                     laserCloudScans[i]->points[j - 1].y -
                     10 * laserCloudScans[i]->points[j].y +
                     laserCloudScans[i]->points[j + 1].y +
                     laserCloudScans[i]->points[j + 2].y +
                     laserCloudScans[i]->points[j + 3].y +
                     laserCloudScans[i]->points[j + 4].y +
                     laserCloudScans[i]->points[j + 5].y;
      double diffZ = laserCloudScans[i]->points[j - 5].z +
                     laserCloudScans[i]->points[j - 4].z +
                     laserCloudScans[i]->points[j - 3].z +
                     laserCloudScans[i]->points[j - 2].z +
                     laserCloudScans[i]->points[j - 1].z -
                     10 * laserCloudScans[i]->points[j].z +
                     laserCloudScans[i]->points[j + 1].z +
                     laserCloudScans[i]->points[j + 2].z +
                     laserCloudScans[i]->points[j + 3].z +
                     laserCloudScans[i]->points[j + 4].z +
                     laserCloudScans[i]->points[j + 5].z;
      Double2d distance(
          j, diffX * diffX + diffY * diffY + diffZ * diffZ / point_distance);
      cloudCurvature.push_back(distance);
    }
    featureExtractionFromSector(laserCloudScans[i], cloudCurvature, edge_raw,
                                surf_raw);
  }

  pcl::PointCloud<PointType>::Ptr edge_filtered(
      new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr surf_filtered(
      new pcl::PointCloud<PointType>());

  // reduce cloud size
  // std::cout << "edge_raw size: " << edge_raw->size() << endl;
  edge_downsize_filter.setInputCloud(edge_raw);
  edge_downsize_filter.filter(*edge_filtered);
  surf_downsize_filter.setInputCloud(surf_raw);
  surf_downsize_filter.filter(*surf_filtered);

  // remove noisy measurement
  // std::cout << "edge_filtered size: " << edge_filtered->size() << endl;
  edge_noise_filter.setInputCloud(edge_filtered);
  edge_noise_filter.filter(*pc_out_edge);
  surf_noise_filter.setInputCloud(surf_filtered);
  surf_noise_filter.filter(*pc_out_surf);
  return;
}

void LaserProcessingClass::featureExtractionFromSector(
    const pcl::PointCloud<PointType>::Ptr& pc_in,
    std::vector<Double2d>& cloudCurvature,
    pcl::PointCloud<PointType>::Ptr& pc_out_edge,
    pcl::PointCloud<PointType>::Ptr& pc_out_surf) {
  std::sort(
      cloudCurvature.begin(), cloudCurvature.end(),
      [](const Double2d& a, const Double2d& b) { return a.value < b.value; });
  // std::cout << "cloudCurvature size: " << cloudCurvature.size() << endl;
  int largestPickedNum = 0;
  std::vector<int> edge_points;
  std::vector<int> picked_points;
  int point_info_count = 0;
  for (int i = cloudCurvature.size() - 1; i >= 0; i--) {
    int ind = cloudCurvature[i].id;
    if (std::find(picked_points.begin(), picked_points.end(), ind) ==
        picked_points.end()) {
      if (cloudCurvature[i].value <= 0.1) {
        break;
      }
      // std::cout << cloudCurvature[i].value << endl;
      largestPickedNum++;
      picked_points.push_back(ind);

      if (largestPickedNum <= 10) {
        edge_points.push_back(ind);
        pc_out_edge->push_back(pc_in->points[ind]);
        point_info_count++;
      } else {
        break;
      }

      for (int k = -5; k <= 5; k++) {
        if (k != 0) picked_points.push_back(ind + k);
      }
    }
  }

  for (int i = 0; i <= (int)cloudCurvature.size() - 1; i++) {
    int ind = cloudCurvature[i].id;
    if (std::find(edge_points.begin(), edge_points.end(), ind) ==
        edge_points.end()) {
      pc_out_surf->push_back(pc_in->points[ind]);
    }
  }
}

LaserProcessingClass::LaserProcessingClass() {}

Double2d::Double2d(int id_in, double value_in) {
  id = id_in;
  value = value_in;
};

PointsInfo::PointsInfo(int layer_in, double time_in) {
  layer = layer_in;
  time = time_in;
};
}  // namespace ORB_SLAM3