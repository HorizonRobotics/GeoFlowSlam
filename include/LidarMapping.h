/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef LidarMapping_H
#define LidarMapping_H
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <atomic>
#include <condition_variable>
#include <pcl/impl/pcl_base.hpp>

#include "System.h"
#include "Tracking.h"
// using namespace ORB_SLAM3;

namespace ORB_SLAM3 {
class Tracking;
class LidarMapping {
 public:
  LidarMapping(double global_resolution_, double local_resolution_,
               double meank_, double thresh_, string save_path_);
  void save();
  void insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth, int idk,
                      vector<KeyFrame *> vpKFs);
  void insertKeyFrame(KeyFrame *kf);
  void shutdown();
  void viewer();
  void SetTracker(Tracking *pTracker);
  void GetLocalMap(pcl::PointCloud<PointType>::Ptr local_map);
  void ClearLocalMap();
  void transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn,
                           pcl::PointCloud<PointType>::Ptr cloudOut,
                           Eigen::Matrix4d transCur);
  int mnloopcount = 0;
  vector<KeyFrame *> currentvpKFs;
  bool cloudbusy = false;
  bool loopbusy = false;
  bool insert_kf = false;
  void updatecloud(Map &curMap);
  void Clear();
  bool bStop = false;

  // 关于更新时的变量
  std::atomic<bool> mabIsUpdating;

 protected:
  void generatePointCloud(KeyFrame *kf);

  std::list<KeyFrame *> mlNewKeyFrames;
  std::list<KeyFrame *> mlAllKeyFrames;
  pcl::PointCloud<PointType>::Ptr globalMap, localMap;
  shared_ptr<thread> viewerThread;
  Tracking *mpTracker;

  bool shutDownFlag = false;
  std::mutex shutDownMutex;

  condition_variable keyFrameUpdated;
  std::mutex mMutexGlobalMap, mMutexLocalMap;
  // vector<PointCloude>     pointcloud;
  // data to generate point clouds
  vector<KeyFrame *> keyframes;
  vector<cv::Mat> colorImgs;
  vector<cv::Mat> depthImgs;
  vector<cv::Mat> colorImgks;
  vector<cv::Mat> depthImgks;
  vector<int> ids;
  std::mutex keyframeMutex;
  std::mutex updateMutex;
  uint16_t lastKeyframeSize = 0;

  double global_resolution = 0.04;
  double local_resolution = 0.04;
  double meank = 50;
  double thresh = 1;
  std::string save_path;
  pcl::VoxelGrid<PointType> *voxel_global;
  pcl::VoxelGrid<PointType> *voxel_local;
  pcl::StatisticalOutlierRemoval<PointType> *statistical_filter;
};
}  // namespace ORB_SLAM3
#endif  // LidarMapping_H
