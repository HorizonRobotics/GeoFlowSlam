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

#include "LidarMapping.h"

#include <KeyFrame.h>
#include <omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sys/time.h>

#include <opencv2/highgui/highgui.hpp>

#include "Converter.h"
#include "System.h"
namespace ORB_SLAM3 {
// int currentloopcount = 0;
LidarMapping::LidarMapping(double global_resolution_, double local_resolution_,
                           double meank_, double thresh_, string save_path_)
    : mabIsUpdating(false) {
  this->global_resolution = global_resolution_;
  this->local_resolution = local_resolution_;
  this->meank = meank_;
  this->thresh = thresh_;
  this->save_path = save_path_;
  statistical_filter = new pcl::StatisticalOutlierRemoval<PointType>(true);
  voxel_global = new pcl::VoxelGrid<PointType>();
  voxel_local = new pcl::VoxelGrid<PointType>();
  statistical_filter->setMeanK(meank);
  statistical_filter->setStddevMulThresh(thresh);
  voxel_global->setLeafSize(global_resolution, global_resolution,
                            global_resolution);
  voxel_local->setLeafSize(local_resolution, local_resolution,
                           local_resolution);
  globalMap = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  localMap = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);

  viewerThread = make_shared<thread>(bind(&LidarMapping::viewer, this));
}

void LidarMapping::shutdown() {
  {
    unique_lock<mutex> lck(shutDownMutex);
    shutDownFlag = true;
    keyFrameUpdated.notify_one();
  }
  viewerThread->join();
}

void LidarMapping::Clear() {
  std::unique_lock<std::mutex> lck(mMutexGlobalMap);
  globalMap.reset(new pcl::PointCloud<PointType>);
}


void LidarMapping::insertKeyFrame(KeyFrame *kf) {
  // cout << "receive a keyframe, 第" << kf->mnId << "个" << endl;
  if (kf->imRGB.empty()) return;
  unique_lock<mutex> lck(keyframeMutex);
  mlNewKeyFrames.emplace_back(kf);
  mlAllKeyFrames.emplace_back(kf);
  insert_kf = true;
  if (mlNewKeyFrames.size() > 30) mlNewKeyFrames.pop_front();
}

void LidarMapping::generatePointCloud(KeyFrame *kf)  //,Eigen::Isometry3d T
{
  pcl::PointCloud<PointType>::Ptr pPointCloud(new pcl::PointCloud<PointType>);
  // point cloud is null ptr
  for (int m = 0; m < kf->imDepth.rows; m += 3) {
    for (int n = 0; n < kf->imDepth.cols; n += 3) {
      float d = kf->imDepth.ptr<float>(m)[n];
      if (d < 0.05 || d > 10) continue;
      PointType p;
      p.z = d;
      p.x = (n - kf->cx) * p.z / kf->fx;
      p.y = (m - kf->cy) * p.z / kf->fy;

      p.b = kf->imRGB.ptr<uchar>(m)[n * 3];
      p.g = kf->imRGB.ptr<uchar>(m)[n * 3 + 1];
      p.r = kf->imRGB.ptr<uchar>(m)[n * 3 + 2];

      pPointCloud->points.push_back(p);
    }
  }
  pPointCloud->height = 1;
  pPointCloud->width = pPointCloud->points.size();
  pPointCloud->is_dense = true;
  kf->mpPointCloudDownsampled = pPointCloud;
}
void LidarMapping::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn,
                                       pcl::PointCloud<PointType>::Ptr cloudOut,
                                       Eigen::Matrix4d transCur) {
  int cloudSize = cloudIn->size();
  cloudOut->resize(cloudSize);
  int numberOfCores = omp_get_max_threads();
#pragma omp parallel for num_threads(numberOfCores)
  for (int i = 0; i < cloudSize; ++i) {
    const auto &pointFrom = cloudIn->points[i];
    cloudOut->points[i].x = transCur(0, 0) * pointFrom.x +
                            transCur(0, 1) * pointFrom.y +
                            transCur(0, 2) * pointFrom.z + transCur(0, 3);
    cloudOut->points[i].y = transCur(1, 0) * pointFrom.x +
                            transCur(1, 1) * pointFrom.y +
                            transCur(1, 2) * pointFrom.z + transCur(1, 3);
    cloudOut->points[i].z = transCur(2, 0) * pointFrom.x +
                            transCur(2, 1) * pointFrom.y +
                            transCur(2, 2) * pointFrom.z + transCur(2, 3);
    cloudOut->points[i].rgba = pointFrom.rgba;
  }
}

void LidarMapping::SetTracker(Tracking *pTracker) { mpTracker = pTracker; }
void LidarMapping::viewer() {
  int nCurKeyFrames, nPreKeyFrames = 0;
  while (1) {
    {
      unique_lock<mutex> lck_shutdown(shutDownMutex);
      if (shutDownFlag) {
        break;
      }
    }
    if (bStop || mabIsUpdating) {
      // cout<<"loopbusy || bStop"<<endl;
      continue;
    }
    int N;
    std::list<KeyFrame *> lNewKeyFrames;
    vector<KeyFrame *> lNewLocalKeyFrames = mpTracker->GetLocalKeyFrames();
    {
      unique_lock<mutex> lck(keyframeMutex);
      N = mlNewKeyFrames.size();
      // 没有新的关键帧
      if (!insert_kf) continue;
      insert_kf = false;
      lNewKeyFrames = mlNewKeyFrames;
      // if (N == 0)
      //   continue;
      // else {
      //   // mlNewKeyFrames.clear();
      // }
    }

    double generatePointCloudTime = 0, transformPointCloudTime = 0;
    std::unique_lock<std::mutex> lck(mMutexLocalMap);
    localMap->clear();
    for (auto pKF : lNewKeyFrames) {
      if (pKF->isBad()) continue;
      if (pKF->mpPointCloudDownsampled == nullptr) continue;
      pcl::PointCloud<PointType>::Ptr p(new pcl::PointCloud<PointType>);
      transformPointCloud((pKF->mpPointCloudDownsampled), (p),
                          Converter::toMatrix4d(pKF->GetPoseInverse()));

      {
        std::unique_lock<std::mutex> lck(mMutexGlobalMap);
        // *globalMap += *p;
        *localMap += *p;
      }
      // gettimeofday(&finish,NULL);//初始化结束时间
      // transformPointCloudTime += finish.tv_sec - start.tv_sec +
      // (finish.tv_usec - start.tv_usec)/1000000.0;
    }


    voxel_local->setInputCloud(localMap);
    voxel_local->filter(*localMap);
  }
  save();
}
void LidarMapping::GetLocalMap(pcl::PointCloud<PointType>::Ptr local_map) {
  std::unique_lock<std::mutex> lck(mMutexLocalMap);
  *local_map = *localMap;
}
void LidarMapping::ClearLocalMap() {
  // std::unique_lock<std::mutex> lck(mMutexLocalMap);
  localMap->clear();
}
void LidarMapping::save() {
  std::unique_lock<std::mutex> lck(mMutexGlobalMap);
  for (auto pKF : mlAllKeyFrames) {
    if (pKF->isBad()) continue;
    if (pKF->mpPointCloudDownsampled == nullptr) continue;
    pcl::PointCloud<PointType>::Ptr p(new pcl::PointCloud<PointType>);
    transformPointCloud((pKF->mpPointCloudDownsampled), (p),
                        Converter::toMatrix4d(pKF->GetPoseInverse()));

    {
      *globalMap += *p;
    }
  }
  voxel_global->setInputCloud(globalMap);
  voxel_global->filter(*globalMap);
  if (!globalMap->empty())
    pcl::io::savePCDFile(save_path + "/globalMap.pcd", *globalMap);
  cout << "globalMap save finished" << endl;
}
void LidarMapping::updatecloud(Map &curMap) {
  // std::unique_lock<std::mutex> lck(updateMutex);

  // mabIsUpdating = true;
  // currentvpKFs = curMap.GetAllKeyFrames();
  // // loopbusy = true;
  // cout << "开始点云更新" << endl;
  // pcl::PointCloud<PointType>::Ptr tmpGlobalMap(new
  // pcl::PointCloud<PointType>); pcl::PointCloud<PointType>::Ptr
  // curPointCloud(new pcl::PointCloud<PointType>);
  // pcl::PointCloud<PointType>::Ptr tmpGlobalMapFilter(
  //     new pcl::PointCloud<PointType>());
  // for (int i = 0; i < currentvpKFs.size(); i++) {
  //   if (!mabIsUpdating) {
  //     return;
  //   }
  //   if (!currentvpKFs[i]->isBad() && currentvpKFs[i]->mptrPointCloud) {
  //     transformPointCloud(
  //         (currentvpKFs[i]->mptrPointCloud), (curPointCloud),
  //         Converter::toMatrix4d(currentvpKFs[i]->GetPoseInverse()));

  //     *tmpGlobalMap += *curPointCloud;

  //     voxel_global->setInputCloud(tmpGlobalMap);
  //     voxel_global->filter(*tmpGlobalMapFilter);
  //     tmpGlobalMap->swap(*tmpGlobalMapFilter);
  //   }
  // }
  // {
  //   std::unique_lock<std::mutex> lck(mMutexGlobalMap);
  //   globalMap = tmpGlobalMap;
  // }
}

}  // namespace ORB_SLAM3