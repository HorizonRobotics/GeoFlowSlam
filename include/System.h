/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <future>
#include <opencv2/core/core.hpp>
#include <string>
#include <thread>

#include "Atlas.h"
#include "FrameDrawer.h"
#include "ImuTypes.h"
#include "KeyFrameDatabase.h"
#include "LidarMapping.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "Settings.h"
#include "ThreadPool.h"
#include "Tracking.h"
#include "Viewer.h"
namespace ORB_SLAM3 {

class Verbose {
 public:
  enum eLevel {
    VERBOSITY_QUIET = 0,
    VERBOSITY_NORMAL = 1,
    VERBOSITY_VERBOSE = 2,
    VERBOSITY_VERY_VERBOSE = 3,
    VERBOSITY_DEBUG = 4
  };

  static eLevel th;

 public:
  static void PrintMess(std::string str, eLevel lev) {
    if (lev <= th) cout << str << endl;
  }

  static void SetTh(eLevel _th) { th = _th; }
};

class Viewer;
class FrameDrawer;
class MapDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;
class LidarMapping;
class System {
 public:
  // Input sensor
  enum eSensor {
    MONOCULAR = 0,
    STEREO = 1,
    RGBD = 2,
    IMU_MONOCULAR = 3,
    IMU_STEREO = 4,
    IMU_RGBD = 5,
  };

  // File type
  enum FileType {
    TEXT_FILE = 0,
    BINARY_FILE = 1,
  };

  struct FrameWrapper {
    std::shared_ptr<Frame> mFrame;
    std::vector<IMU::Point> mImuMeas;
    std::vector<Eigen::Vector3f> mOdomMeas;
    std::shared_ptr<std::promise<Sophus::SE3f>> mPosePromise;
    FrameWrapper(std::shared_ptr<Frame> Frame,
                 const std::vector<IMU::Point> &ImuMeas,
                 const std::vector<Eigen::Vector3f> &OdomMeas,
                 std::shared_ptr<std::promise<Sophus::SE3f>> PosePromise) {
      mFrame = Frame;
      mImuMeas = ImuMeas;
      mOdomMeas = OdomMeas;
      mPosePromise = PosePromise;
    }
  };

  enum IMUMethod { IMU_ORB_SLAM3 = 0, VIG_INIT = 1, IMU_INITIALIZATION = 2 };

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and
  // Viewer threads.
  System(const string &strVocFile, const string &strSettingsFile,
         const eSensor sensor, const bool bUseViewer = true,
         const int initFr = 0, const string &strSequence = std::string(),
         const string &save_dir = std::string());

  Sophus::SE3f TrackStereo(shared_ptr<FrameWrapper> framewrapper);
  // Proccess the given stereo frame. Images must be synchronized and rectified.
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Returns the camera pose (empty if tracking fails).
  Sophus::SE3f TrackStereo(
      const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp,
      const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
      string filename = "");

  Sophus::SE3f TrackRGBD(shared_ptr<FrameWrapper> framewrapper);
  // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
  // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Input depthmap: Float (CV_32F). Returns the camera pose (empty
  // if tracking fails).
  Sophus::SE3f TrackRGBD(
      const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp,
      const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
      const vector<Eigen::Vector3f> &vOdomMeas = vector<Eigen::Vector3f>(),
      string filename = "");

  std::future<Sophus::SE3f> TrackRGBDAsync(
      const cv::Mat &imLeft, const cv::Mat &depthmap, const double &timestamp,
      const std::vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
      const std::vector<Eigen::Vector3f> &vOdomMeas = vector<Eigen::Vector3f>(),
      const std::string &filename = " ");

  Sophus::SE3f TrackMonocular(shared_ptr<FrameWrapper> framewrapper);
  // Proccess the given monocular frame and optionally imu data
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Returns the camera pose (empty if tracking fails).
  Sophus::SE3f TrackMonocular(
      const cv::Mat &im, const double &timestamp,
      const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
      string filename = "");

  std::future<Sophus::SE3f> TrackMonocularAsync(
      const cv::Mat &im, const double &timestamp,
      const std::vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
      const std::vector<Eigen::Vector3f> &vOdomMeas =
          vector<Eigen::Vector3f>(),
      const std::string &filename = "");
  // This stops local mapping thread (map building) and performs only camera
  // tracking.
  void ActivateLocalizationMode();
  // This resumes local mapping thread and performs SLAM again.
  void DeactivateLocalizationMode();

  // Returns true if there have been a big map change (loop closure, global BA)
  // since last call to this function
  bool MapChanged();

  // Reset the system (clear Atlas or the active map)
  void Reset();
  void ResetActiveMap();

  // All threads will be requested to finish.
  // It waits until all threads have finished.
  // This function must be called before saving the trajectory.
  void Shutdown(std::string save_dir = "./");
  bool isShutDown();

  void SaveFrame2FrameReprojErr(const string &filename);
  void SaveFrame2MapReprojErr(const string &filename);

  // Save camera trajectory in the TUM RGB-D dataset format.
  // Only for stereo and RGB-D. This method does not work for monocular.
  // Call first Shutdown()
  // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
  void SaveTrajectoryTUM(const string &filename);
  void SaveTrajectoryTUM2(const string &filename);
  void SaveTrajectory(const string &filename);
  void SaveTrackTimeCost(const string &filename,
                         const vector<float> &vTimesTrack);
  // Save keyframe poses in the TUM RGB-D dataset format.
  // This method works for all sensor input.
  // Call first Shutdown()
  // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
  void SaveKeyFrameTrajectoryTUM(const string &filename);
  void SaveKeyFrameLoopTrajectoryTUM(const string &filename);
  void SaveKeyFrameTrajectoryTUM2(const string &filename);
  void SaveKeyFrameLoopTrajectoryTUM2(const string &filename);

  void SaveTrajectoryEuRoC(const string &filename);
  void SaveKeyFrameTrajectoryEuRoC(const string &filename);

  void SaveTrajectoryEuRoC(const string &filename, Map *pMap);
  void SaveKeyFrameTrajectoryEuRoC(const string &filename, Map *pMap);

  // Save data used for initialization debug
  void SaveDebugData(const int &iniIdx);

  // Save camera trajectory in the KITTI dataset format.
  // Only for stereo and RGB-D. This method does not work for monocular.
  // Call first Shutdown()
  // See format details at:
  // http://www.cvlibs.net/datasets/kitti/eval_odometry.php
  void SaveTrajectoryKITTI(const string &filename);

  // TODO: Save/Load functions
  // SaveMap(const string &filename);
  // LoadMap(const string &filename);

  // Information from most recent processed frame
  // You can call this right after TrackMonocular (or stereo or RGBD)
  int GetTrackingState();
  int GetTrackLostCnt();
  std::vector<MapPoint *> GetTrackedMapPoints();
  std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();
  std::future<Sophus::SE3f> TrackStereoAsync(
      const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp,
      const std::vector<IMU::Point> &vImuMeas, const std::vector<Eigen::Vector3f> &vOdomMeas =
          vector<Eigen::Vector3f>(), const string &filename = "");
  void CreateTrackFrameThread();
  void NotifyProcessingComplete();
  void CreateFrameAndPush(
      const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp,
      const std::string &filename, const std::vector<IMU::Point> &ImuMeas,
      const std::vector<Eigen::Vector3f> &OdomMeas,
      std::shared_ptr<std::promise<Sophus::SE3f>> PosePromise);

  // 添加获取当前地图所有点云接口
  std::vector<MapPoint *> GetAllMapPoints();

  // 添加获取 mpFrameDrawer 成员变量接口
  FrameDrawer *GetmpFrameDrawe();
  int GetFrameQueueSize();

  // For debugging
  double GetTimeFromIMUInit();
  bool isLost();
  bool isFinished();

  void ChangeDataset();

  float GetImageScale();
  LoopClosing *GetLoopCloserVal();
  Settings *GetSetting();
  map<int, vector<float>> *GetFrame2FrameReprojectErr();
  map<int, vector<float>> *GetFrame2MapReprojectErr();

  vector<float> GetTrackTimes();

#ifdef REGISTER_TIMES
  void InsertRectTime(double &time);
  void InsertResizeTime(double &time);
  void InsertTrackTime(double &time);
#endif

 private:
  void SaveAtlas(int type);
  bool LoadAtlas(int type);

  string CalculateCheckSum(string filename, int type);

  // Input sensor
  eSensor mSensor;

  // ORB vocabulary used for place recognition and feature matching.
  ORBVocabulary *mpVocabulary;

  // KeyFrame database for place recognition (relocalization and loop
  // detection).
  KeyFrameDatabase *mpKeyFrameDatabase;

  // Map structure that stores the pointers to all KeyFrames and MapPoints.
  // Map* mpMap;
  Atlas *mpAtlas;

  // Tracker. It receives a frame and computes the associated camera pose.
  // It also decides when to insert a new keyframe, create some new MapPoints
  // and performs relocalization if tracking fails.
  Tracking *mpTracker;

  // Local Mapper. It manages the local map and performs local bundle
  // adjustment.
  LocalMapping *mpLocalMapper;

  // Loop Closer. It searches loops with every new keyframe. If there is a loop
  // it performs a pose graph optimization and full bundle adjustment (in a new
  // thread) afterwards.
  LoopClosing *mpLoopCloser;

  // The viewer draws the map and the current camera pose. It uses Pangolin.
  Viewer *mpViewer;
  LidarMapping *mpLidarMapping;
  FrameDrawer *mpFrameDrawer;
  MapDrawer *mpMapDrawer;

  // System threads: Local Mapping, Loop Closing, Viewer.
  // The Tracking thread "lives" in the main execution thread that creates the
  // System object.
  std::thread *mptLocalMapping;
  std::thread *mptLoopClosing;
  std::thread *mptViewer;

  // Reset flag
  std::mutex mMutexReset;
  bool mbReset;
  bool mbResetActiveMap;

  // Change mode flags
  std::mutex mMutexMode;
  bool mbActivateLocalizationMode;
  bool mbDeactivateLocalizationMode;

  // Shutdown flag
  bool mbShutDown;

  // Tracking state
  int mTrackingState;
  size_t mTrackLostCnt;
  std::vector<MapPoint *> mTrackedMapPoints;
  std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
  std::mutex mMutexState;
  std::mutex mMutexTrackedCnt;
  std::mutex mMutexTrackTimes;
  vector<float> vTimesTrack;

  //
  string mStrLoadAtlasFromFile;
  string mStrSaveAtlasToFile;

  string mStrVocabularyFilePath;

  Settings *settings_;
  std::shared_ptr<hobot::CThreadPool> mthreadPool;
  std::chrono::steady_clock::time_point mLastSubmitTime = std::chrono::steady_clock::now();
  std::atomic_bool mbfinished{false};
  std::mutex mQueueMutex;
  std::condition_variable mQueueCond;
  std::map<double, std::shared_ptr<FrameWrapper>, std::less<double>>
      mFrameQueue;
  std::shared_ptr<std::thread> mTrackThread;
  const size_t kMaxFrameQueueSize = 50;  // Frame queue size limit
  std::condition_variable mQueueNotFullCond;
};

}  // namespace ORB_SLAM3

#endif  // SYSTEM_H
