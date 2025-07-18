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

#ifndef FRAME_H
#define FRAME_H
#include <pcl/kdtree/kdtree_flann.h>  // pcl include kdtree_flann throws error if PCL_NO_PRECOMPILE
// is defined before
#define PCL_NO_PRECOMPILE
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mutex>
#include <opencv2/opencv.hpp>
#include <vector>

#include "Converter.h"
#include "Eigen/Core"
#include "ImuTypes.h"
#include "LidarProcess.h"
#include "ORBVocabulary.h"
#include "Settings.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "Thirdparty/Sophus/sophus/geometry.hpp"
#include "sophus/se3.hpp"

typedef pcl::PointXYZRGBA PointType;
namespace ORB_SLAM3 {
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64
class MapPoint;
class KeyFrame;
class ConstraintPoseImu;
class ConstraintPoseICP;
class GeometricCamera;
class ORBextractor;
class feature_pt {
 public:
  feature_pt(KeyFrame *kf, int id, cv::Mat des, bool is3d = false)
      : first_kf(kf), index(id), is_3d(is3d), des_(des) {}
  KeyFrame *first_kf;  // 第一次提取特征时的关键帧id
  int index;           // 一次提取特征时的在关键帧中所处的位置
  bool is_3d;          // 是否是3d点
  MapPoint *mp = nullptr;
  cv::Mat des_;
};
class Frame {
 public:
  cv::Size frameSize;
  Frame();

  // Copy constructor.
  Frame(const Frame &frame);

  // Constructor for stereo cameras.
  Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
        ORBextractor *extractorLeft, ORBextractor *extractorRight,
        ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf,
        const float &thDepth, GeometricCamera *pCamera,
        Frame *pPrevF = static_cast<Frame *>(NULL),
        const IMU::Calib &ImuCalib = IMU::Calib());

  // Constructor for RGB-D cameras.
  Frame(const cv::Mat &imGray, const cv::Mat &imRGB, const cv::Mat &imDepth,
        const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc,
        Settings *setting, cv::Mat &K, cv::Mat &distCoef, const float &bf,
        const float &thDepth, GeometricCamera *pCamera,
        Frame *pPrevF = static_cast<Frame *>(NULL),
        const IMU::Calib &ImuCalib = IMU::Calib());

  // Constructor for Monocular cameras.
  Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor,
        ORBVocabulary *voc, GeometricCamera *pCamera, Settings *setting,
        cv::Mat &distCoef, const float &bf, const float &thDepth,
        Frame *pPrevF = static_cast<Frame *>(NULL),
        const IMU::Calib &ImuCalib = IMU::Calib());

  // Destructor
  // ~Frame();
  // void extract();
  void AddFeatures(const bool mono);

  void AddPts(const std::vector<cv::KeyPoint> &pts,
              const std::vector<MapPoint *> &mps, const bool mono);

  void ClearKeyPoints();
  // Extract ORB on the image. 0 for left image and 1 for right image.
  void ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1);

  // Compute Bag of Words representation.
  void ComputeBoW();

  // Set the camera pose. (Imu pose is not modified!)
  void SetPose(const Sophus::SE3<float> &Tcw);

  // Set IMU velocity
  void SetVelocity(Eigen::Vector3f Vw);

  Eigen::Vector3f GetVelocity() const;
  void SetPose(const Eigen::Matrix3f &Rwc, const Eigen::Vector3f &twc);
  void SetOnlyICPPose(const Eigen::Matrix3f &Rwc, const Eigen::Vector3f &twc);
  void SetICPDeltaPose(const Eigen::Matrix3d &dR, const Eigen::Vector3d &dt);
  void GetICPDeltaPose(Eigen::Matrix3d &dR, Eigen::Vector3d &dt);
  // Set IMU pose and velocity (implicitly changes camera pose)
  void SetImuPoseVelocity(const Eigen::Matrix3f &Rwb,
                          const Eigen::Vector3f &twb,
                          const Eigen::Vector3f &Vwb);

  Eigen::Matrix<float, 3, 1> GetImuPosition() const;
  Eigen::Matrix<float, 3, 3> GetImuRotation();
  Sophus::SE3<float> GetImuPose();

  Sophus::SE3f GetRelativePoseTrl();
  Sophus::SE3f GetRelativePoseTlr();
  Eigen::Matrix3f GetRelativePoseTlr_rotation();
  Eigen::Vector3f GetRelativePoseTlr_translation();

  void SetNewBias(const IMU::Bias &b);

  // Check if a MapPoint is in the frustum of the camera
  // and fill variables of the MapPoint to be used by the tracking
  bool isInFrustum(MapPoint *pMP, float viewingCosLimit);

  bool ProjectPointDistort(MapPoint *pMP, cv::Point2f &kp, float &u, float &v);

  Eigen::Vector3f inRefCoordinates(Eigen::Vector3f pCw);

  // Compute the cell of a keypoint (return false if outside the grid)
  bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

  vector<size_t> GetFeaturesInArea(const float &x, const float &y,
                                   const float &r, const int minLevel = -1,
                                   const int maxLevel = -1,
                                   const bool bRight = false) const;

  // Search a match for each keypoint in the left image to a keypoint in the
  // right image. If there is a match, depth is computed and the right
  // coordinate associated to the left keypoint is stored.
  void ComputeStereoMatches();

  // Associate a "right" coordinate to a keypoint if there is valid depth in
  // the depthmap.
  void ComputeStereoFromRGBD(const cv::Mat &imDepth);

  // Backprojects a keypoint (if stereo/depth info available) into 3D world
  // coordinates.
  bool UnprojectStereo(const int &i, Eigen::Vector3f &x3D);
  bool UnprojectRGBD(const int &i, Eigen::Vector3f &x3D);

  ConstraintPoseImu *mpcpi;
  ConstraintPoseImu *pre_mpcpi;
  ConstraintPoseICP *mpcp_icp;

  bool imuIsPreintegrated();
  void setIntegrated();

  bool isSet() const;

  // Computes rotation, translation and camera center matrices from the camera
  // pose.
  void UpdatePoseMatrices();

  // Returns the camera center.
  inline Eigen::Vector3f GetCameraCenter() { return mOw; }

  // Returns inverse of rotation
  inline Eigen::Matrix3f GetRotationInverse() { return mRwc; }

  inline Sophus::SE3<float> GetPose() const {
    // TODO: can the Frame pose be accsessed from several threads? should
    // this be protected somehow?
    unique_lock<std::mutex> lock(*mMutexPose);
    return mTcw;
  }

  inline Sophus::SE3<float> GetICPPose() const {
    // TODO: can the Frame pose be accsessed from several threads? should
    // this be protected somehow?
    return mOnlyICPTwc;
  }
  inline Eigen::Matrix3f GetRwc() const { return mRwc; }

  inline Eigen::Vector3f GetOw() const { return mOw; }

  inline bool HasPose() const { return mbHasPose; }

  inline bool HasICPDeltaPose() const { return mbHasICPDeltaPose; }

  inline bool HasVelocity() const { return mbHasVelocity; }

  void SetFrame2FrameReprojError(const float &reproj) {
    // mFrame2FrameReprojError[idx].emplace_back(reproj);
    mFrame2FrameReprojError.emplace_back(reproj);
  }
  void SetFrame2MapReprojError(const float &reproj) {
    // mFrame2MapReprojError[idx].emplace_back(reproj);
    mFrame2MapReprojError.emplace_back(reproj);
  }
  vector<float> GetFrame2FrameReprojError() { return mFrame2FrameReprojError; }
  vector<float> GetFrame2MapReprojError() { return mFrame2MapReprojError; }

 private:
  vector<float> mFrame2FrameReprojError;
  vector<float> mFrame2MapReprojError;
  // Sophus/Eigen migration
  Sophus::SE3<float> mTcw;
  Sophus::SE3<float> mOnlyICPTwc;
  Eigen::Matrix<float, 3, 3> mRwc;
  Eigen::Matrix<float, 3, 1> mOw;
  Eigen::Matrix<float, 3, 3> mRcw;
  Eigen::Matrix<float, 3, 1> mtcw;

  // ICP delta pose
  Eigen::Matrix<double, 3, 3> mICPdR;
  Eigen::Matrix<double, 3, 1> mICPdt;
  bool mbHasPose;
  bool mbHasICPDeltaPose;

  // Rcw_ not necessary as Sophus has a method for extracting the rotation
  // matrix: Tcw_.rotationMatrix() tcw_ not necessary as Sophus has a method
  // for extracting the translation vector: Tcw_.translation() Twc_ not
  // necessary as Sophus has a method for easily computing the inverse pose:
  // Tcw_.inverse()

  Sophus::SE3<float> mTlr, mTrl;
  Eigen::Matrix<float, 3, 3> mRlr;
  Eigen::Vector3f mtlr;

  // IMU linear velocity
  Eigen::Vector3f mVw;
  bool mbHasVelocity;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Vocabulary used for relocalization.
  ORBVocabulary *mpORBvocabulary;

  // Feature extractor. The right is used only in the stereo case.
  ORBextractor *mpORBextractorLeft, *mpORBextractorRight;
  Settings *mpSettings;
  // Frame timestamp.
  double mTimeStamp;

  // Calibration matrix and OpenCV distortion parameters.
  cv::Mat mK;
  Eigen::Matrix3f mK_;
  static float fx;
  static float fy;
  static float cx;
  static float cy;
  static float invfx;
  static float invfy;
  cv::Mat mDistCoef;
  cv::Mat image;
  cv::Mat imageRGB;
  cv::Mat imageDepth;
  pcl::PointCloud<PointType>::Ptr mpPointCloud, mpPointCloudDownsampled;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  std::shared_ptr<std::vector<Eigen::Vector4f>> source_points;  // 深度点云
  std::shared_ptr<LaserProcessingClass> laserProcessing;
  // Stereo baseline multiplied by fx.
  float mbf;

  // Stereo baseline in meters.
  float mb;

  // Threshold close/far points. Close points are inserted from 1 view.
  // Far points are inserted as in the monocular case from 2 views.
  float mThDepth;

  // Number of KeyPoints.
  int N;

  // Vector of keypoints (original for visualization) and undistorted
  // (actually used by the system). In the stereo case, mvKeysUn is redundant
  // as images must be rectified. In the RGB-D case, RGB images can be
  // distorted.
  std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
  std::vector<cv::KeyPoint> mvKeysUn;
  std::vector<cv::KeyPoint> mvProjectedKeys;
  std::vector<cv::Mat> mImGray;
  // Corresponding stereo coordinate and depth for each keypoint.
  std::vector<MapPoint *> mvpMapPoints;
  // "Monocular" keypoints have a negative value.
  std::vector<float> mvuRight;
  std::vector<float> mvDepth;

  // Bag of Words Vector structures.
  DBoW2::BowVector mBowVec;
  DBoW2::FeatureVector mFeatVec;

  // ORB descriptor, each row associated to a keypoint.
  cv::Mat mDescriptors, mDescriptorsRight;

  // MapPoints associated to keypoints, NULL pointer if no association.
  // Flag to identify outlier associations.
  std::vector<bool> mvbOutlier;
  int mnCloseMPs;

  // Keypoints are assigned to cells in a grid to reduce matching complexity
  // when projecting MapPoints.
  static float mfGridElementWidthInv;
  static float mfGridElementHeightInv;
  std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

  IMU::Bias mPredBias;

  // IMU bias
  IMU::Bias mImuBias;

  // Imu calibration
  IMU::Calib mImuCalib;

  // Imu preintegration from last keyframe
  IMU::Preintegrated *mpImuPreintegrated;
  KeyFrame *mpLastKeyFrame;

  // Pointer to previous frame
  Frame *mpPrevFrame;
  IMU::Preintegrated *mpImuPreintegratedFrame;

  // Current and Next Frame id.
  static long unsigned int nNextId;
  long unsigned int mnId;

  // Reference Keyframe.
  KeyFrame *mpReferenceKF;

  // Scale pyramid info.
  int mnScaleLevels;
  float mfScaleFactor;
  float mfLogScaleFactor;
  vector<float> mvScaleFactors;
  vector<float> mvInvScaleFactors;
  vector<float> mvLevelSigma2;
  vector<float> mvInvLevelSigma2;

  // Undistorted Image Bounds (computed once).
  static float mnMinX;
  static float mnMaxX;
  static float mnMinY;
  static float mnMaxY;

  static bool mbInitialComputations;

  map<long unsigned int, cv::Point2f> mmProjectPoints;
  map<long unsigned int, cv::Point2f> mmMatchedInImage;
  std::vector<std::shared_ptr<feature_pt>> track_feature_pts_;
  string mNameFile;

  int mnDataset;
  std::vector<IMU::Point> mvImus;
  int mnWinsizeLK;
  float mfProjError, mfAvgProjError;

  // Current matches in frame
  int mnMatchesInliers;

#ifdef REGISTER_TIMES
  double mTimeORB_Ext;
  double mTimeStereoMatch;
#endif

 private:
  // Undistort keypoints given OpenCV distortion parameters.
  // Only for the RGB-D case. Stereo must be already rectified!
  // (called in the constructor).
  void UndistortKeyPoints();

  // Computes image bounds for the undistorted image (called in the
  // constructor).
  void ComputeImageBounds(const cv::Mat &imLeft);

  // Assign keypoints to the grid for speed up feature matching (called in the
  // constructor).
  // void AssignFeaturesToGrid();

  bool mbIsSet;

  bool mbImuPreintegrated;

  std::mutex *mpMutexImu;
  std::mutex *mpMutexKeyPoints;
  std::mutex *mMutexFeatures;
  std::mutex *mMutexPose;

 public:
  GeometricCamera *mpCamera, *mpCamera2;

  // Number of KeyPoints extracted in the left and right images
  int Nleft, Nright;
  // Number of Non Lapping Keypoints
  int monoLeft, monoRight;

  // For stereo matching
  std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

  // For stereo fisheye matching
  static cv::BFMatcher BFmatcher;

  // Triangulated stereo observations using as reference the left camera.
  // These are computed during ComputeStereoFishEyeMatches
  std::vector<Eigen::Vector3f> mvStereo3Dpoints;

  // Grid for the right image
  std::vector<std::size_t> mGridRight[FRAME_GRID_COLS][FRAME_GRID_ROWS];

  Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
        ORBextractor *extractorLeft, ORBextractor *extractorRight,
        ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf,
        const float &thDepth, GeometricCamera *pCamera,
        GeometricCamera *pCamera2, Sophus::SE3f &Tlr,
        Frame *pPrevF = static_cast<Frame *>(NULL),
        const IMU::Calib &ImuCalib = IMU::Calib());

  // Stereo fisheye
  void ComputeStereoFishEyeMatches();
  void ConvertDepthToPointCloud(int downsample = 1,
                                float DownsizeResolution = 0.1);
  void ConvertKPDepthToPointCloud();
  bool isInFrustumChecks(MapPoint *pMP, float viewingCosLimit,
                         bool bRight = false);

  Eigen::Vector3f UnprojectStereoFishEye(const int &i);
  void AssignFeaturesToGrid();
  cv::Mat imgLeft, imgRight;

  void PrintPointDistribution() {
    int left = 0, right = 0;
    int Nlim = (Nleft != -1) ? Nleft : N;
    for (int i = 0; i < N; i++) {
      if (mvpMapPoints[i] && !mvbOutlier[i]) {
        if (i < Nlim)
          left++;
        else
          right++;
      }
    }
    cout << "Point distribution in Frame: left-> " << left << " --- right-> "
         << right << endl;
  }

  Sophus::SE3<double> T_test;
};

}  // namespace ORB_SLAM3

#endif  // FRAME_H
