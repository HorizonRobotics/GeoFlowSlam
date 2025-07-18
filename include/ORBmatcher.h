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

#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "sophus/sim3.hpp"

namespace ORB_SLAM3 {

class ORBmatcher {
 public:
  ORBmatcher(float nnratio = 0.6, bool checkOri = true);

  // Computes the Hamming distance between two ORB descriptors
  static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

  // Search matches between Frame keypoints and projected MapPoints. Returns
  // number of matches Used to track the local map (Tracking)
  int SearchByProjection(Frame &F, const std::vector<MapPoint *> &vpMapPoints,
                         const float th = 3, const bool bFarPoints = false,
                         const float thFarPoints = 50.0f);
  int FilterOutliers(Frame &F, const float F_THRESHOLD);

  // Project MapPoints tracked in last frame into the current frame and search
  // matches. Used to track from previous frame (Tracking)
  int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame,
                         const float th, const bool bMono);

  bool inBorder(const cv::Point2f &pt, const cv::Mat &im) const;
  void fbKltTracking(const std::vector<cv::Mat> &vprevpyr,
                     const std::vector<cv::Mat> &vcurpyr, int nwinsize,
                     int nbpyrlvl, float ferr, float fmax_fbklt_dist,
                     std::vector<cv::Point2f> &vkps,
                     std::vector<cv::Point2f> &vpriorkps,
                     std::vector<bool> &vkpstatus) const;
  // Project MapPoints seen in KeyFrame into the Frame and search matches.
  // Used in relocalisation (Tracking)
  int SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF,
                         const std::set<MapPoint *> &sAlreadyFound,
                         const float th, const int ORBdist);
  int SearchByProjectionWithOF(Frame &CurrentFrame, const Frame &LastFrame,
                               cv::Mat &mask, const float th, const int winsize,
                               const float F_THRESHOLD,
                               const int DIST_THRESHOLD, const bool bMono);
  int SearchWithGMS(KeyFrame *pKF, Frame &F,
                    std::vector<MapPoint *> &vpMapPointMatches);
  bool isPointNearby(const cv::Mat &mask, const cv::Point2f &pt);
  void updateMask(cv::Mat &mask, const cv::Point2f &pt, int radius);
  float CalculateUniformity(Frame &CurrentFrame);
  // Project MapPoints using a Similarity Transformation and search matches.
  // Used in loop detection (Loop Closing)
  int SearchByProjection(KeyFrame *pKF, Sophus::Sim3<float> &Scw,
                         const std::vector<MapPoint *> &vpPoints,
                         std::vector<MapPoint *> &vpMatched, int th,
                         float ratioHamming = 1.0);

  // Project MapPoints using a Similarity Transformation and search matches.
  // Used in Place Recognition (Loop Closing and Merging)
  int SearchByProjection(KeyFrame *pKF, Sophus::Sim3<float> &Scw,
                         const std::vector<MapPoint *> &vpPoints,
                         const std::vector<KeyFrame *> &vpPointsKFs,
                         std::vector<MapPoint *> &vpMatched,
                         std::vector<KeyFrame *> &vpMatchedKF, int th,
                         float ratioHamming = 1.0);

  // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
  // Brute force constrained to ORB that belong to the same vocabulary node (at
  // a certain level) Used in Relocalisation and Loop Detection
  int SearchByBoW(KeyFrame *pKF, Frame &F,
                  std::vector<MapPoint *> &vpMapPointMatches);
  int SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2,
                  std::vector<MapPoint *> &vpMatches12);
  int SearchByBoW(Frame &F1, Frame &F2, vector<int> &vnMatches12);
  // Matching for the Map Initialization (only used in the monocular case)
  int SearchForInitialization(Frame &F1, Frame &F2,
                              std::vector<cv::Point2f> &vbPrevMatched,
                              std::vector<int> &vnMatches12,
                              int windowSize = 10);
  int SearchForInitializationWithGMS(Frame &Frame_1, Frame &Frame_2,
                                     vector<int> &vnMatches12);  // tracking
  // Matching to triangulate new MapPoints. Check Epipolar Constraint.
  int SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2,
                             std::vector<pair<size_t, size_t> > &vMatchedPairs,
                             const bool bOnlyStereo,
                             const bool bCoarse = false);
  int SearchForTriangulationWithGMS(
      KeyFrame *pKF1, KeyFrame *pKF2, cv::Mat F12,
      std::vector<pair<size_t, size_t> > &vMatchedPairs,
      const bool bOnlyStereo);
  // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3
  // [s12*R12|t12] In the stereo and RGB-D case, s12=1 int
  // SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *>
  // &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12,
  // const float th);
  int SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2,
                   std::vector<MapPoint *> &vpMatches12,
                   const Sophus::Sim3f &S12, const float th);

  // Project MapPoints into KeyFrame and search for duplicated MapPoints.
  int Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints,
           const float th = 3.0, const bool bRight = false);

  // Project MapPoints into KeyFrame using a given Sim3 and search for
  // duplicated MapPoints.
  int Fuse(KeyFrame *pKF, Sophus::Sim3f &Scw,
           const std::vector<MapPoint *> &vpPoints, float th,
           vector<MapPoint *> &vpReplacePoint);

 public:
  static const int TH_LOW;
  static const int TH_HIGH;
  static const int HISTO_LENGTH;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                             const cv::Mat &F12, const KeyFrame *pKF);
  float RadiusByViewingCos(const float &viewCos);

  void ComputeThreeMaxima(std::vector<int> *histo, const int L, int &ind1,
                          int &ind2, int &ind3);

  float mfNNratio;
  bool mbCheckOrientation;
};

}  // namespace ORB_SLAM3

#endif  // ORBMATCHER_H