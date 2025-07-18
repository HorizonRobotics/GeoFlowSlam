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

#include "ORBmatcher.h"

#include <limits.h>
#include <stdint-gcc.h>

#include <opencv2/core/core.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "Thirdparty/GMS/include/Header.h"
#include "Thirdparty/GMS/include/gms_matcher.h"
using namespace std;

namespace ORB_SLAM3 {

const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;

ORBmatcher::ORBmatcher(float nnratio, bool checkOri)
    : mfNNratio(nnratio), mbCheckOrientation(checkOri) {}

int ORBmatcher::SearchByProjection(Frame &F,
                                   const vector<MapPoint *> &vpMapPoints,
                                   const float th, const bool bFarPoints,
                                   const float thFarPoints) {
  int nmatches = 0, left = 0, right = 0;

  const bool bFactor = th != 1.0;

  for (size_t iMP = 0; iMP < vpMapPoints.size(); iMP++) {
    MapPoint *pMP = vpMapPoints[iMP];
    if (!pMP->mbTrackInView && !pMP->mbTrackInViewR) continue;

    if (bFarPoints && pMP->mTrackDepth > thFarPoints) continue;

    if (pMP->isBad()) continue;

    if (pMP->mbTrackInView) {
      const int &nPredictedLevel = pMP->mnTrackScaleLevel;

      // The size of the window will depend on the viewing direction
      float r = RadiusByViewingCos(pMP->mTrackViewCos);

      if (bFactor) r *= th;

      const vector<size_t> vIndices =
          F.GetFeaturesInArea(pMP->mTrackProjX, pMP->mTrackProjY,
                              r * F.mvScaleFactors[nPredictedLevel],
                              nPredictedLevel - 1, nPredictedLevel);

      if (!vIndices.empty()) {
        const cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist = 256;
        int bestLevel = -1;
        int bestDist2 = 256;
        int bestLevel2 = -1;
        int bestIdx = -1;

        // Get best and second matches with near keypoints
        for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                            vend = vIndices.end();
             vit != vend; vit++) {
          const size_t idx = *vit;

          if (F.mvpMapPoints[idx])
            if (F.mvpMapPoints[idx]->Observations() > 0) continue;

          if (F.Nleft == -1 && F.mvuRight[idx] > 0) {
            const float er = fabs(pMP->mTrackProjXR - F.mvuRight[idx]);
            if (er > r * F.mvScaleFactors[nPredictedLevel]) continue;
          }

          const cv::Mat &d = F.mDescriptors.row(idx);

          const int dist = DescriptorDistance(MPdescriptor, d);

          if (dist < bestDist) {
            bestDist2 = bestDist;
            bestDist = dist;
            bestLevel2 = bestLevel;
            bestLevel = (F.Nleft == -1)   ? F.mvKeysUn[idx].octave
                        : (idx < F.Nleft) ? F.mvKeys[idx].octave
                                          : F.mvKeysRight[idx - F.Nleft].octave;
            bestIdx = idx;
          } else if (dist < bestDist2) {
            bestLevel2 = (F.Nleft == -1) ? F.mvKeysUn[idx].octave
                         : (idx < F.Nleft)
                             ? F.mvKeys[idx].octave
                             : F.mvKeysRight[idx - F.Nleft].octave;
            bestDist2 = dist;
          }
        }

        // Apply ratio to second match (only if best and second are in the same
        // scale level)
        if (bestDist <= TH_HIGH) {
          if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
            continue;

          if (bestLevel != bestLevel2 || bestDist <= mfNNratio * bestDist2) {
            F.mvpMapPoints[bestIdx] = pMP;

            if (F.Nleft != -1 && F.mvLeftToRightMatch[bestIdx] !=
                                     -1) {  // Also match with the stereo
                                            // observation at right camera
              F.mvpMapPoints[F.mvLeftToRightMatch[bestIdx] + F.Nleft] = pMP;
              nmatches++;
              right++;
            }

            nmatches++;
            left++;
          }
        }
      }
    }

    if (F.Nleft != -1 && pMP->mbTrackInViewR) {
      const int &nPredictedLevel = pMP->mnTrackScaleLevelR;
      if (nPredictedLevel != -1) {
        float r = RadiusByViewingCos(pMP->mTrackViewCosR);

        const vector<size_t> vIndices =
            F.GetFeaturesInArea(pMP->mTrackProjXR, pMP->mTrackProjYR,
                                r * F.mvScaleFactors[nPredictedLevel],
                                nPredictedLevel - 1, nPredictedLevel, true);

        if (vIndices.empty()) continue;

        const cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist = 256;
        int bestLevel = -1;
        int bestDist2 = 256;
        int bestLevel2 = -1;
        int bestIdx = -1;

        // Get best and second matches with near keypoints
        for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                            vend = vIndices.end();
             vit != vend; vit++) {
          const size_t idx = *vit;

          if (F.mvpMapPoints[idx + F.Nleft])
            if (F.mvpMapPoints[idx + F.Nleft]->Observations() > 0) continue;

          const cv::Mat &d = F.mDescriptors.row(idx + F.Nleft);

          const int dist = DescriptorDistance(MPdescriptor, d);

          if (dist < bestDist) {
            bestDist2 = bestDist;
            bestDist = dist;
            bestLevel2 = bestLevel;
            bestLevel = F.mvKeysRight[idx].octave;
            bestIdx = idx;
          } else if (dist < bestDist2) {
            bestLevel2 = F.mvKeysRight[idx].octave;
            bestDist2 = dist;
          }
        }

        // Apply ratio to second match (only if best and second are in the same
        // scale level)
        if (bestDist <= TH_HIGH) {
          if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
            continue;

          if (F.Nleft != -1 && F.mvRightToLeftMatch[bestIdx] !=
                                   -1) {  // Also match with the stereo
                                          // observation at right camera
            F.mvpMapPoints[F.mvRightToLeftMatch[bestIdx]] = pMP;
            nmatches++;
            left++;
          }

          F.mvpMapPoints[bestIdx + F.Nleft] = pMP;
          nmatches++;
          right++;
        }
      }
    }
  }
  return nmatches;
}
int ORBmatcher::FilterOutliers(Frame &Frame, const float F_THRESHOLD) {
  // 计算当前帧地图投影点
  const Sophus::SE3f Tcw = Frame.GetPose();
  const Eigen::Vector3f twc = Tcw.inverse().translation();
  std::vector<cv::Point2f> un_cur_pts, un_forw_pts;
  for (int i = 0; i < Frame.N; i++) {
    MapPoint *pMP = Frame.mvpMapPoints[i];
    if (pMP) {
      // 默认不是外点
      Frame.mvbOutlier[i] = false;
      // Project
      Eigen::Vector3f x3Dw = pMP->GetWorldPos();
      Eigen::Vector3f x3Dc = Tcw * x3Dw;

      const float xc = x3Dc(0);
      const float yc = x3Dc(1);
      const float invzc = 1.0 / x3Dc(2);
      if (invzc < 0) continue;

      Eigen::Vector2f uv = Frame.mpCamera->project(x3Dc);
      un_forw_pts.emplace_back(Frame.mvKeys.at(i).pt);
      un_cur_pts.emplace_back(uv[0], uv[1]);
    }
  }
  // F check
  int inliner = 0;
  if (un_cur_pts.size() > 8) {
    vector<uchar> status;
    cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD,
                           0.99, status);
    for (int i = 0; i < status.size(); i++) {
      // 如果是外点
      if (!status[i]) {
        Frame.mvbOutlier[i] = true;
      } else {
        inliner++;
      }
    }
  }
  return inliner;
}

float ORBmatcher::RadiusByViewingCos(const float &viewCos) {
  if (viewCos > 0.998)
    return 2.5;
  else
    return 4.0;
}

int ORBmatcher::SearchByBoW(KeyFrame *pKF, Frame &F,
                            vector<MapPoint *> &vpMapPointMatches) {
  const vector<MapPoint *> vpMapPointsKF = pKF->GetMapPointMatches();

  vpMapPointMatches = vector<MapPoint *>(F.N, static_cast<MapPoint *>(NULL));

  const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;

  int nmatches = 0;

  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);
  const float factor = 1.0f / HISTO_LENGTH;

  // We perform the matching over ORB that belong to the same vocabulary node
  // (at a certain level)
  DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
  DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
  DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
  DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

  while (KFit != KFend && Fit != Fend) {
    if (KFit->first == Fit->first) {
      const vector<unsigned int> vIndicesKF = KFit->second;
      const vector<unsigned int> vIndicesF = Fit->second;

      for (size_t iKF = 0; iKF < vIndicesKF.size(); iKF++) {
        const unsigned int realIdxKF = vIndicesKF[iKF];

        MapPoint *pMP = vpMapPointsKF[realIdxKF];

        if (!pMP) continue;

        if (pMP->isBad()) continue;

        const cv::Mat &dKF = pKF->mDescriptors.row(realIdxKF);

        int bestDist1 = 256;
        int bestIdxF = -1;
        int bestDist2 = 256;

        int bestDist1R = 256;
        int bestIdxFR = -1;
        int bestDist2R = 256;

        for (size_t iF = 0; iF < vIndicesF.size(); iF++) {
          if (F.Nleft == -1) {
            const unsigned int realIdxF = vIndicesF[iF];

            if (vpMapPointMatches[realIdxF]) continue;

            const cv::Mat &dF = F.mDescriptors.row(realIdxF);

            const int dist = DescriptorDistance(dKF, dF);

            if (dist < bestDist1) {
              bestDist2 = bestDist1;
              bestDist1 = dist;
              bestIdxF = realIdxF;
            } else if (dist < bestDist2) {
              bestDist2 = dist;
            }
          } else {
            const unsigned int realIdxF = vIndicesF[iF];

            if (vpMapPointMatches[realIdxF]) continue;

            const cv::Mat &dF = F.mDescriptors.row(realIdxF);

            const int dist = DescriptorDistance(dKF, dF);

            if (realIdxF < F.Nleft && dist < bestDist1) {
              bestDist2 = bestDist1;
              bestDist1 = dist;
              bestIdxF = realIdxF;
            } else if (realIdxF < F.Nleft && dist < bestDist2) {
              bestDist2 = dist;
            }

            if (realIdxF >= F.Nleft && dist < bestDist1R) {
              bestDist2R = bestDist1R;
              bestDist1R = dist;
              bestIdxFR = realIdxF;
            } else if (realIdxF >= F.Nleft && dist < bestDist2R) {
              bestDist2R = dist;
            }
          }
        }

        if (bestDist1 <= TH_LOW) {
          if (static_cast<float>(bestDist1) <
              mfNNratio * static_cast<float>(bestDist2)) {
            vpMapPointMatches[bestIdxF] = pMP;

            const cv::KeyPoint &kp =
                (!pKF->mpCamera2) ? pKF->mvKeysUn[realIdxKF]
                : (realIdxKF >= pKF->NLeft)
                    ? pKF->mvKeysRight[realIdxKF - pKF->NLeft]
                    : pKF->mvKeys[realIdxKF];

            if (mbCheckOrientation) {
              cv::KeyPoint &Fkp =
                  (!pKF->mpCamera2 || F.Nleft == -1) ? F.mvKeys[bestIdxF]
                  : (bestIdxF >= F.Nleft) ? F.mvKeysRight[bestIdxF - F.Nleft]
                                          : F.mvKeys[bestIdxF];

              float rot = kp.angle - Fkp.angle;
              if (rot < 0.0) rot += 360.0f;
              int bin = round(rot * factor);
              if (bin == HISTO_LENGTH) bin = 0;
              assert(bin >= 0 && bin < HISTO_LENGTH);
              rotHist[bin].push_back(bestIdxF);
            }
            nmatches++;
          }

          if (bestDist1R <= TH_LOW) {
            if (static_cast<float>(bestDist1R) <
                    mfNNratio * static_cast<float>(bestDist2R) ||
                true) {
              vpMapPointMatches[bestIdxFR] = pMP;

              const cv::KeyPoint &kp =
                  (!pKF->mpCamera2) ? pKF->mvKeysUn[realIdxKF]
                  : (realIdxKF >= pKF->NLeft)
                      ? pKF->mvKeysRight[realIdxKF - pKF->NLeft]
                      : pKF->mvKeys[realIdxKF];

              if (mbCheckOrientation) {
                cv::KeyPoint &Fkp = (!F.mpCamera2) ? F.mvKeys[bestIdxFR]
                                    : (bestIdxFR >= F.Nleft)
                                        ? F.mvKeysRight[bestIdxFR - F.Nleft]
                                        : F.mvKeys[bestIdxFR];

                float rot = kp.angle - Fkp.angle;
                if (rot < 0.0) rot += 360.0f;
                int bin = round(rot * factor);
                if (bin == HISTO_LENGTH) bin = 0;
                assert(bin >= 0 && bin < HISTO_LENGTH);
                rotHist[bin].push_back(bestIdxFR);
              }
              nmatches++;
            }
          }
        }
      }

      KFit++;
      Fit++;
    } else if (KFit->first < Fit->first) {
      KFit = vFeatVecKF.lower_bound(Fit->first);
    } else {
      Fit = F.mFeatVec.lower_bound(KFit->first);
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3) continue;
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
        vpMapPointMatches[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
        nmatches--;
      }
    }
  }

  return nmatches;
}

int ORBmatcher::SearchByProjection(KeyFrame *pKF, Sophus::Sim3f &Scw,
                                   const vector<MapPoint *> &vpPoints,
                                   vector<MapPoint *> &vpMatched, int th,
                                   float ratioHamming) {
  // Get Calibration Parameters for later projection
  const float &fx = pKF->fx;
  const float &fy = pKF->fy;
  const float &cx = pKF->cx;
  const float &cy = pKF->cy;

  Sophus::SE3f Tcw =
      Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
  Eigen::Vector3f Ow = Tcw.inverse().translation();

  // Set of MapPoints already found in the KeyFrame
  set<MapPoint *> spAlreadyFound(vpMatched.begin(), vpMatched.end());
  spAlreadyFound.erase(static_cast<MapPoint *>(NULL));

  int nmatches = 0;

  // For each Candidate MapPoint Project and Match
  for (int iMP = 0, iendMP = vpPoints.size(); iMP < iendMP; iMP++) {
    MapPoint *pMP = vpPoints[iMP];

    // Discard Bad MapPoints and already found
    if (pMP->isBad() || spAlreadyFound.count(pMP)) continue;

    // Get 3D Coords.
    Eigen::Vector3f p3Dw = pMP->GetWorldPos();

    // Transform into Camera Coords.
    Eigen::Vector3f p3Dc = Tcw * p3Dw;

    // Depth must be positive
    if (p3Dc(2) < 0.0) continue;

    // Project into Image
    const Eigen::Vector2f uv = pKF->mpCamera->project(p3Dc);

    // Point must be inside the image
    if (!pKF->IsInImage(uv(0), uv(1))) continue;

    // Depth must be inside the scale invariance region of the point
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    Eigen::Vector3f PO = p3Dw - Ow;
    const float dist = PO.norm();

    if (dist < minDistance || dist > maxDistance) continue;

    // Viewing angle must be less than 60 deg
    Eigen::Vector3f Pn = pMP->GetNormal();

    if (PO.dot(Pn) < 0.5 * dist) continue;

    int nPredictedLevel = pMP->PredictScale(dist, pKF);

    // Search in a radius
    const float radius = th * pKF->mvScaleFactors[nPredictedLevel];

    const vector<size_t> vIndices =
        pKF->GetFeaturesInArea(uv(0), uv(1), radius);

    if (vIndices.empty()) continue;

    // Match to the most similar keypoint in the radius
    const cv::Mat dMP = pMP->GetDescriptor();

    int bestDist = 256;
    int bestIdx = -1;
    for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                        vend = vIndices.end();
         vit != vend; vit++) {
      const size_t idx = *vit;
      if (vpMatched[idx]) continue;

      const int &kpLevel = pKF->mvKeysUn[idx].octave;

      if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel) continue;

      const cv::Mat &dKF = pKF->mDescriptors.row(idx);

      const int dist = DescriptorDistance(dMP, dKF);

      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = idx;
      }
    }

    if (bestDist <= TH_LOW * ratioHamming) {
      vpMatched[bestIdx] = pMP;
      nmatches++;
    }
  }

  return nmatches;
}

int ORBmatcher::SearchByProjection(KeyFrame *pKF, Sophus::Sim3<float> &Scw,
                                   const std::vector<MapPoint *> &vpPoints,
                                   const std::vector<KeyFrame *> &vpPointsKFs,
                                   std::vector<MapPoint *> &vpMatched,
                                   std::vector<KeyFrame *> &vpMatchedKF, int th,
                                   float ratioHamming) {
  // Get Calibration Parameters for later projection
  const float &fx = pKF->fx;
  const float &fy = pKF->fy;
  const float &cx = pKF->cx;
  const float &cy = pKF->cy;

  Sophus::SE3f Tcw =
      Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
  Eigen::Vector3f Ow = Tcw.inverse().translation();

  // Set of MapPoints already found in the KeyFrame
  set<MapPoint *> spAlreadyFound(vpMatched.begin(), vpMatched.end());
  spAlreadyFound.erase(static_cast<MapPoint *>(NULL));

  int nmatches = 0;

  // For each Candidate MapPoint Project and Match
  for (int iMP = 0, iendMP = vpPoints.size(); iMP < iendMP; iMP++) {
    MapPoint *pMP = vpPoints[iMP];
    KeyFrame *pKFi = vpPointsKFs[iMP];

    // Discard Bad MapPoints and already found
    if (pMP->isBad() || spAlreadyFound.count(pMP)) continue;

    // Get 3D Coords.
    Eigen::Vector3f p3Dw = pMP->GetWorldPos();

    // Transform into Camera Coords.
    Eigen::Vector3f p3Dc = Tcw * p3Dw;

    // Depth must be positive
    if (p3Dc(2) < 0.0) continue;

    // Project into Image
    const float invz = 1 / p3Dc(2);
    const float x = p3Dc(0) * invz;
    const float y = p3Dc(1) * invz;

    const float u = fx * x + cx;
    const float v = fy * y + cy;

    // Point must be inside the image
    if (!pKF->IsInImage(u, v)) continue;

    // Depth must be inside the scale invariance region of the point
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    Eigen::Vector3f PO = p3Dw - Ow;
    const float dist = PO.norm();

    if (dist < minDistance || dist > maxDistance) continue;

    // Viewing angle must be less than 60 deg
    Eigen::Vector3f Pn = pMP->GetNormal();

    if (PO.dot(Pn) < 0.5 * dist) continue;

    int nPredictedLevel = pMP->PredictScale(dist, pKF);

    // Search in a radius
    const float radius = th * pKF->mvScaleFactors[nPredictedLevel];

    const vector<size_t> vIndices = pKF->GetFeaturesInArea(u, v, radius);

    if (vIndices.empty()) continue;

    // Match to the most similar keypoint in the radius
    const cv::Mat dMP = pMP->GetDescriptor();

    int bestDist = 256;
    int bestIdx = -1;
    for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                        vend = vIndices.end();
         vit != vend; vit++) {
      const size_t idx = *vit;
      if (vpMatched[idx]) continue;

      const int &kpLevel = pKF->mvKeysUn[idx].octave;

      if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel) continue;

      const cv::Mat &dKF = pKF->mDescriptors.row(idx);

      const int dist = DescriptorDistance(dMP, dKF);

      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = idx;
      }
    }

    if (bestDist <= TH_LOW * ratioHamming) {
      vpMatched[bestIdx] = pMP;
      vpMatchedKF[bestIdx] = pKFi;
      nmatches++;
    }
  }

  return nmatches;
}

int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2,
                                        vector<cv::Point2f> &vbPrevMatched,
                                        vector<int> &vnMatches12,
                                        int windowSize) {
  int nmatches = 0;
  vnMatches12 = vector<int>(F1.mvKeysUn.size(), -1);

  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);
  const float factor = 1.0f / HISTO_LENGTH;

  vector<int> vMatchedDistance(F2.mvKeysUn.size(), INT_MAX);
  vector<int> vnMatches21(F2.mvKeysUn.size(), -1);

  for (size_t i1 = 0, iend1 = F1.mvKeysUn.size(); i1 < iend1; i1++) {
    cv::KeyPoint kp1 = F1.mvKeysUn[i1];
    int level1 = kp1.octave;
    if (level1 > 0) continue;

    vector<size_t> vIndices2 = F2.GetFeaturesInArea(
        vbPrevMatched[i1].x, vbPrevMatched[i1].y, windowSize, level1, level1);

    if (vIndices2.empty()) continue;

    cv::Mat d1 = F1.mDescriptors.row(i1);

    int bestDist = INT_MAX;
    int bestDist2 = INT_MAX;
    int bestIdx2 = -1;

    for (vector<size_t>::iterator vit = vIndices2.begin();
         vit != vIndices2.end(); vit++) {
      size_t i2 = *vit;

      cv::Mat d2 = F2.mDescriptors.row(i2);

      int dist = DescriptorDistance(d1, d2);

      if (vMatchedDistance[i2] <= dist) continue;

      if (dist < bestDist) {
        bestDist2 = bestDist;
        bestDist = dist;
        bestIdx2 = i2;
      } else if (dist < bestDist2) {
        bestDist2 = dist;
      }
    }

    if (bestDist <= TH_LOW) {
      if (bestDist < (float)bestDist2 * mfNNratio) {
        if (vnMatches21[bestIdx2] >= 0) {
          vnMatches12[vnMatches21[bestIdx2]] = -1;
          nmatches--;
        }
        vnMatches12[i1] = bestIdx2;
        vnMatches21[bestIdx2] = i1;
        vMatchedDistance[bestIdx2] = bestDist;
        nmatches++;

        if (mbCheckOrientation) {
          float rot = F1.mvKeysUn[i1].angle - F2.mvKeysUn[bestIdx2].angle;
          if (rot < 0.0) rot += 360.0f;
          int bin = round(rot * factor);
          if (bin == HISTO_LENGTH) bin = 0;
          assert(bin >= 0 && bin < HISTO_LENGTH);
          rotHist[bin].push_back(i1);
        }
      }
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3) continue;
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
        int idx1 = rotHist[i][j];
        if (vnMatches12[idx1] >= 0) {
          vnMatches12[idx1] = -1;
          nmatches--;
        }
      }
    }
  }

  // Update prev matched
  for (size_t i1 = 0, iend1 = vnMatches12.size(); i1 < iend1; i1++)
    if (vnMatches12[i1] >= 0)
      vbPrevMatched[i1] = F2.mvKeysUn[vnMatches12[i1]].pt;

  return nmatches;
}

/**
 * @breif 当基于投影的匹配失败后，采用本函数进行匹配得到更多的匹配结果
 * @param pKF    上一帧关键帧
 * @param F      当前帧
 * @param vpMapPointMatches 匹配的结果
 * @return 匹配数量
 */
int ORBmatcher::SearchWithGMS(KeyFrame *pKF, Frame &F,
                              vector<MapPoint *> &vpMapPointMatches) {
  int nmatches = 0;
  int inliers = 0;
  vpMapPointMatches = vector<MapPoint *>(F.N, static_cast<MapPoint *>(NULL));

  vector<KeyPoint> kp1 = pKF->mvKeys;
  vector<KeyPoint> kp2 = F.mvKeys;
  Mat d1 = pKF->mDescriptors;
  Mat d2 = F.mDescriptors;
  vector<DMatch> matches_all, matches_gms;
  BFMatcher matcher(NORM_HAMMING);
  matcher.match(d1, d2, matches_all);
  // cout<<"all matcher :"<<matches_all.size()<<endl;

  std::vector<bool> vbInliers;
  const Size frameSize = F.frameSize;
  gms_matcher gms(kp1, frameSize, kp2, frameSize, matches_all);
  nmatches = gms.GetInlierMask(vbInliers, false, false);
  // cout<< "num_inliers after GMS :"<<num_inliers<<endl;

  const vector<MapPoint *> vpMapPointsKF = pKF->GetMapPointMatches();

  for (size_t i = 0; i < vbInliers.size(); ++i) {
    if (vbInliers[i] == true) {
      // matches_gms.push_back(matches_all[i]);
      MapPoint *pMP = vpMapPointsKF[matches_all[i].queryIdx];
      if (!pMP) continue;
      if (pMP->isBad()) continue;
      vpMapPointMatches[matches_all[i].trainIdx] = pMP;
      inliers++;
    }
  }
  return nmatches;
}
/**
 * @brief 初始化时调用，计算前后两帧的匹配关系
 * @param Frame_1
 * @param Frame_2
 * @param vnMatches12 匹配的结果
 * @return 匹配数量
 */
int ORBmatcher::SearchForInitializationWithGMS(
    Frame &Frame_1, Frame &Frame_2, vector<int> &vnMatches12)  // tracking
{
  int nmatches = 0;
  int nbestMatches = 0;
  int keySize = Frame_1.mvKeysUn.size();
  // 1 --- > 2
  vnMatches12 = vector<int>(Frame_1.mvKeysUn.size(), -1);
  // vnMatches12.resize(Frame_1.mvKeysUn.size())
  // fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
  //  2 --- > 1
  vector<int> vnMatches21 = vector<int>(Frame_2.mvKeysUn.size(), -1);
  // resize()
  vector<cv::KeyPoint> kp1 = Frame_1.mvKeys;
  vector<cv::KeyPoint> kp2 = Frame_2.mvKeys;
  cv::Mat d1 = Frame_1.mDescriptors;
  cv::Mat d2 = Frame_2.mDescriptors;
  // 1. 通过 bf 进行匹配
  vector<cv::DMatch> matches_all;  //,matches_gms;
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  matcher.match(d1, d2, matches_all);
  // cout<<"all matcher :"<<matches_all.size()<<endl;
  // cout<<"kp1 :"<<kp1.size()<<"  kp2 :"<<kp2.size()<<endl;
  //  2. gms 搜索内点
  std::vector<bool> vbInliers;
  const cv::Size frameSize = Frame_1.frameSize;
  gms_matcher gms(kp1, frameSize, kp2, frameSize, matches_all);
  nmatches = gms.GetInlierMask(vbInliers, false, false);
  // cout<< "num_inliers after GMS :"<<nmatches<<endl;
  //  遍历所有内点
  for (size_t i = 0; i < vbInliers.size(); ++i) {
    //
    if (vbInliers[i] == true) {
      int queryIdx = matches_all[i].queryIdx;
      int trainIdx = matches_all[i].trainIdx;

      int level1 = kp1[queryIdx].octave;
      int level2 = kp2[trainIdx].octave;
      if (level1 > 2)  //(level1>0||level2>0)
        continue;

      if ((vnMatches12[queryIdx] == -1) && (vnMatches21[trainIdx] == -1)) {
        // vnMatches12[matches_all[i].queryIdx] = matches_all[i].trainIdx;
        vnMatches12[queryIdx] = trainIdx;
        vnMatches21[trainIdx] = i;
        nbestMatches++;
      }
      // else
      //     cout<<"repeat ....."<<endl;
      // pt = keyPoints_1[matches[i].queryIdx].pt;
      // pt = keyPoints_2[matches[i].trainIdx].pt;
    }
  }
  // cout<< "return  nmatches:"<<nbestMatches<<endl;
  return nbestMatches;  // nbestMatches
}

/**
 * @brief 计算两个关键帧中未构成地图点的特征的匹配关系
 * @param pKF1  关键帧1，需要添加地图点的关键帧
 * @param pKF2  关键帧2，参考帧
 * @param F12   两帧之间的F矩阵
 * @param vMatchedPairs  匹配结果
 * @param bOnlyStereo
 * @return
 */
int ORBmatcher::SearchForTriangulationWithGMS(
    KeyFrame *pKF1, KeyFrame *pKF2, cv::Mat F12,
    vector<pair<size_t, size_t>> &vMatchedPairs, const bool bOnlyStereo) {
  int nmatches = 0;
  std::vector<int> id1, id2;
  vector<cv::KeyPoint> kps1;
  for (int i = 0; i < pKF1->mvKeys.size(); i++) {
    auto mp = pKF1->GetMapPoint(i);
    // if (mp == nullptr){
    kps1.emplace_back(pKF1->mvKeys.at(i));
    id1.emplace_back(i);
    //}
  }
  vector<cv::KeyPoint> kps2;
  for (int i = 0; i < pKF2->mvKeys.size(); i++) {
    auto mp = pKF2->GetMapPoint(i);
    // if (mp == nullptr){
    kps2.emplace_back(pKF2->mvKeys.at(i));
    id2.emplace_back(i);
    //}
  }
  cv::Mat d1 = pKF1->mDescriptors;
  cv::Mat d2 = pKF2->mDescriptors;
  // 检查输入矩阵的类型和大小
  if (d1.type() != d2.type() || d1.cols != d2.cols) {
    std::cerr << "Error: Descriptor types or sizes do not match!" << std::endl;
    return -1;
  }

  // 确保矩阵类型为浮点数或整数
  if (d1.type() != CV_32F && d1.type() != CV_8U) {
    std::cerr << "Error: Descriptor type must be CV_32F or CV_8U!" << std::endl;
    return -1;
  }
  // 1. 通过 bf 进行匹配
  vector<cv::DMatch> matches_all;  //,matches_gms;
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  matcher.match(d1, d2, matches_all);
  // 2. gms 搜索内点
  std::vector<bool> vbInliers;
  const cv::Size frameSize = pKF1->frameSize;
  gms_matcher gms(kps1, frameSize, kps2, frameSize, matches_all);
  nmatches = gms.GetInlierMask(vbInliers, false, false);
  // 3 遍历所有内点
  for (size_t i = 0; i < vbInliers.size(); ++i) {
    if (vbInliers[i]) {
      int queryIdx = matches_all[i].queryIdx;
      int trainIdx = matches_all[i].trainIdx;
      auto mp1 = pKF1->GetMapPoint(queryIdx);
      auto mp2 = pKF2->GetMapPoint(trainIdx);
      auto kp1 = kps1.at(queryIdx);
      auto kp2 = kps2.at(trainIdx);
      // 去掉已有的地图点
      if (mp1 != nullptr || mp2 != nullptr) continue;
      // 保证足够的视差
      if (!CheckDistEpipolarLine(kp1, kp2, F12, pKF2)) continue;
      // 添加匹配关系
      vMatchedPairs.emplace_back(queryIdx, trainIdx);
    }
  }
  return 0;
}
bool ORBmatcher::CheckDistEpipolarLine(const cv::KeyPoint &kp1,
                                       const cv::KeyPoint &kp2,
                                       const cv::Mat &F12,
                                       const KeyFrame *pKF2) {
  // Epipolar line in second image l = x1'F12 = [a b c]
  const float a = kp1.pt.x * F12.at<float>(0, 0) +
                  kp1.pt.y * F12.at<float>(1, 0) + F12.at<float>(2, 0);
  const float b = kp1.pt.x * F12.at<float>(0, 1) +
                  kp1.pt.y * F12.at<float>(1, 1) + F12.at<float>(2, 1);
  const float c = kp1.pt.x * F12.at<float>(0, 2) +
                  kp1.pt.y * F12.at<float>(1, 2) + F12.at<float>(2, 2);

  const float num = a * kp2.pt.x + b * kp2.pt.y + c;

  const float den = a * a + b * b;

  if (den == 0) return false;

  const float dsqr = num * num / den;
  int oct = min(7, kp2.octave);
  return dsqr < 3.84 * 3;  // pKF2->mvLevelSigma2[oct];
}
int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2,
                            vector<MapPoint *> &vpMatches12) {
  const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeysUn;
  const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
  const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
  const cv::Mat &Descriptors1 = pKF1->mDescriptors;

  const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn;
  const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
  const vector<MapPoint *> vpMapPoints2 = pKF2->GetMapPointMatches();
  const cv::Mat &Descriptors2 = pKF2->mDescriptors;

  vpMatches12 =
      vector<MapPoint *>(vpMapPoints1.size(), static_cast<MapPoint *>(NULL));
  vector<bool> vbMatched2(vpMapPoints2.size(), false);

  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);

  const float factor = 1.0f / HISTO_LENGTH;

  int nmatches = 0;

  DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
  DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
  DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
  DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

  while (f1it != f1end && f2it != f2end) {
    if (f1it->first == f2it->first) {
      for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
        const size_t idx1 = f1it->second[i1];
        if (pKF1->NLeft != -1 && idx1 >= pKF1->mvKeysUn.size()) {
          continue;
        }

        MapPoint *pMP1 = vpMapPoints1[idx1];
        if (!pMP1) continue;
        if (pMP1->isBad()) continue;

        const cv::Mat &d1 = Descriptors1.row(idx1);

        int bestDist1 = 256;
        int bestIdx2 = -1;
        int bestDist2 = 256;

        for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
          const size_t idx2 = f2it->second[i2];

          if (pKF2->NLeft != -1 && idx2 >= pKF2->mvKeysUn.size()) {
            continue;
          }

          MapPoint *pMP2 = vpMapPoints2[idx2];

          if (vbMatched2[idx2] || !pMP2) continue;

          if (pMP2->isBad()) continue;

          const cv::Mat &d2 = Descriptors2.row(idx2);

          int dist = DescriptorDistance(d1, d2);

          if (dist < bestDist1) {
            bestDist2 = bestDist1;
            bestDist1 = dist;
            bestIdx2 = idx2;
          } else if (dist < bestDist2) {
            bestDist2 = dist;
          }
        }

        if (bestDist1 < TH_LOW) {
          if (static_cast<float>(bestDist1) <
              mfNNratio * static_cast<float>(bestDist2)) {
            vpMatches12[idx1] = vpMapPoints2[bestIdx2];
            vbMatched2[bestIdx2] = true;

            if (mbCheckOrientation) {
              float rot = vKeysUn1[idx1].angle - vKeysUn2[bestIdx2].angle;
              if (rot < 0.0) rot += 360.0f;
              int bin = round(rot * factor);
              if (bin == HISTO_LENGTH) bin = 0;
              assert(bin >= 0 && bin < HISTO_LENGTH);
              rotHist[bin].push_back(idx1);
            }
            nmatches++;
          }
        }
      }

      f1it++;
      f2it++;
    } else if (f1it->first < f2it->first) {
      f1it = vFeatVec1.lower_bound(f2it->first);
    } else {
      f2it = vFeatVec2.lower_bound(f1it->first);
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3) continue;
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
        vpMatches12[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
        nmatches--;
      }
    }
  }

  return nmatches;
}

int ORBmatcher::SearchByBoW(Frame &F1, Frame &F2, vector<int> &vnMatches12) {
  const vector<cv::KeyPoint> &vKeysUn1 = F1.mvKeysUn;
  const DBoW2::FeatureVector &vFeatVec1 = F1.mFeatVec;
  const cv::Mat &Descriptors1 = F1.mDescriptors;

  const vector<cv::KeyPoint> &vKeysUn2 = F2.mvKeysUn;
  const DBoW2::FeatureVector &vFeatVec2 = F2.mFeatVec;
  const cv::Mat &Descriptors2 = F2.mDescriptors;

  vnMatches12 = vector<int>(F1.mvKeysUn.size(), -1);

  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);

  const float factor = 1.0f / HISTO_LENGTH;

  int nmatches = 0;

  DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
  DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
  DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
  DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

  while (f1it != f1end && f2it != f2end) {
    if (f1it->first == f2it->first) {
      for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
        const size_t idx1 = f1it->second[i1];
        if (F1.Nleft != -1 && idx1 >= F1.mvKeysUn.size()) {
          continue;
        }

        const cv::Mat &d1 = Descriptors1.row(idx1);

        int bestDist1 = 256;
        int bestIdx2 = -1;
        int bestDist2 = 256;

        for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
          const size_t idx2 = f2it->second[i2];

          if (F2.Nleft != -1 && idx2 >= F2.mvKeysUn.size()) {
            continue;
          }

          const cv::Mat &d2 = Descriptors2.row(idx2);

          int dist = DescriptorDistance(d1, d2);

          if (dist < bestDist1) {
            bestDist2 = bestDist1;
            bestDist1 = dist;
            bestIdx2 = idx2;
          } else if (dist < bestDist2) {
            bestDist2 = dist;
          }
        }

        if (bestDist1 < TH_LOW) {
          if (static_cast<float>(bestDist1) <
              mfNNratio * static_cast<float>(bestDist2)) {
            vnMatches12[idx1] = bestIdx2;

            if (mbCheckOrientation) {
              float rot = vKeysUn1[idx1].angle - vKeysUn2[bestIdx2].angle;
              if (rot < 0.0) rot += 360.0f;
              int bin = round(rot * factor);
              if (bin == HISTO_LENGTH) bin = 0;
              assert(bin >= 0 && bin < HISTO_LENGTH);
              rotHist[bin].push_back(idx1);
            }
            nmatches++;
          }
        }
      }

      f1it++;
      f2it++;
    } else if (f1it->first < f2it->first) {
      f1it = vFeatVec1.lower_bound(f2it->first);
    } else {
      f2it = vFeatVec2.lower_bound(f1it->first);
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3) continue;
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
        vnMatches12[rotHist[i][j]] = -1;
        nmatches--;
      }
    }
  }

  return nmatches;
}

int ORBmatcher::SearchForTriangulation(
    KeyFrame *pKF1, KeyFrame *pKF2, vector<pair<size_t, size_t>> &vMatchedPairs,
    const bool bOnlyStereo, const bool bCoarse) {
  const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
  const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;

  // Compute epipole in second image
  Sophus::SE3f T1w = pKF1->GetPose();
  Sophus::SE3f T2w = pKF2->GetPose();
  Sophus::SE3f Tw2 = pKF2->GetPoseInverse();  // for convenience
  Eigen::Vector3f Cw = pKF1->GetCameraCenter();
  Eigen::Vector3f C2 = T2w * Cw;

  Eigen::Vector2f ep = pKF2->mpCamera->project(C2);
  Sophus::SE3f T12;
  Sophus::SE3f Tll, Tlr, Trl, Trr;
  Eigen::Matrix3f R12;  // for fastest computation
  Eigen::Vector3f t12;  // for fastest computation

  GeometricCamera *pCamera1 = pKF1->mpCamera, *pCamera2 = pKF2->mpCamera;

  if (!pKF1->mpCamera2 && !pKF2->mpCamera2) {
    T12 = T1w * Tw2;
    R12 = T12.rotationMatrix();
    t12 = T12.translation();
  } else {
    Sophus::SE3f Tr1w = pKF1->GetRightPose();
    Sophus::SE3f Twr2 = pKF2->GetRightPoseInverse();
    Tll = T1w * Tw2;
    Tlr = T1w * Twr2;
    Trl = Tr1w * Tw2;
    Trr = Tr1w * Twr2;
  }

  Eigen::Matrix3f Rll = Tll.rotationMatrix(), Rlr = Tlr.rotationMatrix(),
                  Rrl = Trl.rotationMatrix(), Rrr = Trr.rotationMatrix();
  Eigen::Vector3f tll = Tll.translation(), tlr = Tlr.translation(),
                  trl = Trl.translation(), trr = Trr.translation();

  // Find matches between not tracked keypoints
  // Matching speed-up by ORB Vocabulary
  // Compare only ORB that share the same node
  int nmatches = 0;
  vector<bool> vbMatched2(pKF2->N, false);
  vector<int> vMatches12(pKF1->N, -1);

  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);

  const float factor = 1.0f / HISTO_LENGTH;

  DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
  DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
  DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
  DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

  while (f1it != f1end && f2it != f2end) {
    if (f1it->first == f2it->first) {
      for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
        const size_t idx1 = f1it->second[i1];

        MapPoint *pMP1 = pKF1->GetMapPoint(idx1);

        // If there is already a MapPoint skip
        if (pMP1) {
          continue;
        }

        const bool bStereo1 = (!pKF1->mpCamera2 && pKF1->mvuRight[idx1] >= 0);

        if (bOnlyStereo)
          if (!bStereo1) continue;

        const cv::KeyPoint &kp1 = (pKF1->NLeft == -1) ? pKF1->mvKeysUn[idx1]
                                  : (idx1 < pKF1->NLeft)
                                      ? pKF1->mvKeys[idx1]
                                      : pKF1->mvKeysRight[idx1 - pKF1->NLeft];

        const bool bRight1 =
            (pKF1->NLeft == -1 || idx1 < pKF1->NLeft) ? false : true;

        const cv::Mat &d1 = pKF1->mDescriptors.row(idx1);

        int bestDist = TH_LOW;
        int bestIdx2 = -1;

        for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
          size_t idx2 = f2it->second[i2];

          MapPoint *pMP2 = pKF2->GetMapPoint(idx2);

          // If we have already matched or there is a MapPoint skip
          if (vbMatched2[idx2] || pMP2) continue;

          const bool bStereo2 = (!pKF2->mpCamera2 && pKF2->mvuRight[idx2] >= 0);

          if (bOnlyStereo)
            if (!bStereo2) continue;

          const cv::Mat &d2 = pKF2->mDescriptors.row(idx2);

          const int dist = DescriptorDistance(d1, d2);

          if (dist > TH_LOW || dist > bestDist) continue;

          const cv::KeyPoint &kp2 = (pKF2->NLeft == -1) ? pKF2->mvKeysUn[idx2]
                                    : (idx2 < pKF2->NLeft)
                                        ? pKF2->mvKeys[idx2]
                                        : pKF2->mvKeysRight[idx2 - pKF2->NLeft];
          const bool bRight2 =
              (pKF2->NLeft == -1 || idx2 < pKF2->NLeft) ? false : true;

          if (!bStereo1 && !bStereo2 && !pKF1->mpCamera2) {
            const float distex = ep(0) - kp2.pt.x;
            const float distey = ep(1) - kp2.pt.y;
            if (distex * distex + distey * distey <
                100 * pKF2->mvScaleFactors[kp2.octave]) {
              continue;
            }
          }

          if (pKF1->mpCamera2 && pKF2->mpCamera2) {
            if (bRight1 && bRight2) {
              R12 = Rrr;
              t12 = trr;
              T12 = Trr;

              pCamera1 = pKF1->mpCamera2;
              pCamera2 = pKF2->mpCamera2;
            } else if (bRight1 && !bRight2) {
              R12 = Rrl;
              t12 = trl;
              T12 = Trl;

              pCamera1 = pKF1->mpCamera2;
              pCamera2 = pKF2->mpCamera;
            } else if (!bRight1 && bRight2) {
              R12 = Rlr;
              t12 = tlr;
              T12 = Tlr;

              pCamera1 = pKF1->mpCamera;
              pCamera2 = pKF2->mpCamera2;
            } else {
              R12 = Rll;
              t12 = tll;
              T12 = Tll;

              pCamera1 = pKF1->mpCamera;
              pCamera2 = pKF2->mpCamera;
            }
          }

          if (bCoarse ||
              pCamera1->epipolarConstrain(
                  pCamera2, kp1, kp2, R12, t12, pKF1->mvLevelSigma2[kp1.octave],
                  pKF2->mvLevelSigma2[kp2.octave]))  // MODIFICATION_2
          {
            bestIdx2 = idx2;
            bestDist = dist;
          }
        }

        if (bestIdx2 >= 0) {
          const cv::KeyPoint &kp2 =
              (pKF2->NLeft == -1) ? pKF2->mvKeysUn[bestIdx2]
              : (bestIdx2 < pKF2->NLeft)
                  ? pKF2->mvKeys[bestIdx2]
                  : pKF2->mvKeysRight[bestIdx2 - pKF2->NLeft];
          vMatches12[idx1] = bestIdx2;
          vbMatched2[bestIdx2] = true;
          nmatches++;

          if (mbCheckOrientation) {
            float rot = kp1.angle - kp2.angle;
            if (rot < 0.0) rot += 360.0f;
            int bin = round(rot * factor);
            if (bin == HISTO_LENGTH) bin = 0;
            assert(bin >= 0 && bin < HISTO_LENGTH);
            rotHist[bin].push_back(idx1);
          }
        }
      }

      f1it++;
      f2it++;
    } else if (f1it->first < f2it->first) {
      f1it = vFeatVec1.lower_bound(f2it->first);
    } else {
      f2it = vFeatVec2.lower_bound(f1it->first);
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3) continue;
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
        vMatches12[rotHist[i][j]] = -1;
        nmatches--;
      }
    }
  }

  vMatchedPairs.clear();
  vMatchedPairs.reserve(nmatches);

  for (size_t i = 0, iend = vMatches12.size(); i < iend; i++) {
    if (vMatches12[i] < 0) continue;
    vMatchedPairs.push_back(make_pair(i, vMatches12[i]));
  }

  return nmatches;
}

int ORBmatcher::Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints,
                     const float th, const bool bRight) {
  GeometricCamera *pCamera;
  Sophus::SE3f Tcw;
  Eigen::Vector3f Ow;

  if (bRight) {
    Tcw = pKF->GetRightPose();
    Ow = pKF->GetRightCameraCenter();
    pCamera = pKF->mpCamera2;
  } else {
    Tcw = pKF->GetPose();
    Ow = pKF->GetCameraCenter();
    pCamera = pKF->mpCamera;
  }

  const float &fx = pKF->fx;
  const float &fy = pKF->fy;
  const float &cx = pKF->cx;
  const float &cy = pKF->cy;
  const float &bf = pKF->mbf;

  int nFused = 0;

  const int nMPs = vpMapPoints.size();

  // For debbuging
  int count_notMP = 0, count_bad = 0, count_isinKF = 0, count_negdepth = 0,
      count_notinim = 0, count_dist = 0, count_normal = 0, count_notidx = 0,
      count_thcheck = 0;
  for (int i = 0; i < nMPs; i++) {
    MapPoint *pMP = vpMapPoints[i];

    if (!pMP) {
      count_notMP++;
      continue;
    }

    if (pMP->isBad()) {
      count_bad++;
      continue;
    } else if (pMP->IsInKeyFrame(pKF)) {
      count_isinKF++;
      continue;
    }

    Eigen::Vector3f p3Dw = pMP->GetWorldPos();
    Eigen::Vector3f p3Dc = Tcw * p3Dw;

    // Depth must be positive
    if (p3Dc(2) < 0.0f) {
      count_negdepth++;
      continue;
    }

    const float invz = 1 / p3Dc(2);

    const Eigen::Vector2f uv = pCamera->project(p3Dc);

    // Point must be inside the image
    if (!pKF->IsInImage(uv(0), uv(1))) {
      count_notinim++;
      continue;
    }

    const float ur = uv(0) - bf * invz;

    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    Eigen::Vector3f PO = p3Dw - Ow;
    const float dist3D = PO.norm();

    // Depth must be inside the scale pyramid of the image
    if (dist3D < minDistance || dist3D > maxDistance) {
      count_dist++;
      continue;
    }

    // Viewing angle must be less than 60 deg
    Eigen::Vector3f Pn = pMP->GetNormal();

    if (PO.dot(Pn) < 0.5 * dist3D) {
      count_normal++;
      continue;
    }

    int nPredictedLevel = pMP->PredictScale(dist3D, pKF);

    // Search in a radius
    const float radius = th * pKF->mvScaleFactors[nPredictedLevel];

    const vector<size_t> vIndices =
        pKF->GetFeaturesInArea(uv(0), uv(1), radius, bRight);

    if (vIndices.empty()) {
      count_notidx++;
      continue;
    }

    // Match to the most similar keypoint in the radius

    const cv::Mat dMP = pMP->GetDescriptor();

    int bestDist = 256;
    int bestIdx = -1;
    for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                        vend = vIndices.end();
         vit != vend; vit++) {
      size_t idx = *vit;
      const cv::KeyPoint &kp = (pKF->NLeft == -1) ? pKF->mvKeysUn[idx]
                               : (!bRight)        ? pKF->mvKeys[idx]
                                                  : pKF->mvKeysRight[idx];

      const int &kpLevel = kp.octave;

      if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel) continue;

      if (pKF->mvuRight[idx] >= 0) {
        // Check reprojection error in stereo
        const float &kpx = kp.pt.x;
        const float &kpy = kp.pt.y;
        const float &kpr = pKF->mvuRight[idx];
        const float ex = uv(0) - kpx;
        const float ey = uv(1) - kpy;
        const float er = ur - kpr;
        const float e2 = ex * ex + ey * ey + er * er;

        if (e2 * pKF->mvInvLevelSigma2[kpLevel] > 7.8) continue;
      } else {
        const float &kpx = kp.pt.x;
        const float &kpy = kp.pt.y;
        const float ex = uv(0) - kpx;
        const float ey = uv(1) - kpy;
        const float e2 = ex * ex + ey * ey;

        if (e2 * pKF->mvInvLevelSigma2[kpLevel] > 5.99) continue;
      }

      if (bRight) idx += pKF->NLeft;

      const cv::Mat &dKF = pKF->mDescriptors.row(idx);

      const int dist = DescriptorDistance(dMP, dKF);

      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = idx;
      }
    }

    // If there is already a MapPoint replace otherwise add new measurement
    if (bestDist <= TH_LOW) {
      MapPoint *pMPinKF = pKF->GetMapPoint(bestIdx);
      if (pMPinKF) {
        if (!pMPinKF->isBad()) {
          if (pMPinKF->Observations() > pMP->Observations())
            pMP->Replace(pMPinKF);
          else
            pMPinKF->Replace(pMP);
        }
      } else {
        pMP->AddObservation(pKF, bestIdx);
        pKF->AddMapPoint(pMP, bestIdx);
      }
      nFused++;
    } else
      count_thcheck++;
  }

  return nFused;
}

int ORBmatcher::Fuse(KeyFrame *pKF, Sophus::Sim3f &Scw,
                     const vector<MapPoint *> &vpPoints, float th,
                     vector<MapPoint *> &vpReplacePoint) {
  // Get Calibration Parameters for later projection
  const float &fx = pKF->fx;
  const float &fy = pKF->fy;
  const float &cx = pKF->cx;
  const float &cy = pKF->cy;

  // Decompose Scw
  Sophus::SE3f Tcw =
      Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
  Eigen::Vector3f Ow = Tcw.inverse().translation();

  // Set of MapPoints already found in the KeyFrame
  const set<MapPoint *> spAlreadyFound = pKF->GetMapPoints();

  int nFused = 0;

  const int nPoints = vpPoints.size();

  // For each candidate MapPoint project and match
  for (int iMP = 0; iMP < nPoints; iMP++) {
    MapPoint *pMP = vpPoints[iMP];

    // Discard Bad MapPoints and already found
    if (pMP->isBad() || spAlreadyFound.count(pMP)) continue;

    // Get 3D Coords.
    Eigen::Vector3f p3Dw = pMP->GetWorldPos();

    // Transform into Camera Coords.
    Eigen::Vector3f p3Dc = Tcw * p3Dw;

    // Depth must be positive
    if (p3Dc(2) < 0.0f) continue;

    // Project into Image
    const Eigen::Vector2f uv = pKF->mpCamera->project(p3Dc);

    // Point must be inside the image
    if (!pKF->IsInImage(uv(0), uv(1))) continue;

    // Depth must be inside the scale pyramid of the image
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    Eigen::Vector3f PO = p3Dw - Ow;
    const float dist3D = PO.norm();

    if (dist3D < minDistance || dist3D > maxDistance) continue;

    // Viewing angle must be less than 60 deg
    Eigen::Vector3f Pn = pMP->GetNormal();

    if (PO.dot(Pn) < 0.5 * dist3D) continue;

    // Compute predicted scale level
    const int nPredictedLevel = pMP->PredictScale(dist3D, pKF);

    // Search in a radius
    const float radius = th * pKF->mvScaleFactors[nPredictedLevel];

    const vector<size_t> vIndices =
        pKF->GetFeaturesInArea(uv(0), uv(1), radius);

    if (vIndices.empty()) continue;

    // Match to the most similar keypoint in the radius

    const cv::Mat dMP = pMP->GetDescriptor();

    int bestDist = INT_MAX;
    int bestIdx = -1;
    for (vector<size_t>::const_iterator vit = vIndices.begin();
         vit != vIndices.end(); vit++) {
      const size_t idx = *vit;
      const int &kpLevel = pKF->mvKeysUn[idx].octave;

      if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel) continue;

      const cv::Mat &dKF = pKF->mDescriptors.row(idx);

      int dist = DescriptorDistance(dMP, dKF);

      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = idx;
      }
    }

    // If there is already a MapPoint replace otherwise add new measurement
    if (bestDist <= TH_LOW) {
      MapPoint *pMPinKF = pKF->GetMapPoint(bestIdx);
      if (pMPinKF) {
        if (!pMPinKF->isBad()) vpReplacePoint[iMP] = pMPinKF;
      } else {
        pMP->AddObservation(pKF, bestIdx);
        pKF->AddMapPoint(pMP, bestIdx);
      }
      nFused++;
    }
  }

  return nFused;
}

int ORBmatcher::SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2,
                             std::vector<MapPoint *> &vpMatches12,
                             const Sophus::Sim3f &S12, const float th) {
  const float &fx = pKF1->fx;
  const float &fy = pKF1->fy;
  const float &cx = pKF1->cx;
  const float &cy = pKF1->cy;

  // Camera 1 & 2 from world
  Sophus::SE3f T1w = pKF1->GetPose();
  Sophus::SE3f T2w = pKF2->GetPose();

  // Transformation between cameras
  Sophus::Sim3f S21 = S12.inverse();

  const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
  const int N1 = vpMapPoints1.size();

  const vector<MapPoint *> vpMapPoints2 = pKF2->GetMapPointMatches();
  const int N2 = vpMapPoints2.size();

  vector<bool> vbAlreadyMatched1(N1, false);
  vector<bool> vbAlreadyMatched2(N2, false);

  for (int i = 0; i < N1; i++) {
    MapPoint *pMP = vpMatches12[i];
    if (pMP) {
      vbAlreadyMatched1[i] = true;
      int idx2 = get<0>(pMP->GetIndexInKeyFrame(pKF2));
      if (idx2 >= 0 && idx2 < N2) vbAlreadyMatched2[idx2] = true;
    }
  }

  vector<int> vnMatch1(N1, -1);
  vector<int> vnMatch2(N2, -1);

  // Transform from KF1 to KF2 and search
  for (int i1 = 0; i1 < N1; i1++) {
    MapPoint *pMP = vpMapPoints1[i1];

    if (!pMP || vbAlreadyMatched1[i1]) continue;

    if (pMP->isBad()) continue;

    Eigen::Vector3f p3Dw = pMP->GetWorldPos();
    Eigen::Vector3f p3Dc1 = T1w * p3Dw;
    Eigen::Vector3f p3Dc2 = S21 * p3Dc1;

    // Depth must be positive
    if (p3Dc2(2) < 0.0) continue;

    const float invz = 1.0 / p3Dc2(2);
    const float x = p3Dc2(0) * invz;
    const float y = p3Dc2(1) * invz;

    const float u = fx * x + cx;
    const float v = fy * y + cy;

    // Point must be inside the image
    if (!pKF2->IsInImage(u, v)) continue;

    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const float dist3D = p3Dc2.norm();

    // Depth must be inside the scale invariance region
    if (dist3D < minDistance || dist3D > maxDistance) continue;

    // Compute predicted octave
    const int nPredictedLevel = pMP->PredictScale(dist3D, pKF2);

    // Search in a radius
    const float radius = th * pKF2->mvScaleFactors[nPredictedLevel];

    const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u, v, radius);

    if (vIndices.empty()) continue;

    // Match to the most similar keypoint in the radius
    const cv::Mat dMP = pMP->GetDescriptor();

    int bestDist = INT_MAX;
    int bestIdx = -1;
    for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                        vend = vIndices.end();
         vit != vend; vit++) {
      const size_t idx = *vit;

      const cv::KeyPoint &kp = pKF2->mvKeysUn[idx];

      if (kp.octave < nPredictedLevel - 1 || kp.octave > nPredictedLevel)
        continue;

      const cv::Mat &dKF = pKF2->mDescriptors.row(idx);

      const int dist = DescriptorDistance(dMP, dKF);

      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = idx;
      }
    }

    if (bestDist <= TH_HIGH) {
      vnMatch1[i1] = bestIdx;
    }
  }

  // Transform from KF2 to KF2 and search
  for (int i2 = 0; i2 < N2; i2++) {
    MapPoint *pMP = vpMapPoints2[i2];

    if (!pMP || vbAlreadyMatched2[i2]) continue;

    if (pMP->isBad()) continue;

    Eigen::Vector3f p3Dw = pMP->GetWorldPos();
    Eigen::Vector3f p3Dc2 = T2w * p3Dw;
    Eigen::Vector3f p3Dc1 = S12 * p3Dc2;

    // Depth must be positive
    if (p3Dc1(2) < 0.0) continue;

    const float invz = 1.0 / p3Dc1(2);
    const float x = p3Dc1(0) * invz;
    const float y = p3Dc1(1) * invz;

    const float u = fx * x + cx;
    const float v = fy * y + cy;

    // Point must be inside the image
    if (!pKF1->IsInImage(u, v)) continue;

    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const float dist3D = p3Dc1.norm();

    // Depth must be inside the scale pyramid of the image
    if (dist3D < minDistance || dist3D > maxDistance) continue;

    // Compute predicted octave
    const int nPredictedLevel = pMP->PredictScale(dist3D, pKF1);

    // Search in a radius of 2.5*sigma(ScaleLevel)
    const float radius = th * pKF1->mvScaleFactors[nPredictedLevel];

    const vector<size_t> vIndices = pKF1->GetFeaturesInArea(u, v, radius);

    if (vIndices.empty()) continue;

    // Match to the most similar keypoint in the radius
    const cv::Mat dMP = pMP->GetDescriptor();

    int bestDist = INT_MAX;
    int bestIdx = -1;
    for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                        vend = vIndices.end();
         vit != vend; vit++) {
      const size_t idx = *vit;

      const cv::KeyPoint &kp = pKF1->mvKeysUn[idx];

      if (kp.octave < nPredictedLevel - 1 || kp.octave > nPredictedLevel)
        continue;

      const cv::Mat &dKF = pKF1->mDescriptors.row(idx);

      const int dist = DescriptorDistance(dMP, dKF);

      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = idx;
      }
    }

    if (bestDist <= TH_HIGH) {
      vnMatch2[i2] = bestIdx;
    }
  }

  // Check agreement
  int nFound = 0;

  for (int i1 = 0; i1 < N1; i1++) {
    int idx2 = vnMatch1[i1];

    if (idx2 >= 0) {
      int idx1 = vnMatch2[idx2];
      if (idx1 == i1) {
        vpMatches12[i1] = vpMapPoints2[idx2];
        nFound++;
      }
    }
  }

  return nFound;
}
int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame,
                                   const float th, const bool bMono) {
  int nmatches = 0;

  // Rotation Histogram (to check rotation consistency)
  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);
  const float factor = 1.0f / HISTO_LENGTH;

  const Sophus::SE3f Tcw = CurrentFrame.GetPose();
  const Eigen::Vector3f twc = Tcw.inverse().translation();

  const Sophus::SE3f Tlw = LastFrame.GetPose();
  const Eigen::Vector3f tlc = Tlw * twc;

  const bool bForward = tlc(2) > CurrentFrame.mb && !bMono;
  const bool bBackward = -tlc(2) > CurrentFrame.mb && !bMono;

  for (int i = 0; i < LastFrame.N; i++) {
    MapPoint *pMP = LastFrame.mvpMapPoints[i];  // mvpMapPoint是keypoint
    if (pMP) {
      if (!LastFrame.mvbOutlier[i]) {
        // Project
        Eigen::Vector3f x3Dw = pMP->GetWorldPos();
        Eigen::Vector3f x3Dc = Tcw * x3Dw;

        const float xc = x3Dc(0);
        const float yc = x3Dc(1);
        const float invzc = 1.0 / x3Dc(2);

        if (invzc < 0) continue;

        Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dc);

        if (uv(0) < CurrentFrame.mnMinX || uv(0) > CurrentFrame.mnMaxX)
          continue;
        if (uv(1) < CurrentFrame.mnMinY || uv(1) > CurrentFrame.mnMaxY)
          continue;

        int nLastOctave =
            (LastFrame.Nleft == -1 || i < LastFrame.Nleft)
                ? LastFrame.mvKeys[i].octave
                : LastFrame.mvKeysRight[i - LastFrame.Nleft].octave;

        // Search in a window. Size depends on scale
        float radius = th * CurrentFrame.mvScaleFactors[nLastOctave];

        vector<size_t> vIndices2;

        if (bForward)
          vIndices2 =
              CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, nLastOctave);
        else if (bBackward)
          vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, 0,
                                                     nLastOctave);
        else
          vIndices2 = CurrentFrame.GetFeaturesInArea(
              uv(0), uv(1), radius, nLastOctave - 1, nLastOctave + 1);

        if (vIndices2.empty()) continue;

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestIdx2 = -1;

        for (vector<size_t>::const_iterator vit = vIndices2.begin(),
                                            vend = vIndices2.end();
             vit != vend; vit++) {
          const size_t i2 = *vit;

          if (CurrentFrame.mvpMapPoints[i2])
            if (CurrentFrame.mvpMapPoints[i2]->Observations() > 0) continue;

          if (CurrentFrame.Nleft == -1 && CurrentFrame.mvuRight[i2] > 0) {
            const float ur = uv(0) - CurrentFrame.mbf * invzc;
            const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
            if (er > radius) continue;
          }

          const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

          const int dist = DescriptorDistance(dMP, d);

          if (dist < bestDist) {
            bestDist = dist;
            bestIdx2 = i2;
          }
        }

        if (bestDist <= TH_HIGH) {
          CurrentFrame.mvpMapPoints[bestIdx2] = pMP;
          // CurrentFrame.mvbOutlier[bestIdx2] = false;
          nmatches++;

          if (mbCheckOrientation) {
            cv::KeyPoint kpLF =
                (LastFrame.Nleft == -1) ? LastFrame.mvKeysUn[i]
                : (i < LastFrame.Nleft)
                    ? LastFrame.mvKeys[i]
                    : LastFrame.mvKeysRight[i - LastFrame.Nleft];

            cv::KeyPoint kpCF =
                (CurrentFrame.Nleft == -1) ? CurrentFrame.mvKeysUn[bestIdx2]
                : (bestIdx2 < CurrentFrame.Nleft)
                    ? CurrentFrame.mvKeys[bestIdx2]
                    : CurrentFrame.mvKeysRight[bestIdx2 - CurrentFrame.Nleft];
            float rot = kpLF.angle - kpCF.angle;
            if (rot < 0.0) rot += 360.0f;
            int bin = round(rot * factor);
            if (bin == HISTO_LENGTH) bin = 0;
            assert(bin >= 0 && bin < HISTO_LENGTH);
            rotHist[bin].push_back(bestIdx2);
          }
        }
        if (CurrentFrame.Nleft != -1) {
          Eigen::Vector3f x3Dr = CurrentFrame.GetRelativePoseTrl() * x3Dc;
          Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dr);

          int nLastOctave =
              (LastFrame.Nleft == -1 || i < LastFrame.Nleft)
                  ? LastFrame.mvKeys[i].octave
                  : LastFrame.mvKeysRight[i - LastFrame.Nleft].octave;

          // Search in a window. Size depends on scale
          float radius = th * CurrentFrame.mvScaleFactors[nLastOctave];

          vector<size_t> vIndices2;

          if (bForward)
            vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius,
                                                       nLastOctave, -1, true);
          else if (bBackward)
            vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, 0,
                                                       nLastOctave, true);
          else
            vIndices2 = CurrentFrame.GetFeaturesInArea(
                uv(0), uv(1), radius, nLastOctave - 1, nLastOctave + 1, true);

          const cv::Mat dMP = pMP->GetDescriptor();

          int bestDist = 256;
          int bestIdx2 = -1;

          for (vector<size_t>::const_iterator vit = vIndices2.begin(),
                                              vend = vIndices2.end();
               vit != vend; vit++) {
            const size_t i2 = *vit;
            if (CurrentFrame.mvpMapPoints[i2 + CurrentFrame.Nleft])
              if (CurrentFrame.mvpMapPoints[i2 + CurrentFrame.Nleft]
                      ->Observations() > 0)
                continue;

            const cv::Mat &d =
                CurrentFrame.mDescriptors.row(i2 + CurrentFrame.Nleft);

            const int dist = DescriptorDistance(dMP, d);

            if (dist < bestDist) {
              bestDist = dist;
              bestIdx2 = i2;
            }
          }

          if (bestDist <= TH_HIGH) {
            CurrentFrame.mvpMapPoints[bestIdx2 + CurrentFrame.Nleft] = pMP;
            // CurrentFrame.mvbOutlier[bestIdx2] = false;
            nmatches++;
            if (mbCheckOrientation) {
              cv::KeyPoint kpLF =
                  (LastFrame.Nleft == -1) ? LastFrame.mvKeysUn[i]
                  : (i < LastFrame.Nleft)
                      ? LastFrame.mvKeys[i]
                      : LastFrame.mvKeysRight[i - LastFrame.Nleft];

              cv::KeyPoint kpCF = CurrentFrame.mvKeysRight[bestIdx2];

              float rot = kpLF.angle - kpCF.angle;
              if (rot < 0.0) rot += 360.0f;
              int bin = round(rot * factor);
              if (bin == HISTO_LENGTH) bin = 0;
              assert(bin >= 0 && bin < HISTO_LENGTH);
              rotHist[bin].push_back(bestIdx2 + CurrentFrame.Nleft);
            }
          }
        }
      }
    }
  }

  // Apply rotation consistency
  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i != ind1 && i != ind2 && i != ind3) {
        for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
          CurrentFrame.mvpMapPoints[rotHist[i][j]] =
              static_cast<MapPoint *>(NULL);
          nmatches--;
        }
      }
    }
  }

  return nmatches;
}

int ORBmatcher::SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF,
                                   const set<MapPoint *> &sAlreadyFound,
                                   const float th, const int ORBdist) {
  int nmatches = 0;

  const Sophus::SE3f Tcw = CurrentFrame.GetPose();
  Eigen::Vector3f Ow = Tcw.inverse().translation();

  // Rotation Histogram (to check rotation consistency)
  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);
  const float factor = 1.0f / HISTO_LENGTH;

  const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

  for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
    MapPoint *pMP = vpMPs[i];

    if (pMP) {
      if (!pMP->isBad() && !sAlreadyFound.count(pMP)) {
        // Project
        Eigen::Vector3f x3Dw = pMP->GetWorldPos();
        Eigen::Vector3f x3Dc = Tcw * x3Dw;

        const Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dc);

        if (uv(0) < CurrentFrame.mnMinX || uv(0) > CurrentFrame.mnMaxX)
          continue;
        if (uv(1) < CurrentFrame.mnMinY || uv(1) > CurrentFrame.mnMaxY)
          continue;

        // Compute predicted scale level
        Eigen::Vector3f PO = x3Dw - Ow;
        float dist3D = PO.norm();

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();

        // Depth must be inside the scale pyramid of the image
        if (dist3D < minDistance || dist3D > maxDistance) continue;

        int nPredictedLevel = pMP->PredictScale(dist3D, &CurrentFrame);

        // Search in a window
        const float radius = th * CurrentFrame.mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(
            uv(0), uv(1), radius, nPredictedLevel - 1, nPredictedLevel + 1);

        if (vIndices2.empty()) continue;

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestIdx2 = -1;

        for (vector<size_t>::const_iterator vit = vIndices2.begin();
             vit != vIndices2.end(); vit++) {
          const size_t i2 = *vit;
          if (CurrentFrame.mvpMapPoints[i2]) continue;

          const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

          const int dist = DescriptorDistance(dMP, d);

          if (dist < bestDist) {
            bestDist = dist;
            bestIdx2 = i2;
          }
        }

        if (bestDist <= ORBdist) {
          CurrentFrame.mvpMapPoints[bestIdx2] = pMP;
          nmatches++;

          if (mbCheckOrientation) {
            float rot =
                pKF->mvKeysUn[i].angle - CurrentFrame.mvKeysUn[bestIdx2].angle;
            if (rot < 0.0) rot += 360.0f;
            int bin = round(rot * factor);
            if (bin == HISTO_LENGTH) bin = 0;
            assert(bin >= 0 && bin < HISTO_LENGTH);
            rotHist[bin].push_back(bestIdx2);
          }
        }
      }
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i != ind1 && i != ind2 && i != ind3) {
        for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
          CurrentFrame.mvpMapPoints[rotHist[i][j]] = NULL;
          nmatches--;
        }
      }
    }
  }

  return nmatches;
}

/**
 * @brief 光流法
 * @param vprevpyr 上一帧的金字塔
 * @param vcurpyr  当前帧的金字塔
 * @param nwinsize 搜索窗口大小
 * @param nbpyrlvl 金字塔层数默认1
 * @param ferr     误差
 * @param fmax_fbklt_dist 最大距离
 * @param vkps       需要匹配的点
 * @param vpriorkps  匹配点预测位置
 * @param vkpstatus  匹配结果
 */
void ORBmatcher::fbKltTracking(const std::vector<cv::Mat> &vprevpyr,
                               const std::vector<cv::Mat> &vcurpyr,
                               int nwinsize, int nbpyrlvl, float ferr,
                               float fmax_fbklt_dist,
                               std::vector<cv::Point2f> &vkps,
                               std::vector<cv::Point2f> &vpriorkps,
                               std::vector<bool> &vkpstatus) const {
  // if(vprevpyr[0]) return;
  // 金字塔
  if (vprevpyr.size() != vcurpyr.size()) return;
  // 上一帧特征点的位置
  if (vkps.empty()) {
    return;
  }
  // 光流法的窗口
  cv::Size klt_win_size(nwinsize, nwinsize);
  //
  // if ((int)vprevpyr.size() < 2 * (nbpyrlvl + 1)) {
  //   nbpyrlvl = vprevpyr.size() / 2 - 1;
  // }
  // klt 匹配参数
  // Objects for OpenCV KLT
  size_t nbkps = vkps.size();
  vkpstatus.reserve(nbkps);

  std::vector<uchar> vstatus;
  std::vector<float> verr;
  std::vector<int> vkpsidx;
  vstatus.reserve(nbkps);
  verr.reserve(nbkps);
  vkpsidx.reserve(nbkps);
  int nmax_iter = 30;
  float fmax_px_precision = 0.01f;
  cv::TermCriteria klt_convg_crit_(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, nmax_iter,
      fmax_px_precision);
  //         Tracking Forward
  // cout<<"Tracking Forward"<<endl;
  cv::calcOpticalFlowPyrLK(
      vprevpyr, vcurpyr, vkps, vpriorkps, vstatus, verr, klt_win_size, nbpyrlvl,
      klt_convg_crit_,
      (cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS));
  std::vector<cv::Point2f> vnewkps;
  std::vector<cv::Point2f> vbackkps;
  vnewkps.reserve(nbkps);
  vbackkps.reserve(nbkps);

  size_t nbgood = 0;

  // Init outliers vector & update tracked kps
  for (size_t i = 0; i < nbkps; i++) {
    if (!vstatus.at(i)) {
      vkpstatus.push_back(false);
      continue;
    }

    if (verr.at(i) > ferr) {
      vkpstatus.push_back(false);
      continue;
    }

    if (!inBorder(vpriorkps.at(i), vcurpyr.at(0))) {
      vkpstatus.push_back(false);
      continue;
    }

    vnewkps.push_back(vpriorkps.at(i));
    vbackkps.push_back(vkps.at(i));
    vkpstatus.push_back(true);
    vkpsidx.push_back(i);
    nbgood++;
  }

  if (vnewkps.empty()) {
    return;
  }

  vstatus.clear();
  verr.clear();

  // std::cout << "\n \t >>> Forward kltTracking : #" << nbgood << " out
  // of #" << nbkps << " \n";

  // Tracking Backward
  // cout<<"Tracking Backward"<<endl;
  cv::calcOpticalFlowPyrLK(
      vcurpyr, vprevpyr, vnewkps, vbackkps, vstatus, verr, klt_win_size, 0,
      klt_convg_crit_,
      (cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS));
  nbgood = 0;
  for (int i = 0, iend = vnewkps.size(); i < iend; i++) {
    int idx = vkpsidx.at(i);

    if (!vstatus.at(i)) {
      vkpstatus.at(idx) = false;
      continue;
    }

    if (cv::norm(vkps.at(idx) - vbackkps.at(i)) > fmax_fbklt_dist) {
      vkpstatus.at(idx) = false;
      continue;
    }

    nbgood++;
  }

  // std::cout << "\n \t >>> Backward kltTracking : #" << nbgood << " out
  // of #" << vkpsidx.size() << " \n";
}

void ORBmatcher::updateMask(cv::Mat &mask, const cv::Point2f &pt, int radius) {
  cv::circle(mask, pt, radius, cv::Scalar(255), cv::FILLED);
}

bool ORBmatcher::isPointNearby(const cv::Mat &mask, const cv::Point2f &pt) {
  return mask.at<uchar>(cv::Point(pt.x, pt.y)) == 255;
}
int ORBmatcher::SearchByProjectionWithOF(Frame &CurrentFrame,
                                         const Frame &LastFrame, cv::Mat &mask,
                                         const float th, const int winsize,
                                         const float F_THRESHOLD,
                                         const int DIST_THRESHOLD,
                                         const bool bMono) {
  // assert(LastFrame.mvKeys.size() ==
  // LastFrame.track_feature_pts_.size());
  int nbgood = 0;
  std::vector<int> v3dkpids, v2dkpids;
  std::vector<cv::Point2f> v3dkps, v3dpriors, v2dkps, v2dpriors;

  v3dkpids.reserve(LastFrame.mvpMapPoints.size());
  v3dpriors.reserve(LastFrame.mvpMapPoints.size());
  v2dkpids.reserve(LastFrame.mvpMapPoints.size());
  v2dpriors.reserve(LastFrame.mvpMapPoints.size());

  const Sophus::SE3f Tcw = CurrentFrame.GetPose();
  const Eigen::Vector3f twc = Tcw.inverse().translation();
  const Sophus::SE3f Tlw = LastFrame.GetPose();
  const Eigen::Vector3f tlc = Tlw * twc;

  int cnt = -1;
  // 创建掩膜图像
  int imgWidth = CurrentFrame.image.cols;
  int imgHeight = CurrentFrame.image.rows;
  for (auto kp : CurrentFrame.mvKeys) {
    if (kp.pt.x > 0 && kp.pt.x < imgWidth && kp.pt.y > 0 &&
        kp.pt.y < imgHeight) {
      mask.at<uchar>(kp.pt.y, kp.pt.x) = 255;
    }
  }
  size_t none_mps_cnt = 0;
  for (const auto &mp : LastFrame.mvpMapPoints) {
    cnt++;
    // 没有地图点的进行像素跟踪
    if (mp == nullptr) {
      float u = LastFrame.mvKeys.at(cnt).pt.x;
      float v = LastFrame.mvKeys.at(cnt).pt.y;

      v2dkps.emplace_back(u, v);
      v2dpriors.emplace_back(u, v);
      v2dkpids.push_back(cnt);
      none_mps_cnt++;
      continue;
    }
    if (mp->isBad() || LastFrame.mvbOutlier[cnt]) continue;

    // 存在地图点的进行3D点投影跟踪
    Eigen::Vector3f x3Dw = mp->GetWorldPos();
    Eigen::Vector3f x3Dc = Tcw * x3Dw;

    const float xc = x3Dc(0);
    const float yc = x3Dc(1);
    const float invzc = 1.0 / x3Dc(2);

    float u = CurrentFrame.fx * xc * invzc + CurrentFrame.cx;
    float v = CurrentFrame.fy * yc * invzc + CurrentFrame.cy;
    if (invzc < 0 || u < CurrentFrame.mnMinX || u > CurrentFrame.mnMaxX ||
        v < CurrentFrame.mnMinY || v > CurrentFrame.mnMaxY) {
      u = LastFrame.mvKeys.at(cnt).pt.x;
      v = LastFrame.mvKeys.at(cnt).pt.y;
      v2dkps.emplace_back(u, v);
      v2dpriors.emplace_back(u, v);
      v2dkpids.push_back(cnt);
      continue;
    }
    v3dkps.emplace_back(LastFrame.mvKeys.at(cnt).pt);
    v3dpriors.emplace_back(u, v);
    v3dkpids.push_back(cnt);
  }

  int nklt_win_size = winsize;
  float nklt_err = 15;
  float max_fbklt_dist = 0.5f;
  // 1st track 3d kps if using prior
  static const int nbpyrlvl_3d_2d = 3;
  if (!v3dpriors.empty()) {
    // Good / bad kps vector
    std::vector<bool> vkpstatus;
    fbKltTracking(LastFrame.mImGray, CurrentFrame.mImGray, nklt_win_size,
                  nbpyrlvl_3d_2d, nklt_err, max_fbklt_dist, v3dkps, v3dpriors,
                  vkpstatus);

    // F check
    std::vector<int> index;
    std::vector<cv::Point2f> un_cur_pts, un_forw_pts;
    for (int i = 0; i < vkpstatus.size(); i++) {
      if (vkpstatus[i]) {
        index.emplace_back(i);
        un_cur_pts.emplace_back(v3dkps.at(i));
        un_forw_pts.emplace_back(v3dpriors.at(i));
      }
    }
    if (un_cur_pts.size() > 8) {
      vector<uchar> status;
      cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC,
                             F_THRESHOLD, 0.99, status);
      for (int i = 0; i < status.size(); i++) {
        if (!status[i]) {
          vkpstatus.at(index.at(i)) = false;
        }
      }
    }
    size_t nbkps = v3dkps.size();
    // 跟踪到的特征点（上帧）
    std::vector<cv::KeyPoint> tracked_kps;
    std::vector<MapPoint *> tracked_mps;
    tracked_kps.reserve(nbkps);
    tracked_mps.reserve(nbkps);
    CurrentFrame.track_feature_pts_.reserve(nbkps);
    for (size_t i = 0; i < nbkps; i++) {
      if (vkpstatus.at(i)) {
        cv::KeyPoint pt =
            LastFrame.mvKeys.at(v3dkpids.at(i));  // 其他性质和之前一样
        pt.pt = v3dpriors.at(i);                  // 新帧上的位置
        // 判断像素点pt附件是否已经存在mvKeys中的特征点,如果存在则不添加
        if (isPointNearby(mask, pt.pt)) continue;
        tracked_kps.emplace_back(pt);
        tracked_mps.emplace_back(LastFrame.mvpMapPoints.at(v3dkpids.at(i)));
        LastFrame.mvpMapPoints.at(v3dkpids.at(i))->mnLastFrameSeen =
            CurrentFrame.mnId;
        CurrentFrame.track_feature_pts_.emplace_back(
            LastFrame.track_feature_pts_.at(v3dkpids.at(i)));
        nbgood++;
        updateMask(mask, pt.pt, DIST_THRESHOLD);
      } else {  // points not track success
        int id = v3dkpids.at(i);
        float u = LastFrame.mvKeys.at(id).pt.x;
        float v = LastFrame.mvKeys.at(id).pt.y;
        v2dkps.emplace_back(u, v);

        v2dpriors.emplace_back(u, v);
        v2dkpids.push_back(id);
      }
    }
    CurrentFrame.AddPts(tracked_kps, tracked_mps, bMono);
  }

  // track 2d kps and the points tracked fail
  static const int nbpyrlvl_2d_2d = 6;
  if (!v2dkps.empty()) {
    // Good / bad kps vector
    std::vector<bool> vkpstatus;
    fbKltTracking(LastFrame.mImGray, CurrentFrame.mImGray, nklt_win_size,
                  nbpyrlvl_2d_2d, nklt_err, max_fbklt_dist, v2dkps, v2dpriors,
                  vkpstatus);
    // DO F check
    std::vector<int> index;
    std::vector<cv::Point2f> un_cur_pts, un_forw_pts;
    for (int i = 0; i < vkpstatus.size(); i++) {
      if (vkpstatus[i]) {
        index.emplace_back(i);
        un_cur_pts.emplace_back(v2dkps.at(i));
        un_forw_pts.emplace_back(v2dpriors.at(i));
      }
    }

    if (un_cur_pts.size() > 8) {
      vector<uchar> status;
      cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC,
                             F_THRESHOLD * 0.5, 0.99, status);
      for (int i = 0; i < status.size(); i++) {
        if (!status[i]) {
          vkpstatus.at(index.at(i)) = false;
        }
      }
    }

    std::vector<cv::KeyPoint> tracked_kps;
    std::vector<MapPoint *> tracked_mps;
    for (size_t i = 0; i < v2dkps.size(); i++) {
      if (vkpstatus.at(i)) {
        int id = v2dkpids.at(i);
        cv::KeyPoint pt = LastFrame.mvKeys.at(id);  // 其他性质和之前一样
        pt.pt = v2dpriors.at(i);
        if (isPointNearby(mask, pt.pt)) continue;
        tracked_kps.emplace_back(pt);
        tracked_mps.emplace_back(LastFrame.mvpMapPoints.at(id));
        CurrentFrame.track_feature_pts_.emplace_back(
            LastFrame.track_feature_pts_.at(id));
        updateMask(mask, pt.pt, DIST_THRESHOLD);
        if (LastFrame.mvpMapPoints.at(id) != nullptr) {
          LastFrame.mvpMapPoints.at(id)->mnLastFrameSeen = CurrentFrame.mnId;
          // nbgood++;
        }
        nbgood++;
      }
    }
    CurrentFrame.AddPts(tracked_kps, tracked_mps, bMono);
  }

  CurrentFrame.AssignFeaturesToGrid();
  return nbgood;
}


void ORBmatcher::ComputeThreeMaxima(vector<int> *histo, const int L, int &ind1,
                                    int &ind2, int &ind3) {
  int max1 = 0;
  int max2 = 0;
  int max3 = 0;

  for (int i = 0; i < L; i++) {
    const int s = histo[i].size();
    if (s > max1) {
      max3 = max2;
      max2 = max1;
      max1 = s;
      ind3 = ind2;
      ind2 = ind1;
      ind1 = i;
    } else if (s > max2) {
      max3 = max2;
      max2 = s;
      ind3 = ind2;
      ind2 = i;
    } else if (s > max3) {
      max3 = s;
      ind3 = i;
    }
  }

  if (max2 < 0.1f * (float)max1) {
    ind2 = -1;
    ind3 = -1;
  } else if (max3 < 0.1f * (float)max1) {
    ind3 = -1;
  }
}

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b) {
  const int *pa = a.ptr<int32_t>();
  const int *pb = b.ptr<int32_t>();

  int dist = 0;

  for (int i = 0; i < 8; i++, pa++, pb++) {
    unsigned int v = *pa ^ *pb;
    v = v - ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }

  return dist;
}

bool ORBmatcher::inBorder(const cv::Point2f &pt, const cv::Mat &im) const {
  const float BORDER_SIZE = 1.;

  return BORDER_SIZE <= pt.x && pt.x < im.cols - BORDER_SIZE &&
         BORDER_SIZE <= pt.y && pt.y < im.rows - BORDER_SIZE;
}
}  // namespace ORB_SLAM3
