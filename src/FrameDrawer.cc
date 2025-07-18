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

#include "FrameDrawer.h"

#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Tracking.h"

namespace ORB_SLAM3 {

FrameDrawer::FrameDrawer(Atlas *pAtlas) : both(false), mpAtlas(pAtlas) {
  mState = Tracking::SYSTEM_NOT_READY;
  mIm = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  mImRight = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
}

cv::Mat FrameDrawer::DrawFrame(float imageScale) {
  cv::Mat im;
  vector<cv::KeyPoint>
      vIniKeys;  // Initialization: KeyPoints in reference frame
  vector<int>
      vMatches;  // Initialization: correspondeces with reference keypoints
  vector<cv::KeyPoint> vCurrentKeys;  // KeyPoints in current frame
  vector<bool> vbVO, vbMap;           // Tracked MapPoints in current frame
  vector<cv::KeyPoint> vProjectedKeys;
  vector<pair<cv::Point2f, cv::Point2f> > vTracks;
  int state;  // Tracking state
  vector<float> vCurrentDepth;
  float thDepth;

  Frame currentFrame;
  vector<MapPoint *> vpLocalMap;
  vector<cv::KeyPoint> vMatchesKeys;
  vector<MapPoint *> vpMatchedMPs;
  vector<cv::KeyPoint> vOutlierKeys;
  vector<MapPoint *> vpOutlierMPs;
  map<long unsigned int, cv::Point2f> mProjectPoints;
  map<long unsigned int, cv::Point2f> mMatchedInImage;
  // 绿色
  cv::Scalar standardColor(0, 255, 0);
  // 红色
  cv::Scalar projectedColor(0, 0, 255);
  // 蓝色
  cv::Scalar odometryColor(255, 0, 0);
  cv::Scalar skyBlue(135, 206, 235);
  mfProjError = mCurrentFrame.mfProjError;
  mfAvgProjError = mCurrentFrame.mfAvgProjError;
  // Copy variables within scoped mutex
  {
    unique_lock<mutex> lock(mMutex);
    state = mState;
    if (mState == Tracking::SYSTEM_NOT_READY) mState = Tracking::NO_IMAGES_YET;

    mIm.copyTo(im);

    if (mState == Tracking::NOT_INITIALIZED) {
      vCurrentKeys = mvCurrentKeys;
      vIniKeys = mvIniKeys;
      vMatches = mvIniMatches;
      vTracks = mvTracks;
      vProjectedKeys = mvProjectedKeys;
      mnFrameID = currentFrame.mnId;
    } else if (mState == Tracking::OK ||
               mState == Tracking::RECENTLY_LOST)  // TRACKING
    {
      vCurrentKeys = mvCurrentKeys;
      vbVO = mvbVO;
      vbMap = mvbMap;
      vProjectedKeys = mvProjectedKeys;
      currentFrame = mCurrentFrame;
      mnFrameID = currentFrame.mnId;
      vpLocalMap = mvpLocalMap;
      vMatchesKeys = mvMatchedKeys;
      vpMatchedMPs = mvpMatchedMPs;
      vOutlierKeys = mvOutlierKeys;
      vpOutlierMPs = mvpOutlierMPs;
      mProjectPoints = mmProjectPoints;
      mMatchedInImage = mmMatchedInImage;

      vCurrentDepth = mvCurrentDepth;
      thDepth = mThDepth;

    } else if (mState == Tracking::LOST) {
      vCurrentKeys = mvCurrentKeys;
      mnFrameID = currentFrame.mnId;
    }
  }

  if (imageScale != 1.f) {
    int imWidth = im.cols / imageScale;
    int imHeight = im.rows / imageScale;
    cv::resize(im, im, cv::Size(imWidth, imHeight));
  }

  if (im.channels() < 3)  // this should be always true
    cvtColor(im, im, cv::COLOR_GRAY2BGR);

  // Draw
  if (state == Tracking::NOT_INITIALIZED) {
    for (unsigned int i = 0; i < vMatches.size(); i++) {
      if (vMatches[i] >= 0) {
        cv::Point2f pt1, pt2;
        if (imageScale != 1.f) {
          pt1 = vIniKeys[i].pt / imageScale;
          pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
        } else {
          pt1 = vIniKeys[i].pt;
          pt2 = vCurrentKeys[vMatches[i]].pt;
        }
        cv::line(im, pt1, pt2, standardColor);
      }
    }
    for (vector<pair<cv::Point2f, cv::Point2f> >::iterator it = vTracks.begin();
         it != vTracks.end(); it++) {
      cv::Point2f pt1, pt2;
      if (imageScale != 1.f) {
        pt1 = (*it).first / imageScale;
        pt2 = (*it).second / imageScale;
      } else {
        pt1 = (*it).first;
        pt2 = (*it).second;
      }
      cv::line(im, pt1, pt2, standardColor, 5);
    }

  } else if (state == Tracking::OK ||
             state == Tracking::RECENTLY_LOST)  // TRACKING
  {
    mnTracked = 0;
    mnTrackedVO = 0;
    const float r = 4.0;
    int n = vCurrentKeys.size();
    // for (int i = 0; i < n; i++) {
    //   if (vbVO[i] || vbMap[i]) {
    //     cv::Point2f pt1, pt2;
    //     cv::Point2f point;
    //     if (imageScale != 1.f) {
    //       point = vCurrentKeys[i].pt / imageScale;
    //       float px = vCurrentKeys[i].pt.x / imageScale;
    //       float py = vCurrentKeys[i].pt.y / imageScale;
    //       pt1.x = px - r;
    //       pt1.y = py - r;
    //       pt2.x = px + r;
    //       pt2.y = py + r;
    //     } else {
    //       point = vCurrentKeys[i].pt;
    //       pt1.x = vCurrentKeys[i].pt.x - r;
    //       pt1.y = vCurrentKeys[i].pt.y - r;
    //       pt2.x = vCurrentKeys[i].pt.x + r;
    //       pt2.y = vCurrentKeys[i].pt.y + r;
    //     }

    //     // This is a match to a MapPoint in the map
    //     if (vbMap[i]) {
    //       cv::rectangle(im, pt1, pt2, standardColor);
    //       cv::circle(im, point, 2, standardColor, -1);
    //       mnTracked++;
    //     } else  // This is match to a "visual odometry" MapPoint created in
    //     the
    //             // last frame
    //     {
    //       cv::rectangle(im, pt1, pt2, odometryColor);
    //       cv::circle(im, point, 2, odometryColor, -1);
    //       mnTrackedVO++;
    //     }
    //   }
    // }
    for (int i = 0; i < n; i++) {
      if (vbVO[i] || vbMap[i]) {
        cv::Point2f pt1, pt2;
        cv::Point2f project_pt1, project_pt2;
        cv::Point2f point, projected_point;
        if (imageScale != 1.f) {
          point = vCurrentKeys[i].pt / imageScale;
          projected_point = vProjectedKeys[i].pt / imageScale;
          float px = vCurrentKeys[i].pt.x / imageScale;
          float py = vCurrentKeys[i].pt.y / imageScale;
          pt1.x = px - r;
          pt1.y = py - r;
          pt2.x = px + r;
          pt2.y = py + r;
          project_pt1.x = projected_point.x - r;
          project_pt1.y = projected_point.y - r;
          project_pt2.x = projected_point.x + r;
          project_pt2.y = projected_point.y + r;
        } else {
          point = vCurrentKeys[i].pt;
          projected_point = vProjectedKeys[i].pt;
          pt1.x = vCurrentKeys[i].pt.x - r;
          pt1.y = vCurrentKeys[i].pt.y - r;
          pt2.x = vCurrentKeys[i].pt.x + r;
          pt2.y = vCurrentKeys[i].pt.y + r;
          project_pt1.x = projected_point.x - r;
          project_pt1.y = projected_point.y - r;
          project_pt2.x = projected_point.x + r;
          project_pt2.y = projected_point.y + r;
        }

        // This is a match to a MapPoint in the map
        if (vbMap[i]) {
          // cv::rectangle(im, pt1, pt2, standardColor);
          cv::circle(im, point, 2.5, standardColor, -1);
          // cv::rectangle(im, project_pt1, project_pt2, projectedColor);
          cv::circle(im, projected_point, 2.5, projectedColor, -1);
          cv::line(im, point, projected_point, cv::Scalar(255, 0, 0));
          mnTracked++;
        } else  // This is match to a "visual odometry" MapPoint created in the
                // last frame
        {
          cv::rectangle(im, pt1, pt2, skyBlue);
          cv::circle(im, point, 2.5, skyBlue, -1);
          cv::rectangle(im, project_pt1, project_pt2, projectedColor);
          cv::circle(im, projected_point, 2.5, projectedColor, -1);
          // cv::line(im, point, projected_point, cv::Scalar(255, 0, 0));
          mnTrackedVO++;
        }
      }
    }
  }
  // save the image named with frameID
  cv::imwrite("/home/tingyang/3d_recon/orbslam3_res/go2_seq2/images/"+ to_string(mnFrameID) + ".png", im);

  cv::Mat imWithInfo;
  DrawTextInfo(im, state, imWithInfo);

  return imWithInfo;
}

cv::Mat FrameDrawer::DrawRightFrame(float imageScale) {
  cv::Mat im;
  vector<cv::KeyPoint>
      vIniKeys;  // Initialization: KeyPoints in reference frame
  vector<int>
      vMatches;  // Initialization: correspondeces with reference keypoints
  vector<cv::KeyPoint> vCurrentKeys;  // KeyPoints in current frame
  vector<bool> vbVO, vbMap;           // Tracked MapPoints in current frame
  int state;                          // Tracking state

  // Copy variables within scoped mutex
  {
    unique_lock<mutex> lock(mMutex);
    state = mState;
    if (mState == Tracking::SYSTEM_NOT_READY) mState = Tracking::NO_IMAGES_YET;

    mImRight.copyTo(im);

    if (mState == Tracking::NOT_INITIALIZED) {
      vCurrentKeys = mvCurrentKeysRight;
      vIniKeys = mvIniKeys;
      vMatches = mvIniMatches;
    } else if (mState == Tracking::OK) {
      vCurrentKeys = mvCurrentKeysRight;
      vbVO = mvbVO;
      vbMap = mvbMap;
    } else if (mState == Tracking::LOST) {
      vCurrentKeys = mvCurrentKeysRight;
    }
  }  // destroy scoped mutex -> release mutex

  if (imageScale != 1.f) {
    int imWidth = im.cols / imageScale;
    int imHeight = im.rows / imageScale;
    cv::resize(im, im, cv::Size(imWidth, imHeight));
  }

  if (im.channels() < 3)  // this should be always true
    cvtColor(im, im, cv::COLOR_GRAY2BGR);

  // Draw
  if (state == Tracking::NOT_INITIALIZED)  // INITIALIZING
  {
    for (unsigned int i = 0; i < vMatches.size(); i++) {
      if (vMatches[i] >= 0) {
        cv::Point2f pt1, pt2;
        if (imageScale != 1.f) {
          pt1 = vIniKeys[i].pt / imageScale;
          pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
        } else {
          pt1 = vIniKeys[i].pt;
          pt2 = vCurrentKeys[vMatches[i]].pt;
        }

        cv::line(im, pt1, pt2, cv::Scalar(0, 255, 0));
      }
    }
  } else if (state == Tracking::OK)  // TRACKING
  {
    mnTracked = 0;
    mnTrackedVO = 0;
    const float r = 5;
    const int n = mvCurrentKeysRight.size();
    const int Nleft = mvCurrentKeys.size();

    for (int i = 0; i < n; i++) {
      if (vbVO[i + Nleft] || vbMap[i + Nleft]) {
        cv::Point2f pt1, pt2;
        cv::Point2f point;
        if (imageScale != 1.f) {
          point = mvCurrentKeysRight[i].pt / imageScale;
          float px = mvCurrentKeysRight[i].pt.x / imageScale;
          float py = mvCurrentKeysRight[i].pt.y / imageScale;
          pt1.x = px - r;
          pt1.y = py - r;
          pt2.x = px + r;
          pt2.y = py + r;
        } else {
          point = mvCurrentKeysRight[i].pt;
          pt1.x = mvCurrentKeysRight[i].pt.x - r;
          pt1.y = mvCurrentKeysRight[i].pt.y - r;
          pt2.x = mvCurrentKeysRight[i].pt.x + r;
          pt2.y = mvCurrentKeysRight[i].pt.y + r;
        }

        // This is a match to a MapPoint in the map
        if (vbMap[i + Nleft]) {
          cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
          cv::circle(im, point, 2, cv::Scalar(0, 255, 0), -1);
          mnTracked++;
        } else  // This is match to a "visual odometry" MapPoint created in the
                // last frame
        {
          cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 0));
          cv::circle(im, point, 2, cv::Scalar(255, 0, 0), -1);
          mnTrackedVO++;
        }
      }
    }
  }

  cv::Mat imWithInfo;
  DrawTextInfo(im, state, imWithInfo);

  return imWithInfo;
}

void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText) {
  stringstream s;
  if (nState == Tracking::NO_IMAGES_YET)
    s << " WAITING FOR IMAGES";
  else if (nState == Tracking::NOT_INITIALIZED)
    s << " TRYING TO INITIALIZE ";
  else if (nState == Tracking::OK || nState == Tracking::RECENTLY_LOST) {
    if (!mbOnlyTracking)
      s << "SLAM MODE |  ";
    else
      s << "LOCALIZATION | ";
    int nMaps = mpAtlas->CountMaps();
    int nKFs = mpAtlas->KeyFramesInMap();
    int nMPs = mpAtlas->MapPointsInMap();
    s << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs
      << ", Matches: " << mnTracked << ", FrameID: " << mnFrameID;
    if (mnTrackedVO > 0) s << ", + VO matches: " << mnTrackedVO;
  } else if (nState == Tracking::LOST) {
    s << " TRACK LOST. TRYING TO RELOCALIZE ";
    s << ", FrameID: " << mnFrameID;
  } else if (nState == Tracking::SYSTEM_NOT_READY) {
    s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
  }

  int baseline = 0;
  cv::Size textSize =
      cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

  imText = cv::Mat(im.rows + textSize.height + 10, im.cols, im.type());
  im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
  imText.rowRange(im.rows, imText.rows) =
      cv::Mat::zeros(textSize.height + 10, im.cols, im.type());
  cv::putText(imText, s.str(), cv::Point(5, imText.rows - 5),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
  // 在图片左上角显示文字
  stringstream text;
  text << " Avg Proj Error: " << mfAvgProjError;
  cv::Size textSize2 =
      cv::getTextSize(text.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, 0);
  cv::putText(imText, text.str(), cv::Point(5, 20), cv::FONT_HERSHEY_PLAIN, 1,
              cv::Scalar(255, 0, 0), 1, 8);
}

void FrameDrawer::Update(Tracking *pTracker) {
  unique_lock<mutex> lock(mMutex);
  pTracker->mImGray.copyTo(mIm);
  mvCurrentKeys = pTracker->mCurrentFrame.mvKeys;
  mThDepth = pTracker->mCurrentFrame.mThDepth;
  mvCurrentDepth = pTracker->mCurrentFrame.mvDepth;

  if (both) {
    mvCurrentKeysRight = pTracker->mCurrentFrame.mvKeysRight;
    pTracker->mImRight.copyTo(mImRight);
    N = mvCurrentKeys.size() + mvCurrentKeysRight.size();
  } else {
    N = mvCurrentKeys.size();
  }

  mvbVO = vector<bool>(N, false);
  // mvProjectedKeys = vector<cv::KeyPoint>(N);
  mvProjectedKeys = pTracker->mCurrentFrame.mvProjectedKeys;
  mvbMap = vector<bool>(N, false);
  mbOnlyTracking = pTracker->mbOnlyTracking;

  // Variables for the new visualization
  mCurrentFrame = pTracker->mCurrentFrame;
  mmProjectPoints = mCurrentFrame.mmProjectPoints;
  mmMatchedInImage.clear();

  mvpLocalMap = pTracker->GetLocalMapMPS();
  mvMatchedKeys.clear();
  mvMatchedKeys.reserve(N);
  mvpMatchedMPs.clear();
  mvpMatchedMPs.reserve(N);
  mvOutlierKeys.clear();
  mvOutlierKeys.reserve(N);
  mvpOutlierMPs.clear();
  mvpOutlierMPs.reserve(N);

  if (pTracker->mLastProcessedState == Tracking::NOT_INITIALIZED) {
    mvIniKeys = pTracker->mInitialFrame.mvKeys;
    mvIniMatches = pTracker->mvIniMatches;
  } else if (pTracker->mLastProcessedState == Tracking::OK ||
             pTracker->mLastProcessedState == Tracking::RECENTLY_LOST) {
    for (int i = 0; i < N; i++) {
      MapPoint *pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
      if (pMP) {
        if (!pTracker->mCurrentFrame.mvbOutlier[i]) {
          if (pMP->Observations() > 0) {
            mvbMap[i] = true;
          } else
            mvbVO[i] = true;

          mmMatchedInImage[pMP->mnId] = mvCurrentKeys[i].pt;
        } else {
          mvpOutlierMPs.push_back(pMP);
          mvOutlierKeys.push_back(mvCurrentKeys[i]);
        }
        // mvProjectedKeys[i].pt =
        // pTracker->mCurrentFrame.mvProjectedKeys[i].pt;
      }
    }
  }
  mState = static_cast<int>(pTracker->mLastProcessedState);
}

}  // namespace ORB_SLAM3
