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

#include <System.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
// #define ENABLE_ASYNC
// #define ENABLE_OMP
using namespace std;
void LoadImages(const string &strAssociationFilename,
                vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD,
                vector<double> &vTimestamps);

int main(int argc, char **argv) {
  if (argc != 5) {
    cerr << endl
         << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings "
            "path_to_sequence path_to_association"
         << endl;
    return 1;
  }
  cv::FileStorage fsSettings(string(argv[2]), cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    cerr << "Failed to open setting file at: " << string(argv[2]) << endl;
    exit(-1);
  }
  // Retrieve paths to images
  vector<string> vstrImageFilenamesRGB;
  vector<string> vstrImageFilenamesD;
  vector<double> vTimestamps;
  string strAssociationFilename = string(argv[4]);
  cout << "Start Loading Images ..." << endl;
  LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD,
             vTimestamps);
  cout << "End Loadingcd sequence ..." << endl;
  // Check consistency in the number of images and depthmaps
  int nImages = vstrImageFilenamesRGB.size();
  if (vstrImageFilenamesRGB.empty()) {
    cerr << endl << "No images found in provided path." << endl;
    return 1;
  } else if (vstrImageFilenamesD.size() != vstrImageFilenamesRGB.size()) {
    cerr << endl << "Different number of images for rgb and depth." << endl;
    return 1;
  }

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  int bUseViewer = fsSettings["UseViewer"];
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, bUseViewer, 0,
                         std::string(), std::string(argv[3]));
  float imageScale = SLAM.GetImageScale();

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;
  std::cout << "argv[1] : " << argv[1] << std::endl;
  std::cout << "argv[2] : " << argv[2] << std::endl;
  std::cout << "argv[3] : " << argv[3] << std::endl;
  // std::cout <<"nImages : " << nImages << std::endl;

  // Main loop
  cv::Mat imRGB, imD;
#ifdef COMPILEDWITHC11
  std::chrono::steady_clock::time_point start_time =
      std::chrono::steady_clock::now();
#else
  std::chrono::monotonic_clock::time_point start_time =
      std::chrono::monotonic_clock::now();
#endif
  for (int ni = 0; ni < nImages; ni++) {
    std::string rgb_file = string(argv[3]) + "/" + vstrImageFilenamesRGB[ni];
    std::string depth_file = string(argv[3]) + "/" + vstrImageFilenamesD[ni];
    // std::cout << "depth_file :" << depth_file <<std::endl;
    // Read image and depthmap from file
    imRGB = cv::imread(rgb_file.c_str(),
                       cv::IMREAD_UNCHANGED);  //,cv::IMREAD_UNCHANGED);
    imD = cv::imread(depth_file.c_str(),
                     cv::IMREAD_UNCHANGED);  //,cv::IMREAD_UNCHANGED);

    // std::cout << imD.rows << " " << imD.cols <<std::endl;
    // std::cout << "here is ok" << std::endl;

    cv::Mat depth8U;
    imD.convertTo(depth8U, CV_8U, 0.03);

    // // for debug
    // cv::imshow ( "rgb_image", imRGB );
    // cv::Mat depth_color_map;
    // cv::applyColorMap(depth8U, depth_color_map, cv::COLORMAP_JET);
    // cv::imshow ( "depth_image", depth_color_map );
    // cv::waitKey(1);

    double tframe = vTimestamps[ni];

    if (imRGB.empty()) {
      cerr << endl
           << "Failed to load image at: " << string(argv[3]) << "/"
           << vstrImageFilenamesRGB[ni] << endl;
      return 1;
    }

    if (imageScale != 1.f) {
      int width = imRGB.cols * imageScale;
      int height = imRGB.rows * imageScale;
      cv::resize(imRGB, imRGB, cv::Size(width, height));
      cv::resize(imD, imD, cv::Size(width, height));
    }

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 =
        std::chrono::monotonic_clock::now();
#endif

    // Pass the image to the SLAM system
    // SLAM.TrackRGBD(imRGB, imD, tframe);
    // std::cout << tframe << " : " <<
    // SLAM.TrackRGBD(imRGB,imD,tframe).matrix3x4() << std::endl;
#ifdef ENABLE_ASYNC
    SLAM.TrackRGBDAsync(imRGB, imD, tframe);
#else
    SLAM.TrackRGBD(imRGB, imD, tframe);
#endif
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 =
        std::chrono::monotonic_clock::now();
#endif

    double ttrack =
        std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1)
            .count();

    vTimesTrack[ni] = ttrack;

    // Wait to load the next frame
    double T = 0;
    if (ni < nImages - 1)
      T = vTimestamps[ni + 1] - tframe;
    else if (ni > 0)
      T = tframe - vTimestamps[ni - 1];

    int nDelay = fsSettings["DelayScale"];
    if (ttrack < T) usleep((T - ttrack) * 1e6 * nDelay);
  }
#ifdef COMPILEDWITHC11
  std::chrono::steady_clock::time_point end_time =
      std::chrono::steady_clock::now();
#else
  std::chrono::monotonic_clock::time_point end_time =
      std::chrono::monotonic_clock::now();
#endif
  // Stop all threads
  // cv::imshow("final", imRGB);
  // cv::waitKey(0);
  SLAM.Shutdown(std::string(argv[3]));

  // Tracking time statistics
  // sort(vTimesTrack.begin(), vTimesTrack.end());
  // float totaltime = 0;
  // for (int ni = 0; ni < nImages; ni++) {
  //   totaltime += vTimesTrack[ni];
  // }
  // cout << "-------" << endl << endl;
  // cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
  // cout << "mean tracking time: " << totaltime / nImages << endl;
  // cout << "tracking lost count: " << SLAM.GetTrackLostCnt() << endl;
  // cout << "all image count: " << nImages << endl;
  // cout << "tracking lost count: " << float(SLAM.GetTrackLostCnt()) / nImages
  //      << endl;
  double total_time =
      std::chrono::duration_cast<std::chrono::duration<double> >(end_time -
                                                                 start_time)
          .count();
  cout << "-------" << endl << endl;
  cout << "Total time: " << total_time << "s" << endl;
  // cout << "median tracking time: " << vTimesTrack[nImages / 2] * 1e3 << "ms"
  //      << endl;
  double mean_time = total_time / nImages;
  cout << "Mean tracking time: " << mean_time << "s" << endl;

  // Save camera trajectory
  ORB_SLAM3::Settings *setting = SLAM.GetSetting();
  std::string CameraTrajectory_file = string(argv[3]) + "/CameraTrajectory.txt";
  std::string KeyFrameTrajectory_file =
      string(argv[3]) + "/KeyFrameTrajectory.txt";
  std::string KeyFrameLoopTrajectory_file =
      string(argv[3]) + "/KeyFrameLoopTrajectory.txt";
  // Save Reprojection Error
  std::string Frame2FrameReprojectionError_file =
      string(argv[3]) + "/Frame2FrameReprojectionError.txt";
  std::string Frame2MapReprojectionError_file =
      string(argv[3]) + "/Frame2MapReprojectionError.txt";
  if (setting->useOpticalFlow() && setting->useICP() &&
      setting->useLidarObs()) {
    CameraTrajectory_file =
        string(argv[3]) + "/CameraTrajectory_OF_ICP_Lidar_NoImu.txt";
    KeyFrameTrajectory_file =
        string(argv[3]) + "/KeyFrameTrajectory_OF_ICP_Lidar_NoImu.txt";
    KeyFrameLoopTrajectory_file =
        string(argv[3]) + "/KeyFrameLoopTrajectory_OF_ICP_Lidar_NoImu.txt";
    Frame2FrameReprojectionError_file =
        string(argv[3]) + "/Frame2FrameReprojectionError_OF_ICP_Lidar_NoImu.txt";
    Frame2MapReprojectionError_file =
        string(argv[3]) + "/Frame2MapReprojectionError_OF_ICP_Lidar_NoImu.txt";
  } else if (!setting->useOpticalFlow() && setting->useICP()) {
    CameraTrajectory_file = string(argv[3]) + "/CameraTrajectory_NoOF_ICP_NoImu.txt";
    KeyFrameTrajectory_file =
        string(argv[3]) + "/KeyFrameTrajectory_NoOF_ICP_NoImu.txt";
    KeyFrameLoopTrajectory_file =
        string(argv[3]) + "/KeyFrameLoopTrajectory_NoOF_ICP_NoImu.txt";
    Frame2FrameReprojectionError_file =
        string(argv[3]) + "/Frame2FrameReprojectionError_NoOF_ICP_NoImu.txt";
    Frame2MapReprojectionError_file =
        string(argv[3]) + "/Frame2MapReprojectionError_NoOF_ICP_NoImu.txt";
  } else if (!setting->useICP() && setting->useOpticalFlow()) {
    CameraTrajectory_file = string(argv[3]) + "/CameraTrajectory_OF_NoICP_NoImu.txt";
    KeyFrameTrajectory_file =
        string(argv[3]) + "/KeyFrameTrajectory_OF_NoICP_NoImu.txt";
    KeyFrameLoopTrajectory_file =
        string(argv[3]) + "/KeyFrameLoopTrajectory_OF_NoICP_NoImu.txt";
    Frame2FrameReprojectionError_file =
        string(argv[3]) + "/Frame2FrameReprojectionError_OF_NoICP_NoImu.txt";
    Frame2MapReprojectionError_file =
        string(argv[3]) + "/Frame2MapReprojectionError_OF_NoICP_NoImu.txt";
  } else if (setting->useOpticalFlow() && setting->useICP()) {
    CameraTrajectory_file = string(argv[3]) + "/CameraTrajectory_OF_ICP_NoImu.txt";
    KeyFrameTrajectory_file =
        string(argv[3]) + "/KeyFrameTrajectory_OF_ICP_NoImu.txt";
    KeyFrameLoopTrajectory_file =
        string(argv[3]) + "/KeyFrameLoopTrajectory_OF_ICP_NoImu.txt";
    Frame2FrameReprojectionError_file =
        string(argv[3]) + "/Frame2FrameReprojectionError_OF_ICP_NoImu.txt";
    Frame2MapReprojectionError_file =
        string(argv[3]) + "/Frame2MapReprojectionError_OF_ICP_NoImu.txt";
  } else if (!setting->useOpticalFlow() && !setting->useICP() &&
             setting->useLidarObs()) {
    CameraTrajectory_file =
        string(argv[3]) + "/CameraTrajectory_NoOF_NoICP_Lidar_NoImu.txt";
    KeyFrameTrajectory_file =
        string(argv[3]) + "/KeyFrameTrajectory_NoOF_NoICP_Lidar_NoImu.txt";
    KeyFrameLoopTrajectory_file =
        string(argv[3]) + "/KeyFrameLoopTrajectory_NoOF_NoICP_Lidar_NoImu.txt";
    Frame2FrameReprojectionError_file =
        string(argv[3]) + "/Frame2FrameReprojectionError_NoOF_NoICP_Lidar_NoImu.txt";
    Frame2MapReprojectionError_file =
        string(argv[3]) + "/Frame2MapReprojectionError_NoOF_NoICP_Lidar_NoImu.txt";
  }

#ifdef ENABLE_ASYNC
  std::string TimeStamp_file = string(argv[3]) + "/AsyncTrackCost.txt";
#else
  std::string TimeStamp_file = string(argv[3]) + "/NoAsyncTrackCost.txt";
#endif
  // 将total_time、mean_time保存到文件中
  ofstream f;
  f.open(TimeStamp_file);
  f << fixed;
  f << "Total time: " << total_time << "s" << endl;
  f << "Mean tracking time: " << mean_time << "s" << endl;
  f.close();
  SLAM.SaveTrajectoryTUM(CameraTrajectory_file.c_str());
  SLAM.SaveKeyFrameTrajectoryTUM(KeyFrameTrajectory_file.c_str());
  // SLAM.SaveKeyFrameLoopTrajectoryTUM2(KeyFrameLoopTrajectory_file.c_str());
  SLAM.SaveFrame2FrameReprojErr(Frame2FrameReprojectionError_file.c_str());
  SLAM.SaveFrame2MapReprojErr(Frame2MapReprojectionError_file.c_str());
  // SLAM.SaveTrajectoryTUM(CameraTrajectory_file.c_str());
  // SLAM.SaveKeyFrameTrajectoryTUM(KeyFrameTrajectory_file.c_str());
  // SLAM.SaveKeyFrameLoopTrajectoryTUM(KeyFrameLoopTrajectory_file.c_str());
  return 0;
}

void LoadImages(const string &strAssociationFilename,
                vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD,
                vector<double> &vTimestamps) {
  ifstream fAssociation;
  fAssociation.open(strAssociationFilename.c_str());
  int tt = 0;
  while (!fAssociation.eof()) {
    string s;
    getline(fAssociation, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      string sRGB, sD;
      ss >> t;
      // t = tt;
      vTimestamps.push_back(t / 1e3);
      ss >> sRGB;
      vstrImageFilenamesRGB.push_back(sRGB);
      // ss >> t;
      ss >> sD;
      vstrImageFilenamesD.push_back(sD);
    }
    tt++;
  }
}
