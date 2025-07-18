/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez
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

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps,
             vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);
void LoadOdom(const string &strOdomPath, vector<double> &vTimeStamps,
              vector<Eigen::Vector3f> &vPos);
int main(int argc, char **argv) {
  // use the euroc format data package
  if (argc != 5) {
    cerr << endl
         << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings "
            "path_to_sequence path_to_association"
         << endl;
    return 1;
  }
  // open setting to get image resolution
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
  LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD,
             vTimestamps);

  vector<cv::Point3f> vAcc, vGyro;
  vector<double> vTimestampsImu, vTimestampsOdom;
  int first_imu = 0;
  int first_odom = 0;
  std::string imu_path = string(argv[3]) + "/imu/imu.txt";
  std::cout << "IMU path: " << imu_path << std::endl;
  LoadIMU(imu_path, vTimestampsImu, vAcc, vGyro);
  int bUseOdom = fsSettings["UseRobotOdom"];
  vector<Eigen::Vector3f> vPos;
  if (bUseOdom) {
    std::string odom_path = string(argv[3]) + "/imu/odom.txt";
    std::cout << "Odom path: " << odom_path << std::endl;
    LoadOdom(odom_path, vTimestampsOdom, vPos);
  }
  while (vTimestampsImu[first_imu] <= vTimestamps[0]) first_imu++;
  first_imu--;  // first imu measurement to be considered
  if(bUseOdom){
    while (vTimestampsOdom[first_odom] <= vTimestamps[0]) first_odom++;
    first_odom--;  // first odom measurement to be considered
  }

  // Check consistency in the number of images and depthmaps
  int nImages = vstrImageFilenamesRGB.size();
  if (vstrImageFilenamesRGB.empty()) {
    cerr << endl << "No images found in provided path." << endl;
    return 1;
  } else if (vstrImageFilenamesD.size() != vstrImageFilenamesRGB.size()) {
    cerr << endl << "Different number of images for rgb and depth." << endl;
    return 1;
  }
  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;
  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  int bUseViewer = fsSettings["UseViewer"];
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, bUseViewer, 0,
                         std::string(), std::string(argv[3]));
  // ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true,0,
  // std::string(),std::string(argv[3]));
  float imageScale = SLAM.GetImageScale();
  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  // Main loop
  cv::Mat imRGB, imD;
  vector<ORB_SLAM3::IMU::Point> vImuMeas;
  vector<Eigen::Vector3f> vOdomMeas;
#ifdef COMPILEDWITHC11
  std::chrono::steady_clock::time_point start_time =
      std::chrono::steady_clock::now();
#else
  std::chrono::monotonic_clock::time_point start_time =
      std::chrono::monotonic_clock::now();
#endif
  for (int ni = 0; ni < nImages; ni++) {
    // Read image and depthmap from file
    imRGB = cv::imread(string(argv[3]) + "/" + vstrImageFilenamesRGB[ni],
                       cv::IMREAD_UNCHANGED);  //,cv::IMREAD_UNCHANGED);
    imD = cv::imread(string(argv[3]) + "/" + vstrImageFilenamesD[ni],
                     cv::IMREAD_UNCHANGED);  //,cv::IMREAD_UNCHANGED);
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

    // Load imu measurements from previous frame
    vImuMeas.clear();
    vOdomMeas.clear();

    if (ni > 0) {
      while (vTimestampsImu[first_imu] <= vTimestamps[ni]) {
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(
            vAcc[first_imu].x, vAcc[first_imu].y, vAcc[first_imu].z,
            vGyro[first_imu].x, vGyro[first_imu].y, vGyro[first_imu].z,
            vTimestampsImu[first_imu]));
        first_imu++;
      }
    }

    // Load odom measurements from previous frame
    if (bUseOdom&&ni>0) {
      while (vTimestampsOdom[first_odom] <= vTimestamps[ni]) {

        vOdomMeas.push_back(vPos[first_odom]);
        first_odom++;
      }
    }


    if (vImuMeas.size() < 3) continue;
#ifdef ENABLE_ASYNC
    SLAM.TrackRGBDAsync(imRGB, imD, tframe, vImuMeas,vOdomMeas);
#else
    if (bUseOdom){
      SLAM.TrackRGBD(imRGB, imD, tframe, vImuMeas, vOdomMeas);
    }else{
      SLAM.TrackRGBD(imRGB, imD, tframe, vImuMeas);
    }
#endif
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 =
        std::chrono::monotonic_clock::now();
#endif

    double ttrack = 1000.0;

    ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1)
                 .count();
    vTimesTrack.emplace_back(ttrack);


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
  SLAM.Shutdown(std::string(argv[3]));
  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  double totaltime = 0;
  for (int ni = 0; ni < vTimesTrack.size(); ni++) {
    totaltime += vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "total time: " << totaltime << endl;
  cout << "median tracking time: " << vTimesTrack[vTimesTrack.size() / 2]
       << endl;
  cout << "mean tracking time: " << totaltime / vTimesTrack.size() << endl;
  cout << "tracking lost count: " << SLAM.GetTrackLostCnt() << endl;
  cout << "all image count: " << vTimesTrack.size() << endl;
  cout << "tracking lost count: "
       << float(SLAM.GetTrackLostCnt()) / vTimesTrack.size() << endl;

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
        string(argv[3]) + "/CameraTrajectory_OF_ICP_Lidar.txt";
    KeyFrameTrajectory_file =
        string(argv[3]) + "/KeyFrameTrajectory_OF_ICP_Lidar.txt";
    KeyFrameLoopTrajectory_file =
        string(argv[3]) + "/KeyFrameLoopTrajectory_OF_ICP_Lidar.txt";
    Frame2FrameReprojectionError_file =
        string(argv[3]) + "/Frame2FrameReprojectionError_OF_ICP_Lidar.txt";
    Frame2MapReprojectionError_file =
        string(argv[3]) + "/Frame2MapReprojectionError_OF_ICP_Lidar.txt";
  } else if (!setting->useOpticalFlow() && setting->useICP()) {
    CameraTrajectory_file = string(argv[3]) + "/CameraTrajectory_NoOF_ICP.txt";
    KeyFrameTrajectory_file =
        string(argv[3]) + "/KeyFrameTrajectory_NoOF_ICP.txt";
    KeyFrameLoopTrajectory_file =
        string(argv[3]) + "/KeyFrameLoopTrajectory_NoOF_ICP.txt";
    Frame2FrameReprojectionError_file =
        string(argv[3]) + "/Frame2FrameReprojectionError_NoOF_ICP.txt";
    Frame2MapReprojectionError_file =
        string(argv[3]) + "/Frame2MapReprojectionError_NoOF_ICP.txt";
  } else if (!setting->useICP() && setting->useOpticalFlow()) {
    CameraTrajectory_file = string(argv[3]) + "/CameraTrajectory_OF_NoICP.txt";
    KeyFrameTrajectory_file =
        string(argv[3]) + "/KeyFrameTrajectory_OF_NoICP.txt";
    KeyFrameLoopTrajectory_file =
        string(argv[3]) + "/KeyFrameLoopTrajectory_OF_NoICP.txt";
    Frame2FrameReprojectionError_file =
        string(argv[3]) + "/Frame2FrameReprojectionError_OF_NoICP.txt";
    Frame2MapReprojectionError_file =
        string(argv[3]) + "/Frame2MapReprojectionError_OF_NoICP.txt";
  } else if (setting->useOpticalFlow() && setting->useICP()) {
    CameraTrajectory_file = string(argv[3]) + "/CameraTrajectory_OF_ICP.txt";
    KeyFrameTrajectory_file =
        string(argv[3]) + "/KeyFrameTrajectory_OF_ICP.txt";
    KeyFrameLoopTrajectory_file =
        string(argv[3]) + "/KeyFrameLoopTrajectory_OF_ICP.txt";
    Frame2FrameReprojectionError_file =
        string(argv[3]) + "/Frame2FrameReprojectionError_OF_ICP.txt";
    Frame2MapReprojectionError_file =
        string(argv[3]) + "/Frame2MapReprojectionError_OF_ICP.txt";
  } else if (!setting->useOpticalFlow() && !setting->useICP() &&
             setting->useLidarObs()) {
    CameraTrajectory_file =
        string(argv[3]) + "/CameraTrajectory_NoOF_NoICP_Lidar.txt";
    KeyFrameTrajectory_file =
        string(argv[3]) + "/KeyFrameTrajectory_NoOF_NoICP_Lidar.txt";
    KeyFrameLoopTrajectory_file =
        string(argv[3]) + "/KeyFrameLoopTrajectory_NoOF_NoICP_Lidar.txt";
    Frame2FrameReprojectionError_file =
        string(argv[3]) + "/Frame2FrameReprojectionError_NoOF_NoICP_Lidar.txt";
    Frame2MapReprojectionError_file =
        string(argv[3]) + "/Frame2MapReprojectionError_NoOF_NoICP_Lidar.txt";
  }
  // Save timestamp
#ifdef ENABLE_ASYNC
  std::string TimeStamp_file = string(argv[3]) + "/AsyncTrackCost.txt";
#else
  std::string TimeStamp_file = string(argv[3]) + "/NoAsyncTrackCost.txt";
#endif

  ofstream f;
  f.open(TimeStamp_file);
  f << fixed;
  f << "-------" << endl << endl;
  f << "total time: " << totaltime << endl;
  f << "median tracking time: " << vTimesTrack[vTimesTrack.size() / 2] << endl;
  f << "mean tracking time: " << totaltime / vTimesTrack.size() << endl;
  f << "tracking lost count: " << SLAM.GetTrackLostCnt() << endl;
  f << "all image count: " << vTimesTrack.size() << endl;
  f << "tracking lost count: "
    << float(SLAM.GetTrackLostCnt()) / vTimesTrack.size() << endl;
  f.close();
  SLAM.SaveFrame2FrameReprojErr(Frame2FrameReprojectionError_file.c_str());
  SLAM.SaveFrame2MapReprojErr(Frame2MapReprojectionError_file.c_str());
  SLAM.SaveTrajectoryTUM(CameraTrajectory_file.c_str());
  SLAM.SaveKeyFrameTrajectoryTUM(KeyFrameTrajectory_file.c_str());

  return 0;
}

void LoadImages(const string &strAssociationFilename,
                vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD,
                vector<double> &vTimestamps) {
  ifstream fAssociation;
  fAssociation.open(strAssociationFilename.c_str());
  while (!fAssociation.eof()) {
    string s;
    getline(fAssociation, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      string sRGB, sD;
      ss >> t;
      // ms to s
      vTimestamps.push_back(t / 1e3);
      ss >> sRGB;
      vstrImageFilenamesRGB.push_back(sRGB);
      // ss >> t;
      ss >> sD;
      vstrImageFilenamesD.push_back(sD);
    }
  }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps,
             vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro) {
  ifstream fImu;
  fImu.open(strImuPath.c_str());
  vTimeStamps.reserve(40000);
  vAcc.reserve(40000);
  vGyro.reserve(40000);

  while (!fImu.eof()) {
    string s;
    getline(fImu, s);
    if (s[0] == '#') continue;
    // cout<<s<<endl;
    if (!s.empty()) {
      string item;
      size_t pos = 0;
      double data[7];
      int count = 0;
      while ((pos = s.find(',')) != string::npos) {
        item = s.substr(0, pos);
        data[count++] = stod(item);
        s.erase(0, pos + 1);
      }
      item = s.substr(0, pos);
      data[6] = stod(item);
      // cout << data[0] << " " << data[1] << " " << data[2] << " " << data[3]
      //      << " " << data[4] << " " << data[5] << " " << data[6] << endl;
      vTimeStamps.push_back(data[0] / 1e3);
      vGyro.push_back(cv::Point3f(data[4], data[5], data[6]));
      vAcc.push_back(cv::Point3f(data[1], data[2], data[3]));
    }
  }
}

void LoadOdom(const string &strOdomPath, vector<double> &vTimeStamps,
              vector<Eigen::Vector3f> &vPos) {
  ifstream fImu;
  fImu.open(strOdomPath.c_str());
  vTimeStamps.reserve(40000);
  vPos.reserve(40000);

  while (!fImu.eof()) {
    string s;
    getline(fImu, s);
    if (s[0] == '#') continue;
    // cout<<s<<endl;
    if (!s.empty()) {
      string item;
      size_t pos = 0;
      double data[7];
      int count = 0;
      while ((pos = s.find(',')) != string::npos) {
        item = s.substr(0, pos);
        data[count++] = stod(item);
        s.erase(0, pos + 1);
      }
      item = s.substr(0, pos);
      data[6] = stod(item);
      vTimeStamps.push_back(data[0] / 1e3);
      vPos.push_back({data[1], data[2], data[3]});
    }
  }
}